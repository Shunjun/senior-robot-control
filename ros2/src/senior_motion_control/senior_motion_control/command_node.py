"""Publish arbitration results to the safety gate.

This process does not publish /cmd_vel. Manual authority and task source samples
share FinalCommand with immutable local deadlines and source generations.
Estop, release and source retirement publish explicit barriers. The watchdog reads the injected
monotonic clock and keeps running when ROS time is frozen. The timer period is
configuration, not a physical braking guarantee.
"""

import math
import re
import secrets
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any

from geometry_msgs.msg import Twist
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from senior_motion_msgs.msg import DriveExecutionFeedback, FinalCommand
from senior_sensor_ingress.native_receipt import with_dds_info
from std_msgs.msg import String

from senior_control.agent.manual_control import (
    ManualIngress,
    ManualTarget,
    MotionRuntimeReceiver,
)
from senior_control.clock import Clock, SystemClock
from senior_control.errors import ErrorCode
from senior_control.models import Velocity
from senior_control.motion.contracts import ManualIntent
from senior_control.motion.manual_authority import ManualAuthorityError, ManualAuthorityState
from senior_control.protocol import (
    Envelope,
    ManualIntentEnvelope,
    MotionRuntimeState,
    RecoveryEnvelope,
    SimpleEnvelope,
)
from senior_control.settings.models import DrivingSettings
from senior_motion_control.arbiter import Arbiter, CommandResult


@dataclass
class _NavigationSource:
    execution_id: str
    topic: str
    subscription: Subscription
    publisher_gid: bytes
    qualified: Callable[[], bool]
    sequence: int = 0
    received_sequence: int = 0
    last_forwarded: float = float("-inf")
    task_epoch: int = 0
    received_steady: float = 0.0


@dataclass(frozen=True)
class _NavigationRetirement:
    token: str
    generation: int
    # Immutable issued source payload: kind/id/scope/receipt/session/connection.
    source: tuple[str, str, int, float, str, int]
    reason: str
    through_epoch: int
    acknowledged: bool = False
    # Actual retained-task epoch returned by the core, not the issued fence scope.
    task_epoch: int = 0


@dataclass
class _TerminalRelease:
    execution_id: str
    through_epoch: int
    outcome: str
    proof: _NavigationRetirement | None = None
    acknowledged: bool = False
    dispatch_sequence: int = 0


@dataclass
class _TerminalCollection:
    execution_id: str
    through_epoch: int
    dispatch_sequence: int
    frontier: int
    acknowledged: bool = False


class CommandNode(Node):
    def __init__(
        self,
        arbiter: Arbiter | None = None,
        clock: Clock | None = None,
        *,
        parameter_overrides: list[Parameter] | None = None,
    ) -> None:
        super().__init__("command_node", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("robot_id", "robot")
        self.declare_parameter("native_writer_receipts", False)
        self._native_writer_receipts = bool(self.get_parameter("native_writer_receipts").value)
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("command_ttl_ms", 300.0)
        self.declare_parameter("final_command_topic", "/safety/final_command")
        self._monotonic_clock = clock if clock is not None else SystemClock()
        if arbiter is None:
            self._arbiter = Arbiter(
                robot_id=str(self.get_parameter("robot_id").value),
                command_ttl_ms=float(self.get_parameter("command_ttl_ms").value),
            )
        else:
            self._arbiter = arbiter
        self._final_publisher = self.create_publisher(
            FinalCommand, str(self.get_parameter("final_command_topic").value), 10
        )
        self._owner_boot = secrets.token_hex(16)
        self._manual_ingress = ManualIngress(
            str(self.get_parameter("robot_id").value),
            self._owner_boot,
            ttl=float(self.get_parameter("command_ttl_ms").value) / 1000,
        )
        self._manual_target: ManualTarget | None = None
        self._motion_runtime = MotionRuntimeReceiver()
        # Request delivery frontier only; the output state machine owns the latch.
        self._pending_estop_request: FinalCommand | None = None
        self._latched_estop_request: FinalCommand | None = None
        self._latched_estop_revision = 0
        self._pending_estop_reset: FinalCommand | None = None
        self._output_incarnation: tuple[bytes, str] | None = None
        self._output_handoff_boot: str | None = None
        self._pending_output_handoff: FinalCommand | None = None
        self._output_handoff_missing_estop = False
        self._close_reason = "prepared"
        self._last_close_source = ("idle", "", 0, 0.0, "", 0)
        self._consuming_runtime = False
        self._runtime_subscription = None
        if self._native_writer_receipts:
            from senior_sensor_ingress.native_receipt import subscribe

            subscribe(
                self,
                String,
                "/motion/runtime",
                self._on_motion_runtime,
                writer="motion_reporter",
                qos="reliable",
                invalidate=self._invalidate_motion_runtime,
            )
        else:
            # //Compatible: ordinary rclpy consumers retain their metadata path.
            self._runtime_subscription = self.create_subscription(
                String, "/motion/runtime", self._on_motion_runtime, qos_profile_sensor_data
            )
        if self._native_writer_receipts:
            subscribe(
                self,
                DriveExecutionFeedback,
                "/safety/drive_feedback",
                self._on_drive_feedback,
                writer="safety_gate",
                qos="sensor",
                invalidate=self._invalidate_drive_feedback,
            )
        else:
            self.create_subscription(
                DriveExecutionFeedback,
                "/safety/drive_feedback",
                self._on_drive_feedback,
                qos_profile_sensor_data,
            )
        self._navigation_source: _NavigationSource | None = None
        self._retirements: dict[str, _NavigationRetirement] = {}
        self._terminal_requests: dict[str, _TerminalRelease] = {}
        self._terminal_collections: dict[str, _TerminalCollection] = {}
        self._settled_sequence = 0
        self._retirement_core: tuple[bytes, str] | None = None
        self._retirement_feedback_gid: bytes | None = None
        self._generation = 1
        self._estop_requests: dict[str, tuple[FinalCommand, CommandResult]] = {}
        self._active = False
        rate = float(self.get_parameter("command_rate_hz").value)
        ttl = float(self.get_parameter("command_ttl_ms").value) / 1000
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("command_rate_hz must be positive")
        period = min(1.0 / rate, ttl / 2.0)
        steady = RosClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(period, self.publish_command, clock=steady)

    def submit(
        self,
        envelope: Envelope,
        received_at: float | None = None,
    ) -> CommandResult:
        if envelope.robot_id != str(self.get_parameter("robot_id").value):
            return CommandResult(False, ErrorCode.VALIDATION)
        requests = getattr(self, "_estop_requests", None)
        if requests is None:
            requests = self._estop_requests = {}
        if envelope.type == "safety.estop" and envelope.request_id in requests:
            frame, receipt = requests[envelope.request_id]
            self._final_publisher.publish(frame)
            return receipt
        source_closed = False
        if envelope.type == "drive.stop" and hasattr(self, "_manual_ingress"):
            identity, gesture = envelope.payload.authority_id, envelope.payload.gesture
            if identity is not None:
                context = self._manual_ingress.context
                if context is None or (
                    envelope.connection_epoch,
                    envelope.controller_generation,
                    envelope.session_id,
                ) != (context.agent_epoch, context.controller_generation, context.session_id):
                    return CommandResult(False, ErrorCode.CONFLICT)
                if gesture is None or not self._manual_ingress.stop(
                    identity, gesture, self._monotonic_clock.monotonic()
                ):
                    return CommandResult(False, ErrorCode.CONFLICT)
                target = self._manual_target
                if (
                    target is not None
                    and target.sample.intent.authority_id == identity
                    and (target.sample.intent.gesture <= gesture)
                ):
                    self._close_manual("release")
                # A stale stop must not publish a barrier over a newer gesture.
                self.publish_command()
                return CommandResult(True)
            # //Compatible: unscoped safety stops still revoke the whole authority.
            self._manual_ingress.revoke(self._monotonic_clock.monotonic())
            self._close_reason = "release"
            source_closed = self._close_manual("release")
        if envelope.type == "drive.stop":
            result = CommandResult(True)
        else:
            result = self._arbiter.accept(envelope, self._monotonic_clock.monotonic())
        estop_frame = None
        if envelope.type == "safety.estop":
            estop_frame = self._explicit_estop(envelope, received_at)
            self._pending_estop_request = estop_frame
            source_closed = True
        if result.accepted and not source_closed:
            self._generation += 1
            self._active = False
            self._publish_barrier()
        self.publish_command()
        if estop_frame is not None:
            requests[envelope.request_id] = estop_frame, result
        return result

    def _explicit_estop(self, envelope: Envelope, received: float | None) -> FinalCommand:
        now = self._monotonic_clock.monotonic()
        runtime = self._motion_runtime.value
        self._latched_estop_revision = (
            max(
                getattr(self, "_latched_estop_revision", 0),
                runtime.estop_revision if runtime is not None else 0,
            )
            + 1
        )
        self._output_handoff_missing_estop = False
        target, self._manual_target = getattr(self, "_manual_target", None), None
        if hasattr(self, "_manual_ingress"):
            self._manual_ingress.revoke(now)
        self._last_close_source = (
            "manual" if target is not None else "idle",
            target.sample.intent.authority_id if target is not None else "",
            target.source_epoch if target is not None else 0,
            received if received is not None else now,
            envelope.session_id,
            envelope.connection_epoch,
        )
        self._generation += 1
        self._active = False
        self._close_reason = "estop"
        self._publish_barrier(envelope.request_id, reason="estop")
        self._latched_estop_request = self._barrier_frame
        return self._barrier_frame

    def disconnect(self, connection_epoch: int) -> None:
        if hasattr(self, "_manual_ingress"):
            context = self._manual_ingress.context
            if context is not None and context.agent_epoch <= connection_epoch:
                self._manual_ingress.retire(self._monotonic_clock.monotonic())
                self._close_manual("connection_retired")
        if not self._arbiter.task_navigation_active:
            self._generation += 1
            self._active = False
            self._publish_barrier()
        self.publish_command()

    def navigation_qualified(self) -> bool:
        if getattr(self, "_manual_target", None) is not None:
            return False
        return self.task_motion_allowed(continuing=True)

    def _consume_ready_runtime(self) -> None:
        """Consume at most the existing QoS depth through the sole runtime ingress."""
        if self._runtime_subscription is None or self._consuming_runtime:
            return
        self._consuming_runtime = True
        try:
            subscription = self._runtime_subscription
            for _ in range(qos_profile_sensor_data.depth):
                # //Compatible: preserve Humble metadata with MotionExecutor's native take API.
                with subscription.handle:
                    sample = subscription.handle.take_message(
                        subscription.msg_type, subscription.raw
                    )
                if sample is None:
                    break
                self._on_motion_runtime(*sample)
        finally:
            self._consuming_runtime = False

    def task_motion_allowed(self, *, continuing: bool) -> bool:
        # Source ownership only; final control owns physical motion decisions.
        return getattr(self, "_pending_estop_request", None) is None and self._manual_target is None

    def bind_navigation_source(
        self,
        execution_id: str,
        topic: str,
        *,
        qualified: Callable[[], bool],
        task_epoch: int,
    ) -> bool:
        # Bind the exact producer incarnation, not merely its node name.
        endpoints = self.get_publishers_info_by_topic(topic)
        if len(endpoints) != 1 or endpoints[0].node_name != "controller_server":
            return False
        publisher_gid = bytes(endpoints[0].endpoint_gid)
        # An execution-specific topic must be configured at the Nav2 publisher.
        if (
            self._navigation_source is not None
            or not qualified()
            or not re.fullmatch(r"/control/navigation/run_[0-9a-f]{32}/cmd_vel", topic)
            or not self._arbiter.begin_navigation(
                execution_id, self._monotonic_clock.monotonic(), task_epoch=task_epoch
            )
        ):
            return False
        try:

            @with_dds_info
            def on_sample(message: Twist, info: dict[str, Any]) -> None:
                self._on_navigation_sample(topic, message, info)

            subscription = self.create_subscription(
                Twist, topic, on_sample, qos_profile_sensor_data
            )
        except Exception:
            self._arbiter.end_navigation(execution_id)
            raise
        if hasattr(self, "_manual_ingress"):
            self._manual_ingress.revoke(self._monotonic_clock.monotonic())
        self._generation += 1
        self._active = False
        self._last_close_source = (
            "navigation",
            execution_id,
            task_epoch,
            self._monotonic_clock.monotonic(),
            execution_id,
            task_epoch,
        )
        self._close_reason = "prepared"
        self._publish_barrier()
        self._retirements.pop(execution_id, None)
        self._navigation_source = _NavigationSource(
            execution_id,
            topic,
            subscription,
            publisher_gid,
            qualified,
            task_epoch=task_epoch,
        )
        return True

    def navigation_source_matches(self, execution_id: str) -> bool:
        source = self._navigation_source
        return (
            source is not None
            and source.execution_id == execution_id
            and self._arbiter.navigation_source_active(
                execution_id, self._monotonic_clock.monotonic()
            )
        )

    def _other_motion_active(self, execution_id: str) -> bool:
        source = self._navigation_source
        manual = self._manual_ingress.view(self._monotonic_clock.monotonic())
        return (
            self._manual_target is not None
            or (manual is not None and manual.state is not ManualAuthorityState.CLOSED)
            or (source is not None and source.execution_id != execution_id)
        )

    def navigation_source_retired(self, execution_id: str) -> bool:
        receipt = self._retirements.get(execution_id)
        if receipt is None:
            return False
        if receipt.acknowledged:
            return self._retirement_incarnation_current()
        # Missing proof may be reacquired only through the original output boundary;
        # an old task must never displace a later manual/navigation owner.
        if self._other_motion_active(execution_id):
            return False
        if receipt.generation < self._generation:
            self._generation += 1
            receipt = replace(receipt, generation=self._generation)
            self._retirements[execution_id] = receipt
        self._publish_barrier(
            receipt.token,
            execution_id,
            receipt.generation,
            reason=receipt.reason,
            source_override=receipt.source,
        )
        return False

    def complete_navigation_source(self, execution_id: str) -> None:
        self.clear_navigation_source(execution_id, reason="completed")

    def retire_navigation_source(self, execution_id: str, through_epoch: int) -> None:
        source = self._navigation_source
        if self._scope_collected(execution_id, through_epoch):
            return
        if self._other_motion_active(execution_id) or (
            source is not None and (source.task_epoch) > through_epoch
        ):
            return
        self.clear_navigation_source(execution_id, through_epoch=through_epoch)

    def clear_navigation_source(
        self, execution_id: str, *, reason: str = "requested", through_epoch: int | None = None
    ) -> None:
        source = self._navigation_source
        if self._other_motion_active(execution_id):
            return
        if source is not None:
            self._last_close_source = (
                "navigation",
                source.execution_id,
                source.task_epoch,
                source.received_steady,
                source.execution_id,
                source.task_epoch,
            )
            self._close_reason = {
                "requested": "task_revoke",
                "completed": "task_done",
                "qualification_lost": "qualification_lost",
                "navigation_source_inactive": "expiry",
            }.get(reason, "task_revoke")
            self._navigation_source = None
            self._arbiter.end_navigation(execution_id)
            self.destroy_subscription(source.subscription)
            self.get_logger().warning(f"navigation_source_cleared reason={reason}")
        receipt = self._retirements.get(execution_id)
        if receipt is None:
            if len(self._retirements) >= 256:
                # Metadata exhaustion cannot defer the original immediate zero boundary.
                # No nonce/proof is minted, so identity release still fails closed.
                self._generation += 1
                self._active = False
                self._publish_barrier()
                return
            close = self._last_close_source
            if close[0] != "navigation" or close[1] != execution_id:
                if through_epoch is None:
                    return
                close = (
                    "navigation",
                    execution_id,
                    through_epoch,
                    self._monotonic_clock.monotonic(),
                    execution_id,
                    through_epoch,
                )
                self._close_reason = "task_revoke"
            if close[2] < 1 and through_epoch is not None:
                close = (close[0], close[1], through_epoch, close[3], close[4], close[5])
            self._generation += 1
            self._active = False
            receipt = _NavigationRetirement(
                secrets.token_hex(16),
                self._generation,
                close,
                self._close_reason,
                through_epoch or close[2],
            )
        elif through_epoch is not None:
            if receipt.source[2] < 1:
                original = receipt.source
                receipt = replace(
                    receipt,
                    token=secrets.token_hex(16),
                    generation=0,
                    source=(original[0], original[1], through_epoch, *original[3:]),
                    acknowledged=False,
                    task_epoch=0,
                )
            receipt = replace(receipt, through_epoch=max(receipt.through_epoch, through_epoch))
        self._retirements[execution_id] = receipt
        self.navigation_source_retired(execution_id)
        self.publish_command()

    def request_terminal_release(
        self,
        execution_id: str,
        through_epoch: int,
        effect_id: str,
        outcome: str,
        *,
        dispatch_sequence: int = 0,
    ) -> bool | None:
        if (
            type(through_epoch) is not int
            or through_epoch < 1
            or not 1 <= len(execution_id) <= 128
            or not 1 <= len(effect_id) <= 128
            or outcome not in {"succeeded", "canceled", "failed"}
        ):
            return None
        if (
            dispatch_sequence <= self._settled_sequence and self._settled_sequence > 0
        ) or self._scope_collected(execution_id, through_epoch):
            return False
        request = self._terminal_requests.get(effect_id)
        if request is not None:
            if (
                request.execution_id,
                request.through_epoch,
                request.outcome,
                request.dispatch_sequence,
            ) != (execution_id, through_epoch, outcome, dispatch_sequence):
                return False
        else:
            if len(self._terminal_requests) >= 256:
                return None
            self._terminal_requests[effect_id] = _TerminalRelease(
                execution_id, through_epoch, outcome, dispatch_sequence=dispatch_sequence
            )
        return self.terminal_release_status(effect_id)

    def _scope_collected(self, execution_id: str, epoch: int) -> bool:
        # A settled scope behind an unresolved global gap keeps one bounded
        # compact marker; it cannot re-handshake while waiting for the frontier.
        return any(
            item.acknowledged and item.execution_id == execution_id and epoch <= item.through_epoch
            for item in self._terminal_collections.values()
        )

    def _compact_terminal_metadata(self, frontier: int) -> None:
        self._settled_sequence = max(self._settled_sequence, frontier)
        for key, request in list(self._terminal_requests.items()):
            if 0 < request.dispatch_sequence <= self._settled_sequence:
                self._terminal_requests.pop(key)
        for key, item in list(self._terminal_collections.items()):
            if item.acknowledged and item.dispatch_sequence <= self._settled_sequence:
                self._terminal_collections.pop(key)

    def query_terminal_collection(
        self,
        execution_id: str,
        through_epoch: int,
        effect_id: str,
        dispatch_sequence: int,
        frontier: int,
    ) -> tuple[bool, bool | None]:
        """Query only an already issued GC; never allocate a receiver slot."""
        if (
            type(dispatch_sequence) is not int
            or dispatch_sequence < 1
            or type(through_epoch) is not int
            or through_epoch < 1
            or type(frontier) is not int
            or frontier < 0
            or not isinstance(execution_id, str)
            or not 1 <= len(execution_id) <= 128
            or not isinstance(effect_id, str)
            or not 1 <= len(effect_id) <= 128
        ):
            return False, None
        if dispatch_sequence <= self._settled_sequence:
            return True, True  # Previously accepted durable prefix, metadata ACK only.
        known = self._terminal_collections.get(effect_id)
        if known is None:
            return False, None
        if (known.execution_id, known.through_epoch, known.dispatch_sequence) != (
            execution_id,
            through_epoch,
            dispatch_sequence,
        ):
            return True, False
        return True, self.collect_terminal(
            execution_id, through_epoch, effect_id, dispatch_sequence, frontier
        )

    def collect_terminal(
        self,
        execution_id: str,
        through_epoch: int,
        effect_id: str,
        dispatch_sequence: int,
        frontier: int,
    ) -> bool | None:
        """Only the durable, post-settlement outbox may call this metadata boundary."""
        if (
            type(dispatch_sequence) is not int
            or dispatch_sequence < 1
            or type(frontier) is not int
            or frontier < 0
            or type(through_epoch) is not int
            or through_epoch < 1
        ):
            return None
        if dispatch_sequence <= self._settled_sequence:
            return True
        if frontier >= dispatch_sequence:
            return None
        self._compact_terminal_metadata(frontier)
        current = self._terminal_collections.get(effect_id)
        if current is None:
            if len(self._terminal_collections) >= 256:
                return None
            current = _TerminalCollection(execution_id, through_epoch, dispatch_sequence, frontier)
            self._terminal_collections[effect_id] = current
        elif (current.execution_id, current.through_epoch, current.dispatch_sequence) != (
            execution_id,
            through_epoch,
            dispatch_sequence,
        ):
            return False
        current.frontier = max(current.frontier, frontier)
        if not self._retirement_incarnation_current():
            return None
        if current.acknowledged:
            self._compact_terminal_metadata(current.frontier)
            for key, request in list(self._terminal_requests.items()):
                if (
                    0 < request.dispatch_sequence <= self._settled_sequence
                    or request.execution_id == execution_id
                    and request.through_epoch <= through_epoch
                ):
                    self._terminal_requests.pop(key)
            proof = self._retirements.get(execution_id)
            if proof is not None and proof.through_epoch <= through_epoch:
                self._retirements.pop(execution_id)
            for key, item in list(self._terminal_collections.items()):
                if item.acknowledged and item.dispatch_sequence <= self._settled_sequence:
                    self._terminal_collections.pop(key)
            return True
        msg = FinalCommand()
        msg.robot_id = str(self.get_parameter("robot_id").value)
        msg.owner_boot_id = self._owner_boot
        msg.source_kind, msg.source_id, msg.session_id = "navigation", execution_id, execution_id
        msg.source_epoch, msg.connection_epoch = through_epoch, through_epoch
        msg.generation, msg.request_id = 0, effect_id
        msg.received_steady = self._monotonic_clock.monotonic()
        msg.close_reason = "task_settled"
        msg.sequence, msg.gesture = 0, 0
        msg.steady_deadline, msg.vx, msg.wz = 0.0, 0.0, 0.0
        self._final_publisher.publish(msg)
        return None

    def terminal_release_status(self, effect_id: str) -> bool | None:
        request = self._terminal_requests.get(effect_id)
        if request is None:
            return None
        if not self._retirement_incarnation_current():
            return None
        if request.acknowledged:
            return True
        if request.proof is None:
            receipt = self._retirements.get(request.execution_id)
            if receipt is None or not receipt.acknowledged:
                self.retire_navigation_source(request.execution_id, request.through_epoch)
                return None
            if not 0 < receipt.task_epoch <= request.through_epoch:
                return None
            request.proof = receipt
        proof = request.proof
        # Independent identity frame. Never overwrite _barrier_frame or _generation.
        msg = FinalCommand()
        msg.robot_id = str(self.get_parameter("robot_id").value)
        msg.owner_boot_id = self._owner_boot
        msg.source_kind, msg.source_id = "navigation", request.execution_id
        msg.session_id = request.execution_id
        msg.source_epoch, msg.connection_epoch = proof.task_epoch, proof.source[5]
        msg.generation, msg.request_id = proof.generation, proof.token
        msg.received_steady, msg.close_reason = proof.source[3], "task_terminal"
        msg.sequence, msg.gesture = 0, 0
        msg.steady_deadline, msg.vx, msg.wz = 0.0, 0.0, 0.0
        self._final_publisher.publish(msg)
        return None

    def _invalidate_retirement_proofs(self) -> None:
        for execution, proof in self._retirements.items():
            self._retirements[execution] = replace(
                proof,
                token=secrets.token_hex(16),
                generation=0,
                acknowledged=False,
                task_epoch=0,
            )
        for request in self._terminal_requests.values():
            request.proof = None
            request.acknowledged = False
        for collection in self._terminal_collections.values():
            collection.acknowledged = False

    def _retirement_incarnation_current(self) -> bool:
        runtime = self.get_publishers_info_by_topic("/motion/runtime")
        feedback = self.get_publishers_info_by_topic("/safety/drive_feedback")
        good = all(
            len(rows) == 1 and rows[0].node_name == writer and rows[0].node_namespace == "/"
            for rows, writer in ((runtime, "motion_reporter"), (feedback, "safety_gate"))
        )
        current = self._retirement_core
        feedback_gid = bytes(feedback[0].endpoint_gid) if good else None
        if (
            not good
            or current is None
            or bytes(runtime[0].endpoint_gid) != current[0]
            or (
                self._retirement_feedback_gid is not None
                and feedback_gid != self._retirement_feedback_gid
            )
        ):
            self._invalidate_retirement_proofs()
            self._retirement_core = None
            self._retirement_feedback_gid = feedback_gid
            return False
        self._retirement_feedback_gid = feedback_gid
        return True

    def _observe_retirement_core(self, publisher: bytes, boot: str) -> None:
        identity = (publisher, boot)
        previous = self._retirement_core
        if previous == identity:
            return
        if self._retirements:
            self._invalidate_retirement_proofs()
        self._retirement_core = identity
        # ACK admission checks both endpoint frontiers before matching a nonce.
        self._retirement_incarnation_current()

    def _on_navigation_sample(self, topic: str, msg: Twist, info: dict[str, Any]) -> None:
        received = self._monotonic_clock.monotonic()
        source = self._navigation_source
        if source is None or source.topic != topic:
            return
        source.received_sequence += 1
        if not source.qualified():
            self.clear_navigation_source(source.execution_id, reason="qualification_lost")
            return
        endpoints = self.get_publishers_info_by_topic(source.topic)
        # //Compatible: Humble omits per-sample GID. Retain its private run-topic
        # identity contract, bound to this graph incarnation and private run.
        gid = info.get("publisher_gid")
        if (
            len(endpoints) != 1
            or endpoints[0].node_name != "controller_server"
            or bytes(endpoints[0].endpoint_gid) != source.publisher_gid
            or (gid is not None and bytes(gid) != source.publisher_gid)
        ):
            return
        if not all(math.isfinite(v) for v in (msg.linear.x, msg.angular.z)):
            return
        velocity = Velocity(float(msg.linear.x), float(msg.angular.z))
        # A standard Twist has no measurement/production stamp. Qualification
        # work consumes its first local callback receipt, never renews it.
        now = self._monotonic_clock.monotonic()
        # Keep the existing supersedence policy; a producer zero takes precedence.
        if velocity != Velocity(0.0, 0.0) and now - source.last_forwarded < 0.16:
            return
        if self._arbiter.accept_navigation_sample(
            source.execution_id, source.received_sequence, velocity, received, now
        ):
            source.received_steady = received
            source.last_forwarded = now
            source.sequence = source.received_sequence
            self.publish_command()

    def publish_command(self) -> None:
        if getattr(self, "_output_handoff_boot", None) is not None:
            self._publish_barrier()
            return
        if getattr(self, "_pending_estop_request", None) is not None:
            # Retransmit the original request until the output owner confirms it.
            # Source cleanup must not supersede an emergency event still in transit.
            self._publish_barrier()
            return
        if self._publish_manual_target():
            return
        source = self._navigation_source
        if source is not None:
            if not source.qualified():
                self.clear_navigation_source(source.execution_id, reason="qualification_lost")
                return
            if not self._arbiter.navigation_source_active(
                source.execution_id, self._monotonic_clock.monotonic()
            ):
                self.clear_navigation_source(
                    source.execution_id, reason="navigation_source_inactive"
                )
                return
        now = self._monotonic_clock.monotonic()
        final = self._arbiter.final_command(now)
        if final is None:
            if self._active:
                self._generation += 1
                self._active = False
            self._publish_barrier()
            return
        velocity, session, epoch, sequence, deadline = final
        msg = FinalCommand()
        msg.robot_id = str(self.get_parameter("robot_id").value)
        msg.owner_boot_id = self._owner_boot
        msg.session_id = session
        msg.generation = self._generation
        msg.connection_epoch = epoch
        msg.sequence = sequence
        msg.steady_deadline = deadline
        msg.vx, msg.wz = velocity.vx, velocity.wz
        msg.source_kind = "navigation" if source is not None else "idle"
        msg.source_id = source.execution_id if source is not None else ""
        msg.source_epoch = source.task_epoch if source is not None else 0
        msg.received_steady = source.received_steady if source is not None else now
        msg.close_reason = "intent"
        self._final_publisher.publish(msg)
        self._active = True

    def _invalidate_motion_runtime(self) -> None:
        self._motion_runtime.invalidate()

    def _invalidate_drive_feedback(self) -> None:
        self._invalidate_retirement_proofs()
        self._retirement_core = None
        self._retirement_feedback_gid = None

    def _on_output_incarnation_changed(self, previous, current) -> None:
        """Retire old sources before echoing the verified output boot in a zero."""
        now = self._monotonic_clock.monotonic()
        self._output_handoff_boot = current.boot_id
        self._pending_output_handoff = None
        if previous is not None and previous.confirmed_estop_status == "software_latched":
            self._latched_estop_revision = max(
                self._latched_estop_revision, previous.estop_revision
            )
            # Without an original request identity there is no authorized
            # clear-state handoff. Remain stopped until a new explicit estop.
            self._output_handoff_missing_estop = self._latched_estop_request is None
        self._manual_ingress.revoke(now)
        self._manual_target = None
        source = self._navigation_source
        if source is not None:
            self.clear_navigation_source(source.execution_id, reason="requested")
        self._arbiter.shutdown()
        self._generation = max(self._generation, current.output_generation) + 1
        self._active = False
        self._last_close_source = ("idle", "", 0, now, "", 0)
        self._close_reason = "prepared"
        self._publish_barrier()

    @with_dds_info
    def _on_motion_runtime(self, msg: String, info: dict[str, Any] | None = None) -> None:
        endpoints = self.get_publishers_info_by_topic("/motion/runtime")
        if len(endpoints) != 1:
            return
        endpoint = endpoints[0]
        gid = info.get("publisher_gid") if info else None
        if (
            endpoint.node_name != "motion_reporter"
            or endpoint.node_namespace != "/"
            or (gid is not None and bytes(gid) != bytes(endpoint.endpoint_gid))
        ):
            return
        try:
            value = MotionRuntimeState.model_validate_json(msg.data)
        except ValueError:
            return
        now = self._monotonic_clock.monotonic()
        if value.robot_id != str(self.get_parameter("robot_id").value):
            return
        previous = self._motion_runtime.value
        if not self._motion_runtime.accept(value, bytes(endpoint.endpoint_gid), now):
            return
        identity = (bytes(endpoint.endpoint_gid), value.boot_id)
        if identity != self._output_incarnation:
            self._output_incarnation = identity
            self._on_output_incarnation_changed(previous, value)
        self._observe_retirement_core(bytes(endpoint.endpoint_gid), value.boot_id)
        handoff = self._pending_output_handoff
        if (
            handoff is not None
            and value.boot_id == self._output_handoff_boot
            and value.output_generation >= handoff.generation
            and (
                handoff.close_reason != "estop"
                or (
                    value.confirmed_estop_status == "software_latched"
                    and value.estop_revision >= handoff.source_epoch
                )
            )
        ):
            self._output_handoff_boot = None
            self._pending_output_handoff = None
        pending = getattr(self, "_pending_estop_request", None)
        if (
            pending is not None
            and value.output_generation >= pending.generation
            and value.confirmed_estop_status == "software_latched"
        ):
            self._pending_estop_request = None
        if value.confirmed_estop_status == "software_latched":
            self._latched_estop_revision = max(self._latched_estop_revision, value.estop_revision)
        reset = self._pending_estop_reset
        if (
            reset is not None
            and value.output_generation >= reset.generation
            and value.confirmed_estop_status == "clear"
            and self._output_handoff_boot is None
        ):
            self._latched_estop_request = None
            self._pending_estop_reset = None
        if self._output_handoff_boot is not None:
            # Do not expose the replacement's initial clear report as current
            # while the original latch/source handoff is still unconfirmed.
            self._motion_runtime.invalidate()

    def runtime_observation(self) -> dict[str, Any]:
        return self._motion_runtime.observation(self._monotonic_clock.monotonic()).model_dump()

    def motion_runtime(self) -> dict[str, Any] | None:
        value = self._motion_runtime.current(self._monotonic_clock.monotonic())
        return value.model_dump(by_alias=True) if value is not None else None

    def manual_view(self) -> dict[str, Any] | None:
        receipt = self._manual_ingress.view(self._monotonic_clock.monotonic())
        return (
            {
                "authority_id": receipt.authority_id,
                "state": receipt.state.value,
                "controller_generation": receipt.context.controller_generation,
                "connection_epoch": receipt.context.agent_epoch,
            }
            if receipt is not None
            else None
        )

    def manual_request(self, message: dict[str, Any]) -> dict[str, Any]:
        from senior_control.protocol import validate_envelope

        try:
            envelope = validate_envelope(
                message["envelope"].encode(), str(self.get_parameter("robot_id").value)
            )
            now = self._monotonic_clock.monotonic()
            from senior_control.protocol import LearningStartEnvelope, LearningStatusEnvelope

            if isinstance(envelope, (LearningStartEnvelope, LearningStatusEnvelope)):
                if envelope.controller_generation is None:
                    return {"ok": False, "accepted": False, "error": "conflict"}
                runtime = self._motion_runtime.current(now)
                manual = self._manual_ingress.view(now)
                prepared = manual is not None and manual.state is ManualAuthorityState.PREPARED
                context = self._manual_ingress.context
                same_operator = context is not None and (
                    context.agent_epoch,
                    context.controller_generation,
                    context.session_id,
                ) == (
                    envelope.connection_epoch,
                    envelope.controller_generation,
                    envelope.session_id,
                )
                no_holder = manual is None or manual.state is ManualAuthorityState.CLOSED
                can_start = (
                    runtime is not None
                    and runtime.stationary
                    and getattr(self, "_output_handoff_boot", None) is None
                    and runtime.mode in {"idle", "manual"}
                    and self._navigation_source is None
                    and not self._arbiter.task_navigation_active
                    and self._manual_target is None
                    and getattr(self, "_pending_estop_request", None) is None
                    and (no_holder or (prepared and same_operator))
                )
                if isinstance(envelope, LearningStartEnvelope):
                    expected = envelope.payload.expected_authority_id
                    can_start = can_start and (
                        expected is None
                        and no_holder
                        or prepared
                        and same_operator
                        and manual.authority_id == expected
                    )
                    deadline = message.get("deadline_steady")
                    if type(deadline) not in {int, float}:
                        return {"ok": False, "accepted": False, "error": "timeout"}
                    can_start = can_start and now < deadline
                else:
                    deadline = now
                return {
                    "ok": True,
                    "owner_boot_id": self._owner_boot,
                    "output_boot_id": None if runtime is None else runtime.boot_id,
                    "generation": self._generation,
                    "deadline_steady": deadline,
                    "can_start": can_start,
                }
            if isinstance(envelope, RecoveryEnvelope):
                key = "estop_reset" if envelope.type == "safety.estop.reset" else "fault_recover"
                if (
                    getattr(self, "_pending_estop_request", None) is not None
                    or getattr(self, "_output_handoff_boot", None) is not None
                ):
                    return {"ok": False, "accepted": False, "error": "conflict"}
                current = envelope.payload.revision
                self._manual_ingress.revoke(now)
                self._close_manual("fault")
                self._generation += 1
                self._publish_barrier(envelope.request_id, reason=key, revision=current)
                return {"ok": True, "accepted": True}
            if not isinstance(envelope, (ManualIntentEnvelope, SimpleEnvelope)):
                return {"ok": False, "error": "bad_request"}
            generation = envelope.controller_generation
            if generation is None:
                return {"ok": False, "error": "bad_request"}
            previous_context = self._manual_ingress.context
            self._manual_ingress.bind(
                envelope.connection_epoch, generation, envelope.session_id, now
            )
            if previous_context is not None and previous_context != self._manual_ingress.context:
                self._close_manual("connection_retired")
            if envelope.type == "control.release":
                assert isinstance(envelope, SimpleEnvelope)
                identity = envelope.payload.authority_id
                if identity is None:
                    self._manual_ingress.retire(now)
                    self._close_manual("connection_retired")
                elif not self._manual_ingress.release(identity, now):
                    return {"ok": False, "accepted": False, "error": "conflict"}
                else:
                    current = self._manual_ingress.view(now)
                    if current is not None and current.authority_id == identity:
                        self._close_manual("release")
                return {"ok": True, "accepted": True}
            if getattr(self, "_pending_estop_request", None) is not None:
                return {"ok": False, "error": "unavailable"}
            if self._navigation_source is not None or self._arbiter.task_navigation_active:
                return {"ok": False, "accepted": False, "error": "conflict"}
            if envelope.type == "control.acquire":
                previous = self._manual_ingress.view(now)
                receipt = self._manual_ingress.acquire(envelope.request_id, now)
                if receipt.state is ManualAuthorityState.PREPARED and (
                    previous is None or previous.authority_id != receipt.authority_id
                ):
                    self._close_manual("prepared")
                return {
                    "ok": True,
                    "authority_id": receipt.authority_id,
                    "state": receipt.state.value,
                }
            if not isinstance(envelope, ManualIntentEnvelope):
                return {"ok": False, "error": "bad_request"}
            view = self._manual_ingress.view(now)
            if view is None:
                return {"ok": False, "accepted": False, "error": "safety_rejected"}
            sample = self._manual_ingress.accept(
                ManualIntent(**envelope.payload.model_dump()),
                DrivingSettings.model_validate(message["driving"]),
                message["received_steady"],
                message["deadline_steady"],
                now,
            )
            if sample is None:
                return {"ok": False, "accepted": False, "error": "conflict"}
            self._manual_target = replace(sample, request_id=envelope.request_id)
            self.publish_command()
            return {"ok": True, "accepted": True}
        except (ValueError, KeyError, TypeError, ManualAuthorityError):
            return {"ok": False, "accepted": False, "error": "validation"}

    def _close_manual(self, reason: str) -> bool:
        target, self._manual_target = self._manual_target, None
        if target is not None or reason == "prepared":
            self._generation += 1
            self._active = False
            self._close_reason = reason
            context = self._manual_ingress.context
            view = self._manual_ingress.view(self._monotonic_clock.monotonic())
            identity = (
                target.sample.intent.authority_id
                if target is not None
                else (view.authority_id if view is not None else "")
            )
            self._last_close_source = (
                "manual",
                identity,
                target.source_epoch
                if target is not None
                else (context.controller_generation if context else 0),
                target.sample.received_steady
                if target is not None
                else self._monotonic_clock.monotonic(),
                target.session_id
                if target is not None
                else (context.session_id if context else ""),
                target.agent_epoch
                if target is not None
                else (context.agent_epoch if context else 0),
            )
            self._publish_barrier(reason=reason)
            return True
        return False

    def _publish_manual_target(self) -> bool:
        target = getattr(self, "_manual_target", None)
        if target is None:
            return False
        now = self._monotonic_clock.monotonic()
        view = self._manual_ingress.view(now)
        if (
            view is None
            or view.state is not ManualAuthorityState.ACTIVE
            or now >= target.sample.deadline_steady
        ):
            self._close_manual("expiry")
            return False
        message = FinalCommand()
        message.robot_id, message.owner_boot_id = self._manual_ingress.robot_id, self._owner_boot
        message.session_id = target.session_id
        message.generation = self._generation
        message.connection_epoch = target.agent_epoch
        message.sequence = target.sample.intent.sequence
        message.steady_deadline = target.sample.deadline_steady
        message.received_steady = target.sample.received_steady
        message.source_kind, message.source_id = "manual", target.sample.intent.authority_id
        message.source_epoch = target.source_epoch
        message.close_reason = "intent"
        message.vx, message.wz = target.velocity.vx, target.velocity.wz
        message.manual_linear = target.sample.intent.linear
        message.manual_linear_limit = target.linear_limit_mps
        message.degraded_linear_limit = target.degraded_linear_mps
        message.gesture = target.sample.intent.gesture
        message.request_id = target.request_id
        self._final_publisher.publish(message)
        self._active = True
        return True

    @with_dds_info
    def _on_drive_feedback(
        self, msg: DriveExecutionFeedback, info: dict[str, Any] | None = None
    ) -> None:
        endpoints = self.get_publishers_info_by_topic("/safety/drive_feedback")
        if len(endpoints) != 1:
            return
        endpoint = endpoints[0]
        gid = info.get("publisher_gid") if info is not None else None
        if (
            endpoint.node_name == "safety_gate"
            and endpoint.node_namespace == "/"
            and (gid is None or bytes(endpoint.endpoint_gid) == bytes(gid))
            and msg.owner_boot_id == self._owner_boot
            and msg.robot_id == str(self.get_parameter("robot_id").value)
            and msg.status in {"retired", "task_retired", "task_collected"}
        ):
            if not self._retirement_incarnation_current() or msg.reason != self._retirement_core[1]:
                return
            if msg.status == "task_collected":
                collection = self._terminal_collections.get(msg.request_id)
                if (
                    collection is not None
                    and collection.execution_id == msg.session_id
                    and collection.through_epoch == msg.connection_epoch
                    and msg.sequence == 0
                ):
                    collection.acknowledged = True
            elif msg.status == "task_retired":
                for request in self._terminal_requests.values():
                    proof = request.proof
                    if (
                        proof is not None
                        and request.execution_id == msg.session_id
                        and (msg.request_id, msg.sequence, msg.connection_epoch)
                        == (proof.token, proof.generation, proof.task_epoch)
                    ):
                        request.acknowledged = True
            else:
                receipt = self._retirements.get(msg.session_id)
                if (
                    receipt is not None
                    and (msg.request_id, msg.sequence) == (receipt.token, receipt.generation)
                    and 0 < msg.connection_epoch <= receipt.through_epoch
                ):
                    self._retirements[msg.session_id] = replace(
                        receipt, acknowledged=True, task_epoch=msg.connection_epoch
                    )
            return

    def _publish_barrier(
        self,
        request_id: str = "",
        session_id: str = "",
        generation: int | None = None,
        *,
        reason: str | None = None,
        revision: int = 0,
        source_override: tuple[str, str, int, float, str, int] | None = None,
    ) -> None:
        handoff_boot = getattr(self, "_output_handoff_boot", None)
        if handoff_boot is not None:
            if self._output_handoff_missing_estop:
                return
            retained = self._latched_estop_request
            estop_id = (
                request_id
                if reason == "estop"
                else (retained.request_id if retained is not None else "")
            )
            latch_revision = self._latched_estop_revision if estop_id else 0
            cached_handoff = self._pending_output_handoff
            if (
                cached_handoff is not None
                and cached_handoff.generation == self._generation
                and cached_handoff.request_id == (estop_id or handoff_boot)
                and cached_handoff.source_epoch == latch_revision
            ):
                self._final_publisher.publish(cached_handoff)
                return
            # Private idle zero semantics: echo the output boot, preserving the
            # original estop request and revision. Manual/navigation epochs keep
            # their original meaning. Native v3 requires estop gesture == 0.
            request_id = estop_id or handoff_boot
            reason = "estop" if estop_id else "prepared"
            generation = self._generation
            session_id = ""
            source_override = (
                "idle",
                handoff_boot,
                latch_revision,
                self._monotonic_clock.monotonic(),
                "",
                0,
            )
            revision = 0  # gesture is not the restored latch revision in a v3 estop barrier.
        pending = getattr(self, "_pending_estop_request", None)
        if pending is not None and not (reason == "estop" and request_id):
            self._final_publisher.publish(pending)
            return
        output_generation = self._generation if generation is None else generation
        cached = getattr(self, "_barrier_frame", None)
        if (
            not request_id
            and reason is None
            and cached is not None
            and cached.generation == output_generation
        ):
            self._final_publisher.publish(cached)
            return
        msg = FinalCommand()
        msg.robot_id = str(self.get_parameter("robot_id").value)
        msg.owner_boot_id = self._owner_boot
        msg.generation = output_generation
        msg.sequence, msg.steady_deadline, msg.vx, msg.wz = 0, 0.0, 0.0, 0.0
        source = source_override or getattr(self, "_last_close_source", ("idle", "", 0, 0.0, "", 0))
        if reason in {"estop_reset", "fault_recover"}:
            source = ("idle", "", 0, self._monotonic_clock.monotonic(), "", 0)
        msg.request_id = request_id
        msg.session_id = session_id or source[4]
        msg.connection_epoch = source[5]
        msg.source_kind, msg.source_id = ("navigation", session_id) if session_id else source[:2]
        msg.source_epoch = source[2] if not session_id or session_id == source[1] else 0
        msg.received_steady = source[3]
        msg.close_reason = reason or getattr(
            self, "_close_reason", "task_revoke" if session_id else "release"
        )
        msg.gesture = revision
        if msg.generation == self._generation:
            self._barrier_frame = msg
        if handoff_boot is not None:
            self._pending_output_handoff = msg
            if reason == "estop":
                self._pending_estop_request = msg
        elif reason == "estop_reset":
            self._pending_estop_reset = msg
        self._final_publisher.publish(msg)

    def shutdown_motion(self) -> None:
        self._arbiter.shutdown()
        self._generation += 1
        self._active = False
        self._publish_barrier()

    def destroy_node(self):
        self.shutdown_motion()
        return super().destroy_node()


def main() -> None:
    from senior_motion_control.motion_lifecycle import MotionExecutor, run

    run(CommandNode, cli=True, executor_factory=MotionExecutor)
