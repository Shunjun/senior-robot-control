"""Publish arbitration results to the safety gate.

This process does not publish /cmd_vel. Lease expiry, estop, disconnect, and a
zero tick are written to /safety/command_in immediately so the gate does not
keep the previous command until its own TTL. The watchdog reads the injected
monotonic clock and keeps running when ROS time is frozen. The timer period is
simulation configuration, not a physical braking guarantee.
"""

import math
import re
import secrets
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from geometry_msgs.msg import Twist
from rclpy.clock import Clock as RosClock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.subscription import Subscription
from senior_motion_msgs.msg import DriveExecutionFeedback, FinalCommand
from std_msgs.msg import String

from senior_control.agent.manual_control import (
    ManualIngress,
    ManualTarget,
    MotionRuntimeReceiver,
    runtime_healthy,
)
from senior_control.clock import Clock, SystemClock
from senior_control.errors import ErrorCode
from senior_control.models import Velocity
from senior_control.motion.contracts import ManualIntent
from senior_control.motion.manual_authority import ManualAuthorityError, ManualAuthorityState
from senior_control.protocol import (
    DriveExecutionFeedback as ExecutionFeedback,
)
from senior_control.protocol import (
    Envelope,
    ManualIntentEnvelope,
    MotionRuntimeState,
    RecoveryEnvelope,
    SimpleEnvelope,
    VelocityEnvelope,
)
from senior_control.safety.arbiter import Arbiter, CommandResult, PersistentEstop
from senior_control.settings.models import DrivingSettings
from senior_robot.motion_lifecycle import with_dds_info


@dataclass
class _NavigationSource:
    execution_id: str
    topic: str
    offset: float
    subscription: Subscription
    bound_at_ns: int
    qualified: Callable[[], bool]
    sequence: int = 0
    last_stamp: int = 0
    last_forwarded: float = float("-inf")
    task_epoch: int | None = None
    received_steady: float = 0.0


class CommandNode(Node):
    def __init__(
        self,
        arbiter: Arbiter | None = None,
        clock: Clock | None = None,
        *,
        parameter_overrides: list[Parameter] | None = None,
        require_server_heartbeat: bool = False,
    ) -> None:
        super().__init__("command_node", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("robot_id", "robot")
        self.declare_parameter("native_writer_receipts", False)
        self.declare_parameter("motion_control_configured", True)
        self._motion_control_configured = bool(
            self.get_parameter("motion_control_configured").value
        )
        self._native_writer_receipts = bool(self.get_parameter("native_writer_receipts").value)
        self.declare_parameter("safety_state_path", "")
        self.declare_parameter("heartbeat_seconds", 0.2)
        self.declare_parameter("disconnect_seconds", 2.0 if require_server_heartbeat else 1.0)
        self.declare_parameter("command_ttl_seconds", 0.3)
        self.declare_parameter("command_topic", "/safety/command_in")
        self.declare_parameter("final_command_topic", "/safety/final_command")
        topic = str(self.get_parameter("command_topic").value).strip()
        if not topic or self.resolve_topic_name(topic) == "/cmd_vel":
            raise ValueError("command node must publish only to the safety input")
        self._monotonic_clock = clock if clock is not None else SystemClock()
        self._estop_store = None
        if require_server_heartbeat:
            path = str(self.get_parameter("safety_state_path").value)
            if not path:
                raise ValueError("production command node requires safety_state_path")
            self._estop_store = PersistentEstop(
                Path(path), str(self.get_parameter("robot_id").value)
            )
        if arbiter is None:
            self._arbiter = Arbiter(
                robot_id=str(self.get_parameter("robot_id").value),
                heartbeat_seconds=float(self.get_parameter("heartbeat_seconds").value),
                disconnect_seconds=float(self.get_parameter("disconnect_seconds").value),
                command_ttl_seconds=float(self.get_parameter("command_ttl_seconds").value),
                require_server_heartbeat=require_server_heartbeat,
                initial_estop=self._estop_store.latched if self._estop_store else False,
            )
        else:
            self._arbiter = arbiter
        self._publisher = self.create_publisher(Twist, topic, 10)
        self._final_publisher = self.create_publisher(
            FinalCommand, str(self.get_parameter("final_command_topic").value), 10
        )
        self._owner_boot = secrets.token_hex(16)
        self._manual_ingress = ManualIngress(
            str(self.get_parameter("robot_id").value),
            self._owner_boot,
            ttl=float(self.get_parameter("command_ttl_seconds").value),
        )
        self._manual_target: ManualTarget | None = None
        self._motion_runtime = MotionRuntimeReceiver()
        self._local_estop_requested = self._arbiter.estop_latched
        self._pending_recovery: tuple[str, int, str] | None = None
        self._estop_clear_future = None
        self._close_reason = "estop" if self._local_estop_requested else "prepared"
        self._last_close_source = ("idle", "", 0, 0.0, "", 0)
        if self._native_writer_receipts:
            from senior_robot.native_receipt import subscribe

            subscribe(
                self,
                String,
                "/motion/runtime",
                self._on_motion_runtime,
                writer="safety_gate",
                qos="reliable",
                invalidate=self._invalidate_motion_runtime,
            )
        else:
            # //Compatible: isolated simulation retains its existing metadata path.
            self.create_subscription(
                String, "/motion/runtime", self._on_motion_runtime, qos_profile_sensor_data
            )
        self._drive_sample: VelocityEnvelope | None = None
        self._feedback: deque[ExecutionFeedback] = deque(maxlen=24)
        self._awaiting_feedback: dict[str, VelocityEnvelope] = {}
        if require_server_heartbeat:
            if self._native_writer_receipts:
                subscribe(
                    self,
                    DriveExecutionFeedback,
                    "/safety/drive_feedback",
                    self._on_drive_feedback,
                    writer="safety_gate",
                    qos="reliable",
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
        self._retirements: dict[str, tuple[str, int, bool]] = {}
        self._generation = 1
        self._estop_requests: dict[str, tuple[FinalCommand, CommandResult, CommandResult]] = {}
        self._last_server_epoch = 0
        self._active = False
        heartbeat = float(self.get_parameter("heartbeat_seconds").value)
        ttl = float(self.get_parameter("command_ttl_seconds").value)
        period = min(heartbeat, ttl) / 2.0
        if period <= 0.0:
            period = 0.05
        steady = RosClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(period, self.publish_command, clock=steady)

    def submit(
        self,
        envelope: Envelope,
        received_at: float | None = None,
        *,
        local_deadline: float | None = None,
    ) -> CommandResult:
        if envelope.robot_id != str(self.get_parameter("robot_id").value):
            return CommandResult(False, ErrorCode.VALIDATION)
        if envelope.type in {"drive.velocity", "drive.challenge", "safety.reset"} and hasattr(
            self, "_manual_ingress"
        ):
            # //Compatible: only isolated unmigrated nodes retain the old wire.
            return CommandResult(False, ErrorCode.VALIDATION)
        requests = getattr(self, "_estop_requests", None)
        if requests is None:
            requests = self._estop_requests = {}
        if envelope.type == "safety.estop" and envelope.request_id in requests:
            frame, original, receipt = requests[envelope.request_id]
            self._final_publisher.publish(frame)
            # Once a later reset/source generation won, this is only a replay of
            # the old event. Never write a new latch or revoke the new authority.
            if (
                not receipt.accepted
                and frame.generation == self._generation
                and self._local_estop_requested
                and self._estop_store is not None
            ):
                receipt = self._estop_store.commit(envelope, original)
                requests[envelope.request_id] = frame, original, receipt
            return receipt
        source_closed = False
        if envelope.type == "drive.stop" and hasattr(self, "_manual_ingress"):
            self._manual_ingress.revoke(self._monotonic_clock.monotonic())
            self._close_reason = "release"
            source_closed = self._close_manual("release")
        result = self._arbiter.accept(
            envelope,
            self._monotonic_clock.monotonic(),
            received_at=received_at,
            local_deadline=local_deadline,
        )
        estop_frame = None
        if envelope.type == "safety.estop":
            self._local_estop_requested = True
            estop_frame = self._explicit_estop(envelope, received_at)
            source_closed = True
        if result.accepted and isinstance(envelope, VelocityEnvelope):
            self._drive_sample = envelope if envelope.payload.gesture is not None else None
            if self._drive_sample is not None:
                self._awaiting_feedback[envelope.request_id] = envelope
                if len(self._awaiting_feedback) > 32:
                    self._awaiting_feedback.pop(next(iter(self._awaiting_feedback)))
        if result.accepted and envelope.type != "drive.velocity":
            self._drive_sample = None
            if not source_closed:
                self._generation += 1
                self._active = False
                self._publish_barrier()
        self.publish_command()
        receipt = (
            self._estop_store.commit(envelope, result)
            if (estop_frame is not None and self._estop_store is not None)
            else result
        )
        if estop_frame is not None:
            requests[envelope.request_id] = estop_frame, result, receipt
        return receipt

    def _explicit_estop(self, envelope: Envelope, received: float | None) -> FinalCommand:
        now = self._monotonic_clock.monotonic()
        self._pending_recovery = None
        self._estop_clear_future = None
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
        return self._barrier_frame

    def motion_mode(self) -> str:
        if getattr(self, "_manual_target", None) is not None:
            return "manual"
        return self._arbiter.mode(self._monotonic_clock.monotonic())

    def server_heartbeat(self, connection_epoch: int, deadline: float) -> bool:
        accepted = self._arbiter.server_heartbeat(
            connection_epoch, self._monotonic_clock.monotonic(), deadline
        )
        if accepted and connection_epoch != self._last_server_epoch:
            self._last_server_epoch = connection_epoch
            if not self._arbiter.task_navigation_active:
                self._generation += 1
                self._active = False
                self._publish_barrier()
        return accepted

    def disconnect(self, connection_epoch: int) -> None:
        if hasattr(self, "_manual_ingress"):
            context = self._manual_ingress.context
            if context is not None and context.agent_epoch <= connection_epoch:
                self._manual_ingress.retire(self._monotonic_clock.monotonic())
                self._close_manual("connection_retired")
        self._arbiter.disconnect(connection_epoch)
        if not self._arbiter.task_navigation_active:
            self._generation += 1
            self._active = False
            self._publish_barrier()
        self.publish_command()

    def current_navigation_epoch(self) -> int:
        return self._last_server_epoch

    def navigation_qualified(self) -> bool:
        if getattr(self, "_manual_target", None) is not None:
            return False
        return self._arbiter.task_navigation_allowed(self._monotonic_clock.monotonic())

    def task_motion_allowed(self, *, continuing: bool) -> bool:
        now = self._monotonic_clock.monotonic()
        runtime = self._motion_runtime.current(now)
        if (
            not runtime_healthy(runtime, now)
            or runtime is None
            or runtime.mode in {"estop", "execution_fault", "feedback_stop"}
            or self._arbiter.estop_latched
            or self._manual_target is not None
        ):
            return False
        return self._motion_control_configured and (continuing or runtime.stationary)

    def bind_navigation_source(
        self,
        execution_id: str,
        topic: str,
        *,
        qualified: Callable[[], bool],
        task_epoch: int | None = None,
    ) -> bool:
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
            task_epoch or 0,
            self._monotonic_clock.monotonic(),
            execution_id,
            task_epoch or 0,
        )
        self._close_reason = "prepared"
        self._publish_barrier()
        self._retirements.pop(execution_id, None)
        bound_at = time.time_ns()
        self._navigation_source = _NavigationSource(
            execution_id,
            topic,
            bound_at / 1e9 - self._monotonic_clock.monotonic(),
            subscription,
            bound_at,
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

    def navigation_source_retired(self, execution_id: str) -> bool:
        receipt = self._retirements.get(execution_id)
        if receipt is None:
            return False
        token, generation, acknowledged = receipt
        if not acknowledged:
            if generation < self._generation:
                # A newer safety/source fence already superseded this pending
                # boundary. Re-issue its token under a fresh output generation.
                self._generation += 1
                generation = self._generation
                self._retirements[execution_id] = (token, generation, False)
            self._publish_barrier(token, execution_id, generation)
        return acknowledged

    def clear_navigation_source(self, execution_id: str, *, reason: str = "requested") -> None:
        source = self._navigation_source
        if source is not None and source.execution_id != execution_id:
            return
        if source is not None:
            self._last_close_source = (
                "navigation",
                source.execution_id,
                source.task_epoch or 0,
                source.received_steady,
                source.execution_id,
                source.task_epoch or 0,
            )
            self._close_reason = {
                "requested": "task_revoke",
                "qualification_lost": "qualification_lost",
                "clock_offset_changed": "fault",
                "navigation_lease_inactive": "expiry",
            }.get(reason, "task_revoke")
            self._navigation_source = None
            self._arbiter.end_navigation(execution_id)
            self.destroy_subscription(source.subscription)
            self.get_logger().warning(f"navigation_source_cleared reason={reason}")
        if execution_id not in self._retirements:
            self._generation += 1
            self._active = False
            token = secrets.token_hex(16)
            self._retirements[execution_id] = (token, self._generation, False)
        self.navigation_source_retired(execution_id)
        self.publish_command()

    def _on_navigation_sample(self, topic: str, msg: Twist, info: dict[str, Any]) -> None:
        received = self._monotonic_clock.monotonic()
        source = self._navigation_source
        if source is None or source.topic != topic:
            return
        stamp = info.get("source_timestamp")
        if not source.qualified():
            self.clear_navigation_source(source.execution_id, reason="qualification_lost")
            return
        # Pair clocks after potentially slow DDS/reference qualification. Its
        # elapsed time is not clock drift and must not enter this comparison.
        wall_now = time.time()
        clock_now = self._monotonic_clock.monotonic()
        if abs((wall_now - clock_now) - source.offset) > 0.01:
            self.clear_navigation_source(source.execution_id, reason="clock_offset_changed")
            return
        if not isinstance(stamp, int) or stamp <= max(source.bound_at_ns, source.last_stamp):
            return
        endpoints = self.get_publishers_info_by_topic(source.topic)
        if len(endpoints) != 1 or endpoints[0].node_name != "controller_server":
            return
        # //Compatible: Humble's Python callback omits the writer GID. Topic isolation
        # still comes from the controller's run-specific remapping and a private ROS domain.
        gid = info.get("publisher_gid")
        if gid is not None and tuple(gid) != tuple(endpoints[0].endpoint_gid):
            return
        if not (abs(msg.linear.x) <= 0.3 and abs(msg.angular.z) <= 0.5):
            return
        # Keep insignificant controller jitter from superseding identical
        # safety checks. Truncation never increases either velocity component.
        velocity = Velocity(
            math.trunc(msg.linear.x * 100) / 100,
            math.trunc(msg.angular.z * 100) / 100,
        )
        # Publisher graph checks may also take time. Check the original DDS
        # sample's lifetime against current time, never callback-entry time.
        now = self._monotonic_clock.monotonic()
        # The safety worker must finish checking a sample before a different
        # Nav2 output replaces it. A producer zero always takes precedence.
        if velocity != Velocity(0.0, 0.0) and now - source.last_forwarded < 0.16:
            return
        sampled = stamp / 1e9 - source.offset - 0.01
        if self._arbiter.accept_navigation_sample(
            source.execution_id, source.sequence + 1, velocity, sampled, now
        ):
            source.received_steady = received
            source.last_stamp = stamp
            source.last_forwarded = now
            source.sequence += 1
            self.publish_command()

    def publish_command(self) -> None:
        self._settle_local_reset()
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
                    source.execution_id, reason="navigation_lease_inactive"
                )
                return
        now = self._monotonic_clock.monotonic()
        final = self._arbiter.final_command(now)
        if final is None:
            if self._active:
                self._generation += 1
                self._active = False
            self._publish_barrier()
            self._publish(Velocity(0.0, 0.0))
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
        msg.source_epoch = source.task_epoch or 0 if source is not None else 0
        msg.received_steady = source.received_steady if source is not None else now
        msg.close_reason = "intent"
        sample = self._drive_sample
        if sample is not None and (session, epoch, sequence) == (
            sample.session_id,
            sample.connection_epoch,
            sample.seq,
        ):
            msg.request_id = sample.request_id
            msg.gesture = sample.payload.gesture or 0
        self._final_publisher.publish(msg)
        self._active = True
        self._publish(velocity)

    def _invalidate_motion_runtime(self) -> None:
        self._motion_runtime.invalidate()
        self._manual_ingress.revoke(self._monotonic_clock.monotonic())
        self._close_manual("qualification_lost")

    def _invalidate_drive_feedback(self) -> None:
        self._feedback.clear()
        # No retirement acknowledgement or successful execution is fabricated.

    @with_dds_info
    def _on_motion_runtime(self, msg: String, info: dict[str, Any] | None = None) -> None:
        endpoints = self.get_publishers_info_by_topic("/motion/runtime")
        if len(endpoints) != 1:
            return
        endpoint = endpoints[0]
        gid = info.get("publisher_gid") if info else None
        if (
            endpoint.node_name != "safety_gate"
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
        if not self._motion_runtime.accept(value, bytes(endpoint.endpoint_gid), now):
            return
        self._settle_local_reset()
        if value.confirmed_estop_status != "clear":
            self._arbiter.confirm_local_estop(True)
        if value.mode in {"feedback_stop", "execution_fault", "estop"}:
            self._manual_ingress.revoke(now)
            self._close_manual(
                "estop"
                if value.mode == "estop"
                else "expiry"
                if value.mode == "feedback_stop"
                else "fault"
            )

    def _settle_local_reset(self) -> None:
        if not hasattr(self, "_motion_runtime"):
            return  # //Compatible: isolated legacy callback fixtures.
        value = self._motion_runtime.current(self._monotonic_clock.monotonic())
        if value is None or value.confirmed_estop_status != "clear":
            return
        if not getattr(self, "_local_estop_requested", False):
            self._arbiter.confirm_local_estop(False)
            return
        pending = getattr(self, "_pending_recovery", None)
        if (
            pending is None
            or pending[0] != "estop_reset"
            or value.estop_revision <= pending[1]
            or value.boot_id != pending[2]
        ):
            return
        store = self._estop_store
        if store is not None and store.latched:
            future = getattr(self, "_estop_clear_future", None)
            if future is None:
                self._estop_clear_future = store.clear_confirmed()
                return
            if not future.done() or future.exception() is not None:
                return
        self._local_estop_requested = False
        self._pending_recovery = None
        self._arbiter.confirm_local_estop(False)

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
            if not self._motion_control_configured and envelope.type in {
                "control.acquire",
                "drive.intent",
            }:
                return {
                    "ok": False,
                    "error": "unavailable",
                    "reason": "real_motion_not_configured: missing hardware model/ROS parameters",
                }
            now = self._monotonic_clock.monotonic()
            runtime = self._motion_runtime.current(now)
            if isinstance(envelope, RecoveryEnvelope):
                key = "estop_reset" if envelope.type == "safety.estop.reset" else "fault_recover"
                current = (
                    runtime.estop_revision
                    if runtime is not None and key == "estop_reset"
                    else (runtime.fault_revision if runtime is not None else -1)
                )
                if (
                    runtime is None
                    or key not in runtime.allowed_actions
                    or envelope.payload.revision != current
                ):
                    return {"ok": False, "accepted": False, "error": "conflict"}
                self._manual_ingress.revoke(now)
                self._close_manual("fault")
                self._generation += 1
                self._pending_recovery = (key, current, runtime.boot_id)
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
            if (
                getattr(self, "_local_estop_requested", False)
                or runtime is None
                or not runtime_healthy(runtime, now)
                or runtime.mode in {"estop", "execution_fault", "feedback_stop"}
            ):
                return {"ok": False, "error": "unavailable"}
            if self._navigation_source is not None or self._arbiter.task_navigation_active:
                return {"ok": False, "accepted": False, "error": "conflict"}
            if envelope.type == "control.acquire":
                previous = self._manual_ingress.view(now)
                retry = self._manual_ingress.request == (envelope.request_id, "{}")
                if "acquire" not in runtime.allowed_actions and not retry:
                    return {"ok": False, "error": "unavailable"}
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
            if view is None or (
                view.state is ManualAuthorityState.PREPARED
                and "acquire" not in runtime.allowed_actions
            ):
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
        runtime = self._motion_runtime.current(now)
        if not runtime_healthy(runtime, now):
            self._manual_ingress.revoke(now)
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
            and msg.status == "retired"
        ):
            receipt = self._retirements.get(msg.session_id)
            if receipt is not None and (msg.request_id, msg.sequence) == receipt[:2]:
                self._retirements[msg.session_id] = (receipt[0], receipt[1], True)
            return
        sample = self._awaiting_feedback.get(msg.request_id)
        # //Compatible: Humble's Python subscription metadata omits publisher_gid.
        # Require sole expected topology, the command owner boot, and the exact admitted sample.
        gid = info.get("publisher_gid") if info is not None else None
        if (
            endpoint.node_name != "safety_gate"
            or endpoint.node_namespace != "/"
            or (gid is not None and bytes(endpoint.endpoint_gid) != bytes(gid))
            or sample is None
            or (sample.session_id, sample.connection_epoch, sample.seq, sample.payload.gesture)
            != (msg.session_id, msg.connection_epoch, msg.sequence, msg.gesture)
            or msg.owner_boot_id != self._owner_boot
            or msg.robot_id != str(self.get_parameter("robot_id").value)
            or msg.connection_epoch != self._last_server_epoch
        ):
            return
        output = (
            {"vx": msg.vx, "wz": msg.wz, "limited": msg.limited} if msg.status == "applied" else {}
        )
        try:
            feedback = ExecutionFeedback(
                robot_id=msg.robot_id,
                session_id=msg.session_id,
                connection_epoch=msg.connection_epoch,
                request_id=msg.request_id,
                seq=msg.sequence,
                gesture=msg.gesture,
                status=msg.status,
                reason=msg.reason or None,
                **output,
            )
        except ValueError:
            return
        self._awaiting_feedback.pop(msg.request_id)
        self._feedback.append(feedback)

    def take_drive_feedback(self) -> list[dict[str, Any]]:
        return [
            self._feedback.popleft().model_dump(exclude_none=True)
            for _ in range(min(len(self._feedback), 8))
        ]

    def _publish_barrier(
        self,
        request_id: str = "",
        session_id: str = "",
        generation: int | None = None,
        *,
        reason: str | None = None,
        revision: int = 0,
    ) -> None:
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
        source = getattr(self, "_last_close_source", ("idle", "", 0, 0.0, "", 0))
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
        self._final_publisher.publish(msg)

    def _publish(self, velocity: Velocity) -> None:
        message = Twist()
        message.linear.x = float(velocity.vx)
        message.angular.z = float(velocity.wz)
        self._publisher.publish(message)

    def shutdown_motion(self) -> None:
        self._arbiter.shutdown()
        self._generation += 1
        self._active = False
        self._publish_barrier()
        self._publish(Velocity(0, 0))

    def destroy_node(self):
        self.shutdown_motion()
        if self._estop_store is not None:
            self._estop_store.close()
        return super().destroy_node()


def main() -> None:
    from senior_robot.motion_lifecycle import MotionExecutor, run

    run(CommandNode, cli=True, executor_factory=MotionExecutor)
