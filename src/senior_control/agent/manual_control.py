"""Serial ROS-side ingress adapter for the single T2 authority owner.

No second lease or actuator: ManualAuthorityManager admits samples, and T5
consumes the frozen target/deadline and owns closed-loop braking on source close.
"""

from dataclasses import dataclass
from typing import Literal

from senior_control.models import Velocity
from senior_control.motion.contracts import AcceptedManualIntent, ManualIntent
from senior_control.motion.manual_authority import (
    ManualAuthorityContext,
    ManualAuthorityManager,
    ManualAuthorityReceipt,
)
from senior_control.protocol import MotionRuntimeState, RuntimeObservation
from senior_control.settings.models import DrivingSettings


@dataclass(frozen=True)
class ManualTarget:
    sample: AcceptedManualIntent
    velocity: Velocity
    source_epoch: int
    session_id: str
    linear_limit_mps: float
    degraded_linear_mps: float
    request_id: str = ""
    agent_epoch: int = 0


class ManualIngress:
    def __init__(self, robot_id: str, boot_id: str, *, ttl: float):
        self.robot_id, self.boot_id, self.ttl = robot_id, boot_id, ttl
        self.manager: ManualAuthorityManager | None = None
        self.context: ManualAuthorityContext | None = None
        self.request: tuple[str, str] | None = None

    def bind(self, epoch: int, generation: int, session: str, now: float) -> None:
        context = ManualAuthorityContext(self.boot_id, self.robot_id, epoch, generation, session)
        if self.manager is None:
            self.manager = ManualAuthorityManager(context, ttl=self.ttl, max_receipts=4096)
        else:
            self.manager.replace_context(context, now_steady=now)
        if context != self.context:
            self.request = None
        self.context = context

    def acquire(self, request_id: str, now: float) -> ManualAuthorityReceipt:
        assert self.manager is not None and self.context is not None
        receipt = self.manager.acquire(self.context, request_id, "{}", now_steady=now)
        self.request = request_id, "{}"
        return receipt

    def view(self, now: float) -> ManualAuthorityReceipt | None:
        if self.manager is None or self.context is None or self.request is None:
            return None
        return self.manager.acquire(self.context, *self.request, now_steady=now)

    def accept(
        self,
        intent: ManualIntent,
        settings: DrivingSettings,
        received: float,
        deadline: float,
        now: float,
    ) -> ManualTarget | None:
        if self.manager is None or self.context is None:
            return None
        if not self.manager.accept(
            self.context,
            intent,
            now_steady=now,
            received_steady=received,
            queue_deadline_steady=deadline,
        ):
            return None
        sample = self.manager.take_latest(self.context, intent.authority_id, now_steady=now)
        if sample is None:
            return None
        high = intent.gear == "high"
        linear_limit = settings.high_linear_mps if high else settings.low_linear_mps
        if intent.linear < 0:
            linear_limit = min(linear_limit, settings.reverse_linear_mps)
        angular_limit = settings.high_angular_rps if high else settings.low_angular_rps
        return ManualTarget(
            sample,
            Velocity(intent.linear * linear_limit, intent.turn * angular_limit),
            self.context.controller_generation,
            self.context.session_id,
            agent_epoch=self.context.agent_epoch,
            linear_limit_mps=linear_limit,
            degraded_linear_mps=settings.degraded_linear_mps,
        )

    def release(self, authority_id: str, now: float) -> bool:
        return bool(
            self.manager is not None
            and self.context is not None
            and self.manager.release(self.context, authority_id, now_steady=now)
        )

    def retire(self, now: float) -> None:
        if self.manager is not None and self.context is not None:
            self.manager.retire_context(self.context, now_steady=now)

    def revoke(self, now: float) -> None:
        if self.manager is not None:
            self.manager.revoke(now_steady=now)


class MotionRuntimeReceiver:
    """A trusted local sample frontier, with no state-machine transitions."""

    def __init__(self) -> None:
        from senior_control.motion.source_fences import SourceEpochFence

        self._sources = SourceEpochFence(max_retired=32)
        self.value: MotionRuntimeState | None = None
        self._available = False

    def accept(self, value: "MotionRuntimeState", publisher: bytes, now: float) -> bool:
        if not value.sampled_steady <= now < value.valid_until:
            return False
        identity = publisher.hex() + ":" + value.boot_id
        if not self._sources.accept(identity):
            return False
        previous = self.value
        if previous is not None and previous.boot_id == value.boot_id:
            if (
                value.sequence <= previous.sequence
                or value.sampled_steady < previous.sampled_steady
                or value.estop_revision < previous.estop_revision
                or value.fault_revision < previous.fault_revision
            ):
                return False
        self.value = value
        self._available = True
        return True

    def invalidate(self) -> None:
        """Withdraw availability without losing the replay/revision frontier."""
        self._available = False

    def observation(self, now: float) -> RuntimeObservation:
        value = self.value
        if value is None:
            return RuntimeObservation(status="missing")
        status: Literal["current", "revoked", "expired"] = "current"
        if not self._available:
            status = "revoked"
        elif not value.sampled_steady <= now < value.valid_until:
            status = "expired"
        return RuntimeObservation(status=status, last_sampled_steady=value.sampled_steady)

    def current(self, now: float) -> "MotionRuntimeState | None":
        value = self.value
        return (
            value
            if self._available
            and value is not None
            and value.sampled_steady <= now < value.valid_until
            else None
        )


def runtime_healthy(value: "MotionRuntimeState | None", now: float) -> bool:
    return bool(
        value is not None
        and value.healthy
        and value.evidence_stamp_steady is not None
        and value.evidence_deadline_steady is not None
        and value.evidence_stamp_steady <= now < value.evidence_deadline_steady
    )


def vehicle_frame(
    runtime: MotionRuntimeState | None,
    *,
    sequence: int,
    connected: bool,
    task: dict[str, object] | None,
    manual: dict[str, object] | None,
    manual_enabled: bool,
    tasks_enabled: bool,
    runtime_enabled: bool,
    safety_revisions: tuple[int, int],
    now: float,
    observation: RuntimeObservation | None = None,
) -> dict[str, object]:
    """Use T7's motion projection; wire body averages retain their actual times."""
    from senior_control.agent.vehicle_state import (
        ConnectionState,
        RuntimeSnapshot,
        project_vehicle_state,
    )
    from senior_control.motion.contracts import MotionMode, MotionState

    capabilities = ["runtime.v1"] if runtime_enabled else []
    if tasks_enabled:
        capabilities.append("task.execution.v1")
    if manual_enabled:
        capabilities.append("motion.intent.v1")
    # //Compatible: older IPC producers omit reporting metadata.
    if observation is None:
        observation = RuntimeObservation(
            status="missing" if runtime is None else "current",
            last_sampled_steady=None if runtime is None else runtime.sampled_steady,
        )
    if runtime is not None and not runtime.sampled_steady <= now < runtime.valid_until:
        observation = RuntimeObservation(
            status="expired", last_sampled_steady=runtime.sampled_steady
        )
    reporting = observation.model_dump()
    reporting["age_seconds"] = (
        None
        if observation.last_sampled_steady is None
        else max(0.0, now - observation.last_sampled_steady)
    )
    if runtime is None or observation.status != "current":
        return {
            "schema": 1,
            "sequence": sequence,
            "mode": "unknown",
            "reason": observation.status,
            "estop_revision": safety_revisions[0],
            "fault_revision": safety_revisions[1],
            "confirmed_estop_status": "unknown",
            "body_reference": None,
            "wheel_linear_mps": None,
            "task": task,
            "allowed_actions": [],
            "capabilities": capabilities,
            "manual_authority": None,
            "observation": reporting,
        }

    motion = MotionState(
        MotionMode(runtime.mode),
        runtime.reason,
        runtime.task_id,
        safety_revisions[0],
        safety_revisions[1],
        runtime.stationary and runtime_healthy(runtime, now),
        tuple(runtime.allowed_actions) if runtime_healthy(runtime, now) else (),
    )
    view = project_vehicle_state(
        RuntimeSnapshot(sequence, motion),
        ConnectionState(connected),
        now_steady=now,
    )
    reference = runtime.body_reference
    body = (
        None
        if reference is None
        else {
            "kind": "independent_body",
            "average": reference.average,
            "source": reference.source,
            "age_seconds": max(0.0, now - reference.computed_steady),
            "stale": not reference.computed_steady <= now < reference.valid_until_steady,
        }
    )
    return {
        "schema": 1,
        "observation": reporting,
        "sequence": view.sequence,
        "mode": view.mode.value,
        "reason": view.detail_reason,
        "estop_revision": view.estop_revision,
        "fault_revision": view.fault_revision,
        "confirmed_estop_status": runtime.confirmed_estop_status,
        "body_reference": body,
        "wheel_linear_mps": runtime.wheel_linear_mps,
        "task": task,
        "allowed_actions": [
            action
            for action in view.allowed_actions
            if action in {"acquire", "estop_reset", "fault_recover"}
            and (action != "acquire" or manual_enabled)
        ],
        "capabilities": capabilities,
        "manual_authority": None
        if manual is None
        else {
            "authority_id": manual["authority_id"],
            "state": manual["state"],
        },
    }
