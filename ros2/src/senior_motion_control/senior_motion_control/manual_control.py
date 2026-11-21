"""Serial ROS-side ingress adapter for the single T2 authority owner.

No second lease or actuator: ManualAuthorityManager admits samples, and T5
consumes the frozen target/deadline and owns closed-loop braking on source close.
"""

import math
from dataclasses import dataclass
from typing import Literal

from vehicle_manager.settings.models import DrivingSettings

from senior_motion_control.contracts import (
    AcceptedManualIntent,
    ManualIntent,
)
from senior_motion_control.manual_authority import (
    ManualAuthorityContext,
    ManualAuthorityManager,
    ManualAuthorityReceipt,
)

# //Compatible: retain the original value names without loading management projection.
from vehicle_core.motion_state import MotionMode as MotionMode
from vehicle_core.motion_state import MotionState as MotionState
from vehicle_core.runtime_state import MotionRuntimeState, RuntimeObservation
from vehicle_core.values import Velocity


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

    def stop(self, authority_id: str, gesture: int, now: float) -> bool:
        return bool(
            self.manager is not None
            and self.context is not None
            and self.manager.stop(self.context, authority_id, gesture, now_steady=now)
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
        from senior_motion_control.source_fences import SourceEpochFence

        self._sources = SourceEpochFence(max_retired=32)
        self.value: MotionRuntimeState | None = None
        self._available = False

    def accept(self, value: "MotionRuntimeState", publisher: bytes, now: float) -> bool:
        if not math.isfinite(now) or value.sampled_steady > now:
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
        status: Literal["current", "revoked"] = "current"
        if not self._available:
            status = "revoked"
        elif not math.isfinite(now) or value.sampled_steady > now:
            status = "revoked"
        return RuntimeObservation(status=status, last_sampled_steady=value.sampled_steady)

    def current(self, now: float) -> "MotionRuntimeState | None":
        value = self.value
        return (
            value if self._available and value is not None and value.sampled_steady <= now else None
        )


# //Compatible: pure display projection with the original motion/IPC constructors.


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
    # //Compatible: ROS ingress stays independent until display projection is requested.
    from vehicle_manager.agent.snapshot_bindings import vehicle_frame as _vehicle_frame

    return _vehicle_frame(
        runtime,
        sequence=sequence,
        connected=connected,
        task=task,
        manual=manual,
        manual_enabled=manual_enabled,
        tasks_enabled=tasks_enabled,
        runtime_enabled=runtime_enabled,
        safety_revisions=safety_revisions,
        now=now,
        observation=observation,
    )
