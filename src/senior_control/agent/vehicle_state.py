"""Pure display projection of T5 state; connection never changes vehicle runtime."""

import math
from dataclasses import dataclass, replace

from senior_control.models import BodyTwist
from senior_control.motion.contracts import BodyMotion, MotionMode, MotionState


@dataclass(frozen=True)
class ConnectionState:
    connected: bool
    rtt_ms: float | None = None
    reconnecting: bool = False


@dataclass(frozen=True)
class RuntimeSnapshot:
    # Trusted composition sequence; persisted/epoch-normalized by transport, not browser IDs.
    sequence: int
    motion: MotionState
    task_status: str | None = None
    task_progress: float | None = None
    body: BodyMotion | None = None
    body_source: str | None = None
    wheel_linear_mps: float | None = None
    received_steady: float | None = None
    hardware_estop: bool = False


@dataclass(frozen=True)
class VehicleState:
    sequence: int
    mode: MotionMode
    reason: str
    detail_reason: str
    task_id: str | None
    task_status: str | None
    task_progress: float | None
    estop_revision: int
    fault_revision: int
    allowed_actions: tuple[str, ...]
    runtime_actions: tuple[str, ...]
    body_average: BodyTwist | None
    body_source: str | None
    body_age_seconds: float | None
    body_stale: bool
    body_computed_steady: float | None
    body_valid_until_steady: float | None
    wheel_linear_mps: float | None
    received_steady: float | None
    connection: ConnectionState
    last_known: bool
    task_may_continue: bool
    estop_status: str
    confirmed_estop_status: str


def project_vehicle_state(
    runtime: RuntimeSnapshot,
    connection: ConnectionState,
    *,
    now_steady: float | None = None,
    software_estop_pending: bool = False,
    previous: VehicleState | None = None,
) -> VehicleState:
    """T5 already resolves ESTOP > fault > feedback > obstacle > source.

    The mode is the primary reason category; the last reducer event is detail only.
    Keep its exact allowed actions; never infer vehicle stopped from
    disconnection or from a pending browser command. Old snapshots cannot regress
    either safety revision. Transport must normalize reboot epochs before calling.
    runtime.hardware_estop belongs to runtime.sequence and is admitted with that whole
    snapshot; a rejected/same snapshot cannot relabel confirmed vehicle state.
    software_estop_pending is a current request overlay, independent of telemetry.
    """
    if previous is not None and (
        runtime.sequence <= previous.sequence
        or runtime.motion.estop_revision < previous.estop_revision
        or runtime.motion.fault_revision < previous.fault_revision
    ):
        # Update display-only connectivity while retaining every last-known runtime field.
        return replace(
            previous,
            connection=connection,
            last_known=not connection.connected,
            task_may_continue=not connection.connected and previous.task_status == "running",
            allowed_actions=previous.runtime_actions if connection.connected else (),
            estop_status=_estop_overlay(previous.confirmed_estop_status, software_estop_pending),
            body_age_seconds=(
                max(0.0, now_steady - previous.body_computed_steady)
                if now_steady is not None
                and math.isfinite(now_steady)
                and previous.body_computed_steady is not None
                else previous.body_age_seconds
            ),
            body_stale=previous.body_stale
            or (
                previous.body_valid_until_steady is not None
                and now_steady is not None
                and (
                    not math.isfinite(now_steady) or now_steady >= previous.body_valid_until_steady
                )
            ),
        )
    motion = runtime.motion
    hardware_estop = runtime.hardware_estop
    actions = motion.allowed_actions
    if hardware_estop:
        actions = tuple(action for action in actions if action != "estop_reset")
    if motion.mode is MotionMode.ESTOP:
        estop_status = "hardware_latched" if hardware_estop else "software_latched"
    else:
        estop_status = "clear"
    body = runtime.body
    average = body.average if body is not None and body.qualified else None
    stale = body is not None and (
        now_steady is None
        or not math.isfinite(now_steady)
        or not body.computed_steady <= now_steady < body.valid_until_steady
    )
    age = None
    if body is not None and now_steady is not None and math.isfinite(now_steady):
        age = max(0.0, now_steady - body.computed_steady)
    return VehicleState(
        sequence=runtime.sequence,
        mode=motion.mode,
        reason=("hardware_estop" if hardware_estop else "software_estop")
        if motion.mode is MotionMode.ESTOP
        else motion.mode.value,
        detail_reason=motion.reason,
        task_id=motion.task_id,
        task_status=runtime.task_status,
        task_progress=runtime.task_progress,
        estop_revision=motion.estop_revision,
        fault_revision=motion.fault_revision,
        allowed_actions=actions if connection.connected else (),
        runtime_actions=actions,
        body_average=average,
        body_source=runtime.body_source if average is not None else None,
        body_age_seconds=age,
        body_stale=stale,
        body_computed_steady=body.computed_steady if body is not None else None,
        body_valid_until_steady=body.valid_until_steady if body is not None else None,
        wheel_linear_mps=runtime.wheel_linear_mps,
        received_steady=runtime.received_steady,
        connection=connection,
        last_known=not connection.connected,
        task_may_continue=not connection.connected and runtime.task_status == "running",
        estop_status=_estop_overlay(estop_status, software_estop_pending),
        confirmed_estop_status=estop_status,
    )


def _estop_overlay(confirmed: str, pending: bool) -> str:
    return confirmed if confirmed != "clear" else ("pending" if pending else "clear")
