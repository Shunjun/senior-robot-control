"""Existing runtime-to-display projection; no control authority is granted here."""

from vehicle_manager.agent.state_ports import ObservationReading, ProjectionPorts, RuntimeReading
from vehicle_manager.agent.vehicle_state import (
    ConnectionState,
    RuntimeSnapshot,
    project_vehicle_state,
)


def vehicle_frame(
    runtime: RuntimeReading | None,
    *,
    ports: ProjectionPorts,
    sequence: int,
    connected: bool,
    task: dict[str, object] | None,
    manual: dict[str, object] | None,
    manual_enabled: bool,
    tasks_enabled: bool,
    runtime_enabled: bool,
    safety_revisions: tuple[int, int],
    now: float,
    observation: ObservationReading | None = None,
) -> dict[str, object]:
    """Use T7's motion projection; wire body averages retain their actual times."""
    capabilities = ["runtime.v1"] if runtime_enabled else []
    if tasks_enabled:
        capabilities.append("task.execution.v1")
    if manual_enabled:
        capabilities.append("motion.intent.v1")
    # //Compatible: older IPC producers omit reporting metadata.
    if observation is None:
        observation = ports.observation(
            status="missing" if runtime is None else "current",
            last_sampled_steady=None if runtime is None else runtime.sampled_steady,
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

    motion = ports.motion(
        ports.mode(runtime.mode),
        runtime.reason,
        runtime.task_id,
        safety_revisions[0],
        safety_revisions[1],
        runtime.stationary,
        tuple(runtime.allowed_actions),
    )
    view = project_vehicle_state(
        RuntimeSnapshot(sequence, motion),
        ConnectionState(connected),
        now_steady=now,
        estop_mode=ports.estop_mode,
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
