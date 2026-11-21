"""Concrete snapshot/display construction over the existing management projection."""

import time
from collections.abc import Callable
from typing import Any

from vehicle_core.errors import DomainError
from vehicle_core.motion_state import MotionMode, MotionState
from vehicle_core.runtime_state import MotionRuntimeState, RuntimeObservation
from vehicle_core.values import Pose2D
from vehicle_manager.agent.local_adapters import IpcState
from vehicle_manager.agent.request_bindings import LifecycleNavigationOperations
from vehicle_manager.agent.snapshot_state import SnapshotState as _SnapshotState
from vehicle_manager.agent.state_ports import SnapshotPorts
from vehicle_manager.agent.workspace import WorkspaceReader
from vehicle_manager.tasks.store import TaskStore
from vehicle_manager.waypoints.store import WaypointStore


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
    # //Compatible: only the management projection needs these original owners.
    from vehicle_manager.agent.frame_projection import vehicle_frame as _vehicle_frame
    from vehicle_manager.agent.state_ports import ProjectionPorts

    return _vehicle_frame(
        runtime,
        ports=ProjectionPorts(RuntimeObservation, MotionState, MotionMode, MotionMode.ESTOP),
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


class SnapshotState(_SnapshotState):
    def __init__(
        self,
        state: IpcState,
        tasks: TaskStore,
        waypoints: WaypointStore[Any, Any, Pose2D],
        workspace: WorkspaceReader | None = None,
        *,
        navigation: LifecycleNavigationOperations | None = None,
        connected: Callable[[], bool] = lambda: True,
        epoch: Callable[[], int] = lambda: 0,
        manual_enabled: bool = False,
        clock: Callable[[], float] = time.monotonic,
        settings_enabled: bool = False,
        logs_enabled: Callable[[], bool] = lambda: False,
    ) -> None:
        super().__init__(
            state,
            tasks,
            waypoints,
            workspace,
            ports=SnapshotPorts(vehicle_frame, RuntimeObservation, DomainError),
            navigation=navigation,
            connected=connected,
            epoch=epoch,
            manual_enabled=manual_enabled,
            clock=clock,
            settings_enabled=settings_enabled,
            logs_enabled=logs_enabled,
        )
