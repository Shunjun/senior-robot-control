"""Bounded IPC adapters for ROS runtime consumers."""

import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, ValidationError
from vehicle_manager.agent.ipc_motion import TaskAwareMotion as _TaskAwareMotion
from vehicle_manager.agent.ipc_navigation import IpcMapPort as _IpcMapPort
from vehicle_manager.agent.ipc_navigation import IpcNavigation as _IpcNavigation
from vehicle_manager.agent.ipc_state import IpcState as _IpcState
from vehicle_manager.agent.local_ports import NavigationValues
from vehicle_manager.agent.request_ports import Requester as Requester
from vehicle_manager.waypoints.home_capture import HomeCapture as _HomeCapture
from vehicle_manager.waypoints.ports import WaypointValuePorts

from senior_control.agent.connection import IpcMotion
from senior_control.navigation import RoutePlan, RouteProgress, RouteRejected
from senior_control.protocol import RobotPosePayload, ScanPointPayload
from senior_control.tasks.models import ExecutionContext
from senior_control.tasks.store import TaskStore
from senior_control.waypoints.store import WaypointStore
from vehicle_core.errors import DomainError
from vehicle_core.ipc import IpcError
from vehicle_core.runtime_state import MotionRuntimeState, RuntimeObservation
from vehicle_core.values import MapRef, Pose2D


class IpcMapPort(_IpcMapPort):
    """//Compatible: original request signature and concrete transport/domain errors."""

    def __init__(self, client: Requester) -> None:
        super().__init__(client, error=DomainError, ipc_error=IpcError)


class IpcNavigation(_IpcNavigation):
    """//Compatible: original values and Store instances; one request implementation."""

    def __init__(
        self,
        client: Requester,
        waypoints: WaypointStore | None = None,
        *,
        tasks: TaskStore | None = None,
    ) -> None:
        super().__init__(
            client,
            waypoints,
            tasks=tasks,
            values=NavigationValues(DomainError, IpcError, Pose2D, RoutePlan, RouteRejected),
        )


@dataclass(frozen=True)
class RuntimeState:
    localized: bool
    safety_clear: bool
    manual_driving: bool
    map_ref: MapRef | None
    execution_status: str
    map_generation: str | None = None
    localization_generation: str | None = None
    navigation_ready: bool = False
    route_progress: RouteProgress | None = None
    # //Compatible: legacy IPC does not provide live map geometry.
    robot_pose: RobotPosePayload | None = None
    navigation_block_reason: str | None = None
    scan_points: tuple[ScanPointPayload, ...] = ()
    # //Compatible: older IPC omits observation metadata.
    runtime_observation: RuntimeObservation | None = None
    motion_runtime: MotionRuntimeState | None = None
    motion_runtime_version: int = 0
    manual_protocol_version: int = 0
    manual_authority: dict[str, Any] | None = None
    task_lifecycle_version: int = 0
    task_evidence_ready: bool = False
    valid_until: float | None = None

    def for_tasks(self, connected: bool) -> ExecutionContext:
        healthy = self.task_evidence_ready
        admission = self.navigation_ready
        return ExecutionContext(
            connected,
            self.localized,
            healthy,
            self.manual_driving,
            self.map_ref,
            map_generation=self.map_generation,
            localization_generation=self.localization_generation,
            pose_ref=self.robot_pose.model_dump_json() if self.robot_pose else None,
            valid_until=self.valid_until,
            task_lifecycle_version=self.task_lifecycle_version,
            admission_ready=admission,
            motion_halt_reason=None,
        )


class StateReply(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    v: Literal[1]
    ok: Literal[True]
    sampled_at: float
    localized: bool
    safety_clear: bool
    manual_driving: bool
    map_id: str | None
    reference_revision: int | None
    execution_status: str
    # //Compatible: older state replies have no qualified reference or navigation fields.
    map_generation: str | None = None
    localization_generation: str | None = None
    # //Compatible: older IPC omits observation metadata.
    runtime_observation: RuntimeObservation | None = None
    motion_runtime: MotionRuntimeState | None = None
    motion_runtime_version: int = 0
    manual_protocol_version: int = 0
    manual_authority: dict[str, Any] | None = None
    task_lifecycle_version: int = 0
    task_evidence_ready: bool = False
    navigation_ready: bool = False
    route_progress: RouteProgress | None = None
    # //Compatible: older adapters omit live geometry.
    robot_pose: RobotPosePayload | None = None
    navigation_block_reason: str | None = Field(default=None, max_length=256)
    scan_points: list[ScanPointPayload] = Field(default_factory=list, max_length=96)


class IpcState(_IpcState[RuntimeState]):
    """//Compatible: original DTO factories and read return type; no copied state."""

    def __init__(self, client: Requester, *, clock: Callable[[], float] = time.monotonic) -> None:
        super().__init__(
            client,
            parse_state=StateReply.model_validate,
            state_factory=RuntimeState,
            map_reference=MapRef,
            error=DomainError,
            ipc_error=IpcError,
            clock=clock,
        )


class PoseReply(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    v: Literal[1]
    ok: Literal[True]
    sampled_at: float
    healthy: Literal[True]
    map_id: str
    reference_revision: int
    anchor_id: str
    x: float
    y: float
    yaw: float
    anchor_x: float
    anchor_y: float
    anchor_yaw: float
    # //Compatible: legacy pose IPC remains readable but cannot authorize Home.
    map_generation: str | None = None


class IpcHomePort(_HomeCapture[MapRef, Pose2D]):
    """//Compatible: original constructor, state/parser and concrete local values."""

    def __init__(self, client: Requester, store: WaypointStore, *, robot_id: str) -> None:
        self._client = client
        super().__init__(
            lambda: client.request("pose.get"),
            store,
            robot_id=robot_id,
            state=IpcState(client),
            parse_pose=PoseReply.model_validate,
            values=WaypointValuePorts(map_reference=MapRef, pose=Pose2D, error=DomainError),
            transport_errors=(IpcError, OSError, ValueError, ValidationError),
        )


class TaskAwareMotion(_TaskAwareMotion, IpcMotion):
    """//Compatible: original constructor/MRO; task/manual methods have one manager owner."""
