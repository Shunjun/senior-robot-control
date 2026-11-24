"""Vehicle-owned live state and conversion to the shared snapshot envelope."""

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol

from senior_contracts.navigation import RouteProgress
from senior_contracts.wire import (
    HomePayload,
    MapRefPayload,
    PerformancePayload,
    RobotPosePayload,
    ScanPointPayload,
    SnapshotEnvelope,
    SnapshotPayload,
    TaskSummaryPayload,
)

from vehicle_manager.agent.state_ports import WaypointReading
from vehicle_manager.tasks.ports import MapReference as MapRef


@dataclass(frozen=True)
class TaskSummary:
    task_id: str
    revision: int
    enabled: bool


@dataclass(frozen=True)
class LiveState:
    """What the robot's own stores and sensors say right now."""

    localized: bool
    map_ref: MapRef | None
    home: WaypointReading | None
    task_revision: int
    execution_status: str
    runnable_task_ids: tuple[str, ...] = ()
    tasks: tuple[TaskSummary, ...] = ()
    # //Compatible: custom ports without a task cut use the agent journal fallback.
    deleted_task_ids: frozenset[str] | None = None
    map_generation: str | None = None
    route_progress: RouteProgress | None = None
    navigation_ready: bool | None = None
    # //Compatible: older sources do not supply live geometry.
    robot_pose: RobotPosePayload | None = None
    navigation_block_reason: str | None = None
    scan_points: tuple[ScanPointPayload, ...] = ()
    vehicle_ui: dict[str, Any] | None = None
    manual_controller_generation: int | None = None
    manual_agent_epoch: int | None = None
    # //Compatible: legacy producers omit an independent raster version.
    map_grid_id: str | None = None


class StatePort(Protocol):
    async def collect(self) -> LiveState: ...


@dataclass(frozen=True)
class HomeSummary:
    waypoint_id: str
    map_ref: MapRef
    status: str


@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: str
    connected: bool
    localized: bool
    map_ref: MapRef | None
    home: HomeSummary | None
    task_revision: int
    execution_status: str
    collected_at: float
    runnable_task_ids: tuple[str, ...] = ()
    tasks: tuple[TaskSummary, ...] = ()
    home_revision: int = 0
    map_generation: str | None = None
    route_progress: RouteProgress | None = None
    navigation_ready: bool | None = None
    # //Compatible: older sources do not supply live geometry.
    robot_pose: RobotPosePayload | None = None
    navigation_block_reason: str | None = None
    scan_points: tuple[ScanPointPayload, ...] = ()
    vehicle_ui: dict[str, Any] | None = None
    manual_controller_generation: int | None = None
    manual_agent_epoch: int | None = None
    # //Compatible: legacy producers omit an independent raster version.
    map_grid_id: str | None = None
    performance: PerformancePayload | None = None
    mapping_protocol_version: int | None = None


def _map_payload(map_ref: MapRef | None) -> MapRefPayload | None:
    if map_ref is None:
        return None
    return MapRefPayload(map_id=map_ref.map_id, reference_revision=map_ref.reference_revision)


def snapshot_envelope(
    snapshot: RobotSnapshot,
    *,
    robot_id: str,
    session_id: str,
    connection_epoch: int,
    request_id: str,
    seq: int,
    expires_at: float,
    stale: bool = False,
    last_synced_at: float | None = None,
) -> SnapshotEnvelope:
    """The uplink frame a ``Link`` sends for a snapshot.

    ``stale`` and ``last_synced_at`` belong to the cloud's cached copy; an agent leaves them out.
    """
    home = snapshot.home
    frame = SnapshotEnvelope(
        v=1,
        robot_id=robot_id,
        session_id=session_id,
        connection_epoch=connection_epoch,
        request_id=request_id,
        seq=seq,
        expires_at=expires_at,
        type="snapshot",
        payload=SnapshotPayload(
            performance=snapshot.performance,
            mapping_protocol_version=snapshot.mapping_protocol_version,
            connected=snapshot.connected,
            localized=snapshot.localized,
            map_ref=_map_payload(snapshot.map_ref),
            home=None
            if home is None
            else HomePayload(
                waypoint_id=home.waypoint_id,
                map_ref=MapRefPayload(
                    map_id=home.map_ref.map_id,
                    reference_revision=home.map_ref.reference_revision,
                ),
                status=home.status,
            ),
            task_revision=snapshot.task_revision,
            runnable_task_ids=list(snapshot.runnable_task_ids),
            execution_status=snapshot.execution_status,
            collected_at=snapshot.collected_at,
            vehicle_ui=snapshot.vehicle_ui,
            manual_controller_generation=snapshot.manual_controller_generation,
            manual_agent_epoch=snapshot.manual_agent_epoch,
            stale=stale,
            last_synced_at=last_synced_at,
            tasks=[
                TaskSummaryPayload(
                    task_id=item.task_id, revision=item.revision, enabled=item.enabled
                )
                for item in snapshot.tasks
            ],
            home_revision=snapshot.home_revision,
            map_generation=snapshot.map_generation,
            map_grid_id=snapshot.map_grid_id,
            route_progress=snapshot.route_progress if not stale else None,
            navigation_block_reason=snapshot.navigation_block_reason,
            robot_pose=snapshot.robot_pose
            if not stale
            and snapshot.connected
            and snapshot.localized
            and snapshot.map_generation is not None
            else None,
            scan_points=list(snapshot.scan_points)
            if not stale
            and snapshot.connected
            and snapshot.localized
            and snapshot.map_generation is not None
            else [],
        ),
    )
    if snapshot.navigation_ready is not None:
        frame.payload.navigation_ready = (
            snapshot.navigation_ready and snapshot.connected and not stale
        )
    return frame


def snapshot_from_envelope(
    envelope: SnapshotEnvelope, *, map_reference: Callable[[str, int], MapRef]
) -> RobotSnapshot:
    payload = envelope.payload
    home = payload.home
    readiness = payload.navigation_ready if "navigation_ready" in payload.model_fields_set else None
    return RobotSnapshot(
        performance=payload.performance,
        mapping_protocol_version=payload.mapping_protocol_version,
        robot_id=envelope.robot_id,
        connected=payload.connected,
        localized=payload.localized,
        map_ref=None
        if payload.map_ref is None
        else map_reference(payload.map_ref.map_id, payload.map_ref.reference_revision),
        home=None
        if home is None
        else HomeSummary(
            home.waypoint_id,
            map_reference(home.map_ref.map_id, home.map_ref.reference_revision),
            home.status,
        ),
        task_revision=payload.task_revision,
        execution_status=payload.execution_status,
        collected_at=payload.collected_at,
        runnable_task_ids=tuple(payload.runnable_task_ids),
        tasks=tuple(
            TaskSummary(item.task_id, item.revision, item.enabled) for item in payload.tasks
        ),
        home_revision=payload.home_revision,
        map_generation=payload.map_generation,
        map_grid_id=payload.map_grid_id,
        route_progress=payload.route_progress,
        navigation_ready=readiness,
        robot_pose=payload.robot_pose,
        navigation_block_reason=payload.navigation_block_reason,
        scan_points=tuple(payload.scan_points),
        vehicle_ui=payload.vehicle_ui,
        manual_controller_generation=payload.manual_controller_generation,
        manual_agent_epoch=payload.manual_agent_epoch,
    )
