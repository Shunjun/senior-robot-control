"""Concrete local IPC state and task projection; no navigation executor imports."""

from dataclasses import dataclass
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field
from senior_contracts.navigation import RouteProgress
from senior_contracts.wire import RobotPosePayload, ScanPointPayload

from vehicle_core.runtime_state import MotionRuntimeState, RuntimeObservation
from vehicle_core.values import MapRef
from vehicle_manager.tasks.models import ExecutionContext


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
