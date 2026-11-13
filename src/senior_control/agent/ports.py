"""Bounded IPC adapters for ROS runtime consumers."""

import asyncio
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Protocol

from senior_control.agent.connection import IpcMotion
from senior_control.protocol import Envelope, MotionRuntimeState
from senior_control.settings.service import SettingsService

if TYPE_CHECKING:
    from senior_control.tasks.executor import TaskExecutor

from pydantic import BaseModel, ConfigDict, Field, ValidationError

from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef, Pose2D
from senior_control.navigation import RoutePlan, RouteProgress, RouteRejected
from senior_control.protocol import RobotPosePayload, ScanPointPayload
from senior_control.runtime.ipc import IpcError
from senior_control.tasks.lifecycle import Effect
from senior_control.tasks.models import ExecutionContext, TaskDefinition
from senior_control.tasks.store import TaskStore
from senior_control.waypoints.store import WaypointStore


class Requester(Protocol):
    async def request(self, kind: str, **fields: Any) -> dict[str, Any]: ...


class IpcMapPort:
    def __init__(self, client: Requester) -> None:
        self._client = client

    async def _call(self, kind: str, **fields: Any) -> None:
        try:
            reply = await self._client.request(kind, **fields)
        except (IpcError, OSError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS map service unavailable") from exc
        if reply.get("ok") is not True:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS map service unavailable")

    async def export(self, operation_id: str, map_ref: MapRef, destination: Path) -> None:
        await self._call(
            "map.export",
            operation_id=operation_id,
            map_id=map_ref.map_id,
            reference_revision=map_ref.reference_revision,
            path=str(destination),
        )

    async def import_(self, operation_id: str, path: Path) -> None:
        await self._call("map.import", operation_id=operation_id, path=str(path))

    async def clear(self, operation_id: str, expected: MapRef) -> None:
        await self._call(
            "map.clear",
            operation_id=operation_id,
            map_id=expected.map_id,
            reference_revision=expected.reference_revision,
        )


class IpcNavigation:
    """Only a matching explicit acknowledgement counts as an accepted action."""

    def __init__(
        self,
        client: Requester,
        waypoints: WaypointStore | None = None,
        *,
        connection_epoch: Callable[[], int] | None = None,
        tasks: TaskStore | None = None,
    ) -> None:
        self._client = client
        self._waypoints = waypoints
        self._connection_epoch = connection_epoch
        self._tasks = tasks

    async def _call(self, kind: str, execution_id: str, **fields: Any) -> dict[str, Any]:
        try:
            reply = await self._client.request(kind, execution_id=execution_id, **fields)
        except (IpcError, OSError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable") from exc
        if (
            kind == "navigation.task_start"
            and reply.get("ok") is False
            and reply.get("error") == "rejected"
            and reply.get("execution_id") == execution_id
        ):
            raise RouteRejected()
        if reply.get("ok") is not True or reply.get("execution_id") != execution_id:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable")
        return reply

    async def task_poses(self, task: TaskDefinition) -> tuple[Pose2D, ...]:
        if self._waypoints is None:
            raise RouteRejected()
        return await self._waypoints.route_poses(task.map_ref, task.waypoint_ids)

    async def task_effect(self, effect: Effect, task: TaskDefinition) -> bool | None:
        if effect.kind == "plan":
            if effect.map_ref is None:
                raise RouteRejected()
            direct = await self._tasks.execution_route(effect.task_id) if self._tasks else None
            if direct is None:
                raise RouteRejected()
            poses = tuple(
                Pose2D(*direct[int(action.split(":", 1)[0])]) for action in effect.remaining
            )
            plan = RoutePlan(
                execution_id=effect.task_id,
                task_id=task.task_id,
                map_id=task.map_ref.map_id,
                reference_revision=task.map_ref.reference_revision,
                map_generation=effect.map_ref,
                poses=[[p.x, p.y, p.yaw] for p in poses],
                task_epoch=effect.epoch,
                effect_id=effect.effect_id,
                action_ids=list(effect.remaining),
                pose_ref=effect.pose_ref,
            )
            reply = await self._call(
                "navigation.task_start", effect.task_id, plan=plan.model_dump()
            )
        else:
            reply = await self._call(
                "navigation.task_cancel",
                effect.task_id,
                effect_id=effect.effect_id,
                task_epoch=effect.epoch,
            )
        return self._effect_result(reply, effect)

    async def task_effect_status(self, effect: Effect) -> bool | None:
        reply = await self._call(
            "navigation.task_status",
            effect.task_id,
            effect_id=effect.effect_id,
            task_epoch=effect.epoch,
            effect_kind=effect.kind,
        )
        return self._effect_result(reply, effect)

    @staticmethod
    def _effect_result(reply: dict[str, Any], effect: Effect) -> bool | None:
        if (
            reply.get("lifecycle_version") != 2
            or reply.get("effect_id") != effect.effect_id
            or reply.get("task_epoch") != effect.epoch
        ):
            return None
        if effect.kind == "revoke":
            return (
                True
                if (
                    (reply.get("sources_fenced_through_epoch") or -1) >= effect.epoch
                    and reply.get("controller_output_exited") is True
                )
                else None
            )
        result = reply.get("accepted")
        return result if isinstance(result, bool) else None

    async def start(self, execution_id: str, task: TaskDefinition) -> None:
        # //Compatible: legacy task IPC cannot fence an accepted reference generation.
        del execution_id, task
        raise DomainError(ErrorCode.UNAVAILABLE, "qualified ROS navigation unavailable")

    async def start_qualified(
        self, execution_id: str, task: TaskDefinition, map_generation: str
    ) -> None:
        epoch = self._connection_epoch() if self._connection_epoch is not None else None
        if self._waypoints is None:
            raise RouteRejected()
        try:
            poses = await self._waypoints.route_poses(task.map_ref, task.waypoint_ids)
        except DomainError as exc:
            raise RouteRejected() from exc
        await self.start_poses(
            execution_id,
            task.map_ref,
            map_generation,
            poses,
            task_id=task.task_id,
            connection_epoch=epoch,
        )

    async def start_poses(
        self,
        execution_id: str,
        map_ref: MapRef,
        map_generation: str,
        poses: tuple[Pose2D, ...],
        *,
        task_id: str | None = None,
        connection_epoch: int | None = None,
    ) -> None:
        if connection_epoch is None and self._connection_epoch is not None:
            connection_epoch = self._connection_epoch()
        try:
            plan = RoutePlan(
                execution_id=execution_id,
                map_id=map_ref.map_id,
                task_id=task_id,
                reference_revision=map_ref.reference_revision,
                map_generation=map_generation,
                poses=[[pose.x, pose.y, pose.yaw] for pose in poses],
                connection_epoch=connection_epoch,
            )
        except ValidationError as exc:
            raise RouteRejected() from exc
        reply = await self._call("navigation.task_start", execution_id, plan=plan.model_dump())
        if reply.get("map_generation") != map_generation:
            raise DomainError(ErrorCode.CONFLICT, "ROS accepted a different reference generation")

    async def cancel(self, execution_id: str) -> None:
        await self._call("navigation.task_cancel", execution_id)

    async def status(self, execution_id: str) -> RouteProgress | None:
        reply = await self._call("navigation.task_status", execution_id)
        return self._progress(reply)

    async def current(self) -> RouteProgress | None:
        try:
            reply = await self._client.request("navigation.current")
        except (IpcError, OSError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable") from exc
        if reply.get("ok") is not True:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable")
        return self._progress(reply)

    async def cancel_current(self) -> None:
        current = await self.current()
        if current is not None:
            await self.cancel(current.execution_id)

    @staticmethod
    def _progress(reply: dict[str, Any]) -> RouteProgress | None:
        data = reply.get("progress")
        if data is None:
            return None
        try:
            return RouteProgress.model_validate(data)
        except ValidationError as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "invalid ROS navigation status") from exc


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
    motion_runtime: MotionRuntimeState | None = None
    motion_runtime_version: int = 0
    manual_protocol_version: int = 0
    manual_authority: dict[str, Any] | None = None
    task_lifecycle_version: int = 0
    task_evidence_ready: bool = False
    valid_until: float | None = None

    def for_tasks(self, connected: bool) -> ExecutionContext:
        from senior_control.agent.manual_control import runtime_healthy

        runtime = self.motion_runtime
        now = time.monotonic()
        healthy = self.task_evidence_ready
        admission = self.navigation_ready
        if self.motion_runtime_version >= 1 and runtime is None:
            healthy, admission = False, False
        if runtime is not None:
            healthy = (
                healthy
                and runtime_healthy(runtime, now)
                and runtime.mode not in {"estop", "execution_fault", "feedback_stop"}
            )
            admission = admission and runtime.stationary
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
            motion_halt_reason=(
                "estop"
                if runtime is not None and runtime.mode == "estop"
                else "fault"
                if runtime is not None and runtime.mode == "execution_fault"
                else "feedback_lost"
                if self.motion_runtime_version >= 1 and not runtime_healthy(runtime, now)
                else None
            ),
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


class IpcState:
    def __init__(self, client: Requester, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._client = client
        self._clock = clock

    async def read(self) -> RuntimeState:
        try:
            raw = await self._client.request("state.get")
            state = StateReply.model_validate(raw)
            age = self._clock() - state.sampled_at
            if not 0 <= age <= 0.5:
                raise ValueError("state is stale")
            if (state.map_id is None) != (state.reference_revision is None):
                raise ValueError("incomplete map reference")
            map_ref = (
                None
                if state.map_id is None or state.reference_revision is None
                else MapRef(state.map_id, state.reference_revision)
            )
            geometry_live = (
                state.localized
                and map_ref is not None
                and state.map_generation is not None
                and state.map_generation == state.localization_generation
            )
            return RuntimeState(
                state.localized,
                state.safety_clear,
                state.manual_driving,
                map_ref,
                state.execution_status,
                state.map_generation,
                state.localization_generation,
                state.navigation_ready,
                state.route_progress,
                robot_pose=state.robot_pose if geometry_live else None,
                scan_points=tuple(state.scan_points) if geometry_live else (),
                navigation_block_reason=state.navigation_block_reason,
                motion_runtime=state.motion_runtime,
                motion_runtime_version=state.motion_runtime_version,
                manual_protocol_version=state.manual_protocol_version,
                manual_authority=state.manual_authority,
                task_lifecycle_version=state.task_lifecycle_version,
                task_evidence_ready=state.task_evidence_ready,
                valid_until=min(state.sampled_at + 0.5, state.motion_runtime.valid_until)
                if state.motion_runtime is not None
                else state.sampled_at + 0.5,
            )
        except (IpcError, OSError, ValueError, ValidationError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS state unavailable") from exc


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
    simulation: bool = False


class IpcHomePort:
    def __init__(self, client: Requester, store: WaypointStore, *, robot_id: str) -> None:
        self._client = client
        self._store = store
        self._robot_id = robot_id
        self._state = IpcState(client)

    async def home_revision(self) -> int:
        return await self._store.home_revision()

    async def set_current(self, expected_revision: int) -> int:
        try:
            pose = PoseReply.model_validate(await self._client.request("pose.get"))
            map_ref = MapRef(pose.map_id, pose.reference_revision)
            state = await self._state.read()
        except (IpcError, OSError, ValueError, ValidationError) as exc:
            raise DomainError(
                ErrorCode.UNAVAILABLE, "qualified ROS localization unavailable"
            ) from exc
        if (
            not pose.simulation
            or not 0 <= time.monotonic() - pose.sampled_at <= 0.5
            or not state.localized
            or state.map_ref != map_ref
            or not pose.map_generation
            or pose.map_generation != state.map_generation
            or pose.map_generation != state.localization_generation
        ):
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "qualified pose unavailable")
        await self._store.save_home(
            waypoint_id=None,
            pose=Pose2D(pose.x, pose.y, pose.yaw),
            map_ref=map_ref,
            anchor_id=pose.anchor_id,
            anchor_pose=Pose2D(pose.anchor_x, pose.anchor_y, pose.anchor_yaw),
            source_robot_id=self._robot_id,
            status="confirmed",
            adopted=True,
            latitude=None,
            longitude=None,
            expected_revision=expected_revision,
        )
        return expected_revision + 1


class TaskAwareMotion(IpcMotion):
    """Route manual acquisition and explicit resume through the task transaction."""

    def bind_tasks(self, tasks: "TaskExecutor") -> None:
        self.tasks = tasks

    settings: SettingsService | None = None
    _manual_clock: Callable[[], float] = time.monotonic

    def bind_clock(self, clock: Callable[[], float]) -> None:
        """Use the same steady domain as Agent ingress/deadline construction."""
        self._manual_clock = clock

    async def manual(
        self,
        envelope: Envelope,
        *,
        received_steady: float | None = None,
        deadline_steady: float | None = None,
        **fields: Any,
    ) -> dict[str, Any]:
        if envelope.type in {"control.release", "safety.estop.reset", "safety.fault.recover"}:
            return await super().manual(
                envelope,
                received_steady=received_steady,
                deadline_steady=deadline_steady,
            )
        if self.settings is None:
            raise DomainError(ErrorCode.UNAVAILABLE, "vehicle driving profile is not selected")
        for attempt in range(5):
            if envelope.type == "control.acquire":
                try:
                    retired = await self.tasks.control_current("takeover", envelope.request_id)
                except DomainError as exc:
                    if exc.code is not ErrorCode.UNAVAILABLE or attempt == 4:
                        raise
                    await asyncio.sleep(0.05)
                    continue
            async with self.settings.runtime.admission():
                if envelope.type == "control.acquire":
                    if not await self.tasks.manual_handoff_matches(retired):
                        continue  # Release the gate before retiring the changed owner.
                if envelope.type == "drive.intent" and (
                    deadline_steady is None or self._manual_clock() >= deadline_steady
                ):
                    raise DomainError(ErrorCode.TIMEOUT, "intent expired before local handoff")
                snapshot = self.settings.published_snapshot
                return await super().manual(
                    envelope,
                    received_steady=received_steady,
                    deadline_steady=deadline_steady,
                    driving=snapshot.driving.model_dump(),
                    driving_version=snapshot.driving_version,
                )
        raise DomainError(ErrorCode.UNAVAILABLE, "task ownership changed before manual handoff")

    async def forward(self, envelope: Envelope) -> None:
        # //Compatible: native Agent acquire uses manual(), not this legacy forwarded command.
        if envelope.type == "control.acquire":
            for attempt in range(5):
                try:
                    await self.tasks.control_current("takeover", envelope.request_id)
                    break
                except DomainError as exc:
                    if exc.code is not ErrorCode.UNAVAILABLE or attempt == 4:
                        raise
                    await asyncio.sleep(0.05)
        elif envelope.type in {"navigation.start", "navigation.resume"}:
            await self.tasks.control_current("resume", envelope.request_id)
            return
        await super().forward(envelope)
