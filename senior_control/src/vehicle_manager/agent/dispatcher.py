"""Original management edit lanes and durable task admission; no actuator ownership."""

import asyncio
import hashlib
import json
import logging
import re
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any, Generic, Protocol, TypeVar

from senior_contracts.errors import ErrorCode
from senior_contracts.wire import OperationEnvelope, TaskExecutionEnvelope

from vehicle_manager.agent.journal import AgentJournal
from vehicle_manager.agent.request_ports import HomeServicePort, Metrics, RequestValues, ResultT
from vehicle_manager.agent.state_ports import StatePort
from vehicle_manager.agent.workspace import WorkspaceReader
from vehicle_manager.tasks.executor import TaskExecutor
from vehicle_manager.tasks.models import TaskDefinition
from vehicle_manager.tasks.ports import MapReference, PoseReading
from vehicle_manager.tasks.schedule import Schedule
from vehicle_manager.tasks.store import TaskStore
from vehicle_manager.waypoints.store import WaypointStore

logger = logging.getLogger("senior_control.agent")

_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


@dataclass(frozen=True)
class Deadlines:
    """Request budgets: edit/map in seconds, safety in milliseconds."""

    edit: float = 5.0
    map: float = 120.0
    safety_ms: float = 500.0

    def __post_init__(self) -> None:
        if min(self.edit, self.map, self.safety_ms / 1000) <= 0:
            raise ValueError("deadlines must be positive")


class TaskPort(Protocol):
    async def create(self, body: Mapping[str, Any]) -> int: ...

    async def update(self, body: Mapping[str, Any], expected_revision: int) -> int: ...

    async def delete(self, task_id: str, expected_revision: int, *, operation_id: str) -> int: ...

    async def set_enabled(self, task_id: str, enabled: bool, expected_revision: int) -> int: ...


class HomePort(Protocol):
    async def set_current(self, expected_revision: int) -> int: ...

    async def home_revision(self) -> int: ...


MapConsumerT = TypeVar("MapConsumerT", contravariant=True)


class MapPort(Protocol[MapConsumerT]):
    async def export(self, operation_id: str, map_ref: MapConsumerT, destination: Path) -> None: ...

    async def import_(self, operation_id: str, path: Path) -> None: ...

    async def clear(self, operation_id: str, expected: MapConsumerT) -> None: ...


Action = Callable[[], Awaitable[int | None]]
NavigationAction = Callable[[str, str, Mapping[str, Any]], Awaitable[None]]


class Dispatcher(Generic[ResultT]):
    """Runs task, home, and map edits on the robot. Edit and map work use separate lanes."""

    def __init__(
        self,
        robot_id: str,
        *,
        journal: AgentJournal[Any, Any],
        values: RequestValues[ResultT],
        tasks: TaskPort,
        home: HomePort,
        maps: MapPort[Any],
        deadlines: Deadlines,
        metrics: Metrics,
        map_root: Path,
        admit: Callable[[OperationEnvelope], None],
        navigation: NavigationAction | None = None,
        waypoint_create: Callable[[Mapping[str, Any]], Awaitable[None]] | None = None,
    ) -> None:
        self._values = values
        self._robot_id = robot_id
        self._journal = journal
        self._tasks = tasks
        self._home = home
        self._maps = maps
        self._deadlines = deadlines
        self._metrics = metrics
        self._map_root = map_root
        self._admit = admit
        self._navigation = navigation
        self._waypoint_create = waypoint_create
        self._lanes = {"edit": asyncio.Lock(), "map": asyncio.Lock()}
        self._queue_counters = {
            key: values.counter("tasks." + key + ".lock_wait") for key in self._lanes
        }

    async def run(self, envelope: OperationEnvelope) -> ResultT:
        payload = envelope.payload
        operation_id = payload.operation_id
        durable: LifecycleNavigationOperations[ResultT] | None = (
            self._navigation
            if isinstance(self._navigation, LifecycleNavigationOperations)
            else None
        )
        if envelope.type == "operation.query":
            if durable is not None:
                result = await durable.query(operation_id)
                if result is not None:
                    return result
            entry = await self._journal.get(operation_id)
            if entry is None:
                raise self._values.error(ErrorCode.OPERATION_UNKNOWN, "operation was never issued")
            return self._values.result(operation_id, entry.status)

        fingerprint = _fingerprint(envelope)
        existing = await self._journal.get(operation_id)
        if existing is not None and existing.fingerprint != fingerprint:
            raise self._values.error(ErrorCode.CONFLICT, "operation id was used for other content")
        if durable is not None and envelope.type in {"navigation.goal", "navigation.return_home"}:
            if existing is None:
                self._admit(envelope)
            elif await durable.query(operation_id) is None:
                return self._replay(existing, fingerprint)
            return await durable.accept(envelope, fingerprint)
        if existing is not None:
            return self._replay(existing, fingerprint)
        self._admit(envelope)
        lane, action = self._plan(envelope)
        existing = await self._journal.begin(operation_id, fingerprint)
        if existing is not None:
            return self._replay(existing, fingerprint)
        self._note("operation_accepted", envelope)
        return await self._execute(envelope, lane, action)

    def _replay(self, entry: Any, fingerprint: str) -> ResultT:
        if entry.fingerprint != fingerprint:
            raise self._values.error(ErrorCode.CONFLICT, "operation id was used for other content")
        # A retry is acknowledged as success only when the robot's commit is on record.
        if entry.status is self._values.status.SUCCEEDED:
            return self._values.result(entry.operation_id, entry.status)
        if entry.status is self._values.status.FAILED:
            raise self._values.not_committed(
                entry.error or ErrorCode.CONFLICT, "operation failed earlier", entry.status
            )
        if entry.status is self._values.status.NEEDS_REVIEW:
            raise self._values.not_committed(
                ErrorCode.OPERATION_UNKNOWN, "operation outcome needs review", entry.status
            )
        raise self._values.not_committed(
            ErrorCode.CONFLICT, "operation is still running", entry.status
        )

    async def _execute(self, envelope: OperationEnvelope, lane: str, action: Action) -> ResultT:
        operation_id = envelope.payload.operation_id
        started = False

        async def guarded() -> int | None:
            nonlocal started
            token = object()
            counter = self._queue_counters[lane]
            counter.enqueue(token, time.monotonic())
            try:
                async with self._lanes[lane]:
                    counter.start(token, time.monotonic())
                    started = True
                    return await action()
            finally:
                counter.finish(token, time.monotonic())

        deadline = self._deadlines.map if lane == "map" else self._deadlines.edit
        try:
            revision = await asyncio.wait_for(guarded(), timeout=deadline)
        except self._values.error as exc:
            await self._close(envelope, self._values.status.FAILED, exc.code)
            raise
        except asyncio.TimeoutError:
            # A request that never left the queue changed nothing. One that did is unknown.
            status = self._values.status.NEEDS_REVIEW if started else self._values.status.FAILED
            await self._close(envelope, status, ErrorCode.TIMEOUT)
            raise self._values.error(ErrorCode.TIMEOUT, "operation deadline passed") from None
        except asyncio.CancelledError:
            await asyncio.shield(self._close(envelope, self._values.status.NEEDS_REVIEW, None))
            raise
        except Exception:
            logger.exception("operation port failed")
            await self._close(envelope, self._values.status.NEEDS_REVIEW, None)
            raise self._values.error(
                ErrorCode.OPERATION_UNKNOWN, "operation outcome is unknown"
            ) from None
        await self._journal.finish(operation_id, self._values.status.SUCCEEDED, revision=revision)
        self._metrics.inc(
            "agent_operations_total",
            operation=envelope.type,
            result=self._values.status.SUCCEEDED.value,
        )
        self._note("operation_completed", envelope)
        return self._values.result(operation_id, self._values.status.SUCCEEDED)

    async def _close(
        self, envelope: OperationEnvelope, status: Any, error: ErrorCode | None
    ) -> None:
        await self._journal.finish(envelope.payload.operation_id, status, error=error)
        result = error.value if status is self._values.status.FAILED and error else status.value
        self._metrics.inc("agent_operations_total", operation=envelope.type, result=result)
        self._note("operation_failed", envelope)

    def _note(self, event: str, envelope: OperationEnvelope) -> None:
        # Only the event name and correlation ids are logged; bodies may hold secrets.
        logger.info(
            event,
            extra={
                "event": event,
                "robot_id": self._robot_id,
                "request_id": envelope.request_id,
                "operation_id": envelope.payload.operation_id,
            },
        )

    def _plan(self, envelope: OperationEnvelope) -> tuple[str, Action]:
        kind = envelope.type
        payload = envelope.payload
        body = payload.body
        expected = payload.expected_revision
        operation_id = payload.operation_id
        try:
            if kind == "task.create":
                if expected is not None:
                    raise ValueError("create takes no expected revision")
                return "edit", lambda: self._tasks.create(body)
            if kind == "task.update":
                revision = _required(expected)
                return "edit", lambda: self._tasks.update(body, revision)
            if kind == "task.delete":
                revision = _required(expected)
                task_id = _text(body, "task_id")
                return "edit", lambda: self._tasks.delete(
                    task_id, revision, operation_id=operation_id
                )
            if kind == "task.enable":
                revision = _required(expected)
                enabled = body.get("enabled")
                if not isinstance(enabled, bool):
                    raise ValueError("enabled must be a boolean")
                task_id = _text(body, "task_id")
                return "edit", lambda: self._tasks.set_enabled(task_id, enabled, revision)
            if kind == "waypoint.create":
                if expected is not None:
                    raise ValueError("create takes no expected revision")
                waypoint_create = self._waypoint_create
                if waypoint_create is None:
                    raise self._values.error(
                        ErrorCode.UNAVAILABLE, "waypoint editing is unavailable"
                    )
                return "edit", lambda: self._none(waypoint_create(body))
            if kind == "home.set":
                revision = _required(expected)
                return "edit", lambda: self._set_home(revision)
            if kind in {"navigation.goal", "navigation.return_home"}:
                navigation = self._navigation
                if navigation is None:
                    raise self._values.error(
                        ErrorCode.UNAVAILABLE, "qualified navigation is unavailable"
                    )
                if expected is not None:
                    raise ValueError("navigation takes no expected revision")
                return "edit", lambda: self._none(navigation(kind, operation_id, body))
            if kind == "map.export":
                map_ref = _map_ref(body, self._values.map_reference)
                destination = self._transfer_path(body)
                return "map", lambda: self._none(
                    self._maps.export(operation_id, map_ref, destination)
                )
            if kind == "map.import":
                source = self._transfer_path(body)
                return "map", lambda: self._none(self._maps.import_(operation_id, source))
            if kind == "map.clear":
                map_ref = _map_ref(body, self._values.map_reference)
                return "map", lambda: self._none(self._maps.clear(operation_id, map_ref))
        except (KeyError, TypeError, ValueError) as exc:
            raise self._values.error(ErrorCode.VALIDATION, "invalid operation body") from exc
        raise self._values.error(ErrorCode.VALIDATION, "unsupported operation")

    async def _set_home(self, expected_revision: int) -> int:
        return await self._home.set_current(expected_revision)

    @staticmethod
    async def _none(call: Awaitable[None]) -> int | None:
        await call
        return None

    def _transfer_path(self, body: Mapping[str, Any]) -> Path:
        """Map bundles live under one robot-chosen directory; the cloud names only a file."""
        name = _text(body, "name")
        if not _NAME.fullmatch(name):
            raise ValueError("bundle name is not allowed")
        return self._map_root / name


def _fingerprint(envelope: OperationEnvelope) -> str:
    payload = envelope.payload
    canonical = json.dumps(
        [envelope.type, payload.expected_revision, payload.body],
        sort_keys=True,
        separators=(",", ":"),
        default=str,
    )
    return hashlib.sha256(canonical.encode()).hexdigest()


def _required(expected: int | None) -> int:
    if expected is None:
        raise ValueError("expected revision is required")
    return expected


def _text(body: Mapping[str, Any], key: str) -> str:
    value = body[key]
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be text")
    return value


MapT = TypeVar("MapT", bound=MapReference)


def _map_ref(body: Mapping[str, Any], map_reference: Callable[[str, int], MapT]) -> MapT:
    revision = body["reference_revision"]
    if isinstance(revision, bool) or not isinstance(revision, int):
        raise ValueError("reference revision must be an integer")
    return map_reference(_text(body, "map_id"), revision)


# ---------------------------------------------------------------- adapters over the existing stores


class StoreTaskPort(Generic[ResultT]):
    """Task edits through ``TaskStore``; the store keeps its own SQL and revision check."""

    def __init__(
        self,
        store: TaskStore,
        *,
        robot_id: str,
        values: RequestValues[ResultT],
        verify_workspace: Callable[[TaskDefinition, str], Awaitable[None]] | None = None,
    ) -> None:
        self._values = values
        self._store = store
        self._robot_id = robot_id
        self._verify_workspace = verify_workspace

    async def create(self, body: Mapping[str, Any]) -> int:
        definition = self._definition(body)
        await self._qualify_workspace(body, definition)
        stored = await self._store.create(definition)
        return stored.revision

    async def update(self, body: Mapping[str, Any], expected_revision: int) -> int:
        definition = self._definition(body)
        await self._qualify_workspace(body, definition)
        stored = await self._store.update(definition, expected_revision)
        return stored.revision

    async def delete(self, task_id: str, expected_revision: int, *, operation_id: str) -> int:
        return await self._store.delete(task_id, expected_revision, operation_id=operation_id)

    async def set_enabled(self, task_id: str, enabled: bool, expected_revision: int) -> int:
        current = await self._store.get(task_id)
        if current is None:
            raise self._values.error(ErrorCode.CONFLICT, "task does not exist")
        stored = await self._store.update(replace(current, enabled=enabled), expected_revision)
        return stored.revision

    async def _qualify_workspace(self, body: Mapping[str, Any], definition: TaskDefinition) -> None:
        # //Compatible: older clients submit without a workspace raster id.
        if "grid_id" not in body:
            return
        grid_id = body["grid_id"]
        if not isinstance(grid_id, str) or not grid_id.strip() or len(grid_id) > 128:
            raise self._values.error(ErrorCode.VALIDATION, "invalid workspace grid id")
        if any(not isinstance(point, str) for point in body["waypoint_ids"]):
            raise self._values.error(ErrorCode.VALIDATION, "invalid waypoint id")
        if self._verify_workspace is None:
            raise self._values.error(ErrorCode.UNAVAILABLE, "workspace editing is unavailable")
        await self._verify_workspace(definition, grid_id)

    def _definition(self, body: Mapping[str, Any]) -> TaskDefinition:
        try:
            created = datetime.fromisoformat(_text(body, "created_at"))
            raw = body["schedule"]
            timezone_name = _text(body, "timezone")
            if raw["kind"] == "interval":
                schedule = Schedule(
                    "interval",
                    timezone_name,
                    created_at=created,
                    every_seconds=raw["every_seconds"],
                )
            else:
                schedule = Schedule(
                    str(raw["kind"]), timezone_name, expression=raw.get("expression")
                )
            waypoints = body["waypoint_ids"]
            if not isinstance(waypoints, list):
                raise ValueError("waypoint_ids must be a list")
            enabled = body["enabled"]
            return TaskDefinition(
                task_id=_text(body, "task_id"),
                robot_id=self._robot_id,
                map_ref=_map_ref(body, self._values.map_reference),
                waypoint_ids=tuple(str(point) for point in waypoints),
                schedule=schedule,
                timezone=timezone_name,
                enabled=enabled,
                revision=0,
                created_at=created,
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise self._values.error(ErrorCode.VALIDATION, "invalid task definition") from exc


class StoreHomePort(Generic[ResultT]):
    """Sets home from the pose the robot localizes at, never from a pose the cloud supplies."""

    def __init__(
        self,
        service: HomeServicePort,
        localized: Callable[[], Any],
        *,
        values: RequestValues[ResultT],
    ) -> None:
        self._values = values
        self._service = service
        self._localized = localized

    async def set_current(self, expected_revision: int) -> int:
        pose = self._localized()
        if pose is None or not pose.healthy:
            raise self._values.error(ErrorCode.LOCALIZATION_UNAVAILABLE, "robot is not localized")
        await self._service.set_current(pose, expected_revision=expected_revision)
        return await self._service.home_revision()

    async def home_revision(self) -> int:
        return await self._service.home_revision()


class StoreMapPort:
    """Map export, import, and clear through ``MapOperations``, which owns its own journal."""

    def __init__(self, operations: Any, *, robot_id: str) -> None:
        self._operations = operations
        self._robot_id = robot_id

    async def export(self, operation_id: str, map_ref: MapReference, destination: Path) -> None:
        await self._operations.export_bundle(map_ref, destination, operation_id=operation_id)

    async def import_(self, operation_id: str, path: Path) -> None:
        await self._operations.import_bundle(path, self._robot_id, operation_id=operation_id)

    async def clear(self, operation_id: str, expected: MapReference) -> None:
        await self._operations.clear_map(expected, operation_id=operation_id)


class _TaskRouteReceiver(Generic[ResultT]):
    def __init__(
        self,
        executor: TaskExecutor,
        fingerprint: str,
        state: StatePort,
        *,
        values: RequestValues[ResultT],
    ) -> None:
        self._values = values
        self.executor, self.fingerprint, self.state = executor, fingerprint, state

    async def start_poses(
        self,
        execution_id: str,
        map_ref: MapReference,
        map_generation: str,
        poses: tuple[PoseReading, ...],
    ) -> None:
        live = await self.state.read()
        self.executor.context = live.for_tasks(self.executor.context.connected)
        await self.executor.submit_route(
            execution_id, self.fingerprint, map_ref, map_generation, poses
        )

    async def cancel_current(self) -> None:
        raise self._values.error(ErrorCode.VALIDATION, "use lifecycle cancellation")


class LifecycleNavigationOperations(Generic[ResultT]):
    """Direct target validation plus atomic task acceptance; no outer journal.begin."""

    def __init__(
        self,
        executor: "TaskExecutor",
        state: StatePort,
        workspace: WorkspaceReader,
        waypoints: WaypointStore[Any, Any, Any],
        *,
        values: RequestValues[ResultT],
    ) -> None:
        self._values = values
        self.executor, self.state, self.workspace, self.waypoints = (
            executor,
            state,
            workspace,
            waypoints,
        )

    async def query(self, operation_id: str, *, acceptance: bool = False) -> ResultT | None:
        service = self.executor.lifecycle
        assert service is not None
        await service.start()
        receipt = await service.store.query(operation_id)
        if receipt is None:
            return None
        if not receipt.accepted:
            raise self._values.error(ErrorCode.UNAVAILABLE, receipt.reason)
        state = await service.store.load(receipt.task_id)
        status = self._values.status.ACCEPTED
        if not acceptance:
            status = {
                "starting": self._values.status.DISPATCHING,
                "running": self._values.status.RUNNING,
                "succeeded": self._values.status.SUCCEEDED,
                "failed": self._values.status.FAILED,
            }.get(state.phase, status)
        return self._values.task_result(
            operation_id,
            status,
            {
                "execution_id": receipt.task_id,
                "phase": state.phase,
                "epoch": state.epoch,
                "completed": list(state.completed),
                "remaining": list(state.remaining),
            },
        )

    async def execution_view(self, execution_id: str) -> dict[str, Any]:
        """T6 projection only; T2/T5 own the enclosing vehicle frame and authority."""
        service = self.executor.lifecycle
        assert service is not None
        await service.start()
        state = await service.store.load(execution_id)
        actions: list[str] = []
        if state.phase == "paused":
            if (
                self.executor.current_context().motion_allowed
                and await service.store.tasks.execution_route(execution_id) is not None
            ):
                actions.append("resume")
            actions.extend(("cancel", "takeover"))
        elif state.phase in {"starting", "running", "revoking"}:
            actions.extend(("pause", "cancel", "takeover"))
        return {
            "task_id": execution_id,
            "version": state.version,
            "phase": state.phase,
            "completed": len(state.completed),
            "total": len(state.actions),
            "stopped_for": state.stopped_for,
            "allowed_actions": actions,
        }

    async def execution_command(self, envelope: TaskExecutionEnvelope) -> ResultT:
        from dataclasses import asdict

        from vehicle_manager.tasks.lifecycle import Command
        from vehicle_manager.tasks.lifecycle_store import RequestConflict, VersionConflict

        service = self.executor.lifecycle
        assert service is not None
        await service.start()
        payload = envelope.payload
        previous = await service.store.query(payload.request_id)
        if previous is None and payload.action == "resume":
            live = await self.state.read()
            self.executor.context = live.for_tasks(self.executor.context.connected)
            if await service.store.tasks.execution_route(payload.task_id) is None:
                raise self._values.error(ErrorCode.UNAVAILABLE, "accepted task targets need review")
        # The durable request identity is nested, so a transport retry may change
        # its outer session/epoch/id without changing the accepted task command.
        fingerprint = hashlib.sha256(
            json.dumps(payload.model_dump(), sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        try:
            result = await service.command(
                Command(payload.request_id, payload.action, payload.task_id),
                expected_version=payload.expected_version,
                operation_fingerprint=fingerprint,
            )
        except (VersionConflict, RequestConflict) as exc:
            raise self._values.error(
                ErrorCode.CONFLICT, "task version or request conflicts"
            ) from exc
        except KeyError as exc:
            raise self._values.error(ErrorCode.VALIDATION, "unknown task execution") from exc
        assert result.receipt is not None
        # Commit/receipt means accepted, not completion of the external boundary.
        # The regular runtime also reconciles if this request is interrupted here.
        await service.reconcile(payload.task_id)
        return self._values.task_result(
            payload.request_id, self._values.status.ACCEPTED, asdict(result.receipt)
        )

    async def accept(self, envelope: OperationEnvelope, fingerprint: str) -> ResultT:
        operation_id = envelope.payload.operation_id
        previous = await self.query(operation_id, acceptance=True)
        if previous is not None:
            return previous
        receiver = _TaskRouteReceiver(self.executor, fingerprint, self.state, values=self._values)
        validate = self._values.direct_navigation(
            self.state, self.workspace, self.waypoints, receiver
        )
        await asyncio.wait_for(validate(envelope.type, operation_id, envelope.payload.body), 3.0)
        result = await self.query(operation_id, acceptance=True)
        assert result is not None
        return result

    async def __call__(self, kind: str, execution_id: str, body: Mapping[str, Any]) -> None:
        if kind != "navigation.cancel":
            raise self._values.error(
                ErrorCode.VALIDATION, "navigation submission requires its operation envelope"
            )
        await self.executor.control_current("cancel", execution_id)
