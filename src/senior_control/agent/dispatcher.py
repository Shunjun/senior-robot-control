"""Device-owned edits: dedupe by operation id, guard by revision, ack after the robot commits."""

import asyncio
import hashlib
import json
import logging
import re
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any, Protocol

from senior_control.agent.journal import AgentJournal, JournalEntry
from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.operations import MapOperations
from senior_control.models import MapRef, OperationResult, OperationStatus
from senior_control.observability import Metrics
from senior_control.protocol import OperationEnvelope
from senior_control.tasks import Schedule, TaskDefinition, TaskStore
from senior_control.waypoints import HomeService, LocalizedPose

logger = logging.getLogger("senior_control.agent")

_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")


@dataclass(frozen=True)
class Deadlines:
    """Seconds each work queue may spend on one request before it is abandoned."""

    edit: float = 5.0
    map: float = 120.0
    safety: float = 0.5

    def __post_init__(self) -> None:
        if min(self.edit, self.map, self.safety) <= 0:
            raise ValueError("deadlines must be positive")


class TaskPort(Protocol):
    async def create(self, body: Mapping[str, Any]) -> int: ...

    async def update(self, body: Mapping[str, Any], expected_revision: int) -> int: ...

    async def delete(self, task_id: str, expected_revision: int, *, operation_id: str) -> int: ...

    async def set_enabled(self, task_id: str, enabled: bool, expected_revision: int) -> int: ...


class HomePort(Protocol):
    async def set_current(self) -> None: ...


class MapPort(Protocol):
    async def export(self, operation_id: str, map_ref: MapRef, destination: Path) -> None: ...

    async def import_(self, operation_id: str, path: Path) -> None: ...

    async def clear(self, operation_id: str, expected: MapRef) -> None: ...


Action = Callable[[], Awaitable[int | None]]


class Dispatcher:
    """Runs task, home, and map edits on the robot. Edit and map work use separate lanes."""

    def __init__(
        self,
        robot_id: str,
        *,
        journal: AgentJournal,
        tasks: TaskPort,
        home: HomePort,
        maps: MapPort,
        deadlines: Deadlines,
        metrics: Metrics,
        map_root: Path,
        admit: Callable[[OperationEnvelope], None],
    ) -> None:
        self._robot_id = robot_id
        self._journal = journal
        self._tasks = tasks
        self._home = home
        self._maps = maps
        self._deadlines = deadlines
        self._metrics = metrics
        self._map_root = map_root
        self._admit = admit
        self._lanes = {"edit": asyncio.Lock(), "map": asyncio.Lock()}

    async def run(self, envelope: OperationEnvelope) -> OperationResult:
        payload = envelope.payload
        operation_id = payload.operation_id
        if envelope.type == "operation.query":
            entry = await self._journal.get(operation_id)
            if entry is None:
                raise DomainError(ErrorCode.OPERATION_UNKNOWN, "operation was never issued")
            return OperationResult(operation_id, entry.status)

        fingerprint = _fingerprint(envelope)
        existing = await self._journal.get(operation_id)
        if existing is not None:
            return self._replay(existing, fingerprint)
        self._admit(envelope)
        lane, action = self._plan(envelope)
        existing = await self._journal.begin(operation_id, fingerprint)
        if existing is not None:
            return self._replay(existing, fingerprint)
        self._note("operation_accepted", envelope)
        return await self._execute(envelope, lane, action)

    def _replay(self, entry: JournalEntry, fingerprint: str) -> OperationResult:
        if entry.fingerprint != fingerprint:
            raise DomainError(ErrorCode.CONFLICT, "operation id was used for other content")
        if entry.status is OperationStatus.FAILED:
            raise DomainError(entry.error or ErrorCode.CONFLICT, "operation failed earlier")
        return OperationResult(entry.operation_id, entry.status)

    async def _execute(
        self, envelope: OperationEnvelope, lane: str, action: Action
    ) -> OperationResult:
        operation_id = envelope.payload.operation_id
        started = False

        async def guarded() -> int | None:
            nonlocal started
            async with self._lanes[lane]:
                started = True
                return await action()

        deadline = self._deadlines.map if lane == "map" else self._deadlines.edit
        try:
            revision = await asyncio.wait_for(guarded(), timeout=deadline)
        except DomainError as exc:
            await self._close(envelope, OperationStatus.FAILED, exc.code)
            raise
        except asyncio.TimeoutError:
            # A request that never left the queue changed nothing. One that did is unknown.
            status = OperationStatus.NEEDS_REVIEW if started else OperationStatus.FAILED
            await self._close(envelope, status, ErrorCode.TIMEOUT)
            raise DomainError(ErrorCode.TIMEOUT, "operation deadline passed") from None
        except asyncio.CancelledError:
            await asyncio.shield(self._close(envelope, OperationStatus.NEEDS_REVIEW, None))
            raise
        except Exception:
            logger.exception("operation port failed")
            await self._close(envelope, OperationStatus.NEEDS_REVIEW, None)
            raise DomainError(ErrorCode.OPERATION_UNKNOWN, "operation outcome is unknown") from None
        await self._journal.finish(operation_id, OperationStatus.SUCCEEDED, revision=revision)
        self._metrics.inc(
            "agent_operations_total",
            operation=envelope.type,
            result=OperationStatus.SUCCEEDED.value,
        )
        self._note("operation_completed", envelope)
        return OperationResult(operation_id, OperationStatus.SUCCEEDED)

    async def _close(
        self, envelope: OperationEnvelope, status: OperationStatus, error: ErrorCode | None
    ) -> None:
        await self._journal.finish(envelope.payload.operation_id, status, error=error)
        result = error.value if status is OperationStatus.FAILED and error else status.value
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
            if kind == "home.set":
                return "edit", self._set_home
            if kind == "map.export":
                map_ref = _map_ref(body)
                destination = self._transfer_path(body)
                return "map", lambda: self._none(
                    self._maps.export(operation_id, map_ref, destination)
                )
            if kind == "map.import":
                source = self._transfer_path(body)
                return "map", lambda: self._none(self._maps.import_(operation_id, source))
            if kind == "map.clear":
                map_ref = _map_ref(body)
                return "map", lambda: self._none(self._maps.clear(operation_id, map_ref))
        except (KeyError, TypeError, ValueError) as exc:
            raise DomainError(ErrorCode.VALIDATION, "invalid operation body") from exc
        raise DomainError(ErrorCode.VALIDATION, "unsupported operation")

    async def _set_home(self) -> int | None:
        await self._home.set_current()
        return None

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


def _map_ref(body: Mapping[str, Any]) -> MapRef:
    revision = body["reference_revision"]
    if isinstance(revision, bool) or not isinstance(revision, int):
        raise ValueError("reference revision must be an integer")
    return MapRef(_text(body, "map_id"), revision)


# ---------------------------------------------------------------- adapters over the existing stores


class StoreTaskPort:
    """Task edits through ``TaskStore``; the store keeps its own SQL and revision check."""

    def __init__(self, store: TaskStore, *, robot_id: str, journal: AgentJournal) -> None:
        self._store = store
        self._robot_id = robot_id
        self._journal = journal

    async def create(self, body: Mapping[str, Any]) -> int:
        stored = await self._store.create(self._definition(body))
        return stored.revision

    async def update(self, body: Mapping[str, Any], expected_revision: int) -> int:
        definition = self._definition(body)
        await self._require_live(definition.task_id)
        stored = await self._store.update(definition, expected_revision)
        return stored.revision

    async def delete(self, task_id: str, expected_revision: int, *, operation_id: str) -> int:
        """Disable through the store, then record the delete. Row and history are kept."""
        current = await self._store.get(task_id)
        if current is None:
            raise DomainError(ErrorCode.CONFLICT, "task does not exist")
        stored = await self._store.update(replace(current, enabled=False), expected_revision)
        await self._journal.mark_deleted(task_id, operation_id, stored.revision)
        return stored.revision

    async def set_enabled(self, task_id: str, enabled: bool, expected_revision: int) -> int:
        await self._require_live(task_id)
        current = await self._store.get(task_id)
        if current is None:
            raise DomainError(ErrorCode.CONFLICT, "task does not exist")
        stored = await self._store.update(replace(current, enabled=enabled), expected_revision)
        return stored.revision

    async def _require_live(self, task_id: str) -> None:
        if task_id in await self._journal.deleted_task_ids():
            raise DomainError(ErrorCode.CONFLICT, "task was deleted")

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
                map_ref=_map_ref(body),
                waypoint_ids=tuple(str(point) for point in waypoints),
                schedule=schedule,
                timezone=timezone_name,
                enabled=enabled,
                revision=0,
                created_at=created,
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise DomainError(ErrorCode.VALIDATION, "invalid task definition") from exc


class StoreHomePort:
    """Sets home from the pose the robot localizes at, never from a pose the cloud supplies."""

    def __init__(self, service: HomeService, localized: Callable[[], LocalizedPose | None]) -> None:
        self._service = service
        self._localized = localized

    async def set_current(self) -> None:
        pose = self._localized()
        if pose is None or not pose.healthy:
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "robot is not localized")
        await self._service.set_current(pose)


class StoreMapPort:
    """Map export, import, and clear through ``MapOperations``, which owns its own journal."""

    def __init__(self, operations: MapOperations, *, robot_id: str) -> None:
        self._operations = operations
        self._robot_id = robot_id

    async def export(self, operation_id: str, map_ref: MapRef, destination: Path) -> None:
        await self._operations.export_bundle(map_ref, destination, operation_id=operation_id)

    async def import_(self, operation_id: str, path: Path) -> None:
        await self._operations.import_bundle(path, self._robot_id, operation_id=operation_id)

    async def clear(self, operation_id: str, expected: MapRef) -> None:
        await self._operations.clear_map(expected, operation_id=operation_id)
