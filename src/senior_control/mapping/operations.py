"""Transactional map import, export, and clear."""

import asyncio
import hashlib
import json
import shutil
import tempfile
import uuid
from collections.abc import AsyncIterator, Awaitable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, cast

import aiosqlite

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.archive import BundleManifest, MapArchive
from senior_control.mapping.references import (
    SlamSnapshot,
    durable_mkdir,
    durable_move_tree,
    recover_move_tree,
)
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationResult, OperationStatus
from senior_control.storage.database import Database

_GRAPH_WAIT_SECONDS = 0.25

OPERATION_JOURNAL_STATEMENTS: tuple[str, ...] = (
    """
    CREATE TABLE map_operation (
        operation_id TEXT PRIMARY KEY,
        action TEXT NOT NULL,
        payload TEXT NOT NULL,
        status TEXT NOT NULL,
        rollback_confirmed INTEGER NOT NULL,
        relocalized INTEGER NOT NULL
    )
    """,
    """
    CREATE TABLE map_operation_phase (
        operation_id TEXT NOT NULL,
        seq INTEGER NOT NULL,
        phase TEXT NOT NULL,
        PRIMARY KEY (operation_id, seq),
        FOREIGN KEY (operation_id) REFERENCES map_operation (operation_id)
    )
    """,
    """
    CREATE TABLE map_task_link (
        task_id TEXT PRIMARY KEY,
        definition TEXT NOT NULL,
        map_id TEXT NOT NULL,
        reference_revision INTEGER NOT NULL,
        invalidated INTEGER NOT NULL
    )
    """,
    """
    CREATE TABLE map_home_link (
        robot_id TEXT PRIMARY KEY,
        source_robot_id TEXT NOT NULL,
        map_id TEXT NOT NULL,
        reference_revision INTEGER NOT NULL,
        waypoint_id TEXT NOT NULL,
        status TEXT NOT NULL,
        adopted INTEGER NOT NULL
    )
    """,
)


class ProcessCrash(BaseException):
    """Injected process death. It is not an operation result."""


class MotionGate(Protocol):
    async def stop_and_block_navigation(self) -> None: ...


class PoseGraphLoader(Protocol):
    async def load(self, directory: Path, hint: object) -> None: ...


class Checkpoint(Protocol):
    async def checkpoint(self) -> None: ...


class PoseGraphVerifier(Protocol):
    """Backend confirmation for the complete saved graph and its map identity.

    A true result requires the live backend generation to match both the directory
    contents and MapRef, with all earlier mutations complete or fenced so that none
    can later change it. A service acknowledgement or file copy is insufficient.
    """

    async def matches(self, directory: Path, map_ref: MapRef) -> bool: ...


class QuiescenceFence(Protocol):
    """Return only when earlier graph mutations cannot complete or take effect later."""

    async def quiesce(self) -> None: ...


@dataclass(frozen=True)
class HomeLink:
    robot_id: str
    source_robot_id: str
    map_ref: MapRef
    waypoint_id: str
    status: str
    adopted: bool


@dataclass(frozen=True)
class TaskLink:
    task_id: str
    definition: str
    map_ref: MapRef
    invalidated: bool


@dataclass(frozen=True)
class OperationRecord:
    result: OperationResult
    rollback_confirmed: bool
    relocalized: bool


@dataclass(frozen=True)
class _StoredOperation:
    payload: str
    status: OperationStatus
    rollback_confirmed: bool
    relocalized: bool


class MapOperations:
    def __init__(
        self,
        database: Database,
        store: MapReferenceStore,
        archive: MapArchive,
        motion: MotionGate,
        *,
        robot_id: str,
        map_root: Path,
        config_dir: Path,
        loader: PoseGraphLoader | None = None,
        checkpoint: Checkpoint | None = None,
        fault_after: str | None = None,
    ) -> None:
        self._database = database
        self.store = store
        self._archive = archive
        self._motion = motion
        self.robot_id = robot_id
        self.map_root = map_root
        self.config_dir = config_dir
        self._loader = loader
        self._checkpoint = checkpoint
        self._fault_after = fault_after
        self._map_lock = asyncio.Lock()
        self._graph_work: asyncio.Task[None] | None = None
        self._recovery_required = False

    async def prepare(self) -> None:
        from senior_control.storage.core_schema import migrate_core
        from senior_control.storage.migrations import migrate
        from senior_control.storage.schema import DOMAIN_MIGRATIONS

        await migrate(self._database, list(DOMAIN_MIGRATIONS))
        await migrate_core(self._database)
        for name in ("history", ".staging", "active", ".active.previous"):
            _owned_path(self.map_root, name)
        recover_move_tree(self.map_root / "active")
        guard = await self._graph_guard()
        active = self.map_root / "active"
        stopped = False
        if guard is None and active.exists():
            await self._motion.stop_and_block_navigation()
            stopped = True
            try:
                snapshot = SlamSnapshot.read(active)
                await self._confirm_graph(active / "pose_graph", snapshot.map_ref)
            except Exception:
                await self._set_graph_guard("startup-" + uuid.uuid4().hex, active)
                guard = await self._graph_guard()
            else:
                self.store.replace(snapshot)
        if guard is not None:
            self._recovery_required = True
            self.store.review_required = True
            if not stopped:
                await self._motion.stop_and_block_navigation()
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE map_operation
                    SET status = ?
                    WHERE status IN (?, ?, ?)
                    """,
                    (
                        OperationStatus.NEEDS_REVIEW.value,
                        OperationStatus.ACCEPTED.value,
                        OperationStatus.DISPATCHING.value,
                        OperationStatus.RUNNING.value,
                    ),
                )

    async def export_bundle(
        self, map_ref: MapRef, destination: Path, *, operation_id: str | None = None
    ) -> BundleManifest:
        operation_id = _operation_id(operation_id)
        payload = _payload(
            "export",
            {
                "map_id": map_ref.map_id,
                "reference_revision": map_ref.reference_revision,
                "destination": str(destination),
            },
        )
        if not await self._accept(operation_id, "export", payload):
            return await self._archived_manifest(destination)
        await self._phase(operation_id, "dispatching", OperationStatus.DISPATCHING)
        baseline = self.store.map_ref
        try:
            async with self._map_lock:
                self._require_reconciled()
                baseline = self.store.map_ref
                if self.store.map_ref != map_ref:
                    await self._finish(
                        operation_id,
                        OperationStatus.FAILED,
                        rollback_confirmed=False,
                        relocalized=False,
                    )
                    raise DomainError(ErrorCode.CONFLICT, "active map reference does not match")
                await self._phase(operation_id, "running", OperationStatus.RUNNING)
                manifest = await self._archive.export_bundle(self.store, map_ref, destination)
            await self._finish(
                operation_id,
                OperationStatus.SUCCEEDED,
                rollback_confirmed=False,
                relocalized=False,
            )
            return manifest
        except asyncio.CancelledError:
            await self._needs_review(operation_id)
            raise
        except Exception:
            await self._close_open(operation_id, baseline)
            raise

    async def import_bundle(
        self, path: Path, robot_id: str, *, operation_id: str | None = None
    ) -> OperationResult:
        operation_id = _operation_id(operation_id)
        with tempfile.TemporaryDirectory() as raw:
            staging = Path(raw)
            manifest = await self._archive.validate_and_extract(path, staging)
            try:
                snapshot = SlamSnapshot.read(staging / "occupancy")
            except ValueError as exc:
                raise DomainError(ErrorCode.VALIDATION, "stable layer is unreadable") from exc
            if snapshot.map_ref != manifest.map_ref:
                raise DomainError(
                    ErrorCode.VALIDATION, "stable layer does not match the bundle map"
                )
            payload = _payload(
                "import",
                {
                    "robot_id": robot_id,
                    "map_id": manifest.map_ref.map_id,
                    "reference_revision": manifest.map_ref.reference_revision,
                    "package_sha256": _package_digest(staging),
                },
            )
            if not await self._accept(operation_id, "import", payload):
                return await self.query(operation_id)
            self._crash_if("accepted")
            baseline = self.store.map_ref
            previous: SlamSnapshot | None = None
            history: Path | None = None
            graph_switched = False
            try:
                async with self._map_lock:
                    self._require_reconciled()
                    baseline = self.store.map_ref
                    await self._phase(operation_id, "dispatching", OperationStatus.DISPATCHING)
                    self._crash_if("dispatching")
                    await self._motion.stop_and_block_navigation()
                    await self._pause_or_review(operation_id)
                    previous = self.store.snapshot(Path("pose_graph"), ())
                    history = _history_dir(self.map_root, previous.map_ref)
                    await self._backup(previous, history)
                    await self._phase(operation_id, "history_durable", None)
                    await self._phase(operation_id, "running", OperationStatus.RUNNING)
                    graph_stage = self._stage_dir() / "pose_graph"
                    _copy_tree(staging / "pose_graph", graph_stage)
                    await self._set_graph_guard(operation_id, history)
                    graph_switched = True
                    self._graph_verifier()
                    await self._graph_step(self._archive.read_pose_graph(graph_stage))
                    if self._loader is not None:
                        await self._graph_step(
                            self._loader.load(graph_stage, manifest.last_pose_hint)
                        )
                    await self._confirm_graph(graph_stage, snapshot.map_ref)
                    self.store.replace(snapshot)
                    self.store.review_required = True
                    await self._publish_active(snapshot, graph_stage)
                    await self._phase(operation_id, "active_durable", None)
                    await self._finish_import(operation_id, robot_id, manifest)
                    self.store.review_required = snapshot.review_required
                return OperationResult(operation_id, OperationStatus.SUCCEEDED)
            except asyncio.CancelledError:
                if graph_switched:
                    self.store.review_required = True
                await self._needs_review(operation_id)
                raise
            except Exception:
                if graph_switched:
                    self.store.review_required = True
                    await self._needs_review(operation_id)
                else:
                    await self._close_open(operation_id, baseline)
                raise

    async def clear_map(
        self, expected: MapRef, *, operation_id: str | None = None
    ) -> OperationResult:
        operation_id = _operation_id(operation_id)
        payload = _payload(
            "clear",
            {"map_id": expected.map_id, "reference_revision": expected.reference_revision},
        )
        if not await self._accept(operation_id, "clear", payload):
            return await self.query(operation_id)
        self._crash_if("accepted")
        baseline = self.store.map_ref
        previous: SlamSnapshot | None = None
        history: Path | None = None
        graph_switched = False
        try:
            async with self._map_lock:
                self._require_reconciled()
                baseline = self.store.map_ref
                if self.store.map_ref != expected:
                    await self._finish(
                        operation_id,
                        OperationStatus.FAILED,
                        rollback_confirmed=False,
                        relocalized=False,
                    )
                    raise DomainError(ErrorCode.CONFLICT, "active map reference does not match")
                await self._phase(operation_id, "dispatching", OperationStatus.DISPATCHING)
                self._crash_if("dispatching")
                await self._motion.stop_and_block_navigation()
                await self._pause_or_review(operation_id)
                previous = self.store.snapshot(Path("pose_graph"), ())
                history = _history_dir(self.map_root, previous.map_ref)
                await self._backup(previous, history)
                await self._phase(operation_id, "history_durable", None)
                await self._phase(operation_id, "running", OperationStatus.RUNNING)
                await self._set_graph_guard(operation_id, history)
                graph_switched = True
                self._graph_verifier()
                await self._graph_step(self._archive.reset_pose_graph())
                cleared = MapRef(f"unmapped-{uuid.uuid4().hex}", 0)
                cleared_store = MapReferenceStore(
                    cleared, previous.odom_owner, previous.cell_size_m
                )
                snapshot = cleared_store.snapshot(Path("pose_graph"), ())
                graph_stage = self._stage_dir() / "pose_graph"
                await self._archive.write_pose_graph(graph_stage)
                await self._confirm_graph(graph_stage, cleared)
                self.store.replace(snapshot)
                self.store.review_required = True
                await self._publish_active(snapshot, graph_stage)
                await self._phase(operation_id, "active_durable", None)
                await self._invalidate(expected)
                await self._unadopt_home(expected)
                await self._finish(
                    operation_id,
                    OperationStatus.SUCCEEDED,
                    rollback_confirmed=False,
                    relocalized=False,
                    clear_graph_guard=True,
                )
                self.store.review_required = snapshot.review_required
            return OperationResult(operation_id, OperationStatus.SUCCEEDED)
        except asyncio.CancelledError:
            if graph_switched:
                self.store.review_required = True
            await self._needs_review(operation_id)
            raise
        except Exception:
            if graph_switched:
                self.store.review_required = True
                await self._needs_review(operation_id)
            else:
                await self._close_open(operation_id, baseline)
            raise

    async def query(self, operation_id: str) -> OperationResult:
        stored = await self._stored(operation_id)
        if stored is None:
            raise DomainError(ErrorCode.OPERATION_UNKNOWN, "operation was never issued")
        return OperationResult(operation_id, stored.status)

    async def operation_record(self, operation_id: str) -> OperationRecord:
        stored = await self._stored(operation_id)
        if stored is None:
            raise DomainError(ErrorCode.OPERATION_UNKNOWN, "operation was never issued")
        return OperationRecord(
            OperationResult(operation_id, stored.status),
            stored.rollback_confirmed,
            stored.relocalized,
        )

    async def phases(self, operation_id: str) -> tuple[str, ...]:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT phase
                FROM map_operation_phase
                WHERE operation_id = ?
                ORDER BY seq
                """,
                (operation_id,),
            ) as cursor:
                rows = await cursor.fetchall()
            await connection.rollback()
        found: list[str] = []
        for row in rows:
            found.append(_text(row[0]))
        return tuple(found)

    async def home_for(self, robot_id: str) -> HomeLink | None:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT robot_id, source_robot_id, map_id, reference_revision,
                       waypoint_id, status, adopted
                FROM map_home_link
                WHERE robot_id = ?
                """,
                (robot_id,),
            ) as cursor:
                row = await cursor.fetchone()
            await connection.rollback()
        if row is None:
            return None
        return HomeLink(
            _text(row[0]),
            _text(row[1]),
            MapRef(_text(row[2]), _whole(row[3])),
            _text(row[4]),
            _text(row[5]),
            _flag(row[6]),
        )

    async def bind_task(self, task_id: str, definition: str, map_ref: MapRef) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    INSERT INTO map_task_link (
                        task_id, definition, map_id, reference_revision, invalidated
                    )
                    VALUES (?, ?, ?, ?, 0)
                    ON CONFLICT(task_id) DO UPDATE SET
                        definition = excluded.definition,
                        map_id = excluded.map_id,
                        reference_revision = excluded.reference_revision,
                        invalidated = 0
                    """,
                    (task_id, definition, map_ref.map_id, map_ref.reference_revision),
                )

    async def task_links(self) -> tuple[TaskLink, ...]:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT task_id, definition, map_id, reference_revision, invalidated
                FROM map_task_link
                ORDER BY rowid
                """
            ) as cursor:
                rows = await cursor.fetchall()
            await connection.rollback()
        return tuple(
            TaskLink(
                _text(row[0]),
                _text(row[1]),
                MapRef(_text(row[2]), _whole(row[3])),
                _flag(row[4]),
            )
            for row in rows
        )

    async def _accept(self, operation_id: str, action: str, payload: str) -> bool:
        async with self._database.write_lock:
            existing = await self._locked_stored(operation_id)
            if existing is not None:
                if existing.payload != payload:
                    raise DomainError(ErrorCode.CONFLICT, "operation payload does not match")
                return False
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    INSERT INTO map_operation (
                        operation_id, action, payload, status, rollback_confirmed, relocalized
                    )
                    VALUES (?, ?, ?, ?, 0, 0)
                    """,
                    (operation_id, action, payload, OperationStatus.ACCEPTED.value),
                )
                await connection.execute(
                    """
                    INSERT INTO map_operation_phase (operation_id, seq, phase)
                    VALUES (?, 1, 'accepted')
                    """,
                    (operation_id,),
                )
        return True

    async def _phase(self, operation_id: str, phase: str, status: OperationStatus | None) -> None:
        async with self._database.write_lock:
            connection = self._database.connection
            async with connection.execute(
                """
                SELECT COALESCE(MAX(seq), 0)
                FROM map_operation_phase
                WHERE operation_id = ?
                """,
                (operation_id,),
            ) as cursor:
                row = await cursor.fetchone()
            await connection.rollback()
            seq = 1 if row is None else _whole(row[0]) + 1
            async with self._transaction() as writing:
                if status is not None:
                    await writing.execute(
                        "UPDATE map_operation SET status = ? WHERE operation_id = ?",
                        (status.value, operation_id),
                    )
                await writing.execute(
                    """
                    INSERT INTO map_operation_phase (operation_id, seq, phase)
                    VALUES (?, ?, ?)
                    """,
                    (operation_id, seq, phase),
                )

    async def _finish(
        self,
        operation_id: str,
        status: OperationStatus,
        *,
        rollback_confirmed: bool,
        relocalized: bool,
        clear_graph_guard: bool = False,
    ) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE map_operation
                    SET status = ?, rollback_confirmed = ?, relocalized = ?
                    WHERE operation_id = ?
                    """,
                    (
                        status.value,
                        int(rollback_confirmed),
                        int(relocalized),
                        operation_id,
                    ),
                )
                if clear_graph_guard:
                    await connection.execute(
                        "DELETE FROM map_graph_guard WHERE robot_id = ?", (self.robot_id,)
                    )
        if clear_graph_guard:
            self._recovery_required = False

    async def _finish_import(
        self, operation_id: str, robot_id: str, manifest: BundleManifest
    ) -> None:
        from senior_control.waypoints.store import (
            adopted_home_blocks_foreign,
            replace_imported_home,
        )

        home = next((item for item in manifest.waypoints if item.kind == "home"), None)
        # Another robot's home stays unconfirmed. The last pose is only a load hint.
        pending = manifest.source_robot_id != robot_id
        anchor_pose = home.pose if home is not None else None
        if home is not None:
            for binding in self.store.snapshot(Path("pose_graph"), ()).waypoint_bindings:
                if binding.waypoint.waypoint_id == home.waypoint_id:
                    anchor_pose = binding.anchor_pose
                    break
        async with self._database.write_lock:
            async with self._transaction() as connection:
                if home is not None and not (
                    pending and await adopted_home_blocks_foreign(connection, robot_id)
                ):
                    assert anchor_pose is not None
                    await replace_imported_home(
                        connection,
                        robot_id=robot_id,
                        waypoint_id=home.waypoint_id,
                        pose=home.pose,
                        map_ref=home.map_ref,
                        anchor_id=home.anchor_id,
                        anchor_pose=anchor_pose,
                        source_robot_id=manifest.source_robot_id,
                        status="pending_confirmation" if pending else "confirmed",
                    )
                    await connection.execute(
                        """
                        INSERT INTO map_home_link (
                            robot_id, source_robot_id, map_id, reference_revision,
                            waypoint_id, status, adopted
                        )
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                        ON CONFLICT(robot_id) DO UPDATE SET
                            source_robot_id = excluded.source_robot_id,
                            map_id = excluded.map_id,
                            reference_revision = excluded.reference_revision,
                            waypoint_id = excluded.waypoint_id,
                            status = excluded.status,
                            adopted = excluded.adopted
                        """,
                        (
                            robot_id,
                            manifest.source_robot_id,
                            home.map_ref.map_id,
                            home.map_ref.reference_revision,
                            home.waypoint_id,
                            "pending_confirmation" if pending else "confirmed",
                            0 if pending else 1,
                        ),
                    )
                await connection.execute(
                    """
                    UPDATE map_operation
                    SET status = ?, rollback_confirmed = 0, relocalized = 0
                    WHERE operation_id = ?
                    """,
                    (OperationStatus.SUCCEEDED.value, operation_id),
                )
                await connection.execute(
                    "DELETE FROM map_graph_guard WHERE robot_id = ?", (self.robot_id,)
                )
        self._recovery_required = False

    async def _invalidate(self, map_ref: MapRef) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE map_task_link
                    SET invalidated = 1
                    WHERE map_id = ? AND reference_revision = ?
                    """,
                    (map_ref.map_id, map_ref.reference_revision),
                )

    async def _unadopt_home(self, map_ref: MapRef) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    """
                    UPDATE map_home_link
                    SET adopted = 0, status = 'unadopted'
                    WHERE map_id = ? AND reference_revision = ?
                    """,
                    (map_ref.map_id, map_ref.reference_revision),
                )

    async def _stored(self, operation_id: str) -> _StoredOperation | None:
        async with self._database.write_lock:
            return await self._locked_stored(operation_id)

    async def _locked_stored(self, operation_id: str) -> _StoredOperation | None:
        connection = self._database.connection
        async with connection.execute(
            """
            SELECT payload, status, rollback_confirmed, relocalized
            FROM map_operation
            WHERE operation_id = ?
            """,
            (operation_id,),
        ) as cursor:
            row = await cursor.fetchone()
        await connection.rollback()
        if row is None:
            return None
        return _StoredOperation(
            _text(row[0]),
            OperationStatus(_text(row[1])),
            _flag(row[2]),
            _flag(row[3]),
        )

    @asynccontextmanager
    async def _transaction(self) -> AsyncIterator[aiosqlite.Connection]:
        connection = self._database.connection
        await connection.rollback()
        async with connection.execute("BEGIN IMMEDIATE"):
            pass
        try:
            yield connection
        except BaseException:
            await connection.rollback()
            raise
        else:
            await connection.commit()

    async def _pause_or_review(self, operation_id: str) -> None:
        try:
            if self._checkpoint is not None:
                await self._checkpoint.checkpoint()
        except asyncio.CancelledError:
            await self._needs_review(operation_id)
            raise

    async def _needs_review(self, operation_id: str) -> None:
        task = asyncio.current_task()
        if task is not None:
            uncancel = getattr(task, "uncancel", None)
            if callable(uncancel):
                uncancel()
        stored = await self._stored(operation_id)
        if stored is None or _closed(stored.status):
            return
        await self._phase(operation_id, "needs_review", OperationStatus.NEEDS_REVIEW)
        await self._finish(
            operation_id,
            OperationStatus.NEEDS_REVIEW,
            rollback_confirmed=False,
            relocalized=False,
        )

    async def _close_open(self, operation_id: str, baseline: MapRef) -> None:
        stored = await self._stored(operation_id)
        if stored is None or _closed(stored.status):
            return
        if self.store.map_ref != baseline:
            await self._needs_review(operation_id)
            return
        await self._finish(
            operation_id,
            OperationStatus.FAILED,
            rollback_confirmed=False,
            relocalized=False,
        )

    async def _backup(self, snapshot: SlamSnapshot, destination: Path) -> None:
        _check_owned_path(self.map_root, destination)
        stage = self._stage_dir()
        await self._archive.write_pose_graph(stage / "pose_graph")
        snapshot.write(stage)
        _check_owned_path(self.map_root, destination)
        durable_move_tree(stage, destination)

    async def _publish_active(self, snapshot: SlamSnapshot, pose_graph: Path | None) -> None:
        stage = self._stage_dir()
        if pose_graph is None:
            await self._archive.write_pose_graph(stage / "pose_graph")
        else:
            _copy_tree(pose_graph, stage / "pose_graph")
        snapshot.write(stage)
        durable_move_tree(stage, _owned_path(self.map_root, "active"))

    def _require_reconciled(self) -> None:
        if self._recovery_required:
            raise DomainError(ErrorCode.CONFLICT, "map graph outcome needs review")

    async def _set_graph_guard(self, operation_id: str, history: Path) -> None:
        _check_owned_path(self.map_root, history)
        relative = history.relative_to(self.map_root).as_posix()
        self._recovery_required = True
        self.store.review_required = True
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    "INSERT INTO map_graph_guard (robot_id, operation_id, history) "
                    "VALUES (?, ?, ?)",
                    (self.robot_id, operation_id, relative),
                )

    def _graph_verifier(self) -> PoseGraphVerifier:
        graph = self._archive._pose_graph
        if not callable(getattr(graph, "matches", None)):
            raise DomainError(ErrorCode.UNAVAILABLE, "graph identity verification is unavailable")
        return cast(PoseGraphVerifier, graph)

    async def _confirm_graph(self, directory: Path, map_ref: MapRef) -> None:
        verifier = self._graph_verifier()
        try:
            verified = await asyncio.wait_for(
                verifier.matches(directory, map_ref), timeout=_GRAPH_WAIT_SECONDS
            )
        except asyncio.TimeoutError as exc:  # //Compatible: Python 3.10 timeout type.
            raise DomainError(
                ErrorCode.OPERATION_UNKNOWN, "graph identity confirmation timed out"
            ) from exc
        if verified is not True:
            raise DomainError(ErrorCode.OPERATION_UNKNOWN, "graph identity is unconfirmed")

    async def _graph_guard(self) -> tuple[str, str] | None:
        async with self._database.write_lock:
            async with self._database.connection.execute(
                "SELECT operation_id, history FROM map_graph_guard WHERE robot_id = ?",
                (self.robot_id,),
            ) as cursor:
                row = await cursor.fetchone()
        return None if row is None else (_text(row[0]), _text(row[1]))

    async def _clear_graph_guard(self) -> None:
        async with self._database.write_lock:
            async with self._transaction() as connection:
                await connection.execute(
                    "DELETE FROM map_graph_guard WHERE robot_id = ?", (self.robot_id,)
                )
        self._recovery_required = False

    async def _graph_step(self, operation: Awaitable[None], *, bounded: bool = False) -> None:
        async def run() -> None:
            await operation

        task = asyncio.create_task(run())
        self._graph_work = task
        task.add_done_callback(_consume_task_exception)
        try:
            if bounded:
                await asyncio.wait_for(asyncio.shield(task), timeout=_GRAPH_WAIT_SECONDS)
            else:
                await asyncio.shield(task)
        except asyncio.CancelledError:
            await asyncio.wait({task}, timeout=_GRAPH_WAIT_SECONDS)
            raise
        finally:
            if task.done() and self._graph_work is task:
                self._graph_work = None

    async def reconcile(self) -> None:
        """Restore only after the backend fences pending work and confirms the saved graph."""
        async with self._map_lock:
            guard = await self._graph_guard()
            if guard is None:
                return
            await self._motion.stop_and_block_navigation()
            if self._graph_work is not None and not self._graph_work.done():
                raise DomainError(ErrorCode.OPERATION_UNKNOWN, "graph operation is still running")
            graph = self._archive._pose_graph
            quiesce = getattr(graph, "quiesce", None)
            matches = getattr(graph, "matches", None)
            loader_quiesce = getattr(self._loader, "quiesce", None)
            if (
                not callable(quiesce)
                or not callable(matches)
                or (self._loader is not None and not callable(loader_quiesce))
            ):
                raise DomainError(ErrorCode.UNAVAILABLE, "graph recovery fence is unavailable")
            try:
                await asyncio.wait_for(quiesce(), timeout=_GRAPH_WAIT_SECONDS)
                if loader_quiesce is not None:
                    await asyncio.wait_for(loader_quiesce(), timeout=_GRAPH_WAIT_SECONDS)
                history = self.map_root.joinpath(*Path(guard[1]).parts)
                _check_owned_path(self.map_root, history)
                snapshot = SlamSnapshot.read(history)
                await self._graph_step(
                    self._archive.read_pose_graph(history / "pose_graph"), bounded=True
                )
                verified = await asyncio.wait_for(
                    matches(history / "pose_graph", snapshot.map_ref), timeout=_GRAPH_WAIT_SECONDS
                )
                if verified is not True:
                    raise DomainError(ErrorCode.OPERATION_UNKNOWN, "graph identity is unconfirmed")
            except asyncio.TimeoutError as exc:  # //Compatible: Python 3.10 timeout type.
                raise DomainError(ErrorCode.TIMEOUT, "graph recovery timed out") from exc
            stage = self._stage_dir()
            _copy_tree(history / "pose_graph", stage / "pose_graph")
            snapshot.write(stage)
            durable_move_tree(stage, _owned_path(self.map_root, "active"))
            self.store.replace(snapshot)
            await self._clear_graph_guard()

    def _stage_dir(self) -> Path:
        stage = _owned_path(self.map_root, ".staging", uuid.uuid4().hex)
        durable_mkdir(stage)
        return stage

    async def _archived_manifest(self, destination: Path) -> BundleManifest:
        with tempfile.TemporaryDirectory() as raw:
            return await self._archive.validate_and_extract(destination, Path(raw))

    def _crash_if(self, phase: str) -> None:
        if self._fault_after == phase:
            raise ProcessCrash(phase)


def _closed(status: OperationStatus) -> bool:
    return status in {
        OperationStatus.SUCCEEDED,
        OperationStatus.FAILED,
        OperationStatus.NEEDS_REVIEW,
    }


def _history_dir(root: Path, map_ref: MapRef) -> Path:
    identity = map_ref.map_id
    if (
        identity in {".", ".."}
        or identity.startswith("~")
        or any(char in identity for char in ("/", "\\", "\0"))
        or len(identity.encode("utf-8")) > 200
    ):
        identity = "~" + hashlib.sha256(identity.encode("utf-8")).hexdigest()
    return _owned_path(root, "history", identity, str(map_ref.reference_revision))


def _owned_path(root: Path, *parts: str) -> Path:
    path = root.joinpath(*parts)
    _check_owned_path(root, path)
    return path


def _check_owned_path(root: Path, path: Path) -> None:
    try:
        relative = path.relative_to(root)
    except ValueError as exc:
        raise DomainError(ErrorCode.VALIDATION, "map path escapes map root") from exc
    current = root
    if current.is_symlink():
        raise DomainError(ErrorCode.VALIDATION, "map path contains a symlink")
    for part in relative.parts:
        if part in {".", ".."}:
            raise DomainError(ErrorCode.VALIDATION, "map path escapes map root")
        current = current / part
        if current.is_symlink():
            raise DomainError(ErrorCode.VALIDATION, "map path contains a symlink")


def _consume_task_exception(task: asyncio.Task[None]) -> None:
    if not task.cancelled():
        task.exception()


def _package_digest(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        name = path.relative_to(root).as_posix().encode("utf-8")
        digest.update(len(name).to_bytes(8, "big"))
        digest.update(name)
        digest.update(path.stat().st_size.to_bytes(8, "big"))
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(65_536), b""):
                digest.update(chunk)
    return digest.hexdigest()


def _copy_tree(source: Path, destination: Path) -> None:
    if source.is_symlink() or any(path.is_symlink() for path in source.rglob("*")):
        raise DomainError(ErrorCode.VALIDATION, "map tree contains a symlink")
    shutil.copytree(source, destination, dirs_exist_ok=True)


def _operation_id(operation_id: str | None) -> str:
    if operation_id is None or not operation_id.strip():
        return uuid.uuid4().hex
    return operation_id


def _payload(action: str, fields: dict[str, object]) -> str:
    body = {"action": action, **fields}
    return json.dumps(body, sort_keys=True, separators=(",", ":"))


def _text(value: object) -> str:
    if not isinstance(value, str):
        raise DomainError(ErrorCode.VALIDATION, "expected text")
    return value


def _whole(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DomainError(ErrorCode.VALIDATION, "expected integer")
    return value


def _flag(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if value == 0:
        return False
    if value == 1:
        return True
    raise DomainError(ErrorCode.VALIDATION, "expected flag")
