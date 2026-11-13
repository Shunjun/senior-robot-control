"""Same-database lifecycle transactions and durable effect obligations.

agent_operation remains the request identity authority. Receipt rows below are
its task-specific result detail, not another admission registry. Every admission
Context is sampled from a synchronous provider inside its transaction; passing a
Context value directly is unsupported. Providers do no external I/O. No external I/O
is performed while Database.write_lock or a SQLite transaction is held.
"""

import asyncio
import hashlib
import json
from collections.abc import Awaitable, Callable
from contextlib import AbstractAsyncContextManager
from contextvars import ContextVar
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Any, Literal, TypeVar

import aiosqlite

from senior_control.agent.journal import AGENT_JOURNAL_STATEMENTS
from senior_control.models import Pose2D
from senior_control.tasks import lifecycle as core
from senior_control.tasks.models import ExecutionRecord, TaskDefinition
from senior_control.tasks.store import TaskStore

T = TypeVar("T")
EffectStatus = Literal["ready", "sending", "done", "retired"]
ContextProvider = Callable[[], core.Context]
Admission = Callable[[], AbstractAsyncContextManager[ContextProvider]]
MAX_EFFECT_BATCH = 100


class VersionConflict(ValueError):
    """The caller must reload the committed snapshot."""


class RequestConflict(ValueError):
    """An operation id already belongs to different content or another domain."""


class StaleEffect(ValueError):
    """An effect no longer owns the current execution boundary."""


@dataclass(frozen=True)
class StoredEffect:
    effect: core.Effect
    status: EffectStatus
    result: bool | None


@dataclass(frozen=True)
class EffectPage:
    effects: tuple[StoredEffect, ...]
    next_cursor: int | None


SCHEMA = (
    "CREATE TABLE IF NOT EXISTS task_lifecycle ("
    "execution_id TEXT PRIMARY KEY REFERENCES task_lifecycle_binding(execution_id),"
    "robot_id TEXT NOT NULL, version INTEGER NOT NULL, boot_id TEXT NOT NULL, state TEXT NOT NULL)",
    "CREATE INDEX IF NOT EXISTS task_lifecycle_active_idx ON task_lifecycle(robot_id) "
    "WHERE json_extract(state, '$.phase') NOT IN ('succeeded','failed','canceled')",
    "CREATE TABLE IF NOT EXISTS task_lifecycle_receipt ("
    "operation_id TEXT PRIMARY KEY REFERENCES agent_operation(operation_id),"
    "execution_id TEXT NOT NULL, receipt TEXT NOT NULL)",
    "CREATE TABLE IF NOT EXISTS task_lifecycle_effect ("
    "effect_id TEXT PRIMARY KEY, execution_id TEXT NOT NULL "
    "REFERENCES task_lifecycle(execution_id),"
    "payload TEXT NOT NULL, status TEXT NOT NULL, result INTEGER)",
    "CREATE INDEX IF NOT EXISTS task_lifecycle_effect_active_idx "
    "ON task_lifecycle_effect(execution_id) WHERE status IN ('ready','sending')",
    "CREATE INDEX IF NOT EXISTS task_lifecycle_effect_audit_idx "
    "ON task_lifecycle_effect(execution_id)",
)


def _json(value: object) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), default=_json_default)


def _json_default(value: object) -> str:
    if isinstance(value, datetime):
        return value.astimezone(timezone.utc).isoformat()
    raise TypeError(f"unsupported persistent value: {type(value)}")


def _effect(payload: str) -> core.Effect:
    raw: dict[str, Any] = json.loads(payload)
    raw["remaining"] = tuple(raw["remaining"])
    return core.Effect(**raw)


def _decode(payload: str) -> core.Lifecycle:
    raw: dict[str, Any] = json.loads(payload)
    raw["actions"] = tuple(raw["actions"])
    raw["completed"] = tuple(raw["completed"])
    raw["pending"] = _effect(_json(raw["pending"])) if raw["pending"] else None
    requests = []
    for cmd, receipt in raw["requests"]:
        cmd["actions"] = tuple(cmd["actions"])
        requests.append((core.Command(**cmd), core.Receipt(**receipt)))
    raw["requests"] = tuple(requests)
    return core.Lifecycle(**raw)


class LifecycleStore:
    def __init__(
        self, tasks: TaskStore, *, boot_id: str, admission: Admission | None = None
    ) -> None:
        if not boot_id:
            raise ValueError("process boot identity required")
        self.tasks = tasks
        self.database = tasks._database
        self.robot_id = tasks._robot_id
        self.boot_id = boot_id
        self._admission = admission
        self._sample: ContextVar[ContextProvider | None] = ContextVar(
            "task_commit_context", default=None
        )

    def _context(self, fallback: ContextProvider) -> core.Context:
        return (self._sample.get() or fallback)()

    async def _atomic(
        self, work: Callable[[aiosqlite.Connection], Awaitable[T]], *, mutation: bool = True
    ) -> T:
        async def locked() -> T:
            async with self.database.write_lock:
                async with self.tasks._transaction() as connection:
                    result = await work(connection)
                    sample = self._sample.get()
                    if sample is not None and isinstance(result, core.Transition):
                        if any(effect.kind == "plan" for effect in result.effects):
                            if not sample().motion_allowed:
                                raise VersionConflict("task admission changed before commit")
                    return result

        async def guarded() -> T:
            if not mutation or self._admission is None:
                # //Compatible: isolated stores retain their existing synchronous context port.
                return await locked()
            async with self._admission() as sample:
                token = self._sample.set(sample)
                try:
                    return await locked()
                finally:
                    self._sample.reset(token)

        # Cancellation cannot release the shared lock with an uncompleted commit
        # or queued SQLite write. The caller may receive cancellation after commit;
        # its stable operation id remains queryable in that case.
        pending = asyncio.create_task(guarded())
        try:
            return await asyncio.shield(pending)
        except asyncio.CancelledError:
            while not pending.done():
                try:
                    await asyncio.shield(pending)
                except asyncio.CancelledError:
                    continue
            pending.result()
            raise

    async def prepare(self) -> None:
        async def work(connection: aiosqlite.Connection) -> None:
            for statement in (*AGENT_JOURNAL_STATEMENTS, *SCHEMA):
                await connection.execute(statement)

        await self._atomic(work, mutation=False)

    async def _load(
        self, connection: aiosqlite.Connection, execution_id: str
    ) -> core.Lifecycle | None:
        async with connection.execute(
            "SELECT state FROM task_lifecycle WHERE execution_id = ? AND robot_id = ?",
            (execution_id, self.robot_id),
        ) as cursor:
            row = await cursor.fetchone()
        return _decode(str(row[0])) if row else None

    async def bind_existing(
        self, record: ExecutionRecord, task: TaskDefinition, context: ContextProvider
    ) -> None:
        """Migrate an old instance by retiring output, never by resending navigation.

        Old coarse records have no committed action ledger. Preserve their identity
        as paused/restart after retirement; do not invent completed progress.
        """

        async def work(connection: aiosqlite.Connection) -> None:
            if await self._load(connection, record.execution_id) is not None:
                return
            actual = await self.tasks._execution_in(connection, record.execution_id)
            if actual != record:
                raise VersionConflict("legacy execution changed during ownership transfer")
            await connection.execute(
                "INSERT INTO task_lifecycle_binding VALUES (?)", (record.execution_id,)
            )
            await self.tasks.snapshot_execution_definition(
                connection, record.execution_id, task.task_id
            )
            state = core.Lifecycle(
                task_id=record.execution_id,
                actions=tuple(f"{i}:{point}" for i, point in enumerate(task.waypoint_ids)),
            )
            result = core.event(state, core.Event("restart"), self._context(context))
            await self._save(connection, None, result)

        await self._atomic(work)

    async def instances(self, *, active_only: bool = False) -> tuple[str, ...]:
        async def work(connection: aiosqlite.Connection) -> tuple[str, ...]:
            suffix = (
                " AND json_extract(state, '$.phase') NOT IN ('succeeded','failed','canceled')"
                if active_only
                else ""
            )
            async with connection.execute(
                "SELECT execution_id FROM task_lifecycle WHERE robot_id=?" + suffix + " LIMIT 4097",
                (self.robot_id,),
            ) as cursor:
                result = tuple(str(row[0]) for row in await cursor.fetchall())
            if len(result) > 4096:
                raise RuntimeError("task instance scan limit exceeded")
            return result

        return await self._atomic(work, mutation=False)

    async def load(self, execution_id: str) -> core.Lifecycle:
        async def work(connection: aiosqlite.Connection) -> core.Lifecycle:
            state = await self._load(connection, execution_id)
            if state is None:
                raise KeyError(execution_id)
            return state

        return await self._atomic(work, mutation=False)

    async def query(self, request_id: str) -> core.Receipt | None:
        async def work(connection: aiosqlite.Connection) -> core.Receipt | None:
            async with connection.execute(
                "SELECT r.receipt FROM task_lifecycle_receipt r JOIN agent_operation a "
                "ON a.operation_id = r.operation_id WHERE r.operation_id = ? AND a.robot_id = ?",
                (request_id, self.robot_id),
            ) as cursor:
                row = await cursor.fetchone()
            return core.Receipt(**json.loads(str(row[0]))) if row else None

        return await self._atomic(work, mutation=False)

    async def _replay(
        self,
        connection: aiosqlite.Connection,
        cmd: core.Command,
        fingerprint: str,
    ) -> core.Transition | None:
        async with connection.execute(
            "SELECT a.robot_id, a.fingerprint, r.receipt FROM agent_operation a "
            "LEFT JOIN task_lifecycle_receipt r ON a.operation_id = r.operation_id "
            "WHERE a.operation_id = ?",
            (cmd.request_id,),
        ) as cursor:
            row = await cursor.fetchone()
        if row is None:
            return None
        if row[0] != self.robot_id or row[1] != fingerprint or row[2] is None:
            raise RequestConflict(cmd.request_id)
        receipt = core.Receipt(**json.loads(str(row[2])))
        state = await self._load(connection, receipt.task_id)
        return core.Transition(state or core.Lifecycle(), receipt=receipt)

    async def _receipt(
        self,
        connection: aiosqlite.Connection,
        receipt: core.Receipt,
        fingerprint: str,
    ) -> None:
        await connection.execute(
            "INSERT INTO agent_operation(operation_id,robot_id,fingerprint,status) "
            "VALUES (?,?,?,?)",
            (
                receipt.request_id,
                self.robot_id,
                fingerprint,
                "succeeded" if receipt.accepted else "failed",
            ),
        )
        await connection.execute(
            "INSERT INTO task_lifecycle_receipt VALUES (?,?,?)",
            (receipt.request_id, receipt.task_id, _json(asdict(receipt))),
        )

    async def _save(
        self,
        connection: aiosqlite.Connection,
        before: core.Lifecycle | None,
        result: core.Transition,
    ) -> None:
        state = result.state
        if before is not None and state == before:
            return
        if state.phase in {"starting", "running"}:
            async with connection.execute(
                "SELECT 1 FROM task_execution e JOIN task_definition t ON t.task_id=e.task_id "
                "WHERE t.robot_id=? AND e.execution_id<>? "
                "AND e.status IN ('dispatching','running','cancel_pending') LIMIT 1",
                (self.robot_id, state.task_id),
            ) as cursor:
                if await cursor.fetchone():
                    raise VersionConflict("another motion execution is active")
        payload = _json(asdict(state))
        if before is None:
            await connection.execute(
                "INSERT INTO task_lifecycle VALUES (?,?,?,?,?)",
                (state.task_id, self.robot_id, state.version, self.boot_id, payload),
            )
        else:
            async with connection.execute(
                "UPDATE task_lifecycle SET version=?,boot_id=?,state=? "
                "WHERE execution_id=? AND robot_id=? AND version=?",
                (
                    state.version,
                    self.boot_id,
                    payload,
                    state.task_id,
                    self.robot_id,
                    before.version,
                ),
            ) as cursor:
                if cursor.rowcount != 1:
                    raise VersionConflict("snapshot compare-and-swap failed")
        await self._outbox(connection, before, result)
        # Only this writer projects coarse status for an explicitly bound instance.
        status = {
            "starting": "dispatching",
            "running": "running",
            "revoking": "cancel_pending",
            "paused": "paused",
            "succeeded": "succeeded",
            "failed": "failed",
            "canceled": "paused",
        }[state.phase]
        await connection.execute(
            "UPDATE task_execution SET status=?,navigation_sent=?,rollback_confirmed=? "
            "WHERE execution_id=?",
            (
                status,
                int(state.phase == "running"),
                int(state.phase in core.TERMINAL or state.phase == "paused"),
                state.task_id,
            ),
        )

    async def _outbox(
        self,
        connection: aiosqlite.Connection,
        before: core.Lifecycle | None,
        result: core.Transition,
    ) -> None:
        state = result.state
        keep = [effect.effect_id for effect in result.effects]
        if state.pending:
            keep.append(state.pending.effect_id)
        # A new receipt alone (e.g. a rejected resume) does not consume an
        # outstanding handoff. Explicit stop/restart/replan does retire it.
        if (
            before is not None
            and before.epoch == state.epoch
            and state.stopped_for == "takeover"
            and state.phase in {"paused", "succeeded"}
            and not any(effect.kind == "manual_ready" for effect in result.effects)
        ):
            async with connection.execute(
                "SELECT effect_id,payload FROM task_lifecycle_effect WHERE execution_id=? "
                "AND status IN ('ready','sending')",
                (state.task_id,),
            ) as cursor:
                for effect_id, payload in await cursor.fetchall():
                    if _effect(str(payload)).kind == "manual_ready":
                        keep.append(str(effect_id))
        exclusion = " AND effect_id NOT IN (" + ",".join("?" for _ in keep) + ")" if keep else ""
        await connection.execute(
            "UPDATE task_lifecycle_effect SET status='retired' WHERE execution_id=? "
            "AND status IN ('ready','sending')" + exclusion,
            (state.task_id, *keep),
        )
        for effect in result.effects:
            await connection.execute(
                "INSERT INTO task_lifecycle_effect VALUES (?,?,?,'ready',NULL)",
                (effect.effect_id, state.task_id, _json(asdict(effect))),
            )

    async def submit(
        self,
        cmd: core.Command,
        task: TaskDefinition,
        occurrence: datetime,
        context: ContextProvider,
        *,
        operation_fingerprint: str | None = None,
        route_poses: tuple[Pose2D, ...] | None = None,
        execution_poses: tuple[Pose2D, ...] | None = None,
    ) -> core.Transition:
        if cmd.action != "submit":
            raise ValueError("submit command required")
        fingerprint = (
            operation_fingerprint
            or hashlib.sha256(
                _json(
                    {"command": asdict(cmd), "task": asdict(task), "occurrence": occurrence}
                ).encode()
            ).hexdigest()
        )

        async def work(connection: aiosqlite.Connection) -> core.Transition:
            replay = await self._replay(connection, cmd, fingerprint)
            if replay is not None:
                return replay
            if await self._load(connection, cmd.task_id) is not None:
                raise VersionConflict("instance identity is already owned")
            result = core.command(core.Lifecycle(), cmd, self._context(context))
            assert result.receipt is not None
            if result.receipt.accepted:
                if route_poses is not None:
                    await self.tasks._insert_task(connection, task, revision=task.revision)
                    await self.tasks._bind_link(connection, task)
                    await connection.execute(
                        "INSERT INTO task_direct_route VALUES (?)", (task.task_id,)
                    )
                await self.tasks.claim_in_transaction(
                    connection,
                    task,
                    occurrence,
                    execution_id=cmd.task_id,
                    lifecycle=True,
                )
                poses = execution_poses if execution_poses is not None else route_poses
                if poses is not None:
                    if len(poses) != len(cmd.actions):
                        raise ValueError("every task action needs its accepted target")
                    await connection.execute(
                        "INSERT INTO task_execution_route VALUES (?,?)",
                        (cmd.task_id, _json([[p.x, p.y, p.yaw] for p in poses])),
                    )
                await self._save(connection, None, result)
            await self._receipt(connection, result.receipt, fingerprint)
            return result

        return await self._atomic(work)

    async def command(
        self,
        cmd: core.Command,
        context: ContextProvider,
        *,
        expected_version: int,
        operation_fingerprint: str | None = None,
    ) -> core.Transition:
        if cmd.action == "submit":
            raise ValueError("use submit to bind an execution")
        fingerprint = (
            operation_fingerprint or hashlib.sha256(_json(asdict(cmd)).encode()).hexdigest()
        )

        async def work(connection: aiosqlite.Connection) -> core.Transition:
            replay = await self._replay(connection, cmd, fingerprint)
            if replay is not None:
                return replay
            state = await self._require(connection, cmd.task_id, expected_version)
            result = core.command(state, cmd, self._context(context))
            assert result.receipt is not None
            await self._save(connection, state, result)
            await self._receipt(connection, result.receipt, fingerprint)
            return result

        return await self._atomic(work)

    async def _require(
        self,
        connection: aiosqlite.Connection,
        execution_id: str,
        version: int,
    ) -> core.Lifecycle:
        state = await self._load(connection, execution_id)
        if state is None:
            raise KeyError(execution_id)
        if state.version != version:
            raise VersionConflict(execution_id)
        return state

    async def event(
        self,
        execution_id: str,
        incoming: core.Event,
        context: ContextProvider,
        *,
        expected_version: int,
    ) -> core.Transition:
        async def work(connection: aiosqlite.Connection) -> core.Transition:
            state = await self._require(connection, execution_id, expected_version)
            if (
                incoming.kind in {"action_completed", "completed", "failed"}
                and incoming.epoch != state.epoch
            ):
                raise StaleEffect("old execution epoch")
            result = core.event(state, incoming, self._context(context))
            await self._save(connection, state, result)
            return result

        return await self._atomic(work)

    async def _require_instance(self, connection: aiosqlite.Connection, execution_id: str) -> None:
        # Existence checks for outbox reads must not decode an entire snapshot.
        async with connection.execute(
            "SELECT 1 FROM task_lifecycle WHERE execution_id=? AND robot_id=?",
            (execution_id, self.robot_id),
        ) as cursor:
            if await cursor.fetchone() is None:
                raise KeyError(execution_id)

    async def active_effects(
        self,
        execution_id: str,
        *,
        limit: int = 16,
    ) -> tuple[StoredEffect, ...]:
        """Bounded coordinator input; historical payloads are never loaded."""
        if not 1 <= limit <= MAX_EFFECT_BATCH:
            raise ValueError("effect limit must be between 1 and 100")

        async def work(connection: aiosqlite.Connection) -> tuple[StoredEffect, ...]:
            await self._require_instance(connection, execution_id)
            async with connection.execute(
                "SELECT payload,status,result FROM task_lifecycle_effect "
                "INDEXED BY task_lifecycle_effect_active_idx WHERE execution_id=? "
                "AND status IN ('ready','sending') ORDER BY rowid LIMIT ?",
                (execution_id, limit),
            ) as cursor:
                rows = await cursor.fetchall()
            return tuple(
                StoredEffect(_effect(str(row[0])), row[1], None if row[2] is None else bool(row[2]))
                for row in rows
            )

        return await self._atomic(work, mutation=False)

    async def effect_page(
        self,
        execution_id: str,
        *,
        after: int = 0,
        limit: int = 100,
    ) -> EffectPage:
        """Keyset-paginated audit input, independent of the coordinator."""
        if after < 0 or not 1 <= limit <= MAX_EFFECT_BATCH:
            raise ValueError("invalid effect audit cursor or limit")

        async def work(connection: aiosqlite.Connection) -> EffectPage:
            await self._require_instance(connection, execution_id)
            async with connection.execute(
                "SELECT rowid,payload,status,result FROM task_lifecycle_effect "
                "WHERE execution_id=? AND rowid>? ORDER BY rowid LIMIT ?",
                (execution_id, after, limit + 1),
            ) as cursor:
                rows = list(await cursor.fetchall())
            page = rows[:limit]
            effects = tuple(
                StoredEffect(_effect(str(row[1])), row[2], None if row[3] is None else bool(row[3]))
                for row in page
            )
            cursor_value = int(page[-1][0]) if len(rows) > limit else None
            return EffectPage(effects, cursor_value)

        return await self._atomic(work, mutation=False)

    async def effects(self, execution_id: str) -> tuple[StoredEffect, ...]:
        # //Compatible: full diagnostic history for existing callers/tests only.
        # Production coordination uses active_effects; audit clients use effect_page.
        async def work(connection: aiosqlite.Connection) -> tuple[StoredEffect, ...]:
            if await self._load(connection, execution_id) is None:
                raise KeyError(execution_id)
            async with connection.execute(
                "SELECT payload,status,result FROM task_lifecycle_effect WHERE execution_id=? "
                "ORDER BY rowid",
                (execution_id,),
            ) as cursor:
                rows = await cursor.fetchall()
            return tuple(
                StoredEffect(_effect(str(row[0])), row[1], None if row[2] is None else bool(row[2]))
                for row in rows
            )

        return await self._atomic(work, mutation=False)

    async def claim_effect(self, execution_id: str, effect_id: str) -> StoredEffect | None:
        async def work(connection: aiosqlite.Connection) -> StoredEffect | None:
            if await self._load(connection, execution_id) is None:
                raise KeyError(execution_id)
            async with connection.execute(
                "UPDATE task_lifecycle_effect SET status='sending' "
                "WHERE effect_id=? AND execution_id=? AND status='ready' RETURNING payload",
                (effect_id, execution_id),
            ) as cursor:
                row = await cursor.fetchone()
            return StoredEffect(_effect(str(row[0])), "sending", None) if row else None

        return await self._atomic(work)

    async def acknowledge(
        self,
        execution_id: str,
        effect_id: str,
        epoch: int,
        ok: bool,
        context: ContextProvider,
        *,
        expected_version: int,
    ) -> core.Transition:
        async def work(connection: aiosqlite.Connection) -> core.Transition:
            state = await self._require(connection, execution_id, expected_version)
            async with connection.execute(
                "SELECT payload,status FROM task_lifecycle_effect "
                "WHERE effect_id=? AND execution_id=?",
                (effect_id, execution_id),
            ) as cursor:
                row = await cursor.fetchone()
            if row is None:
                raise StaleEffect(effect_id)
            effect = _effect(str(row[0]))
            if effect.epoch != epoch or state.epoch != epoch or row[1] == "retired":
                raise StaleEffect(effect_id)
            if row[1] == "done":
                return core.Transition(state)
            if row[1] != "sending":
                raise StaleEffect("effect was not claimed for sending")
            if effect.kind == "manual_ready":
                result = core.Transition(state)
            else:
                if state.pending != effect:
                    raise StaleEffect(effect_id)
                result = core.acknowledge(
                    state, effect_id, epoch, ok=ok, context=self._context(context)
                )
            await self._save(connection, state, result)
            status = "sending" if effect.kind == "revoke" and not ok else "done"
            await connection.execute(
                "UPDATE task_lifecycle_effect SET status=?,result=? WHERE effect_id=?",
                (status, int(ok), effect_id),
            )
            return result

        return await self._atomic(work)

    async def recover(self, context: ContextProvider) -> tuple[str, ...]:
        async def work(connection: aiosqlite.Connection) -> tuple[str, ...]:
            async with connection.execute(
                "SELECT execution_id,state,boot_id FROM task_lifecycle WHERE robot_id=?",
                (self.robot_id,),
            ) as cursor:
                rows = await cursor.fetchall()
            for execution_id, payload, boot_id in rows:
                if boot_id == self.boot_id:
                    continue
                # Retire old handoff notifications even when the core is already
                # terminal and restart is a no-op; never grant old manual authority.
                await connection.execute(
                    "UPDATE task_lifecycle_effect SET status='retired' WHERE execution_id=? "
                    "AND status IN ('ready','sending')",
                    (execution_id,),
                )
                record = await self.tasks._execution_in(connection, str(execution_id))
                if record is not None:
                    await self.tasks.snapshot_execution_definition(
                        connection, str(execution_id), record.task_id
                    )
                state = _decode(str(payload))
                result = core.event(state, core.Event("restart"), self._context(context))
                await self._save(connection, state, result)
                await connection.execute(
                    "UPDATE task_lifecycle SET boot_id=? WHERE execution_id=?",
                    (self.boot_id, execution_id),
                )
            return tuple(str(row[0]) for row in rows)

        return await self._atomic(work)
