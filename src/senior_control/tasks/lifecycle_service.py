"""Execute committed outbox obligations without holding a database lock over I/O."""

import asyncio
from dataclasses import dataclass
from datetime import datetime
from typing import Protocol

from senior_control.models import Pose2D
from senior_control.tasks import lifecycle as core
from senior_control.tasks.lifecycle_store import (
    MAX_EFFECT_BATCH,
    ContextProvider,
    LifecycleStore,
    StaleEffect,
    VersionConflict,
)
from senior_control.tasks.models import TaskDefinition


class LifecyclePort(Protocol):
    """Stable effect identity is mandatory at the receiver.

    plan=True confirms reservation, not task completion. revoke=True proves that
    ALL task sources through epoch are fenced AND old controller output exited;
    accepting a cancel RPC is insufficient. manual_ready is a notification to T5,
    which must check current task/authority admission before granting anything.
    Physical execution must recheck current local permission/fences: transaction
    admission cannot freeze live motion safety until execution. The caller/adapter
    imposes finite I/O deadlines; a port that never returns is not time-bounded.
    query never initiates a goal or grants authority. None means unknown and is
    never permission to resend. Physical actions deduplicate by task/action ID.
    """

    async def execute(self, effect: core.Effect) -> bool | None: ...

    async def query(self, effect: core.Effect) -> bool | None: ...


@dataclass(frozen=True)
class ReconcileResult:
    operations: int
    ack_attempts: int
    needs_continue: bool


class LifecycleService:
    def __init__(
        self,
        store: LifecycleStore,
        port: LifecyclePort,
        *,
        context: ContextProvider,
        effect_limit: int = 16,
        ack_attempt_limit: int = 3,
    ) -> None:
        if not 1 <= effect_limit <= MAX_EFFECT_BATCH or not 1 <= ack_attempt_limit <= 16:
            raise ValueError("invalid reconciliation operation or acknowledgement budget")
        self.effect_limit = effect_limit
        self.ack_attempt_limit = ack_attempt_limit
        self.store = store
        self.port = port
        self.context = context
        self._start_lock = asyncio.Lock()
        self._started = False

    async def start(self) -> None:
        async with self._start_lock:
            if not self._started:
                await self.store.prepare()
                await self.store.recover(self.context)
                self._started = True

    async def submit(
        self,
        cmd: core.Command,
        task: TaskDefinition,
        occurrence: datetime,
        *,
        operation_fingerprint: str | None = None,
        route_poses: tuple[Pose2D, ...] | None = None,
        execution_poses: tuple[Pose2D, ...] | None = None,
    ) -> core.Transition:
        await self.start()
        return await self.store.submit(
            cmd,
            task,
            occurrence,
            self.context,
            operation_fingerprint=operation_fingerprint,
            route_poses=route_poses,
            execution_poses=execution_poses,
        )

    async def command(
        self,
        cmd: core.Command,
        *,
        expected_version: int,
        operation_fingerprint: str | None = None,
    ) -> core.Transition:
        await self.start()
        return await self.store.command(
            cmd,
            self.context,
            expected_version=expected_version,
            operation_fingerprint=operation_fingerprint,
        )

    async def event(
        self,
        execution_id: str,
        incoming: core.Event,
        *,
        expected_version: int,
    ) -> core.Transition:
        await self.start()
        return await self.store.event(
            execution_id, incoming, self.context, expected_version=expected_version
        )

    async def acknowledge(
        self,
        effect: core.Effect,
        ok: bool,
        *,
        expected_version: int,
    ) -> core.Transition:
        await self.start()
        return await self.store.acknowledge(
            effect.task_id,
            effect.effect_id,
            effect.epoch,
            ok,
            self.context,
            expected_version=expected_version,
        )

    async def _confirm(self, effect: core.Effect, answer: bool, budget: int) -> int:
        for attempt in range(1, budget + 1):
            state = await self.store.load(effect.task_id)
            try:
                await self.acknowledge(effect, answer, expected_version=state.version)
            except VersionConflict:
                continue
            except StaleEffect:
                pass
            return attempt
        # Nothing was committed. Keep sending/pending; the next pass queries the
        # receiver by the same effect ID instead of re-executing its side effect.
        return budget

    async def reconcile(self, execution_id: str) -> ReconcileResult:
        """Bound operations/ack attempts after startup, not elapsed wall time.

        The local caller/port owns deadlines and cancellation of external waits;
        cancellation leaves sending durable. New effects wait for the next pass.
        Neither a database lock nor an instance lock is held during port I/O.
        """
        await self.start()
        operations = attempts = 0
        batch = await self.store.active_effects(execution_id, limit=self.effect_limit)
        for stored in batch:
            if attempts >= self.ack_attempt_limit:
                break
            effect = stored.effect
            if stored.status == "ready":
                claimed = await self.store.claim_effect(execution_id, effect.effect_id)
                if claimed is None:
                    continue
                operation = self.port.execute
            else:
                operation = self.port.query
            operations += 1
            try:
                answer = await operation(effect)
            except Exception:
                continue
            if answer is not None:
                attempts += await self._confirm(effect, answer, self.ack_attempt_limit - attempts)
        remaining = await self.store.active_effects(execution_id, limit=1)
        return ReconcileResult(operations, attempts, bool(remaining))
