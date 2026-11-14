"""One task-effect adapter over the real qualified navigation IPC contract."""

import asyncio
from typing import Protocol

from senior_control.models import Pose2D
from senior_control.navigation import RouteProgress, RouteRejected
from senior_control.tasks.lifecycle import Context, Effect, Event
from senior_control.tasks.lifecycle_store import LifecycleStore
from senior_control.tasks.models import TaskDefinition


class TaskNavigation(Protocol):
    async def task_poses(self, task: TaskDefinition) -> tuple[Pose2D, ...]: ...
    async def task_effect(self, effect: Effect, task: TaskDefinition) -> bool | None: ...
    async def task_effect_status(self, effect: Effect) -> bool | None: ...
    async def status(self, execution_id: str) -> RouteProgress | None: ...


class NavigationEffects:
    def __init__(self, store: LifecycleStore, navigation: TaskNavigation, *, timeout: float = 0.3):
        self.store = store
        self.navigation = navigation
        self.timeout = timeout

    async def resolve(self, task: TaskDefinition) -> tuple[Pose2D, ...]:
        return await asyncio.wait_for(self.navigation.task_poses(task), self.timeout)

    async def execute(self, effect: Effect) -> bool | None:
        if effect.kind == "manual_ready":
            return await self._handoff(effect)
        task = await self.store.tasks.execution_definition(effect.task_id)
        if task is None:
            return None
        try:
            result = await asyncio.wait_for(self.navigation.task_effect(effect, task), self.timeout)
        except RouteRejected:
            return False
        if effect.kind == "revoke" and result is True:
            await self._committed_actions(effect)
        return result

    async def query(self, effect: Effect) -> bool | None:
        if effect.kind == "manual_ready":
            return await self._handoff(effect)
        result = await asyncio.wait_for(self.navigation.task_effect_status(effect), self.timeout)
        if effect.kind == "revoke" and result is True:
            await self._committed_actions(effect)
        return result

    async def _committed_actions(self, effect: Effect) -> None:
        progress = await self.progress(effect.task_id)
        if progress is None or progress.task_epoch != effect.epoch:
            return
        for action in progress.completed_action_ids:
            state = await self.store.load(effect.task_id)
            if action not in state.completed:
                # Persist action commitments before the output-boundary ack.
                await self.store.event(
                    effect.task_id,
                    Event("action_completed", state.epoch, action),
                    lambda: self._context_for_progress(),
                    expected_version=state.version,
                )

    @staticmethod
    def _context_for_progress() -> Context:
        return Context(False, False)

    async def _handoff(self, effect: Effect) -> bool:
        # This consumes the notification, NOT a manual authority. TaskAwareMotion
        # forwards the real T2 acquire only after reading this durable result.
        state = await self.store.load(effect.task_id)
        return (
            state.epoch == effect.epoch
            and state.phase in {"paused", "succeeded"}
            and state.stopped_for == "takeover"
            and state.pending is None
        )

    async def progress(self, execution_id: str) -> RouteProgress | None:
        return await asyncio.wait_for(self.navigation.status(execution_id), self.timeout)
