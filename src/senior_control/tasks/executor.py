"""Drive accepted tasks through the durable lifecycle and bounded navigation port.

The isolated legacy adapter below remains for unbound callers only. Production
uses LifecycleService as the sole writer of execution identity and progress.
"""

import asyncio
import json
import time
import uuid
from datetime import datetime, timezone
from typing import TYPE_CHECKING, Any

from senior_control.models import Pose2D
from senior_control.tasks.lifecycle import Action, Command, Context, Event, EventKind
from senior_control.tasks.schedule import Schedule

if TYPE_CHECKING:
    from senior_control.tasks.lifecycle_service import LifecycleService
from collections.abc import Callable
from typing import Protocol, cast

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef
from senior_control.navigation import RouteRejected
from senior_control.storage.database import Database
from senior_control.tasks.models import (
    ExecutionContext,
    ExecutionRecord,
    ExecutionStatus,
    IntentAction,
    SkipRecord,
    TaskDefinition,
    TaskIntent,
)
from senior_control.tasks.store import TaskStore


class ProcessCrash(BaseException):
    """Injected process death. It is not an execution result."""


class NavigationPort(Protocol):
    async def cancel(self, execution_id: str) -> None: ...


class QualifiedNavigationPort(Protocol):
    """Atomically fence task acceptance on the reference nonce and live localization.

    The receiver must reject stale or unknown qualification before accepting a goal;
    checking the token only in the caller is insufficient.
    """

    async def start_qualified(
        self, execution_id: str, task: TaskDefinition, map_generation: str
    ) -> None: ...


class NavigationFailure(Exception):
    """A navigation RPC failed; persisted state retains any cancellation obligation."""


class QualificationUnavailable(DomainError):
    """The occurrence was rejected before claiming or invoking navigation."""

    def __init__(self) -> None:
        super().__init__(ErrorCode.UNAVAILABLE, "qualified task navigation is unavailable")


# //Compatible: unbound pre-migration records retain this isolated SQL adapter.
class _UnmigratedExecutor:
    def __init__(
        self,
        database: Database,
        store: TaskStore,
        navigation: NavigationPort | None,
        *,
        fault_after: str | None = None,
        link_up: Callable[[], bool] | None = None,
        generation: Callable[[], int] | None = None,
        # //Compatible: legacy callers without a shared reference remain unavailable.
        map_reference: MapReferenceStore | None = None,
        qualified_reference: Callable[[MapRef, str], bool] | None = None,
    ) -> None:
        if store._database is not database:
            raise ValueError("executor store must use the given database")
        self._database = database
        self._store = store
        self._navigation = navigation
        self.map_reference = map_reference
        self._qualified_reference = qualified_reference
        self._fault_after = fault_after
        self._link_up = link_up if link_up is not None else (lambda: True)
        # //Compatible: injected legacy callers without a connection use one fixed generation.
        self._generation = generation if generation is not None else (lambda: 0)

    async def persist_and_dispatch(
        self,
        intent: TaskIntent,
        task: TaskDefinition,
        *,
        expected_generation: int | None = None,
        map_generation: str | None = None,
        localization_generation: str | None = None,
        localized: bool = False,
    ) -> ExecutionRecord:
        if intent.action is not IntentAction.DISPATCH or intent.occurrence_utc is None:
            raise ValueError("dispatch intent required")
        if intent.task_id != task.task_id:
            raise ValueError("dispatch task does not match the intent")
        if not self._link_up():
            raise QualificationUnavailable()
        started_generation = self._generation()
        if expected_generation is not None and expected_generation != started_generation:
            raise QualificationUnavailable()
        if not self._map_qualified(
            task, map_generation, localization_generation, localized
        ) or not callable(getattr(self._navigation, "start_qualified", None)):
            raise QualificationUnavailable()
        record = await self._store.claim(task, intent.occurrence_utc, generation=started_generation)
        self._crash_if("committed")
        if self._navigation is None or not self._map_qualified(
            task, map_generation, localization_generation, localized
        ):
            return await self._store.mark_needs_review(record.execution_id)
        assert map_generation is not None
        navigation = cast(QualifiedNavigationPort, self._navigation)
        try:
            await navigation.start_qualified(record.execution_id, task, map_generation)
        except RouteRejected:
            return await self._store.mark_needs_review(record.execution_id)
        except asyncio.CancelledError:
            pending = asyncio.create_task(self._store.mark_cancel_pending(record.execution_id))
            await asyncio.shield(pending)
            raise
        except Exception as exc:
            await self._store.mark_cancel_pending(record.execution_id)
            try:
                await self._navigation.cancel(record.execution_id)
            except Exception:
                raise NavigationFailure("start and cancellation are unconfirmed") from exc
            await self._store.mark_needs_review(record.execution_id)
            raise NavigationFailure("start outcome is unknown") from exc
        if not self._map_qualified(task, map_generation, localization_generation, localized):
            await self._store.mark_cancel_pending(record.execution_id)
            try:
                await self._navigation.cancel(record.execution_id)
            except Exception as exc:
                raise NavigationFailure("reconnected action cancellation is unconfirmed") from exc
            return await self._store.mark_needs_review(record.execution_id)
        running = await self._store.mark_running(record.execution_id)
        if running.status is ExecutionStatus.CANCEL_PENDING or not self._map_qualified(
            task, map_generation, localization_generation, localized
        ):
            await self._store.mark_cancel_pending(record.execution_id)
            try:
                await self._navigation.cancel(record.execution_id)
            except Exception as exc:
                raise NavigationFailure("reconnected action cancellation is unconfirmed") from exc
            return await self._store.mark_needs_review(record.execution_id)
        return running

    async def apply(
        self,
        intent: TaskIntent,
        task: TaskDefinition,
        *,
        expected_generation: int | None = None,
        map_generation: str | None = None,
        localization_generation: str | None = None,
        localized: bool = False,
    ) -> ExecutionRecord | SkipRecord:
        if intent.task_id != task.task_id:
            raise ValueError("intent task does not match")
        if intent.action is IntentAction.DISPATCH:
            return await self.persist_and_dispatch(
                intent,
                task,
                expected_generation=expected_generation,
                map_generation=map_generation,
                localization_generation=localization_generation,
                localized=localized,
            )
        if intent.action is IntentAction.PAUSE:
            if intent.execution_id is None:
                raise ValueError("pause requires an execution id")
            pending = await self._store.mark_cancel_pending(intent.execution_id)
            if self._navigation is None:
                raise NavigationFailure("navigation cancellation is unavailable")
            try:
                await self._navigation.cancel(intent.execution_id)
            except Exception as exc:
                raise NavigationFailure("navigation cancellation is unconfirmed") from exc
            if not pending.navigation_sent:
                return await self._store.mark_needs_review(intent.execution_id)
            return await self._store.pause(intent.execution_id)
        if intent.action is IntentAction.SKIP and intent.reason == "clock_jump":
            if intent.skip_from is None or intent.skip_until is None:
                raise ValueError("clock jump requires an interval")
            if intent.skip_from > intent.skip_until:
                raise ValueError("skip interval is inverted")
            return await self._store.record_skip(
                task.task_id, intent.skip_from, intent.skip_until, intent.reason
            )
        if intent.action is IntentAction.SKIP and intent.occurrence_utc is not None:
            return await self._store.skip_occurrence(task, intent.occurrence_utc)
        raise ValueError("unsupported intent")

    def _map_qualified(
        self,
        task: TaskDefinition,
        map_generation: str | None,
        localization_generation: str | None,
        localized: bool,
    ) -> bool:
        if not localized or map_generation is None or localization_generation != map_generation:
            return False
        if self._qualified_reference is not None:
            return self._qualified_reference(task.map_ref, map_generation)
        reference = self.map_reference
        return (
            reference is not None
            and not reference.review_required
            and reference.map_ref == task.map_ref
            and map_generation == reference.map_generation
        )

    async def reconcile(self, record: ExecutionRecord, map_generation: str | None) -> None:
        status = getattr(self._navigation, "status", None)
        if record.status is not ExecutionStatus.RUNNING or not callable(status):
            return
        try:
            progress = await status(record.execution_id)
        except DomainError:
            return
        if progress is None or progress.execution_id != record.execution_id:
            return
        if progress.map_generation != map_generation or progress.status == "cancel_pending":
            await self._store.mark_cancel_pending(record.execution_id)
        elif progress.status == "canceled":
            await self._store.pause(record.execution_id)
        elif progress.status in {"succeeded", "failed"}:
            await self._store.finish(record.execution_id, succeeded=progress.status == "succeeded")

    def _crash_if(self, phase: str) -> None:
        if self._fault_after == phase:
            raise ProcessCrash(phase)


class TaskExecutor(_UnmigratedExecutor):
    """Production task driver: the lifecycle service is the sole bound-instance writer."""

    def __init__(
        self,
        database: Database,
        store: TaskStore,
        navigation: NavigationPort | None,
        *,
        lifecycle: "LifecycleService | None" = None,
        **legacy: Any,
    ) -> None:
        super().__init__(database, store, navigation, **legacy)
        self.lifecycle = lifecycle
        self.context = ExecutionContext(False, False, False, False, None)

    def current_context(self, context: ExecutionContext | None = None) -> Context:
        context = self.context if context is None else context
        fresh = context.valid_until is None or time.monotonic() < context.valid_until
        ready = (
            fresh
            and context.admission_ready
            and context.safety_clear
            and not context.manual_driving
            and context.localized
            and context.map_generation is not None
            and context.map_generation == context.localization_generation
            and context.task_lifecycle_version >= 2
        )
        return Context(
            context.connected,
            ready,
            context.pose_ref if fresh else None,
            context.map_generation if fresh else None,
        )

    async def persist_and_dispatch(
        self, intent: TaskIntent, task: TaskDefinition, **qualification: Any
    ) -> ExecutionRecord:
        if self.lifecycle is None:
            # //Compatible: isolated/unbound callers only; production always injects lifecycle.
            return await super().persist_and_dispatch(intent, task, **qualification)
        if intent.occurrence_utc is None:
            raise ValueError("occurrence is required")
        if (
            self.context.active_map != task.map_ref
            or task.task_id in self.context.invalidated_task_ids
        ):
            raise QualificationUnavailable()
        identity = uuid.uuid5(
            uuid.NAMESPACE_URL,
            f"{task.robot_id}:{task.task_id}:{intent.occurrence_utc.isoformat()}",
        ).hex
        cmd = Command(
            f"schedule:{identity}",
            "submit",
            identity,
            tuple(f"{index}:{point}" for index, point in enumerate(task.waypoint_ids)),
            True,
        )
        from senior_control.tasks.navigation_port import NavigationEffects

        previous = await self.lifecycle.store.query(cmd.request_id)
        if previous is not None:
            if not previous.accepted:
                raise QualificationUnavailable()
            await self.lifecycle.reconcile(identity)
        else:
            if not isinstance(self.lifecycle.port, NavigationEffects):
                raise QualificationUnavailable()
            try:
                poses = await self.lifecycle.port.resolve(task)
            except (DomainError, asyncio.TimeoutError) as exc:
                raise QualificationUnavailable() from exc
            result = await self.lifecycle.submit(
                cmd, task, intent.occurrence_utc, execution_poses=poses
            )
            if result.receipt is None or not result.receipt.accepted:
                raise QualificationUnavailable()
        await self.lifecycle.reconcile(identity)
        stored = await self._store.execution_by_id(identity)
        assert stored is not None
        return stored

    async def apply(
        self, intent: TaskIntent, task: TaskDefinition, **qualification: Any
    ) -> ExecutionRecord | SkipRecord:
        if self.lifecycle is None or intent.action is IntentAction.SKIP:
            return await super().apply(intent, task, **qualification)
        if intent.action is IntentAction.DISPATCH:
            return await self.persist_and_dispatch(intent, task, **qualification)
        assert intent.execution_id is not None
        action: Action = "takeover" if intent.reason == "manual" else "pause"
        await self.control(
            intent.execution_id, action, f"runtime:{intent.execution_id}:{intent.reason}"
        )
        record = await self._store.execution_by_id(intent.execution_id)
        assert record is not None
        return record

    async def submit_route(
        self,
        operation_id: str,
        fingerprint: str,
        ref: MapRef,
        generation: str,
        poses: tuple[Pose2D, ...],
    ) -> str:
        assert self.lifecycle is not None
        if self.context.active_map != ref or self.context.map_generation != generation:
            raise QualificationUnavailable()
        now = datetime.now(timezone.utc)
        identity = uuid.uuid5(uuid.NAMESPACE_URL, f"route:{operation_id}").hex
        definition = TaskDefinition(
            identity,
            self._store._robot_id,
            ref,
            tuple(str(i) for i in range(len(poses))),
            Schedule.interval(now, 60, "UTC"),
            "UTC",
            True,
            1,
            now,
        )
        cmd = Command(
            operation_id, "submit", identity, tuple(f"{i}:{i}" for i in range(len(poses)))
        )
        result = await self.lifecycle.submit(
            cmd, definition, now, operation_fingerprint=fingerprint, route_poses=poses
        )
        if result.receipt is None or not result.receipt.accepted:
            raise QualificationUnavailable()
        await self.lifecycle.reconcile(identity)
        return identity

    async def control(self, execution_id: str, action: Action, request_id: str) -> None:
        from senior_control.tasks.lifecycle_store import RequestConflict, VersionConflict

        assert self.lifecycle is not None
        for _ in range(3):
            state = await self.lifecycle.store.load(execution_id)
            try:
                result = await self.lifecycle.command(
                    Command(request_id, action, execution_id), expected_version=state.version
                )
                break
            except VersionConflict:
                continue
            except RequestConflict as exc:
                raise DomainError(
                    ErrorCode.CONFLICT, "request id belongs to another command"
                ) from exc
        else:
            raise DomainError(ErrorCode.CONFLICT, "task changed; reload before continuing")
        if result.receipt is not None and not result.receipt.accepted:
            if action == "resume":
                raise DomainError(ErrorCode.CONFLICT, result.receipt.reason)
        await self.lifecycle.reconcile(execution_id)
        await self.lifecycle.reconcile(execution_id)

    async def control_current(
        self, action: Action, request_id: str
    ) -> tuple[str, int | None] | None:
        """Return the exact durable owner/revision checked after retirement."""
        assert self.lifecycle is not None
        record = await self._store.latest_execution()
        if record is None:
            if action == "resume":
                raise DomainError(ErrorCode.CONFLICT, "no paused task")
            return None
        try:
            state = await self.lifecycle.store.load(record.execution_id)
        except KeyError:
            if record.status in {ExecutionStatus.SUCCEEDED, ExecutionStatus.FAILED}:
                if action == "resume":
                    raise DomainError(ErrorCode.CONFLICT, "task is terminal") from None
                return record.execution_id, None
            definition = await self._store.get(record.task_id)
            if definition is None:
                raise DomainError(ErrorCode.UNAVAILABLE, "task definition is unavailable") from None
            await self.lifecycle.store.bind_existing(record, definition, self.current_context)
            state = await self.lifecycle.store.load(record.execution_id)
        if action == "resume" and await self._store.execution_route(record.execution_id) is None:
            raise DomainError(
                ErrorCode.UNAVAILABLE, "legacy task targets need review; create a confirmed route"
            )
        if state.phase in {"succeeded", "failed", "canceled"}:
            if action == "resume":
                raise DomainError(ErrorCode.CONFLICT, "task is terminal")
            return record.execution_id, state.version
        if action == "takeover":
            # A retry may retire a new owner or a resumed epoch. Its old receipt
            # must not settle this new obligation under the same acquire request.
            request_id = uuid.uuid5(
                uuid.NAMESPACE_URL, json.dumps([request_id, record.execution_id, state.epoch])
            ).hex
        await self.control(record.execution_id, action, request_id)
        state = await self.lifecycle.store.load(record.execution_id)
        if action == "takeover" and state.phase not in {"succeeded", "failed", "canceled"}:
            notices = await self.lifecycle.store.active_effects(record.execution_id)
            if state.phase != "paused" or state.stopped_for != "takeover" or notices:
                raise DomainError(ErrorCode.UNAVAILABLE, "task output retirement is pending")
        if action == "cancel" and state.phase not in {"canceled", "succeeded", "failed"}:
            raise DomainError(ErrorCode.UNAVAILABLE, "task cancellation is pending")

        return record.execution_id, state.version

    async def manual_handoff_matches(self, retired: tuple[str, int | None] | None) -> bool:
        """Read the authoritative owner under composition's handoff gate.

        Lifecycle mutations use that same gate. This method neither dispatches
        effects nor waits for a port; changed ownership is retired outside it.
        """
        assert self.lifecycle is not None
        record = await self._store.latest_execution()
        if record is None:
            return retired is None
        if retired is None or record.execution_id != retired[0]:
            return False
        try:
            state = await self.lifecycle.store.load(record.execution_id)
        except KeyError:
            # //Compatible: completed unbound history has no lifecycle revision.
            if retired[1] is not None or record.status not in {
                ExecutionStatus.SUCCEEDED,
                ExecutionStatus.FAILED,
            }:
                return False
        else:
            if state.version != retired[1]:
                return False
            if state.phase not in {"succeeded", "failed", "canceled"}:
                if (
                    state.phase != "paused"
                    or state.stopped_for != "takeover"
                    or state.pending is not None
                    or await self.lifecycle.store.active_effects(record.execution_id)
                ):
                    return False
        # The existing coarse projection also exposes an older resumed instance.
        return not await self._store.executions_for_scan(None, datetime.now(timezone.utc))

    async def reconcile(self, record: ExecutionRecord, map_generation: str | None) -> None:
        if self.lifecycle is None:
            await super().reconcile(record, map_generation)
            return
        state = await self.lifecycle.store.load(record.execution_id)
        context = self.context
        if context.motion_halt_reason is not None and state.phase not in {
            "succeeded",
            "failed",
            "canceled",
        }:
            # T5 is the health/ESTOP owner. Preserve this cause even when a
            # previous feedback boundary is still awaiting its real revoke ack.
            await self.lifecycle.event(
                record.execution_id,
                Event(context.motion_halt_reason),
                expected_version=state.version,
            )
        elif state.phase in {"starting", "running"}:
            kind: EventKind | None = None
            if not context.localized or context.map_generation != context.localization_generation:
                kind = "feedback_lost"
            elif not context.safety_clear:
                kind = "fault"
            elif context.manual_driving:
                await self.control(
                    record.execution_id, "takeover", f"observed-takeover:{state.epoch}"
                )
            if kind is not None:
                await self.lifecycle.event(
                    record.execution_id, Event(kind), expected_version=state.version
                )
        elif state.phase in {"paused", "revoking"} and state.stopped_for == "feedback":
            await self.lifecycle.event(
                record.execution_id, Event("feedback_recovered"), expected_version=state.version
            )
        await self.lifecycle.reconcile(record.execution_id)
        from senior_control.tasks.navigation_port import NavigationEffects

        port = self.lifecycle.port
        if not isinstance(port, NavigationEffects):
            return
        try:
            progress = await port.progress(record.execution_id)
        except (DomainError, asyncio.TimeoutError):
            # A bounded status read is retried next tick; it never resends a plan.
            return
        state = await self.lifecycle.store.load(record.execution_id)
        if progress is None or progress.task_epoch != state.epoch:
            return
        for action in progress.completed_action_ids:
            state = await self.lifecycle.store.load(record.execution_id)
            if action not in state.completed:
                await self.lifecycle.event(
                    record.execution_id,
                    Event("action_completed", state.epoch, action),
                    expected_version=state.version,
                )
        state = await self.lifecycle.store.load(record.execution_id)
        if progress.status == "succeeded" and progress.controller_output_exited:
            await self.lifecycle.event(
                record.execution_id, Event("completed", state.epoch), expected_version=state.version
            )
        elif progress.status == "failed" and state.phase in {"starting", "running"}:
            # A Nav2 execution failure is recoverable task suspension; it is not
            # proof of an unrecoverable task-definition failure.
            await self.lifecycle.event(
                record.execution_id, Event("fault"), expected_version=state.version
            )
