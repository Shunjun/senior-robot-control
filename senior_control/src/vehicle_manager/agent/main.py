"""Original management Agent/session; actuator and backend owners are injected."""

import asyncio
import json
import logging
import signal
import time
from collections.abc import Awaitable, Callable, Mapping, MutableMapping
from pathlib import Path
from typing import Any, Generic

from senior_contracts.errors import ErrorCode
from senior_contracts.wire import (
    HEARTBEAT_INTERVAL_MS,
    EmptyPayload,
    Envelope,
    LearningStartEnvelope,
    LearningStatusEnvelope,
    ManualIntentEnvelope,
    OperationEnvelope,
    PerformancePayload,
    RecoveryEnvelope,
    ResponseEnvelope,
    ResponsePayload,
    SettingsApplyEnvelope,
    SettingsEnvelope,
    SettingsGetEnvelope,
    SettingsResultEnvelope,
    SimpleEnvelope,
    SnapshotEnvelope,
    TaskExecutionEnvelope,
    VideoEnvelope,
    WorkspaceEnvelope,
)

from vehicle_manager.agent.connection import HeartbeatFrame, Link, MotionPort
from vehicle_manager.agent.dispatcher import (
    Deadlines,
    HomePort,
    MapPort,
    NavigationAction,
    TaskPort,
)
from vehicle_manager.agent.journal import AgentJournal
from vehicle_manager.agent.request_ports import Metrics
from vehicle_manager.agent.runtime_ports import AgentRuntimePorts, Clock, ResultT, Signaling
from vehicle_manager.agent.settings_wire import settings_command
from vehicle_manager.agent.snapshots import HomeSummary, RobotSnapshot, StatePort
from vehicle_manager.agent.supervisor import AuthRejected, Supervisor
from vehicle_manager.agent.workspace import WorkspaceReader
from vehicle_manager.settings.service import RuntimeGate, SettingsService

logger = logging.getLogger("vehicle_manager.agent")
HALT_TYPES = frozenset({"drive.stop", "safety.estop"})
MAX_INFLIGHT = 64
MAX_SAFETY_PENDING = 64


def _snapshot_change_key(snapshot: RobotSnapshot) -> tuple[Any, ...]:
    """Retain values, so producer-owned mutable geometry cannot rewrite the sent baseline."""
    pose, route = snapshot.robot_pose, snapshot.route_progress
    return (
        snapshot.localized,
        None if snapshot.performance is None else snapshot.performance.sequence,
        snapshot.map_ref,
        snapshot.map_generation,
        snapshot.map_grid_id,
        None
        if route is None
        else (
            route.execution_id,
            route.task_id,
            route.map_generation,
            route.status,
            route.active_waypoint_index,
        ),
        snapshot.navigation_ready,
        snapshot.home,
        snapshot.task_revision,
        snapshot.execution_status,
        snapshot.runnable_task_ids,
        None if pose is None else (pose.x, pose.y, pose.yaw),
        tuple((point.x, point.y) for point in snapshot.scan_points[:64]),
        snapshot.navigation_block_reason,
        repr(snapshot.vehicle_ui),
    )


class Agent(Generic[ResultT]):
    def __init__(
        self,
        robot_id: str,
        *,
        ports: AgentRuntimePorts[ResultT],
        connector: Callable[[], Awaitable[Link]],
        journal: AgentJournal[Any, Any],
        tasks: TaskPort,
        home: HomePort,
        maps: MapPort[Any],
        state: StatePort,
        motion: MotionPort,
        settings: SettingsService | None = None,
        runtime_gate: RuntimeGate | None = None,
        workspace: WorkspaceReader | None = None,
        navigation: NavigationAction | None = None,
        video_session: Callable[[], object | None] | None = None,
        deadlines: Deadlines | None = None,
        map_root: Path = Path("map-transfer"),
        background: Mapping[str, Callable[[], Awaitable[None]]] | None = None,
        cleanup: Callable[[], Awaitable[None]] | None = None,
        background_fault: Callable[[str], Awaitable[None]] | None = None,
        heartbeat_ms: float = HEARTBEAT_INTERVAL_MS,
        retry_delay_ms: float = 1000,
        command_ttl_ms: float = 300,
        snapshot_seconds: float = 2.0,
        state_poll_ms: float = 250,
        metrics: Metrics | None = None,
        clock: Clock | None = None,
        performance: Callable[[], PerformancePayload | None] = lambda: None,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        if not robot_id.strip():
            raise ValueError("robot id is required")
        self._performance = performance
        self._ports = ports
        self.robot_id = robot_id
        self._settings = settings
        # //Compatible: callers injecting only settings still share that owner's gate.
        self._runtime_gate = runtime_gate or (settings.runtime if settings is not None else None)
        self._intent_queue = ports.counter("agent.manual_intent", 1)
        self._queued_intent: tuple[Link, ManualIntentEnvelope, float, float] | None = None
        self._intent_task: asyncio.Task[None] | None = None
        self.metrics = metrics if metrics is not None else ports.metrics()
        self._connector = connector
        self._journal = journal
        self._state = state
        self._workspace = workspace
        self._navigation = navigation
        self._workspace_request: asyncio.Task[None] | None = None
        self._motion = motion
        self._video_session = video_session or (lambda: None)
        self._deadlines = deadlines if deadlines is not None else Deadlines()
        self._background = dict(background or {})
        self._cleanup = cleanup
        self._background_fault_handler = background_fault
        self.module_faults: dict[str, dict[str, Any]] = {}
        self._failed_modules: set[str] = set()
        self._failed_manual_authority: str | None = None
        self._manual_epoch_failed: int | None = None
        self._safety_fault_task: asyncio.Task[None] | None = None
        self._stragglers: set[asyncio.Task[Any]] = set()
        self._heartbeat_ms = heartbeat_ms
        self._retry_delay_ms = retry_delay_ms
        if command_ttl_ms <= 0:
            raise ValueError("command ttl must be positive")
        self._command_ttl_seconds = command_ttl_ms / 1000
        self._wall_clock = wall_clock
        self._clock = clock if clock is not None else ports.clock()
        # //Compatible: isolated/non-native MotionPort implementations have no admission clock.
        bind_clock = getattr(motion, "bind_clock", None)
        if bind_clock is not None:
            bind_clock(self._clock.monotonic)
        self._home = home
        self._dispatcher = ports.dispatcher(
            robot_id,
            journal=journal,
            tasks=tasks,
            home=home,
            maps=maps,
            deadlines=self._deadlines,
            metrics=self.metrics,
            map_root=map_root,
            admit=self._admit,
            navigation=navigation,
            waypoint_create=workspace.create_waypoint if workspace is not None else None,
        )
        self._link: Link | None = None
        self._revoked_link: Link | None = None
        self._epoch = 0
        self._last_epoch = 0
        self._supervisor: Supervisor | None = None
        self._stopping = False
        self.auth_rejected = False
        self.runtime_failed = False
        self._stop = asyncio.Event()
        self._inflight: set[asyncio.Task[None]] = set()
        self._safety_task: asyncio.Task[None] | None = None
        self._safety_pending: MutableMapping[tuple[str, int, str], tuple[Link, Envelope]] = (
            ports.safety_pending("agent.safety_pending", MAX_SAFETY_PENDING)
        )
        self._estop_receipts: dict[str, tuple[str, bool]] = {}
        self._estop_inflight = False
        self._signals: list[signal.Signals] = []
        if snapshot_seconds <= 0 or state_poll_ms <= 0:
            raise ValueError("snapshot intervals must be positive")
        self._snapshot_seconds = snapshot_seconds
        self._state_poll_seconds = state_poll_ms / 1000

    # ------------------------------------------------------------------ state

    @property
    def connected(self) -> bool:
        return self._link is not None

    @property
    def connection_epoch(self) -> int:
        return self._epoch

    @property
    def failed_modules(self) -> frozenset[str]:
        # Local lifecycle policy is never sourced from reporting-channel messages.
        return frozenset(self._failed_modules)

    @property
    def motion_allowed(self) -> bool:
        supervisor = self._supervisor
        return supervisor is not None and supervisor.motion_allowed and not self._stopping

    @property
    def _safety_blocked(self) -> bool:
        return self._estop_inflight or any(key[0] == "safety.estop" for key in self._safety_pending)

    def _invalidate_gate(self) -> None:
        if self._runtime_gate is not None:
            self._runtime_gate.invalidate()

    # ------------------------------------------------------------------ lifecycle

    async def run(self) -> None:
        supervisor = Supervisor(
            send_heartbeat=self._beat,
            call_ros=self._motion.call,
            connect=self._session,
            background=self._background,
            heartbeat_ms=self._heartbeat_ms,
            retry_delay_ms=self._retry_delay_ms,
            on_fault=self._background_fault,
        )
        self._supervisor = supervisor
        self._log("startup")
        try:
            async with supervisor:
                try:
                    await self._stop.wait()
                finally:
                    await self._disable_motion(supervisor)
        finally:
            await self._release()

    def report_runtime_event(self, event: dict[str, Any]) -> None:
        # Only reporting metadata. No event grants motion or changes the safety owner.
        module = event["module"]
        if module not in self.module_faults and len(self.module_faults) >= 32:
            module = "observer"
        if event["phase"] == "ready":
            if module in self.failed_modules:
                return  # Reporting cannot revive a stopped local module.
            self.module_faults.pop(module, None)
        else:
            self.module_faults[module] = dict(event)
        level = logging.INFO if event["phase"] in {"ready", "starting"} else logging.ERROR
        logger.log(
            level, "runtime module %s: %s %s", module, event["phase"], event.get("detail", "")
        )

    async def _background_fault(self, name: str, reason: str) -> None:
        self._failed_modules.add(name)
        if name != "observer" or "observer" not in self.module_faults:
            self.report_runtime_event({"module": name, "phase": "failed", "detail": reason})
        if name == "tasks" and self._background_fault_handler is not None:
            await self._quietly(
                self._background_fault_handler(name), context="failed task revocation"
            )

    def _supervisor_fault(self, reason: str) -> None:
        self.report_runtime_event(
            {"module": "motion_delivery", "phase": "unconfirmed", "detail": reason}
        )
        self._manual_epoch_failed = self._last_epoch
        self._discard_queued_intent()
        self._invalidate_gate()
        if not self._stopping and (
            self._safety_fault_task is None or self._safety_fault_task.done()
        ):
            self._safety_fault_task = self._spawn(self._retire_failed_delivery(self._last_epoch))

    async def _retire_failed_delivery(self, epoch: int) -> None:
        async def disable() -> None:
            await self._quietly(self._motion.disable(epoch), context="manual source retirement")

        async def pause_tasks() -> None:
            if self._background_fault_handler is not None:
                await self._quietly(
                    self._background_fault_handler("motion_delivery"),
                    context="task source revocation",
                )

        await asyncio.gather(disable(), pause_tasks())

    def _safety_fault(self) -> None:
        self._supervisor_fault("safety delivery unconfirmed; vehicle owner state is unknown")

    def request_shutdown(self) -> None:
        """Synchronous and signal-safe: motion is revoked before anything else happens."""
        self._stopping = True
        self._invalidate_gate()
        if self._supervisor is not None:
            self._supervisor.revoke("shutdown requested")
        self._stop.set()

    def install_signal_handlers(self) -> None:
        loop = asyncio.get_running_loop()
        for number in (signal.SIGTERM, signal.SIGINT):
            loop.add_signal_handler(number, self.request_shutdown)
            self._signals.append(number)

    async def _disable_motion(self, supervisor: Supervisor) -> None:
        self._stopping = True
        supervisor.revoke("shutting down")
        await self._quietly(self._motion.disable(self._last_epoch))
        await self._settle_safety()

    async def _release(self) -> None:
        self._stopping = True
        self._log("shutdown")
        inflight = list(self._inflight)
        for task in inflight:
            if task is not self._safety_task:
                task.cancel()
        await self._settle_safety()
        await self._wait_inflight(inflight)
        link, self._link = self._link, None
        if link is not None:
            await self._quietly(link.close())
        if self._cleanup is not None:
            await self._quietly(self._cleanup())
        await self._quietly(self._motion.exit())
        await self._quietly(self._motion.close())
        loop = asyncio.get_running_loop()
        for number in self._signals:
            loop.remove_signal_handler(number)
        self._signals.clear()

    async def _wait_inflight(self, tasks: list[asyncio.Task[None]]) -> None:
        if not tasks:
            return
        done, pending = await asyncio.wait(tasks, timeout=self._deadlines.safety_ms / 1000)
        if done:
            await asyncio.gather(*done, return_exceptions=True)
        if pending:
            self._stragglers.update(pending)
            for task in pending:
                task.add_done_callback(self._straggler_finished)
            if self._stopping:
                self.runtime_failed = True
            self.report_runtime_event(
                {
                    "module": "retired_work",
                    "phase": "unconfirmed",
                    "detail": "inflight work did not stop before shutdown deadline",
                }
            )

    def _straggler_finished(self, task: asyncio.Task[Any]) -> None:
        self._stragglers.discard(task)
        if not task.cancelled():
            task.exception()

    async def _quietly(self, call: Awaitable[Any], *, context: str = "shutdown step") -> None:
        task = asyncio.ensure_future(call)
        try:
            done, _pending = await asyncio.wait({task}, timeout=self._deadlines.safety_ms / 1000)
        except BaseException:
            task.cancel()
            self._stragglers.add(task)
            task.add_done_callback(self._straggler_finished)
            raise
        if not done:
            task.cancel()
            self._stragglers.add(task)
            task.add_done_callback(self._straggler_finished)
            if self._stopping:
                self.runtime_failed = True
            logger.error("%s exceeded its deadline", context)
            return
        try:
            task.result()
        except Exception:
            logger.exception("%s failed", context)

    # ------------------------------------------------------------------ cloud link

    async def _beat(self) -> None:
        link = self._link
        if link is None:
            return
        try:
            await link.send_heartbeat()
        except Exception:
            if self._link is link:
                self._revoked_link = link
                await self._quietly(link.close())
            raise

    async def _session(self) -> bool:
        try:
            link = await self._connector()
        except AuthRejected:
            self.auth_rejected = True
            self.request_shutdown()
            raise
        self._revoked_link = None
        self._link, self._epoch, self._last_epoch = link, link.epoch, link.epoch
        self.metrics.inc("agent_reconnects_total")
        self._log("connection_opened")
        received_heartbeat = False
        # Snapshot collection is independent of heartbeat and control reception.
        keepalive = asyncio.ensure_future(self._keep_snapshot_fresh(link))
        video: Signaling | None = None
        try:
            while True:
                frame = await link.recv()
                if isinstance(frame, HeartbeatFrame):
                    if frame.robot_id != self.robot_id or self._revoked_link is link:
                        raise ConnectionError("invalid heartbeat")
                    received_heartbeat = True
                elif isinstance(frame, VideoEnvelope):
                    if video is None:
                        video = self._ports.video_signaling(
                            self.robot_id,
                            link.epoch,
                            send=link.send_video,
                            make_session=self._video_session,
                        )
                    video.submit(frame)
                else:
                    await self._receive_command(link, frame)
        except AuthRejected:
            self.auth_rejected = True
            self.request_shutdown()
            raise
        except (ConnectionError, OSError):
            return received_heartbeat
        finally:
            epoch = link.epoch
            if self._link is link:
                self._invalidate_gate()
                self._link, self._epoch = None, 0
            self._log("connection_closed")
            if video is not None:
                video.retire()
            keepalive.cancel()
            self._discard_queued_intent()
            inflight = list(self._inflight)
            for task in inflight:
                if task is not self._safety_task:
                    task.cancel()
            await self._quietly(self._motion.disable(epoch))
            await self._settle_safety()
            await self._wait_inflight([keepalive, *inflight])
            if video is not None:
                await self._quietly(video.close())
            await self._quietly(link.close())

    async def _keep_snapshot_fresh(self, link: Link) -> None:
        """Push on a state change, and at least every ``snapshot_seconds``.

        The relay refreshes ``last_synced_at`` only from snapshots, so an idle robot must keep
        sending them or the cloud would show it stale.
        """
        loop = asyncio.get_running_loop()
        last_key: tuple[Any, ...] | None = None
        last_sent = loop.time()
        if self._link is not link:
            return
        try:
            snapshot = await self._collect()
            if self._link is not link:
                return
            key = _snapshot_change_key(snapshot)
            await link.send_snapshot(snapshot)
            last_key, last_sent = key, loop.time()
        except Exception:
            logger.warning("initial snapshot unavailable", exc_info=True)
        while self._link is link:
            await asyncio.sleep(self._state_poll_seconds)
            if self._link is not link:
                return
            try:
                snapshot = await self._collect()
            except Exception:
                # Any failed read, ``OSError`` and a timeout included, only skips this round: the
                # link is still up, so the next successful collect must keep the cloud fresh.
                logger.warning("periodic snapshot collect failed", exc_info=True)
                continue
            if self._link is not link:
                return
            key = _snapshot_change_key(snapshot)
            if key == last_key and loop.time() - last_sent < self._snapshot_seconds:
                continue
            try:
                await link.send_snapshot(snapshot)
            except (ConnectionError, OSError):
                return  # only a dead send ends this keepalive
            except Exception:
                logger.warning("periodic snapshot send failed", exc_info=True)
                continue
            last_key, last_sent = key, loop.time()

    def _safety_key(self, envelope: Envelope) -> tuple[str, int, str]:
        identity = envelope.session_id
        if envelope.type == "drive.stop" and envelope.payload.authority_id is not None:
            identity = json.dumps(
                (identity, envelope.payload.authority_id, envelope.payload.gesture)
            )
        if envelope.type == "safety.estop":
            # Distinct user events require distinct request receipts, even when
            # queued in the same session. Only identical transport retries merge.
            identity = json.dumps((envelope.request_id, envelope.seq))
        return envelope.type, envelope.connection_epoch, identity

    async def _receive_command(self, link: Link, envelope: Envelope) -> None:
        if envelope._received_steady is None:
            envelope._received_steady = self._clock.monotonic()
        if envelope.robot_id != self.robot_id or envelope.connection_epoch != link.epoch:
            if not isinstance(envelope, ManualIntentEnvelope):
                self._spawn(self._reply(link, envelope, error=ErrorCode.CONFLICT))
            return
        if envelope.type in HALT_TYPES | {
            "control.release",
            "safety.estop.reset",
            "safety.fault.recover",
        }:
            self._invalidate_gate()
        key = self._safety_key(envelope)
        overflow = (
            envelope.type in HALT_TYPES
            and key not in self._safety_pending
            and len(self._safety_pending) >= MAX_SAFETY_PENDING
        )
        if envelope.type != "safety.estop" and not overflow:
            self._accept(link, envelope)
            return
        if overflow:
            self._safety_fault()
            await self._reply(link, envelope, error=ErrorCode.UNAVAILABLE)
            return
        try:
            if self._admit_native_estop(envelope):
                self._spawn(self._reply(link, envelope, status=self._ports.status.SUCCEEDED))
                return
            self._accept(link, envelope)
        except self._ports.error as exc:
            self._spawn(self._reply(link, envelope, error=exc.code))

    def _admit_native_estop(self, envelope: Envelope) -> bool:
        # Process-local event deduplication, never a second software estop latch.
        fingerprint = json.dumps(
            {
                "type": envelope.type,
                "session_id": envelope.session_id,
                "payload": envelope.payload.model_dump(mode="json"),
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        previous = self._estop_receipts.get(envelope.request_id)
        if previous is not None:
            if previous[0] != fingerprint:
                raise self._ports.error(ErrorCode.CONFLICT, "safety request content changed")
            return previous[1]
        self._estop_receipts[envelope.request_id] = (fingerprint, False)
        return False

    def _accept(self, link: Link, envelope: Envelope) -> None:
        if isinstance(envelope, ResponseEnvelope):
            return
        if isinstance(envelope, (LearningStartEnvelope, LearningStatusEnvelope)):
            if len(self._inflight) >= MAX_INFLIGHT:
                raise ConnectionError("agent command capacity exceeded")
            self._spawn(self._answer_learning(link, envelope))
            return
        if isinstance(
            envelope, (SettingsGetEnvelope, SettingsApplyEnvelope, SettingsResultEnvelope)
        ):
            if len(self._inflight) >= MAX_INFLIGHT:
                raise ConnectionError("agent command capacity exceeded")
            self._spawn(self._answer_settings(link, envelope))
            return
        if isinstance(envelope, WorkspaceEnvelope):
            # The relay bounds reads to one in flight. Dropping excess here reserves
            # the ordinary and safety lanes even when the relay violates that limit.
            if self._workspace_request is None or self._workspace_request.done():
                self._workspace_request = self._spawn(self._answer_workspace(link, envelope))
            return
        if isinstance(envelope, ManualIntentEnvelope):
            now = (
                envelope._received_steady
                if envelope._received_steady is not None
                else self._clock.monotonic()
            )
            self._discard_queued_intent(overwritten=True)
            self._queued_intent = (link, envelope, now, now + self._command_ttl_seconds)
            self._intent_queue.enqueue(id(envelope), time.monotonic())
            if self._intent_task is None or self._intent_task.done():
                self._intent_task = self._spawn(self._drain_intent())
            return
        if envelope.type == "drive.stop" and envelope.payload.authority_id is not None:
            queued = self._queued_intent
            if queued is not None:
                intent = queued[1]
                if (
                    intent.session_id == envelope.session_id
                    and intent.connection_epoch == envelope.connection_epoch
                    and intent.controller_generation == envelope.controller_generation
                    and intent.payload.authority_id == envelope.payload.authority_id
                    and intent.payload.gesture <= envelope.payload.gesture
                ):
                    self._discard_queued_intent()
        elif envelope.type in HALT_TYPES or envelope.type == "control.release":
            self._discard_queued_intent()
        if envelope.type in HALT_TYPES:
            key = self._safety_key(envelope)
            existing = self._safety_pending.get(key)
            if existing is not None:
                if envelope.seq > existing[1].seq:
                    self._safety_pending[key] = (link, envelope)
            elif len(self._safety_pending) >= MAX_SAFETY_PENDING:
                self._safety_fault()
                raise self._ports.error(ErrorCode.UNAVAILABLE, "agent safety capacity exceeded")
            else:
                self._safety_pending[key] = (link, envelope)
            if self._safety_task is None or self._safety_task.done():
                self._safety_task = self._spawn(self._drain_safety())
                self._safety_task.add_done_callback(self._safety_finished)
            return
        safety_active = self._safety_task is not None and not self._safety_task.done()
        workspace_active = self._workspace_request in self._inflight
        if len(self._inflight) - int(safety_active) - int(workspace_active) >= MAX_INFLIGHT:
            raise ConnectionError("agent command capacity exceeded")
        self._spawn(self._answer(link, envelope))

    def _discard_queued_intent(self, *, overwritten: bool = False) -> None:
        pending, self._queued_intent = self._queued_intent, None
        if pending is not None:
            self._intent_queue.discard(id(pending[1]), time.monotonic(), overwritten=overwritten)

    async def _drain_intent(self) -> None:
        while self._queued_intent is not None:
            link, envelope, received, deadline = self._queued_intent
            self._queued_intent = None
            self._intent_queue.start(id(envelope), time.monotonic())
            try:
                self._admit(envelope)
                if link is not self._link or self._clock.monotonic() >= deadline:
                    continue
                # Completion is local socket handoff only; status travels independently.
                await self._motion.send_intent(
                    envelope,
                    received_steady=received,
                    deadline_steady=deadline,
                )
            except self._ports.error as exc:
                logger.debug("manual stream sample discarded: %s", exc.code.value)
            except (OSError, ConnectionError, self._ports.ipc_error):
                logger.exception("manual stream transport failed")
                self._invalidate_gate()
                self._failed_manual_authority = envelope.payload.authority_id
                self._discard_queued_intent()
                self.report_runtime_event(
                    {
                        "module": "manual",
                        "phase": "unconfirmed",
                        "detail": "manual IPC failed",
                    }
                )
                # Retire exactly this authority through the existing native owner.
                release = SimpleEnvelope(
                    **envelope.model_dump(exclude={"type", "payload"}),
                    type="control.release",
                    payload=EmptyPayload(authority_id=envelope.payload.authority_id),
                )
                await self._quietly(
                    self._motion.manual(release), context="manual authority release"
                )
                return
            finally:
                self._intent_queue.finish(id(envelope), time.monotonic())

    def _safety_finished(self, task: asyncio.Task[None]) -> None:
        if task.cancelled():
            self._safety_fault()
        elif (error := task.exception()) is not None:
            logger.error("safety delivery worker failed", exc_info=error)
            self._safety_fault()

    async def _drain_safety(self) -> None:
        # Revoking motion or faulting the transport does not revoke accepted halt
        # events. Shutdown's _settle_safety bounds this drain before IPC is closed.
        while self._safety_pending:
            key = next(
                (key for key in self._safety_pending if key[0] == "safety.estop"),
                next(iter(self._safety_pending)),
            )
            kind = key[0]
            link, envelope = self._safety_pending.pop(key)
            self._estop_inflight = kind == "safety.estop"
            try:
                result = await self.dispatch(envelope)
            except asyncio.CancelledError:
                logger.error(
                    "safety delivery interrupted; request_id=%s type=%s pending=%d; "
                    "vehicle owner state is unconfirmed",
                    envelope.request_id,
                    kind,
                    len(self._safety_pending),
                )
                raise
            except self._ports.error as exc:
                logger.error(
                    "safety delivery not confirmed; request_id=%s type=%s error=%s",
                    envelope.request_id,
                    kind,
                    exc.code.value,
                )
                if len(self._inflight) < MAX_INFLIGHT + 1:
                    self._spawn(self._reply(link, envelope, error=exc.code))
                if (
                    exc.code is not ErrorCode.CONFLICT
                    and link.epoch == self._last_epoch
                    and kind == "safety.estop"
                ):
                    self._safety_fault()
                continue
            finally:
                self._estop_inflight = False
            if len(self._inflight) < MAX_INFLIGHT + 1:
                self._spawn(self._reply(link, envelope, result=result))

    async def _settle_safety(self) -> bool:
        task = self._safety_task
        if task is None:
            return True
        done, _pending = await asyncio.wait({task}, timeout=self._deadlines.safety_ms / 1000)
        failed = task not in done or task.cancelled()
        if not failed:
            failed = task.exception() is not None or bool(self._safety_pending)
        if failed:
            self._safety_fault()
            logger.error(
                "safety drain did not complete; pending=%d; "
                "vehicle owner state is unconfirmed; restart is disabled",
                len(self._safety_pending),
            )
            if not task.done():
                task.cancel()
                done, _ = await asyncio.wait({task}, timeout=self._deadlines.safety_ms / 1000)
                if not done:
                    self._stragglers.add(task)
                    task.add_done_callback(self._straggler_finished)
            return False
        return True

    def _report_queues(self) -> None:
        self._ports.report_queues(self, len(self._inflight), len(self._stragglers))

    def _spawn(self, work: Awaitable[None]) -> asyncio.Task[None]:
        task = asyncio.ensure_future(work)
        self._inflight.add(task)
        self._report_queues()
        task.add_done_callback(self._inflight.discard)
        task.add_done_callback(self._work_finished)
        task.add_done_callback(lambda _: self._report_queues())
        return task

    def _work_finished(self, task: asyncio.Task[None]) -> None:
        if not task.cancelled() and (error := task.exception()) is not None:
            logger.error("agent request failed", exc_info=error)

    async def _answer(self, link: Link, envelope: Envelope) -> None:
        try:
            result = await self.dispatch(envelope)
        except self._ports.not_committed as exc:
            await self._reply(link, envelope, error=exc.code, status=exc.status)
            return
        except self._ports.error as exc:
            await self._reply(link, envelope, error=exc.code)
            return
        if (
            result.status is not self._ports.status.SUCCEEDED
            and envelope.type != "operation.query"
            and not isinstance(result, self._ports.task_result)
        ):
            # Only a query may report a status that is not committed, and it says so honestly.
            await self._reply(link, envelope, error=ErrorCode.CONFLICT, status=result.status)
            return
        await self._reply(link, envelope, result=result)
        if isinstance(envelope, OperationEnvelope) and envelope.type != "operation.query":
            try:
                await self.publish_snapshot(self.robot_id)
            except Exception:
                logger.warning("snapshot after edit failed", exc_info=True)

    async def _answer_learning(self, link: Link, envelope) -> None:
        try:
            self._admit(envelope)
            received = envelope._received_steady
            assert received is not None
            deadline = received + max(0.0, envelope.expires_at - self._wall_clock())
            data = await self._motion.learning(
                envelope, received_steady=received, deadline_steady=deadline
            )
        except self._ports.error as exc:
            await self._reply(link, envelope, error=exc.code)
        except (OSError, ValueError, RuntimeError):
            await self._reply(link, envelope, error=ErrorCode.UNAVAILABLE)
        else:
            await self._reply(link, envelope, data=data)

    async def _answer_settings(self, link: Link, envelope: SettingsEnvelope) -> None:
        try:
            self._admit(envelope)
            if self._settings is None:
                raise self._ports.error(ErrorCode.UNAVAILABLE, "settings are not configured")
            data = await settings_command(self._settings, envelope)
        except self._ports.error as exc:
            await self._reply(link, envelope, error=exc.code)
        except (ValueError, RuntimeError):
            await self._reply(link, envelope, error=ErrorCode.UNAVAILABLE)
        else:
            await self._reply(link, envelope, data=data)

    async def _answer_workspace(self, link: Link, envelope: WorkspaceEnvelope) -> None:
        try:
            self._admit(envelope)
            if self._workspace is None:
                raise self._ports.error(ErrorCode.UNAVAILABLE, "workspace reads unavailable")
            data = await asyncio.wait_for(self._workspace.read(envelope), self._deadlines.edit)
            self._admit(envelope)
        except asyncio.TimeoutError:
            await self._reply(link, envelope, error=ErrorCode.TIMEOUT)
        except self._ports.error as exc:
            await self._reply(link, envelope, error=exc.code)
        except Exception:
            logger.exception("workspace read failed")
            await self._reply(link, envelope, error=ErrorCode.UNAVAILABLE)
        else:
            await self._reply(link, envelope, data=data)

    async def _reject(self, link: Link, envelope: Envelope, error: ErrorCode) -> None:
        await self._reply(link, envelope, error=error)

    async def _reply(
        self,
        link: Link,
        envelope: Envelope,
        *,
        result: Any | None = None,
        error: ErrorCode | None = None,
        status: Any | None = None,
        data: dict[str, Any] | None = None,
    ) -> None:
        if (
            link is not self._link
            or envelope.connection_epoch != self._epoch
            or asyncio.current_task() in self._stragglers
        ):
            return
        request_type: Any = envelope.type
        shown = result.status if result is not None else status
        operation_id = (
            envelope.payload.operation_id if isinstance(envelope, OperationEnvelope) else None
        )
        if isinstance(result, self._ports.task_result):
            data = result.task_state
        elif isinstance(result, self._ports.manual_result_type):
            data = result.authority
        response = ResponseEnvelope(
            v=1,
            robot_id=envelope.robot_id,
            session_id=envelope.session_id,
            # The relay drops replies that do not carry the epoch of the socket they leave on.
            connection_epoch=link.epoch,
            request_id=envelope.request_id,
            seq=envelope.seq,
            expires_at=envelope.expires_at,
            type="response",
            controller_generation=getattr(envelope, "controller_generation", None),
            payload=ResponsePayload(
                request_type=request_type,
                ok=error is None,
                error=error,
                operation_id=operation_id,
                data=data,
                status=shown.value if shown is not None else None,
            ),
        )
        logger.info(
            "agent_command_result",
            extra={
                "event": "agent_command_result",
                "request_id": envelope.request_id,
                "request_type": envelope.type,
                "task_id": getattr(envelope.payload, "task_id", None),
                "result": error.value if error else "response_ok",
            },
        )
        try:
            await link.send_response(response)
        except Exception:
            # The result stays in the journal; the controller asks again after reconnecting.
            logger.warning("response could not be sent", exc_info=True)

    # ------------------------------------------------------------------ commands

    async def dispatch(self, envelope: Envelope) -> ResultT:
        if envelope.type in HALT_TYPES | {
            "control.release",
            "safety.estop.reset",
            "safety.fault.recover",
        }:
            self._invalidate_gate()
        # Native manual admission lives at TaskAwareMotion's physical handoff;
        # task mutations serialize inside LifecycleStore, never around port I/O.
        return await self._dispatch(envelope)

    async def _dispatch(self, envelope: Envelope) -> ResultT:
        if isinstance(
            envelope,
            (
                ResponseEnvelope,
                SnapshotEnvelope,
                WorkspaceEnvelope,
                SettingsGetEnvelope,
                LearningStartEnvelope,
                LearningStatusEnvelope,
                SettingsApplyEnvelope,
                SettingsResultEnvelope,
            ),
        ):
            raise self._ports.error(ErrorCode.VALIDATION, "command is not for this robot")
        if envelope.robot_id != self.robot_id:
            raise self._ports.error(ErrorCode.VALIDATION, "command is not for this robot")
        if isinstance(envelope, ManualIntentEnvelope):
            raise self._ports.error(ErrorCode.VALIDATION, "intent uses the send-only stream")
        if isinstance(envelope, RecoveryEnvelope) or envelope.type in {
            "control.acquire",
            "control.release",
        }:
            received = envelope._received_steady
            if received is None:
                received = self._clock.monotonic()
            return await self._manual_control(
                envelope, received, received + self._command_ttl_seconds
            )
        if isinstance(envelope, TaskExecutionEnvelope):
            self._admit(envelope)
            if not isinstance(self._navigation, self._ports.lifecycle_navigation):
                raise self._ports.error(ErrorCode.UNAVAILABLE, "task lifecycle is not wired")
            return await self._navigation.execution_command(envelope)
        if isinstance(envelope, OperationEnvelope):
            return await self._dispatcher.run(envelope)
        try:
            result = await self._control(envelope)
        except self._ports.error as exc:
            self.metrics.inc(
                "agent_operations_total", operation=envelope.type, result=exc.code.value
            )
            raise
        self.metrics.inc(
            "agent_operations_total", operation=envelope.type, result=result.status.value
        )
        return result

    async def _manual_control(
        self,
        envelope: Envelope,
        received: float,
        deadline: float,
    ) -> ResultT:
        if envelope.robot_id != self.robot_id:
            raise self._ports.error(ErrorCode.VALIDATION, "manual command is for another robot")
        self._admit(envelope)
        if self._safety_blocked and envelope.type not in {
            "control.release",
            "safety.estop.reset",
            "safety.fault.recover",
        }:
            raise self._ports.error(ErrorCode.SAFETY_REJECTED, "local safety recovery is pending")
        if isinstance(envelope, RecoveryEnvelope):
            resolver = getattr(self._state, "recovery_revision", None)
            if resolver is None:
                raise self._ports.error(
                    ErrorCode.UNAVAILABLE, "motion recovery projection is not wired"
                )
            kind = "estop_reset" if envelope.type == "safety.estop.reset" else "fault_recover"
            revision = await resolver(kind, envelope.payload.revision)
            envelope = envelope.model_copy(
                update={
                    "payload": envelope.payload.model_copy(update={"revision": revision}),
                }
            )
        try:
            reply = await asyncio.wait_for(
                self._motion.manual(envelope, received_steady=received, deadline_steady=deadline),
                timeout=self._deadlines.edit
                if envelope.type == "control.acquire"
                else self._deadlines.safety_ms / 1000,
            )
        except asyncio.TimeoutError as exc:
            raise self._ports.error(
                ErrorCode.OPERATION_UNKNOWN, "manual acknowledgement is unknown"
            ) from exc
        if envelope.type == "control.acquire":
            if reply["authority_id"] == self._failed_manual_authority:
                raise self._ports.error(
                    ErrorCode.UNAVAILABLE, "manual authority retirement is unconfirmed"
                )
            return self._ports.manual_result(
                envelope.request_id,
                self._ports.status.SUCCEEDED,
                {"authority_id": reply["authority_id"], "state": reply["state"]},
            )
        return self._ports.result(envelope.request_id, self._ports.status.SUCCEEDED)

    async def _control(self, envelope: SimpleEnvelope) -> ResultT:
        request_id = envelope.request_id
        if envelope.type == "safety.estop":
            if self._admit_native_estop(envelope):
                return self._ports.result(request_id, self._ports.status.SUCCEEDED)
        if isinstance(envelope, SimpleEnvelope) and envelope.type in HALT_TYPES:
            # A stop is honored whatever the epoch, expiry, or link state: it only removes motion.
            supervisor = self._supervisor
            try:
                if envelope.type == "safety.estop":
                    received = envelope._received_steady
                    if received is None:
                        received = envelope._received_steady = self._clock.monotonic()
                    await asyncio.wait_for(
                        self._motion.estop_received(envelope, received_steady=received),
                        self._deadlines.safety_ms / 1000,
                    )
                    fingerprint, _ = self._estop_receipts[request_id]
                    self._estop_receipts[request_id] = (fingerprint, True)
                else:
                    await asyncio.wait_for(
                        self._motion.stop(envelope), self._deadlines.safety_ms / 1000
                    )
            except self._ports.motion_rejected as exc:
                raise self._ports.error(
                    exc.code or ErrorCode.SAFETY_REJECTED, "stop was rejected"
                ) from exc
            except Exception as exc:
                if supervisor is not None:
                    self._supervisor_fault("stop was not acknowledged")
                raise self._ports.error(ErrorCode.TIMEOUT, "stop was not acknowledged") from exc
            return self._ports.result(request_id, self._ports.status.SUCCEEDED)
        raise self._ports.error(ErrorCode.VALIDATION, "unsupported control request")

    def _admit(self, envelope: Envelope) -> None:
        """A first-time command needs a live link, the current epoch, and an unexpired deadline."""
        if asyncio.current_task() in self._stragglers:
            raise self._ports.error(ErrorCode.CONFLICT, "retired work cannot issue commands")
        starts_task = envelope.type in {"navigation.goal", "navigation.return_home"} or (
            isinstance(envelope, TaskExecutionEnvelope) and envelope.payload.action == "resume"
        )
        if starts_task and "tasks" in self.failed_modules:
            raise self._ports.error(ErrorCode.UNAVAILABLE, "task runtime is unavailable")
        if envelope.type in {"control.acquire", "drive.intent"}:
            if self._manual_epoch_failed == envelope.connection_epoch:
                raise self._ports.error(ErrorCode.UNAVAILABLE, "manual delivery is unavailable")
            if (
                isinstance(envelope, ManualIntentEnvelope)
                and envelope.payload.authority_id == self._failed_manual_authority
            ):
                raise self._ports.error(ErrorCode.UNAVAILABLE, "manual authority was retired")
        if self._link is None or self._revoked_link is self._link:
            raise self._ports.error(ErrorCode.OFFLINE, "robot is not connected to the server")
        if envelope.connection_epoch != self._epoch:
            raise self._ports.error(ErrorCode.CONFLICT, "command belongs to an earlier connection")
        if (
            envelope.type not in {"control.acquire", "control.release", "drive.intent"}
            and envelope.expires_at < self._wall_clock()
        ):
            raise self._ports.error(ErrorCode.TIMEOUT, "command expired")

    # ------------------------------------------------------------------ snapshots

    async def publish_snapshot(self, robot_id: str) -> RobotSnapshot:
        if robot_id != self.robot_id:
            raise self._ports.error(ErrorCode.VALIDATION, "snapshot is for another robot")
        link, epoch = self._link, self._epoch
        snapshot = await self._collect()
        if link is not None and self._link is link and self._epoch == epoch:
            await link.send_snapshot(snapshot)
        return snapshot

    async def _collect(self) -> RobotSnapshot:
        live, home_revision = await asyncio.wait_for(
            asyncio.gather(self._state.collect(), self._home.home_revision()),
            timeout=self._deadlines.edit,
        )
        deleted = live.deleted_task_ids
        if deleted is None:
            deleted = await self._journal.deleted_task_ids()
        home = live.home
        ui = dict(live.vehicle_ui or {})
        if "runtime.v1" in ui.get("capabilities", []) and await self._motion.learning_available():
            ui["capabilities"] = [*ui.get("capabilities", []), "learning.v1"]
        if self.module_faults:
            ui["module_faults"] = dict(self.module_faults)
            capabilities = list(ui.get("capabilities", []))
            if "tasks" in self.failed_modules:
                capabilities = [c for c in capabilities if c != "task.execution.v1"]
            if self._manual_epoch_failed == self._epoch:
                capabilities = [c for c in capabilities if c != "motion.intent.v1"]
            if "logs" in self.failed_modules:
                capabilities = [c for c in capabilities if c != "logs.v1"]
            ui["capabilities"] = capabilities
        snapshot = RobotSnapshot(
            robot_id=self.robot_id,
            connected=self.connected,
            localized=live.localized,
            map_ref=live.map_ref,
            home=None if home is None else HomeSummary(home.id, home.map_ref, home.status),
            task_revision=live.task_revision,
            runnable_task_ids=tuple(i for i in live.runnable_task_ids if i not in deleted),
            execution_status=live.execution_status,
            collected_at=self._wall_clock(),
            performance=self._performance(),
            tasks=tuple(item for item in live.tasks if item.task_id not in deleted),
            home_revision=home_revision,
            map_generation=live.map_generation,
            map_grid_id=live.map_grid_id,
            route_progress=live.route_progress,
            navigation_ready=live.navigation_ready,
            robot_pose=live.robot_pose,
            navigation_block_reason=live.navigation_block_reason,
            scan_points=live.scan_points,
            vehicle_ui=ui if self.module_faults else live.vehicle_ui,
            manual_controller_generation=live.manual_controller_generation,
            manual_agent_epoch=live.manual_agent_epoch,
        )
        return snapshot

    def _log(self, event: str) -> None:
        logger.info(event, extra={"event": event, "robot_id": self.robot_id})
