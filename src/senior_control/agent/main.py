"""The on-robot agent: owns the robot database and the cloud link, never an rclpy executor."""

import argparse
import asyncio
import logging
import os
import signal
import sys
import time
from collections import deque
from collections.abc import Awaitable, Callable, Mapping
from pathlib import Path
from typing import Any

from senior_control.agent.connection import (
    HeartbeatFrame,
    HeartbeatLedger,
    Link,
    MotionPort,
    MotionRejected,
)
from senior_control.agent.dispatcher import (
    Deadlines,
    Dispatcher,
    HomePort,
    MapPort,
    NavigationAction,
    OperationNotCommitted,
    TaskPort,
)
from senior_control.agent.journal import AgentJournal
from senior_control.agent.snapshots import HomeSummary, RobotSnapshot, StatePort
from senior_control.agent.workspace import WorkspaceReader
from senior_control.clock import Clock, SystemClock
from senior_control.errors import DomainError, ErrorCode
from senior_control.models import OperationResult, OperationStatus
from senior_control.observability import Metrics
from senior_control.protocol import (
    DriveExecutionFeedback,
    Envelope,
    OperationEnvelope,
    ResponseEnvelope,
    ResponsePayload,
    SimpleEnvelope,
    SnapshotEnvelope,
    VelocityEnvelope,
    VideoEnvelope,
    WorkspaceEnvelope,
)
from senior_control.runtime.ipc import IpcError
from senior_control.runtime.supervisor import AuthRejected, Supervisor
from senior_control.vision.signaling import VideoSignaling
from senior_control.vision.stream import VideoSession

logger = logging.getLogger("senior_control.agent")

HALT_TYPES = frozenset({"drive.stop", "safety.estop"})
MAX_INFLIGHT = 64
MAX_SAFETY_PENDING = 64
# Browser commands may be admitted for this long. The vehicle does not execute for that long:
# a velocity older than command_ttl_seconds is refused on the local clock.
ADMISSION_SECONDS = 5.0
MAX_COMMAND_CLOCK_ERROR = 0.05


async def _never_publish(vx: float, wz: float) -> None:
    raise AssertionError("the agent must not publish /cmd_vel")


class Agent:
    def __init__(
        self,
        robot_id: str,
        *,
        connector: Callable[[], Awaitable[Link]],
        journal: AgentJournal,
        tasks: TaskPort,
        home: HomePort,
        maps: MapPort,
        state: StatePort,
        motion: MotionPort,
        workspace: WorkspaceReader | None = None,
        navigation: NavigationAction | None = None,
        video_session: Callable[[], VideoSession | None] | None = None,
        deadlines: Deadlines | None = None,
        map_root: Path = Path("map-transfer"),
        background: Mapping[str, Callable[[], Awaitable[None]]] | None = None,
        cleanup: Callable[[], Awaitable[None]] | None = None,
        on_eligibility_lost: Callable[[int], Awaitable[None]] | None = None,
        heartbeat_seconds: float = 1.0,
        retry_delay: float = 1.0,
        heartbeat_window: float = 2.0,
        command_ttl_seconds: float = 0.3,
        source_clock_verified: bool = False,
        snapshot_seconds: float = 2.0,
        state_poll_seconds: float = 0.25,
        metrics: Metrics | None = None,
        clock: Clock | None = None,
        wall_clock: Callable[[], float] = time.time,
    ) -> None:
        if not robot_id.strip():
            raise ValueError("robot id is required")
        self.robot_id = robot_id
        self.metrics = metrics if metrics is not None else Metrics()
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
        self._on_eligibility_lost = on_eligibility_lost
        self._loss_tasks: set[asyncio.Task[None]] = set()
        self._stragglers: set[asyncio.Task[Any]] = set()
        self._continuity_generation = 0
        self._was_eligible = False
        self._heartbeat_seconds = heartbeat_seconds
        self._retry_delay = retry_delay
        if command_ttl_seconds <= 0:
            raise ValueError("command ttl must be positive")
        self._command_ttl_seconds = command_ttl_seconds
        self._source_clock_verified = source_clock_verified
        self._wall_clock = wall_clock
        self._clock = clock if clock is not None else SystemClock()
        self._ledger = HeartbeatLedger(self._clock, heartbeat_window)
        self._home = home
        self._dispatcher = Dispatcher(
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
        )
        self._link: Link | None = None
        self._revoked_link: Link | None = None
        self._clock_revoked = False
        self._epoch = 0
        self._last_epoch = 0
        self._heartbeat_generation = 0
        self._latest_velocity: VelocityEnvelope | None = None
        self._supervisor: Supervisor | None = None
        self._stopping = False
        self.auth_rejected = False
        self.runtime_failed = False
        self.recovery_required = False
        self._stop = asyncio.Event()
        self._inflight: set[asyncio.Task[None]] = set()
        self._safety_task: asyncio.Task[None] | None = None
        self._velocity_task: asyncio.Task[None] | None = None
        self._queued_velocity: tuple[Link, VelocityEnvelope, int] | None = None
        self._replaced_velocity: deque[DriveExecutionFeedback] = deque(maxlen=16)
        self._safety_pending: dict[tuple[str, int, str], tuple[Link, Envelope]] = {}
        self._safety_cleanups: set[asyncio.Task[None]] = set()
        self._safety_lock = asyncio.Lock()
        self._safety_admitting = 0
        self._safety_receipts = 0
        self._safety_resolving = False
        self._estop_inflight = False
        self._signals: list[signal.Signals] = []
        if snapshot_seconds <= 0 or state_poll_seconds <= 0:
            raise ValueError("snapshot intervals must be positive")
        self._snapshot_seconds = snapshot_seconds
        self._state_poll_seconds = state_poll_seconds

    # ------------------------------------------------------------------ state

    @property
    def connected(self) -> bool:
        return self._link is not None

    @property
    def motion_allowed(self) -> bool:
        supervisor = self._supervisor
        return supervisor is not None and supervisor.motion_allowed and not self._stopping

    @property
    def eligible_for_motion(self) -> bool:
        """Needs this robot's own heartbeat. Another robot's heartbeat counts for nothing."""
        self._expire_eligibility()
        return (
            self._link is not None
            and self._link.command_clock_valid()
            and self.motion_allowed
            and not self._safety_blocked
            and self._ledger.eligible(self.robot_id)
        )

    @property
    def _safety_blocked(self) -> bool:
        return (
            self.recovery_required
            or self._safety_admitting > 0
            or self._safety_resolving
            or self._estop_inflight
            or any(key[0] == "safety.estop" for key in self._safety_pending)
        )

    @property
    def continuity_generation(self) -> int:
        self._expire_eligibility()
        return self._continuity_generation

    def _expire_eligibility(self) -> None:
        if self._was_eligible and not self._ledger.eligible(self.robot_id):
            self._lose_eligibility()

    def _lose_eligibility(self) -> None:
        if not self._was_eligible:
            return
        self._was_eligible = False
        lost_generation = self._continuity_generation
        self._continuity_generation += 1
        self._heartbeat_generation += 1
        self._revoked_link = self._link
        self._ledger.forget(self.robot_id)
        if self._on_eligibility_lost is not None:
            task = asyncio.ensure_future(self._on_eligibility_lost(lost_generation))
            self._loss_tasks.add(task)
            task.add_done_callback(self._loss_finished)

    def _loss_finished(self, task: asyncio.Task[None]) -> None:
        self._loss_tasks.discard(task)
        if not task.cancelled() and task.exception() is not None:
            self._supervisor_fault("failed to persist connection loss")

    def note_heartbeat(self, robot_id: str) -> None:
        self._ledger.note(robot_id)

    # ------------------------------------------------------------------ lifecycle

    async def run(self) -> None:
        try:
            unresolved = await self._safety_storage(self._journal.safety.pending())
        except Exception:
            logger.exception("could not read persistent safety state")
            self._safety_fault()
            await self._release()
            return
        if unresolved:
            logger.warning("unresolved safety intent: status-only recovery channel")
            self.recovery_required = True
        supervisor = Supervisor(
            send_heartbeat=self._beat,
            call_ros=self._motion.call,
            drive_sink=self._drive,
            publish_cmd_vel=_never_publish,
            connect=self._session,
            background=self._background,
            heartbeat_seconds=self._heartbeat_seconds,
            retry_delay=self._retry_delay,
            on_fault=self._supervisor_fault,
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

    def _supervisor_fault(self, reason: str) -> None:
        self.runtime_failed = True
        self.request_shutdown()

    def _safety_fault(self) -> None:
        self.recovery_required = True
        self.auth_rejected = True
        self.runtime_failed = True
        self.request_shutdown()

    def request_shutdown(self) -> None:
        """Synchronous and signal-safe: motion is revoked before anything else happens."""
        self._stopping = True
        self._lose_eligibility()
        self._heartbeat_generation += 1
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
        self._log("shutdown")
        inflight = list(self._inflight)
        for task in inflight:
            if task is not self._safety_task:
                task.cancel()
        await self._settle_safety()
        await self._wait_inflight(list(self._safety_cleanups))
        await self._wait_inflight(inflight)
        await self._wait_inflight(list(self._loss_tasks))
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
        done, pending = await asyncio.wait(tasks, timeout=self._deadlines.safety)
        if done:
            await asyncio.gather(*done, return_exceptions=True)
        if pending:
            self._stragglers.update(pending)
            for task in pending:
                task.add_done_callback(self._straggler_finished)
            self.runtime_failed = True
            self.request_shutdown()
            logger.error("inflight work did not stop before shutdown deadline")

    def _straggler_finished(self, task: asyncio.Task[Any]) -> None:
        self._stragglers.discard(task)
        if not task.cancelled():
            task.exception()

    async def _quietly(self, call: Awaitable[Any]) -> None:
        task = asyncio.ensure_future(call)
        try:
            done, _pending = await asyncio.wait({task}, timeout=self._deadlines.safety)
        except BaseException:
            task.cancel()
            self._stragglers.add(task)
            task.add_done_callback(self._straggler_finished)
            raise
        if not done:
            task.cancel()
            self._stragglers.add(task)
            task.add_done_callback(self._straggler_finished)
            self.runtime_failed = True
            logger.error("shutdown step exceeded its deadline")
            return
        try:
            task.result()
        except Exception:
            logger.exception("shutdown step failed")

    # ------------------------------------------------------------------ cloud link

    async def _beat(self) -> None:
        link = self._link
        if link is None or not self.motion_allowed:
            return
        self._expire_eligibility()
        if self._revoked_link is link and not self._clock_revoked:
            await self._quietly(self._motion.disable(link.epoch))
            await self._quietly(link.close())
            return
        try:
            await link.send_heartbeat()
        except Exception:
            if self._link is not link:
                raise
            self._heartbeat_generation += 1
            self._revoked_link = link
            self._lose_eligibility()
            self._ledger.forget(self.robot_id)
            epoch = link.epoch
            if epoch:
                await self._quietly(self._motion.disable(epoch))
            await self._quietly(link.close())
            raise

    async def _session(self) -> bool:
        try:
            link = await self._connector()
        except AuthRejected:
            self.auth_rejected = True
            self.request_shutdown()
            raise
        self._heartbeat_generation += 1
        self._queued_velocity = None
        self._replaced_velocity.clear()
        self._revoked_link = None
        self._clock_revoked = False
        self._link, self._epoch, self._last_epoch = link, link.epoch, link.epoch
        self.metrics.inc("agent_reconnects_total")
        self._log("connection_opened")
        received_heartbeat = False
        # Snapshot collection is independent of heartbeat and control reception.
        keepalive = asyncio.ensure_future(self._keep_snapshot_fresh(link))
        feedback = asyncio.ensure_future(self._forward_drive_feedback(link))
        video: VideoSignaling | None = None
        try:
            while True:
                frame = await link.recv()
                if isinstance(frame, HeartbeatFrame):
                    if frame.robot_id != self.robot_id or frame.deadline <= self._clock.monotonic():
                        raise ConnectionError("invalid heartbeat")
                    received_heartbeat = True
                    if not link.command_clock_valid():
                        if self._was_eligible:
                            self._clock_revoked = True
                            self._lose_eligibility()
                            await self._quietly(self._motion.disable(link.epoch))
                        continue
                    if self._clock_revoked:
                        # The old epoch was disabled. Recover motion only on a new epoch.
                        raise ConnectionError("command clock recovered")
                    if self._safety_blocked:
                        continue
                    was_eligible = self._was_eligible
                    self._expire_eligibility()
                    if was_eligible and not self._was_eligible:
                        raise ConnectionError("heartbeat continuity expired")
                    if not self.motion_allowed or self._revoked_link is link:
                        raise ConnectionError("motion is revoked")
                    generation = self._heartbeat_generation
                    await asyncio.wait_for(
                        self._motion.server_heartbeat(link.epoch, frame.deadline),
                        timeout=self._deadlines.safety,
                    )
                    was_eligible = self._was_eligible
                    self._expire_eligibility()
                    if (
                        (was_eligible and not self._was_eligible)
                        or frame.deadline <= self._clock.monotonic()
                        or self._link is not link
                        or self._revoked_link is link
                        or generation != self._heartbeat_generation
                        or not self.motion_allowed
                    ):
                        raise ConnectionError("heartbeat expired before ROS acknowledgement")
                    self._ledger.note(frame.robot_id, deadline=frame.deadline)
                    self._was_eligible = self._ledger.eligible(frame.robot_id)
                elif isinstance(frame, VideoEnvelope):
                    if video is None:
                        video = VideoSignaling(
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
                self._lose_eligibility()
                self._link, self._epoch = None, 0
                self._ledger.forget(self.robot_id)
            self._log("connection_closed")
            if video is not None:
                video.retire()
            keepalive.cancel()
            feedback.cancel()
            inflight = list(self._inflight)
            for task in inflight:
                if task is not self._safety_task:
                    task.cancel()
            await self._quietly(self._motion.disable(epoch))
            await self._settle_safety()
            await self._wait_inflight([keepalive, feedback, *inflight])
            if video is not None:
                await self._quietly(video.close())
            await self._quietly(link.close())

    async def _forward_drive_feedback(self, link: Link) -> None:
        take = getattr(self._motion, "take_drive_feedback", None)
        send = getattr(link, "send_drive_feedback", None)
        if take is None or send is None:
            return
        while self._link is link:
            await asyncio.sleep(0.05)
            try:
                results = list(self._replaced_velocity)
                self._replaced_velocity.clear()
                for result in (*results, *(await take())):
                    if self._link is link and result.robot_id == self.robot_id:
                        await send(result)
            except (ConnectionError, OSError):
                return
            except (DomainError, IpcError, ValueError):
                # //Compatible: feedback is optional when the local adapter is older.
                await asyncio.sleep(0.5)

    async def _keep_snapshot_fresh(self, link: Link) -> None:
        """Push on a state change, and at least every ``snapshot_seconds``.

        The relay refreshes ``last_synced_at`` only from snapshots, so an idle robot must keep
        sending them or the cloud would show it stale.
        """
        try:
            await self.publish_snapshot(self.robot_id)
        except Exception:
            logger.warning("initial snapshot unavailable", exc_info=True)
        loop = asyncio.get_running_loop()
        last_key: tuple[Any, ...] | None = None
        last_sent = loop.time()
        while True:
            await asyncio.sleep(self._state_poll_seconds)
            try:
                snapshot = await self._collect()
            except Exception:
                # Any failed read, ``OSError`` and a timeout included, only skips this round: the
                # link is still up, so the next successful collect must keep the cloud fresh.
                logger.warning("periodic snapshot collect failed", exc_info=True)
                continue
            key = (
                snapshot.localized,
                snapshot.map_ref,
                snapshot.home,
                snapshot.task_revision,
                snapshot.execution_status,
                snapshot.runnable_task_ids,
            )
            if last_key is None:
                last_key = key
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

    async def _safety_storage(self, work: Awaitable[Any], deadline: float | None = None) -> Any:
        """Bound storage waits without assuming cancellation can stop SQLite immediately."""
        task = asyncio.ensure_future(work)
        loop = asyncio.get_running_loop()
        timeout = self._deadlines.safety if deadline is None else max(0.0, deadline - loop.time())
        try:
            done, _ = await asyncio.wait({task}, timeout=timeout)
        except BaseException:
            task.cancel()
            self._stragglers.add(task)
            task.add_done_callback(self._straggler_finished)
            raise
        if not done:
            task.cancel()
            self._stragglers.add(task)
            task.add_done_callback(self._straggler_finished)
            raise TimeoutError("safety storage deadline exceeded")
        if deadline is not None and loop.time() >= deadline:
            if not task.cancelled():
                task.exception()
            raise TimeoutError("safety storage deadline exceeded")
        return task.result()

    async def _acquire_safety_lock(self, deadline: float) -> None:
        """Wait for the lock within the receipt budget, including cancellation races."""
        loop = asyncio.get_running_loop()
        task = asyncio.create_task(self._safety_lock.acquire())
        try:
            done, _ = await asyncio.wait({task}, timeout=max(0.0, deadline - loop.time()))
        except BaseException:
            if task.done() and not task.cancelled() and task.exception() is None:
                self._safety_lock.release()
            else:
                task.cancel()
            raise
        if not done:
            task.cancel()
            raise TimeoutError("safety lock deadline exceeded")
        task.result()
        if loop.time() >= deadline:
            self._safety_lock.release()
            raise TimeoutError("safety lock deadline exceeded")

    async def _receive_command(self, link: Link, envelope: Envelope) -> None:
        if envelope.robot_id != self.robot_id or envelope.connection_epoch != link.epoch:
            self._spawn(self._reply(link, envelope, error=ErrorCode.CONFLICT))
            return
        key = (envelope.type, envelope.connection_epoch, envelope.session_id)
        overflow = (
            envelope.type in HALT_TYPES
            and key not in self._safety_pending
            and len(self._safety_pending) >= MAX_SAFETY_PENDING
        )
        if envelope.type != "safety.estop" and not overflow:
            self._accept(link, envelope)
            return
        # From receipt until durable admission, no new motion or ROS heartbeat is authorized.
        deadline = asyncio.get_running_loop().time() + self._deadlines.safety
        self._safety_admitting += 1
        self._safety_receipts += 1
        self._latest_velocity = None
        try:
            await self._acquire_safety_lock(deadline)
            try:
                await self._safety_storage(self._journal.safety.mark(), deadline)
                if overflow:
                    self.recovery_required = True
                self._accept(link, envelope)
            finally:
                self._safety_lock.release()
        except BaseException as exc:
            self._safety_fault()
            if isinstance(exc, asyncio.CancelledError):
                raise
            raise ConnectionError("safety admission failed") from exc
        finally:
            self._safety_admitting -= 1

    async def _clear_replayed_safety(self) -> None:
        interrupted = asyncio.Event()
        task = asyncio.create_task(self._clear_replayed_safety_locked(interrupted))
        self._safety_cleanups.add(task)
        task.add_done_callback(self._safety_cleanups.discard)
        self._stragglers.add(task)
        task.add_done_callback(self._straggler_finished)
        try:
            await asyncio.shield(task)
        except asyncio.CancelledError:
            self._safety_fault()
            interrupted.set()
            raise

    async def _clear_replayed_safety_locked(self, interrupted: asyncio.Event) -> None:
        async with self._safety_lock:
            if (
                self.recovery_required
                or self._safety_admitting
                or self._estop_inflight
                or any(key[0] == "safety.estop" for key in self._safety_pending)
            ):
                return
            receipts = self._safety_receipts
            clear = asyncio.create_task(self._journal.safety.clear_rejected())
            interruption: BaseException | None = None
            cancel = asyncio.create_task(interrupted.wait())
            try:
                done, _ = await asyncio.wait(
                    {clear, cancel},
                    timeout=self._deadlines.safety,
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if cancel in done:
                    raise asyncio.CancelledError()
                if clear not in done:
                    raise TimeoutError("safety storage deadline exceeded")
                clear.result()
            except BaseException as exc:
                interruption = exc
                logger.exception("could not resolve rejected safety intent")
                self._safety_fault()
                if not clear.done():
                    clear.cancel()
                while not clear.done():
                    try:
                        await asyncio.shield(clear)
                    except asyncio.CancelledError:
                        continue
                    except BaseException:
                        break
                if clear.done() and not clear.cancelled():
                    clear.exception()
            finally:
                cancel.cancel()
            if self._safety_receipts != receipts:
                # A canceled or timed-out receipt still needs the old delete's outcome repaired.
                repair = asyncio.create_task(self._safety_storage(self._journal.safety.mark()))
                try:
                    while not repair.done():
                        try:
                            await asyncio.shield(repair)
                        except asyncio.CancelledError:
                            self._safety_fault()
                    repair.result()
                except BaseException:
                    logger.exception("could not retain safety intent after interrupted clear")
                    self._safety_fault()
            if isinstance(interruption, asyncio.CancelledError):
                raise interruption

    def _accept(self, link: Link, envelope: Envelope) -> None:
        if isinstance(envelope, ResponseEnvelope):
            return
        if isinstance(envelope, WorkspaceEnvelope):
            # The relay bounds reads to one in flight. Dropping excess here reserves
            # the ordinary and safety lanes even when the relay violates that limit.
            if self._workspace_request is None or self._workspace_request.done():
                self._workspace_request = self._spawn(self._answer_workspace(link, envelope))
            return
        if envelope.type in HALT_TYPES:
            self._queued_velocity = None
            key = (envelope.type, envelope.connection_epoch, envelope.session_id)
            existing = self._safety_pending.get(key)
            if existing is not None:
                if envelope.seq > existing[1].seq:
                    self._safety_pending[key] = (link, envelope)
            elif len(self._safety_pending) >= MAX_SAFETY_PENDING:
                self.auth_rejected = True
                self.runtime_failed = True
                self.request_shutdown()
                raise ConnectionError("agent safety capacity exceeded")
            else:
                self._safety_pending[key] = (link, envelope)
            if self._safety_task is None or self._safety_task.done():
                self._safety_task = self._spawn(self._drain_safety())
                self._safety_task.add_done_callback(self._safety_finished)
            return
        if isinstance(envelope, VelocityEnvelope) and envelope.payload.gesture is not None:
            if self._queued_velocity is not None:
                previous = self._queued_velocity[1]
                self._replaced_velocity.append(DriveExecutionFeedback.superseded(previous))
            self._queued_velocity = (link, envelope, self.continuity_generation)
            if self._velocity_task is None or self._velocity_task.done():
                self._velocity_task = self._spawn(self._drain_velocity())
            return
        if envelope.type in {"control.acquire", "control.release"}:
            self._queued_velocity = None
        safety_active = self._safety_task is not None and not self._safety_task.done()
        workspace_active = self._workspace_request in self._inflight
        if len(self._inflight) - int(safety_active) - int(workspace_active) >= MAX_INFLIGHT:
            raise ConnectionError("agent command capacity exceeded")
        if isinstance(envelope, VelocityEnvelope) and envelope.payload.server_deadline is not None:
            # //Compatible: only legacy grants use the wall-anchored conversion.
            convert = getattr(link, "local_command_deadline", None)
            deadline = (
                convert(envelope.payload.server_deadline)
                if convert is not None
                else None
            )
            generation = self.continuity_generation
            self._spawn(self._answer(link, envelope, deadline, generation))
        else:
            self._spawn(self._answer(link, envelope))

    async def _drain_velocity(self) -> None:
        while self._queued_velocity is not None:
            link, envelope, generation = self._queued_velocity
            self._queued_velocity = None
            await self._answer(link, envelope, generation=generation)

    def _safety_finished(self, task: asyncio.Task[None]) -> None:
        if task.cancelled() or task.exception() is not None:
            self._safety_fault()

    async def _drain_safety(self) -> None:
        while self._safety_pending:
            if self.recovery_required and self._stopping:
                return
            key = next(
                (key for key in self._safety_pending if key[0] == "safety.estop"),
                next(iter(self._safety_pending)),
            )
            kind = key[0]
            link, envelope = self._safety_pending.pop(key)
            self._estop_inflight = kind == "safety.estop"
            replay = False
            try:
                result = await self.dispatch(envelope)
            except DomainError as exc:
                if len(self._inflight) < MAX_INFLIGHT + 1:
                    self._spawn(self._reply(link, envelope, error=exc.code))
                if (
                    exc.code is not ErrorCode.CONFLICT
                    and link.epoch == self._last_epoch
                    and kind == "safety.estop"
                ):
                    self._safety_fault()
                replay = exc.code is ErrorCode.CONFLICT
                continue
            finally:
                if kind == "safety.estop":
                    self._estop_inflight = False
                    if replay:
                        self._safety_resolving = True
                        try:
                            await self._clear_replayed_safety()
                        finally:
                            self._safety_resolving = False
            if len(self._inflight) < MAX_INFLIGHT + 1:
                self._spawn(self._reply(link, envelope, result=result))

    async def _settle_safety(self) -> bool:
        task = self._safety_task
        if task is None:
            return True
        done, _pending = await asyncio.wait({task}, timeout=self._deadlines.safety)
        failed = task not in done or task.cancelled()
        if not failed:
            failed = task.exception() is not None
        if failed:
            self._safety_fault()
            logger.error("safety command did not complete; restart is disabled")
            if not task.done():
                task.cancel()
                done, _ = await asyncio.wait({task}, timeout=self._deadlines.safety)
                if not done:
                    self._stragglers.add(task)
                    task.add_done_callback(self._straggler_finished)
            return False
        return True

    def _spawn(self, work: Awaitable[None]) -> asyncio.Task[None]:
        task = asyncio.ensure_future(work)
        self._inflight.add(task)
        task.add_done_callback(self._inflight.discard)
        return task

    async def _answer(
        self,
        link: Link,
        envelope: Envelope,
        deadline: float | None = None,
        generation: int | None = None,
    ) -> None:
        try:
            result = await self.dispatch(
                envelope, deadline=deadline, link=link, generation=generation
            )
        except OperationNotCommitted as exc:
            await self._reply(link, envelope, error=exc.code, status=exc.status)
            return
        except DomainError as exc:
            await self._reply(link, envelope, error=exc.code)
            return
        if result.status is not OperationStatus.SUCCEEDED and envelope.type != "operation.query":
            # Only a query may report a status that is not committed, and it says so honestly.
            await self._reply(link, envelope, error=ErrorCode.CONFLICT, status=result.status)
            return
        await self._reply(link, envelope, result=result)
        if isinstance(envelope, OperationEnvelope) and envelope.type != "operation.query":
            try:
                await self.publish_snapshot(self.robot_id)
            except Exception:
                logger.warning("snapshot after edit failed", exc_info=True)

    async def _answer_workspace(self, link: Link, envelope: WorkspaceEnvelope) -> None:
        try:
            self._admit(envelope)
            if self._workspace is None:
                raise DomainError(ErrorCode.UNAVAILABLE, "workspace reads unavailable")
            data = await asyncio.wait_for(self._workspace.read(envelope), self._deadlines.edit)
            self._admit(envelope)
        except asyncio.TimeoutError:
            await self._reply(link, envelope, error=ErrorCode.TIMEOUT)
        except DomainError as exc:
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
        result: OperationResult | None = None,
        error: ErrorCode | None = None,
        status: OperationStatus | None = None,
        data: dict[str, Any] | None = None,
    ) -> None:
        request_type: Any = envelope.type
        shown = result.status if result is not None else status
        operation_id = (
            envelope.payload.operation_id if isinstance(envelope, OperationEnvelope) else None
        )
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
            payload=ResponsePayload(
                request_type=request_type,
                ok=error is None,
                error=error,
                operation_id=operation_id,
                data=data,
                status=shown.value if shown is not None else None,
            ),
        )
        try:
            await link.send_response(response)
        except Exception:
            # The result stays in the journal; the controller asks again after reconnecting.
            logger.warning("response could not be sent", exc_info=True)

    # ------------------------------------------------------------------ commands

    async def dispatch(
        self,
        envelope: Envelope,
        *,
        deadline: float | None = None,
        link: Link | None = None,
        generation: int | None = None,
    ) -> OperationResult:
        if isinstance(envelope, (ResponseEnvelope, SnapshotEnvelope, WorkspaceEnvelope)):
            raise DomainError(ErrorCode.VALIDATION, "command is not for this robot")
        if envelope.robot_id != self.robot_id:
            raise DomainError(ErrorCode.VALIDATION, "command is not for this robot")
        if isinstance(envelope, OperationEnvelope):
            return await self._dispatcher.run(envelope)
        try:
            result = await self._control(
                envelope, deadline=deadline, link=link, generation=generation
            )
        except DomainError as exc:
            self.metrics.inc(
                "agent_operations_total", operation=envelope.type, result=exc.code.value
            )
            raise
        self.metrics.inc(
            "agent_operations_total", operation=envelope.type, result=result.status.value
        )
        return result

    async def _control(
        self,
        envelope: VelocityEnvelope | SimpleEnvelope,
        *,
        deadline: float | None = None,
        link: Link | None = None,
        generation: int | None = None,
    ) -> OperationResult:
        request_id = envelope.request_id
        if isinstance(envelope, SimpleEnvelope) and envelope.type in HALT_TYPES:
            # A stop is honored whatever the epoch, expiry, or link state: it only removes motion.
            supervisor = self._supervisor
            self._latest_velocity = None
            if supervisor is not None:
                supervisor.submit_drive(0.0, 0.0)
            try:
                if envelope.type == "safety.estop":
                    await asyncio.wait_for(self._motion.estop(envelope), self._deadlines.safety)
                    self.recovery_required = True
                else:
                    await asyncio.wait_for(self._motion.stop(envelope), self._deadlines.safety)
            except MotionRejected as exc:
                raise DomainError(
                    exc.code or ErrorCode.SAFETY_REJECTED, "stop was rejected"
                ) from exc
            except Exception as exc:
                if supervisor is not None:
                    self._supervisor_fault("stop was not acknowledged")
                raise DomainError(ErrorCode.TIMEOUT, "stop was not acknowledged") from exc
            return OperationResult(request_id, OperationStatus.SUCCEEDED)
        if isinstance(envelope, SimpleEnvelope) and envelope.type == "navigation.cancel":
            if self._navigation is not None:
                self._admit(envelope)
                await self._navigation("navigation.cancel", request_id, {})
                return OperationResult(request_id, OperationStatus.SUCCEEDED)
        received_at = None
        if isinstance(envelope, VelocityEnvelope) and envelope.payload.gesture is not None:
            convert_timing = getattr(link, "local_drive_timing", None)
            issued = envelope.payload.server_issued_at
            remote_deadline = envelope.payload.server_deadline
            timing = (
                convert_timing(issued, remote_deadline)
                if convert_timing is not None and issued is not None and remote_deadline is not None
                else None
            )
            if timing is None:
                raise DomainError(ErrorCode.TIMEOUT, "motion clock evidence expired")
            received_at, deadline = timing
        if isinstance(envelope, VelocityEnvelope) and envelope.payload.server_deadline is not None:
            if (
                deadline is None
                or deadline <= self._clock.monotonic()
                or self._link is not link
                or generation != self.continuity_generation
            ):
                raise DomainError(ErrorCode.TIMEOUT, "motion authority expired")
        self._admit(envelope)
        if not self.eligible_for_motion:
            raise DomainError(ErrorCode.SAFETY_REJECTED, "robot is not eligible to move")
        if isinstance(envelope, VelocityEnvelope):
            if not self._source_clock_verified and (
                envelope.payload.server_deadline is None
                or (envelope.payload.gesture is None and not envelope.payload.challenge_id)
            ):
                raise DomainError(ErrorCode.UNAVAILABLE, "controller clock is not verified")
            if self._link is None or not self._link.command_clock_valid():
                raise DomainError(ErrorCode.TIMEOUT, "command clock is not synchronized")
            if envelope.payload.server_deadline is None:
                # //Compatible: isolated older tests inject a verified source clock.
                implied_send = envelope.expires_at - ADMISSION_SECONDS
                age = self._wall_clock() - implied_send
                if (
                    age < -MAX_COMMAND_CLOCK_ERROR
                    or age + MAX_COMMAND_CLOCK_ERROR > self._command_ttl_seconds
                ):
                    raise DomainError(
                        ErrorCode.TIMEOUT, "command time is outside the motion window"
                    )
                received_at = self._clock.monotonic() - max(0.0, age + MAX_COMMAND_CLOCK_ERROR)
            try:
                await asyncio.wait_for(
                    self._motion.drive(envelope, received_at=received_at, local_deadline=deadline)
                    if envelope.payload.server_deadline is not None
                    else self._motion.drive(envelope, received_at=received_at),
                    self._deadlines.safety,
                )
            except Exception as exc:
                raise DomainError(ErrorCode.SAFETY_REJECTED, "motion was not accepted") from exc
            if envelope.payload.gesture is None:
                # //Compatible: isolated legacy supervisor users may resubmit local samples.
                self._latest_velocity = envelope
            return OperationResult(request_id, OperationStatus.SUCCEEDED)
        try:
            await asyncio.wait_for(self._motion.forward(envelope), self._deadlines.safety)
        except DomainError:
            raise
        except Exception as exc:
            raise DomainError(ErrorCode.TIMEOUT, "ros side did not accept the command") from exc
        return OperationResult(request_id, OperationStatus.SUCCEEDED)

    async def _drive(self, vx: float, wz: float) -> None:
        """Supervisor sink. The arbiter gets the controller's own validated envelope."""
        if self._safety_blocked:
            return
        latest = self._latest_velocity
        if latest is None or (latest.payload.vx, latest.payload.wz) != (vx, wz):
            return
        await self._motion.drive(latest)

    def _admit(self, envelope: Envelope) -> None:
        """A first-time command needs a live link, the current epoch, and an unexpired deadline."""
        if self._link is None:
            raise DomainError(ErrorCode.OFFLINE, "robot is not connected to the server")
        if envelope.connection_epoch != self._epoch:
            raise DomainError(ErrorCode.CONFLICT, "command belongs to an earlier connection")
        if (
            envelope.type not in {"control.acquire", "control.release", "drive.velocity"}
            and envelope.expires_at < self._wall_clock()
        ):
            raise DomainError(ErrorCode.TIMEOUT, "command expired")

    # ------------------------------------------------------------------ snapshots

    async def publish_snapshot(self, robot_id: str) -> RobotSnapshot:
        if robot_id != self.robot_id:
            raise DomainError(ErrorCode.VALIDATION, "snapshot is for another robot")
        snapshot = await self._collect()
        link = self._link
        if link is not None:
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
            tasks=tuple(item for item in live.tasks if item.task_id not in deleted),
            home_revision=home_revision,
        )
        return snapshot

    def _log(self, event: str) -> None:
        logger.info(event, extra={"event": event, "robot_id": self.robot_id})


def main() -> int:
    """Load explicit configuration and run the owned agent composition."""
    from senior_control.agent.composition import AgentConfig, run_agent

    parser = argparse.ArgumentParser(description="Robot cloud agent")
    parser.add_argument("--robot-id")
    parser.add_argument("--cloud-url")
    parser.add_argument("--database-path")
    parser.add_argument("--socket-path")
    parser.add_argument("--token-file", type=Path)
    parser.add_argument("--credentials-file", type=Path)
    parser.add_argument("--pair", action="store_true")
    parser.add_argument("--device-name")
    args = parser.parse_args()
    values = dict(os.environ)
    for name, value in (
        ("SENIOR_ROBOT_ID", args.robot_id),
        ("SENIOR_CLOUD_URL", args.cloud_url),
        ("SENIOR_DATABASE_PATH", args.database_path),
        ("SENIOR_AGENT_SOCKET", args.socket_path),
    ):
        if value is not None:
            values[name] = value
    try:
        from senior_control.agent.pairing import pair_device, read_credentials, read_token

        credentials_file = args.credentials_file
        if credentials_file is None and values.get("SENIOR_CREDENTIALS_FILE"):
            credentials_file = Path(values["SENIOR_CREDENTIALS_FILE"])
        if args.pair:
            import getpass

            cloud_url = values.get("SENIOR_CLOUD_URL")
            if not cloud_url or credentials_file is None or not args.device_name:
                raise ValueError("pair requires cloud URL, device name and credentials file")
            code = getpass.getpass("Pairing code: ")
            robot_id = asyncio.run(pair_device(cloud_url, code, args.device_name, credentials_file))
            print(f"Paired robot {robot_id}")
            return 0
        if credentials_file is not None:
            credentials = read_credentials(credentials_file)
            values["SENIOR_ROBOT_ID"] = credentials.robot_id
            values["SENIOR_DEVICE_TOKEN"] = credentials.device_token
        if args.token_file is not None:
            values["SENIOR_DEVICE_TOKEN"] = read_token(args.token_file)
        config = AgentConfig.from_environment(values)
    except (OSError, ValueError) as exc:
        print(f"invalid agent configuration: {exc}", file=sys.stderr)
        return 78
    from senior_control.observability import configure_logging

    configure_logging()
    return asyncio.run(run_agent(config))


if __name__ == "__main__":
    raise SystemExit(main())
