"""The on-robot agent: owns the robot database and the cloud link, never an rclpy executor."""

import argparse
import asyncio
import logging
import os
import signal
import sys
import time
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
    OperationNotCommitted,
    TaskPort,
)
from senior_control.agent.journal import AgentJournal
from senior_control.agent.snapshots import HomeSummary, RobotSnapshot, StatePort
from senior_control.clock import Clock, SystemClock
from senior_control.errors import DomainError, ErrorCode
from senior_control.models import OperationResult, OperationStatus
from senior_control.observability import Metrics
from senior_control.protocol import (
    Envelope,
    OperationEnvelope,
    ResponseEnvelope,
    ResponsePayload,
    SimpleEnvelope,
    SnapshotEnvelope,
    VelocityEnvelope,
)
from senior_control.runtime.supervisor import AuthRejected, Supervisor

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
        deadlines: Deadlines | None = None,
        map_root: Path = Path("map-transfer"),
        background: Mapping[str, Callable[[], Awaitable[None]]] | None = None,
        cleanup: Callable[[], Awaitable[None]] | None = None,
        on_eligibility_lost: Callable[[int], Awaitable[None]] | None = None,
        heartbeat_seconds: float = 0.2,
        retry_delay: float = 1.0,
        heartbeat_window: float = 1.0,
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
        self._motion = motion
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
        )
        self._link: Link | None = None
        self._revoked_link: Link | None = None
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
        self._safety_pending: dict[tuple[str, int, str], tuple[Link, Envelope]] = {}
        self._safety_lock = asyncio.Lock()
        self._safety_admitting = False
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
            self.connected
            and self.motion_allowed
            and not self._safety_blocked
            and self._ledger.eligible(self.robot_id)
        )

    @property
    def _safety_blocked(self) -> bool:
        return (
            self.recovery_required
            or self._safety_admitting
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
            logger.error("unresolved safety intent requires controlled recovery")
            self._safety_fault()
            await self._release()
            return
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
        if link is None or not self.motion_allowed or self._safety_blocked:
            return
        self._expire_eligibility()
        if self._revoked_link is link:
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

    async def _session(self) -> None:
        try:
            link = await self._connector()
        except AuthRejected:
            self.auth_rejected = True
            self.request_shutdown()
            raise
        self._heartbeat_generation += 1
        self._revoked_link = None
        self._link, self._epoch, self._last_epoch = link, link.epoch, link.epoch
        self.metrics.inc("agent_reconnects_total")
        self._log("connection_opened")
        try:
            # Snapshot collection is independent of heartbeat and control reception.
            keepalive = asyncio.ensure_future(self._keep_snapshot_fresh(link))
            try:
                while True:
                    frame = await link.recv()
                    if isinstance(frame, HeartbeatFrame):
                        if self._safety_blocked:
                            continue
                        was_eligible = self._was_eligible
                        self._expire_eligibility()
                        if was_eligible and not self._was_eligible:
                            raise ConnectionError("heartbeat continuity expired")
                        if not self.motion_allowed or self._revoked_link is link:
                            raise ConnectionError("motion is revoked")
                        if (
                            not self.motion_allowed
                            or frame.robot_id != self.robot_id
                            or frame.deadline <= self._clock.monotonic()
                        ):
                            raise ConnectionError("invalid heartbeat")
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
                    else:
                        await self._receive_command(link, frame)
            finally:
                keepalive.cancel()
                await self._wait_inflight([keepalive])
        except AuthRejected:
            self.auth_rejected = True
            self.request_shutdown()
            raise
        except (ConnectionError, OSError):
            return
        finally:
            epoch = link.epoch
            if self._link is link:
                self._lose_eligibility()
                self._link, self._epoch = None, 0
                self._ledger.forget(self.robot_id)
            self._log("connection_closed")
            inflight = list(self._inflight)
            for task in inflight:
                if task is not self._safety_task:
                    task.cancel()
            await self._quietly(self._motion.disable(epoch))
            await self._settle_safety()
            if self.recovery_required:
                self.request_shutdown()
            await self._wait_inflight(inflight)
            await self._quietly(link.close())

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

    async def _safety_storage(self, work: Awaitable[Any]) -> Any:
        """Bound storage waits without assuming cancellation can stop SQLite immediately."""
        task = asyncio.ensure_future(work)
        try:
            done, _ = await asyncio.wait({task}, timeout=self._deadlines.safety)
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
        return task.result()

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
        self._safety_admitting = True
        self._latest_velocity = None
        try:
            async with self._safety_lock:
                await self._safety_storage(self._journal.safety.mark())
                if overflow:
                    self.recovery_required = True
                self._accept(link, envelope)
        except BaseException as exc:
            self._safety_fault()
            if isinstance(exc, asyncio.CancelledError):
                raise
            raise ConnectionError("safety admission failed") from exc
        finally:
            self._safety_admitting = False

    async def _clear_replayed_safety(self) -> None:
        async with self._safety_lock:
            if (
                self.recovery_required
                or self._safety_admitting
                or self._estop_inflight
                or any(key[0] == "safety.estop" for key in self._safety_pending)
            ):
                return
            try:
                await self._safety_storage(self._journal.safety.clear_rejected())
            except BaseException as exc:
                logger.exception("could not resolve rejected safety intent")
                if self._safety_admitting:
                    # A new receipt can be waiting while the delete's outcome is unknown.
                    try:
                        await self._safety_storage(self._journal.safety.mark())
                    except BaseException:
                        logger.exception("could not retain safety intent after interrupted clear")
                self._safety_fault()
                if isinstance(exc, asyncio.CancelledError):
                    raise

    def _accept(self, link: Link, envelope: Envelope) -> None:
        if isinstance(envelope, ResponseEnvelope):
            return
        if envelope.type in HALT_TYPES:
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
        safety_active = self._safety_task is not None and not self._safety_task.done()
        if len(self._inflight) - int(safety_active) >= MAX_INFLIGHT:
            raise ConnectionError("agent command capacity exceeded")
        self._spawn(self._answer(link, envelope))

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

    async def _answer(self, link: Link, envelope: Envelope) -> None:
        try:
            result = await self.dispatch(envelope)
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
                status=shown.value if shown is not None else None,
            ),
        )
        try:
            await link.send_response(response)
        except Exception:
            # The result stays in the journal; the controller asks again after reconnecting.
            logger.warning("response could not be sent", exc_info=True)

    # ------------------------------------------------------------------ commands

    async def dispatch(self, envelope: Envelope) -> OperationResult:
        if isinstance(envelope, (ResponseEnvelope, SnapshotEnvelope)):
            raise DomainError(ErrorCode.VALIDATION, "command is not for this robot")
        if envelope.robot_id != self.robot_id:
            raise DomainError(ErrorCode.VALIDATION, "command is not for this robot")
        if isinstance(envelope, OperationEnvelope):
            return await self._dispatcher.run(envelope)
        try:
            result = await self._control(envelope)
        except DomainError as exc:
            self.metrics.inc(
                "agent_operations_total", operation=envelope.type, result=exc.code.value
            )
            raise
        self.metrics.inc(
            "agent_operations_total", operation=envelope.type, result=result.status.value
        )
        return result

    async def _control(self, envelope: VelocityEnvelope | SimpleEnvelope) -> OperationResult:
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
        self._admit(envelope)
        if not self.eligible_for_motion:
            raise DomainError(ErrorCode.SAFETY_REJECTED, "robot is not eligible to move")
        if isinstance(envelope, VelocityEnvelope):
            if not self._source_clock_verified:
                raise DomainError(ErrorCode.UNAVAILABLE, "controller clock is not verified")
            if self._link is None or not self._link.command_clock_valid():
                raise DomainError(ErrorCode.TIMEOUT, "command clock is not synchronized")
            implied_send = envelope.expires_at - ADMISSION_SECONDS
            age = self._wall_clock() - implied_send
            if (
                age < -MAX_COMMAND_CLOCK_ERROR
                or age + MAX_COMMAND_CLOCK_ERROR > self._command_ttl_seconds
            ):
                raise DomainError(ErrorCode.TIMEOUT, "command time is outside the motion window")
            received_at = self._clock.monotonic() - max(0.0, age + MAX_COMMAND_CLOCK_ERROR)
            try:
                await asyncio.wait_for(
                    self._motion.drive(envelope, received_at=received_at),
                    self._deadlines.safety,
                )
            except Exception as exc:
                raise DomainError(ErrorCode.SAFETY_REJECTED, "motion was not accepted") from exc
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
        if envelope.expires_at < self._wall_clock():
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
            tasks=live.tasks,
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
