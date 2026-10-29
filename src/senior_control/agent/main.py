"""The on-robot agent: owns the robot database and the cloud link, never an rclpy executor."""

import asyncio
import logging
import signal
import time
from collections.abc import Awaitable, Callable, Mapping
from pathlib import Path
from typing import Any

from senior_control.agent.connection import HeartbeatFrame, HeartbeatLedger, Link, MotionPort
from senior_control.agent.dispatcher import Deadlines, Dispatcher, HomePort, MapPort, TaskPort
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
        heartbeat_seconds: float = 0.2,
        retry_delay: float = 1.0,
        heartbeat_window: float = 1.0,
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
        self._heartbeat_seconds = heartbeat_seconds
        self._retry_delay = retry_delay
        self._wall_clock = wall_clock
        self._ledger = HeartbeatLedger(
            clock if clock is not None else SystemClock(), heartbeat_window
        )
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
        self._epoch = 0
        self._last_epoch = 0
        self._latest_velocity: VelocityEnvelope | None = None
        self._supervisor: Supervisor | None = None
        self._stopping = False
        self._stop = asyncio.Event()
        self._inflight: set[asyncio.Task[None]] = set()
        self._signals: list[signal.Signals] = []

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
        return self.connected and self.motion_allowed and self._ledger.eligible(self.robot_id)

    def note_heartbeat(self, robot_id: str) -> None:
        self._ledger.note(robot_id)

    # ------------------------------------------------------------------ lifecycle

    async def run(self) -> None:
        supervisor = Supervisor(
            send_heartbeat=self._beat,
            call_ros=self._motion.call,
            drive_sink=self._drive,
            publish_cmd_vel=_never_publish,
            connect=self._session,
            background=self._background,
            heartbeat_seconds=self._heartbeat_seconds,
            retry_delay=self._retry_delay,
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

    def request_shutdown(self) -> None:
        """Synchronous and signal-safe: motion is revoked before anything else happens."""
        self._stopping = True
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
        try:
            await asyncio.wait_for(
                self._motion.disable(self._last_epoch), timeout=self._deadlines.safety
            )
        except Exception:
            logger.exception("ros side did not acknowledge disabling motion")

    async def _release(self) -> None:
        self._log("shutdown")
        inflight = list(self._inflight)
        for task in inflight:
            task.cancel()
        await asyncio.gather(*inflight, return_exceptions=True)
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

    async def _quietly(self, call: Awaitable[Any]) -> None:
        try:
            await asyncio.wait_for(call, timeout=self._deadlines.safety)
        except Exception:
            logger.exception("shutdown step failed")

    # ------------------------------------------------------------------ cloud link

    async def _beat(self) -> None:
        link = self._link
        if link is not None:
            await link.send_heartbeat()

    async def _session(self) -> None:
        try:
            link = await self._connector()
        except AuthRejected:
            self.request_shutdown()
            raise
        self._link, self._epoch, self._last_epoch = link, link.epoch, link.epoch
        self.metrics.inc("agent_reconnects_total")
        self._log("connection_opened")
        try:
            # The cloud holds no state it can trust after a reconnect: send everything again.
            await self.publish_snapshot(self.robot_id)
            while True:
                frame = await link.recv()
                if isinstance(frame, HeartbeatFrame):
                    self._ledger.note(frame.robot_id)
                else:
                    self._accept(link, frame)
        except (ConnectionError, OSError):
            return
        finally:
            if self._link is link:
                self._link, self._epoch = None, 0
            self._log("connection_closed")
            await self._quietly(link.close())

    def _accept(self, link: Link, envelope: Envelope) -> None:
        if isinstance(envelope, ResponseEnvelope):
            return
        if len(self._inflight) >= MAX_INFLIGHT:
            self._spawn(self._reject(link, envelope, ErrorCode.CONFLICT))
            return
        self._spawn(self._answer(link, envelope))

    def _spawn(self, work: Awaitable[None]) -> None:
        task = asyncio.ensure_future(work)
        self._inflight.add(task)
        task.add_done_callback(self._inflight.discard)

    async def _answer(self, link: Link, envelope: Envelope) -> None:
        try:
            result = await self.dispatch(envelope)
        except DomainError as exc:
            await self._reply(link, envelope, error=exc.code)
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
    ) -> None:
        request_type: Any = envelope.type
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
                status=result.status.value if result is not None else None,
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
                else:
                    await asyncio.wait_for(self._motion.stop(envelope), self._deadlines.safety)
            except Exception as exc:
                if supervisor is not None:
                    supervisor.revoke("stop was not acknowledged")
                raise DomainError(ErrorCode.TIMEOUT, "stop was not acknowledged") from exc
            return OperationResult(request_id, OperationStatus.SUCCEEDED)
        self._admit(envelope)
        if isinstance(envelope, VelocityEnvelope):
            supervisor = self._supervisor
            if supervisor is None or not self.eligible_for_motion:
                raise DomainError(ErrorCode.SAFETY_REJECTED, "robot is not eligible to move")
            self._latest_velocity = envelope
            if not supervisor.submit_drive(envelope.payload.vx, envelope.payload.wz):
                self._latest_velocity = None
                raise DomainError(ErrorCode.SAFETY_REJECTED, "robot is not eligible to move")
            return OperationResult(request_id, OperationStatus.SUCCEEDED)
        try:
            await asyncio.wait_for(self._motion.forward(envelope), self._deadlines.safety)
        except Exception as exc:
            raise DomainError(ErrorCode.TIMEOUT, "ros side did not accept the command") from exc
        return OperationResult(request_id, OperationStatus.SUCCEEDED)

    async def _drive(self, vx: float, wz: float) -> None:
        """Supervisor sink. The arbiter gets the controller's own validated envelope."""
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
        live = await asyncio.wait_for(self._state.collect(), timeout=self._deadlines.edit)
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
        )
        link = self._link
        if link is not None:
            await link.send_snapshot(snapshot)
        return snapshot

    def _log(self, event: str) -> None:
        logger.info(event, extra={"event": event, "robot_id": self.robot_id})
