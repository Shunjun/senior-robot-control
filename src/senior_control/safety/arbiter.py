"""Per-robot lease, heartbeat, and command arbitration.

Motion uses a monotonic deadline of the local receive time plus the command TTL.
``expires_at`` is not compared with that clock. A lease that has already died can be
acquired again on the same connection generation. A latched estop outranks
manual, then recovery, then navigation. Recovery has no remote command yet.
Disconnect and lease expiry stop motion and do not resume navigation.
"""

import fcntl
import json
import math
import os
import tempfile
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from senior_control.errors import ErrorCode
from senior_control.models import Velocity
from senior_control.protocol import (
    DRIVE_CHALLENGE_SECONDS,
    DRIVE_HOLD_FLOOR_SECONDS,
    DRIVE_TRANSPORT_SECONDS,
    Envelope,
    SimpleEnvelope,
    VelocityEnvelope,
)

ModeName = Literal["estop", "manual", "recovery", "navigation", "idle"]
_ZERO = Velocity(0.0, 0.0)
_SAFETY = frozenset({"safety.estop", "safety.reset"})
_NAV_START = frozenset({"navigation.start", "navigation.resume"})


@dataclass(frozen=True)
class CommandResult:
    accepted: bool
    error: ErrorCode | None = None

    def __post_init__(self) -> None:
        if self.accepted == (self.error is not None):
            raise ValueError("command result conflicts")


@dataclass
class _Lease:
    session_id: str
    epoch: int
    seq: int
    refreshed_at: float
    alive: bool
    gesture: int = 0
    closed_gesture: int = 0


def _ok() -> CommandResult:
    return CommandResult(True, None)


def _reject(error: ErrorCode) -> CommandResult:
    return CommandResult(False, error)


class PersistentEstop:
    """Single-owner durable latch; missing, invalid, or foreign state is latched."""

    def __init__(self, path: Path, robot_id: str) -> None:
        if not path.is_absolute() or not robot_id or len(robot_id) > 128:
            raise ValueError("absolute state path and robot identity required")
        self._path, self._robot_id = path, robot_id
        self._lock = open(str(path) + ".lock", "a+b")
        try:
            fcntl.flock(self._lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BaseException:
            self._lock.close()
            raise
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="estop-state")
        self._pending: Future[None] | None = None
        self._pending_request: tuple[str, int, int] | None = None
        self.latched, self.revision = True, 0
        try:
            with path.open("r", encoding="utf-8") as stream:
                value = json.load(stream)
            if (
                set(value) == {"version", "robot_id", "latched", "revision"}
                and value["version"] == 1
                and value["robot_id"] == robot_id
                and type(value["latched"]) is bool
                and type(value["revision"]) is int
                and value["revision"] >= 0
            ):
                self.latched, self.revision = value["latched"], value["revision"]
        except (OSError, ValueError, TypeError):
            pass

    @staticmethod
    def initialize(path: Path, robot_id: str) -> None:
        if not path.is_absolute() or not robot_id or len(robot_id) > 128:
            raise ValueError("absolute state path and robot identity required")
        with open(str(path) + ".lock", "a+b") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
            fd = os.open(path, flags, 0o600)
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(
                    {"version": 1, "robot_id": robot_id, "latched": False, "revision": 0},
                    stream,
                )
                stream.flush()
                os.fsync(stream.fileno())
            PersistentEstop._sync_directory(path.parent)

    @staticmethod
    def _sync_directory(directory: Path) -> None:
        fd = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def estop(self) -> None:
        self.latched = True
        self.revision += 1
        fd, name = tempfile.mkstemp(prefix=".estop-", dir=self._path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(
                    {
                        "version": 1,
                        "robot_id": self._robot_id,
                        "latched": True,
                        "revision": self.revision,
                    },
                    stream,
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, self._path)
            self._sync_directory(self._path.parent)
        finally:
            try:
                os.unlink(name)
            except FileNotFoundError:
                pass

    def commit(self, command: Envelope, result: CommandResult) -> CommandResult:
        """Acknowledge an accepted estop only after its latch is durable."""
        if not result.accepted and (
            result.error is not ErrorCode.CONFLICT or self._pending is None
        ):
            return result
        request = (command.session_id, command.connection_epoch, command.seq)
        retry = request == self._pending_request
        if result.accepted:
            self._pending_request = request
        pending = self._pending
        if pending is None or (pending.done() and pending.exception() is not None):
            pending = self._pending = self._worker.submit(self.estop)
        try:
            pending.result(timeout=0.2)
        except Exception:
            # Retain the write and its identity so a replay can finish it.
            return _reject(ErrorCode.UNAVAILABLE)
        self._pending = None
        self._pending_request = None
        return _ok() if retry else result

    def close(self) -> None:
        self._worker.shutdown(wait=False, cancel_futures=True)
        if self._pending is not None and not self._pending.done():
            self._pending.add_done_callback(lambda _future: self._lock.close())
        else:
            self._lock.close()


class FinalCommandFence:
    """Gate authority against the actual ROS publisher and immutable steady deadline."""

    def __init__(self) -> None:
        self._publisher: bytes | None = None
        self._retired: set[bytes] = set()
        self._retired_boots: set[str] = set()
        self._boot: str | None = None
        self._generation = -1
        self._sequence = -1
        self._deadline = float("-inf")
        self._velocity = _ZERO
        self._authority: tuple[str, int] | None = None
        self._invalid = False

    def set_publisher(self, publisher: bytes | None) -> None:
        if publisher != self._publisher:
            if self._publisher is not None:
                self._retired.add(self._publisher)
            if self._boot is not None:
                self._retired_boots.add(self._boot)
            self._publisher = publisher
            self._boot = None
            self._generation = -1
            self._sequence = -1
            self._deadline = float("-inf")
            self._velocity = _ZERO
            self._authority = None
            self._invalid = (
                publisher in self._retired
                or len(self._retired) > 64
                or len(self._retired_boots) > 64
            )

    def accept(
        self,
        publisher: bytes,
        boot: str,
        generation: int,
        sequence: int,
        deadline: float,
        velocity: Velocity,
        now: float,
        *,
        session: str = "",
        epoch: int = 0,
    ) -> bool:
        if (
            not publisher
            or publisher != self._publisher
            or self._invalid
            or not boot
            or boot in self._retired_boots
            or len(boot) > 128
            or generation < 0
            or sequence < 0
            or not all(math.isfinite(v) for v in (deadline, now, velocity.vx, velocity.wz))
        ):
            return False
        if self._boot is not None and boot != self._boot:
            self._invalid = True
            self._deadline = float("-inf")
            return False
        if self._boot is None or generation > self._generation:
            # DDS may drop intermediate barriers; any newer zero barrier supersedes them.
            if velocity != _ZERO or sequence != 0:
                return False
            self._boot, self._generation, self._sequence = boot, generation, sequence
            self._deadline, self._velocity = deadline, velocity
            self._authority = None
            return True
        if generation != self._generation or sequence < self._sequence:
            return False
        authority = (session, epoch)
        if self._authority is not None and authority != self._authority:
            return False
        if sequence == self._sequence:
            return deadline == self._deadline and velocity == self._velocity
        if deadline <= now:
            return False
        self._authority = authority
        self._sequence, self._deadline, self._velocity = sequence, deadline, velocity
        return True

    def usable(self, now: float) -> bool:
        return (
            not self._invalid
            and self._publisher is not None
            and self._boot is not None
            and math.isfinite(now)
            and now < self._deadline
            and self._velocity != _ZERO
        )


class Arbiter:
    def __init__(
        self,
        robot_id: str,
        heartbeat_seconds: float,
        disconnect_seconds: float,
        command_ttl_seconds: float,
        require_server_heartbeat: bool = False,
        initial_estop: bool = False,
    ) -> None:
        if not (
            math.isfinite(heartbeat_seconds)
            and math.isfinite(disconnect_seconds)
            and math.isfinite(command_ttl_seconds)
            and heartbeat_seconds > 0.0
            and command_ttl_seconds > 0.0
            and heartbeat_seconds < disconnect_seconds
        ):
            raise ValueError("heartbeat must be shorter than disconnect threshold")
        self._robot_id = robot_id
        # //Compatible: isolated legacy callers do not have a server heartbeat port.
        self._require_server_heartbeat = require_server_heartbeat
        self._server_epoch = 0
        self._server_deadline = float("-inf")
        self._server_disabled_epoch = 0
        self._disconnect_seconds = disconnect_seconds
        self._command_ttl_seconds = command_ttl_seconds
        self._lease: _Lease | None = None
        self._history: dict[tuple[str, int], int] = {}
        self._estop = initial_estop
        self._manual = False
        self._recovery = False
        self._navigation = False
        self._navigation_run: str | None = None
        self._navigation_seq = 0
        self._velocity = _ZERO
        self._deadline: float | None = None
        self._strict_deadline = False

    def accept(
        self,
        command: Envelope,
        now: float,
        *,
        received_at: float | None = None,
        local_deadline: float | None = None,
    ) -> CommandResult:
        self._expire(now)
        if not math.isfinite(now):
            return _reject(ErrorCode.VALIDATION)
        if command.robot_id != self._robot_id:
            return _reject(ErrorCode.SAFETY_REJECTED)
        if not isinstance(command, (VelocityEnvelope, SimpleEnvelope)):
            return _reject(ErrorCode.VALIDATION)
        if (
            self._require_server_heartbeat
            and command.type not in _SAFETY
            and command.type != "drive.stop"
            and (
                command.connection_epoch != self._server_epoch
                or now > self._server_deadline
                or command.connection_epoch <= self._server_disabled_epoch
            )
        ):
            return _reject(ErrorCode.OFFLINE)
        lease = self._lease
        if self._require_server_heartbeat and command.type in _SAFETY:
            if self._server_epoch == 0:
                if command.type == "safety.estop" and command.connection_epoch > 0:
                    key = (command.session_id, command.connection_epoch)
                    if command.seq <= self._history.get(key, -1):
                        return _reject(ErrorCode.CONFLICT)
                    self._history[key] = command.seq
                    return self._apply_safety(command.type)
                return _reject(ErrorCode.OFFLINE)
            if command.connection_epoch < self._server_epoch:
                return _reject(ErrorCode.CONFLICT)
            if command.connection_epoch != self._server_epoch and command.type != "safety.estop":
                return _reject(ErrorCode.OFFLINE)
            if command.type == "safety.estop":
                key = (command.session_id, command.connection_epoch)
                if command.seq <= self._history.get(key, -1):
                    return _reject(ErrorCode.CONFLICT)
                self._history[key] = command.seq
                if lease is not None and lease.session_id == command.session_id:
                    lease.seq = command.seq
                return self._apply_safety(command.type)
            # No reset may clear an estop until the command carries an estop fence.
            return _reject(ErrorCode.UNAVAILABLE)
        if (
            lease is None
            or command.session_id != lease.session_id
            or command.connection_epoch != lease.epoch
        ) and command.seq <= self._history.get((command.session_id, command.connection_epoch), -1):
            return _reject(ErrorCode.CONFLICT)
        if lease is None:
            return self._accept_without_lease(command, now, received_at)
        if command.session_id != lease.session_id:
            if command.connection_epoch < lease.epoch:
                return _reject(ErrorCode.CONFLICT)
            if command.type == "control.acquire" and not lease.alive:
                # The reconnected page may take this generation once the old lease is dead.
                return self._open_lease(command, now)
            if (
                command.type in _SAFETY
                and not lease.alive
                and command.connection_epoch > lease.epoch
            ):
                return self._accept_safety(command, now, advance_epoch=True)
            return _reject(ErrorCode.CONFLICT)
        if command.connection_epoch < lease.epoch:
            return _reject(ErrorCode.CONFLICT)
        if command.connection_epoch > lease.epoch:
            if command.type == "control.acquire":
                return self._open_lease(command, now)
            if command.type in _SAFETY and not lease.alive:
                return self._accept_safety(command, now, advance_epoch=True)
            return _reject(ErrorCode.OFFLINE)
        if command.seq <= lease.seq:
            return _reject(ErrorCode.CONFLICT)
        if not lease.alive:
            if command.type == "control.acquire":
                return self._open_lease(command, now)
            if command.type in _SAFETY:
                return self._accept_safety(command, now, advance_epoch=False)
            return _reject(ErrorCode.OFFLINE)
        return self._accept_live(command, now, received_at, local_deadline)

    def server_heartbeat(self, epoch: int, now: float, deadline: float) -> bool:
        """Renew only a fresh, bounded server lease; expiry requires a new epoch."""
        if not math.isfinite(now) or not math.isfinite(deadline):
            return False
        if deadline <= now or deadline - now > self._disconnect_seconds:
            return False
        if self._server_epoch and now > self._server_deadline:
            self._server_disabled_epoch = max(self._server_disabled_epoch, self._server_epoch)
            if self._lease is not None:
                self._lease.alive = False
            self._clear_motion()
        if epoch <= self._server_disabled_epoch or epoch < self._server_epoch:
            return False
        if epoch > self._server_epoch:
            if self._lease is not None:
                self._lease.alive = False
            self._clear_motion()
            self._server_epoch = epoch
        self._server_deadline = deadline
        return True

    def tick(self, now: float) -> Velocity:
        self._expire(now)
        if self._estop or not (self._connected(now) or (
            self._navigation_run is not None and self.navigation_qualified(now)
        )):
            return _ZERO
        if (
            (self._manual or self._navigation or self._recovery)
            and self._deadline is not None
            and (now < self._deadline or (not self._strict_deadline and now == self._deadline))
        ):
            return self._velocity
        return _ZERO

    def final_command(self, now: float) -> tuple[Velocity, str, int, int, float] | None:
        velocity = self.tick(now)
        if self._navigation_run is not None and velocity != _ZERO and self._deadline is not None:
            return (
                velocity,
                self._navigation_run,
                self._server_epoch,
                self._navigation_seq,
                min(self._deadline, self._server_deadline),
            )
        lease = self._lease
        if lease is None or self._deadline is None:
            return None
        deadline = self._deadline
        if velocity == _ZERO and not (
            self._strict_deadline
            and self._manual
            and not self._estop
            and self._connected(now)
            and now < deadline
        ):
            return None
        if self._require_server_heartbeat and not self._strict_deadline:
            # //Compatible: legacy commands retain the original heartbeat cap on publication.
            deadline = min(deadline, self._server_deadline)
        return velocity, lease.session_id, lease.epoch, lease.seq, deadline

    def navigation_qualified(self, now: float) -> bool:
        self._expire(now)
        return (
            self._require_server_heartbeat
            and math.isfinite(now)
            and self._server_epoch > self._server_disabled_epoch
            and now < self._server_deadline
            and not self._estop
            and not self._connected(now)
        )

    def navigation_source_active(self, execution_id: str, now: float) -> bool:
        return self.navigation_qualified(now) and self._navigation_run == execution_id

    def begin_navigation(self, execution_id: str, now: float) -> bool:
        if (
            not execution_id
            or len(execution_id) > 128
            or self._navigation_run is not None
            or not self.navigation_qualified(now)
        ):
            return False
        self._clear_motion()
        self._navigation_run = execution_id
        self._navigation = True
        self._strict_deadline = True
        return True

    def accept_navigation_sample(
        self, execution_id: str, sequence: int, velocity: Velocity, sampled: float, now: float
    ) -> bool:
        if (
            not self.navigation_source_active(execution_id, now)
            or sequence <= self._navigation_seq
            or not math.isfinite(sampled)
            or not sampled <= now < sampled + self._command_ttl_seconds
        ):
            return False
        self._navigation_seq = sequence
        self._velocity = velocity
        self._deadline = sampled + self._command_ttl_seconds
        return True

    def end_navigation(self, execution_id: str) -> None:
        if self._navigation_run == execution_id:
            self._clear_motion()

    def accept_autonomous(
        self, source: Literal["navigation", "recovery"], velocity: Velocity, now: float
    ) -> bool:
        """Reject unbound commands until a source-stamped generation bridge exists."""
        del source, velocity, now
        return False

    def begin_recovery(self, now: float) -> bool:
        self._expire(now)
        if not self._connected(now) or self._estop or self._manual or not self._navigation:
            return False
        self._navigation = False
        self._recovery = True
        self._velocity = _ZERO
        self._deadline = None
        return True

    def disconnect(self, connection_epoch: int) -> None:
        if self._navigation_run is not None and connection_epoch == self._server_epoch:
            self._server_disabled_epoch = max(connection_epoch, self._server_disabled_epoch)
            self._clear_motion()
        lease = self._lease
        if lease is not None and lease.epoch == connection_epoch:
            lease.alive = False
            self._clear_motion()

    def mode(self, now: float) -> ModeName:
        self._expire(now)
        if self._estop:
            return "estop"
        if not self._connected(now):
            return "idle"
        if self._manual:
            return "manual"
        if self._recovery:
            return "recovery"
        if self._navigation:
            return "navigation"
        return "idle"

    def _accept_without_lease(
        self,
        command: VelocityEnvelope | SimpleEnvelope,
        now: float,
        _received_at: float | None,
    ) -> CommandResult:
        if command.type == "control.acquire":
            return self._open_lease(command, now)
        if command.type in _SAFETY:
            return self._accept_safety(command, now, advance_epoch=False)
        if command.type in _NAV_START and not self._estop:
            self._open_lease(command, now)
            self._navigation = True
            return _ok()
        return _reject(ErrorCode.OFFLINE)

    def _accept_live(
        self,
        command: VelocityEnvelope | SimpleEnvelope,
        now: float,
        received_at: float | None,
        local_deadline: float | None,
    ) -> CommandResult:
        kind = command.type
        if kind == "control.acquire":
            self._refresh(command, now)
            self._clear_motion()
            return _ok()
        if kind == "control.release":
            self._refresh(command, now)
            lease = self._lease
            if lease is None:
                raise RuntimeError("lease missing")
            lease.alive = False
            self._clear_motion()
            return _ok()
        if kind == "drive.stop":
            if self._estop:
                return _reject(ErrorCode.SAFETY_REJECTED)
            lease = self._lease
            assert lease is not None
            if command.payload.gesture is not None and command.payload.gesture < lease.gesture:
                return _reject(ErrorCode.CONFLICT)
            lease.closed_gesture = max(
                lease.gesture, lease.closed_gesture, command.payload.gesture or 0
            )
            self._refresh(command, now)
            self._hold_manual_stop()
            return _ok()
        if kind == "drive.velocity":
            return self._accept_velocity(command, now, received_at, local_deadline)
        if kind in _SAFETY:
            self._refresh(command, now)
            return self._apply_safety(kind)
        if kind in _NAV_START:
            if self._estop:
                return _reject(ErrorCode.SAFETY_REJECTED)
            if self._manual:
                return _reject(ErrorCode.CONFLICT)
            self._refresh(command, now)
            self._clear_motion()
            self._navigation = True
            return _ok()
        if kind == "navigation.cancel":
            self._refresh(command, now)
            self._clear_motion()
            return _ok()
        return _reject(ErrorCode.VALIDATION)

    def _accept_velocity(
        self,
        command: VelocityEnvelope | SimpleEnvelope,
        now: float,
        received_at: float | None,
        local_deadline: float | None,
    ) -> CommandResult:
        if self._estop:
            return _reject(ErrorCode.SAFETY_REJECTED)
        if not isinstance(command, VelocityEnvelope):
            return _reject(ErrorCode.VALIDATION)
        if self._require_server_heartbeat and received_at is None and local_deadline is None:
            return _reject(ErrorCode.VALIDATION)
        gesture = command.payload.gesture
        if gesture is not None:
            lease = self._lease
            assert lease is not None
            if gesture < lease.gesture or gesture <= lease.closed_gesture:
                return _reject(ErrorCode.CONFLICT)
            if (
                received_at is None
                or local_deadline is None
                or command.payload.server_issued_at is None
                or not math.isfinite(received_at)
                or not math.isfinite(local_deadline)
                or received_at > now
            ):
                return _reject(ErrorCode.VALIDATION)
            if now >= local_deadline or now > received_at + DRIVE_TRANSPORT_SECONDS:
                return _reject(ErrorCode.TIMEOUT)
            deadline = max(local_deadline, now + DRIVE_HOLD_FLOOR_SECONDS)
            lease.gesture = gesture
        elif local_deadline is not None:
            # //Compatible: preserve the original challenge deadline for older controllers.
            source_window = (
                DRIVE_CHALLENGE_SECONDS
                if command.payload.server_deadline is not None
                else self._command_ttl_seconds
            )
            if not math.isfinite(local_deadline) or local_deadline > now + source_window:
                return _reject(ErrorCode.VALIDATION)
            # Shorter local TTLs expire relative to issuance, even after transport delay.
            deadline = local_deadline - max(0.0, source_window - self._command_ttl_seconds)
        else:
            # //Compatible: isolated legacy callers provide original local receipt time.
            received = now if received_at is None else received_at
            if not math.isfinite(received) or received > now:
                return _reject(ErrorCode.VALIDATION)
            deadline = received + self._command_ttl_seconds
        if self._require_server_heartbeat:
            deadline = min(deadline, self._server_deadline)
        # A command that already used up its TTL while queued does not start another window.
        if now > deadline:
            return _reject(ErrorCode.TIMEOUT)
        self._refresh(command, now)
        self._manual = True
        self._navigation = False
        self._recovery = False
        self._velocity = Velocity(command.payload.vx, command.payload.wz)
        self._deadline = deadline
        self._strict_deadline = gesture is not None
        return _ok()

    def _accept_safety(
        self,
        command: VelocityEnvelope | SimpleEnvelope,
        _now: float,
        *,
        advance_epoch: bool,
    ) -> CommandResult:
        """Apply estop or reset. `_now` is ignored so a dead lease stays dead."""
        lease = self._lease
        self._history[(command.session_id, command.connection_epoch)] = command.seq
        if lease is None:
            self._lease = _Lease(
                session_id=command.session_id,
                epoch=command.connection_epoch,
                seq=command.seq,
                refreshed_at=float("-inf"),
                alive=False,
            )
        else:
            lease.seq = command.seq
            if advance_epoch:
                lease.epoch = command.connection_epoch
            lease.alive = False
        return self._apply_safety(command.type)

    def _apply_safety(self, kind: str) -> CommandResult:
        if kind == "safety.estop":
            self._estop = True
            self._clear_motion()
            return _ok()
        if kind == "safety.reset":
            self._estop = False
            return _ok()
        return _reject(ErrorCode.VALIDATION)

    def _open_lease(self, command: Envelope, now: float) -> CommandResult:
        self._history[(command.session_id, command.connection_epoch)] = command.seq
        self._lease = _Lease(
            session_id=command.session_id,
            epoch=command.connection_epoch,
            seq=command.seq,
            refreshed_at=now,
            alive=True,
        )
        self._clear_motion()
        return _ok()

    def _refresh(self, command: Envelope, now: float) -> None:
        lease = self._lease
        if lease is None:
            raise RuntimeError("lease missing")
        lease.seq = command.seq
        self._history[(command.session_id, command.connection_epoch)] = command.seq
        lease.refreshed_at = now
        lease.alive = True

    def _hold_manual_stop(self) -> None:
        self._manual = True
        self._navigation = False
        self._recovery = False
        self._velocity = _ZERO
        self._deadline = None

    def _clear_motion(self) -> None:
        self._manual = False
        self._recovery = False
        self._navigation = False
        self._navigation_run = None
        self._navigation_seq = 0
        self._velocity = _ZERO
        self._deadline = None

    def _expire(self, now: float) -> None:
        if self._navigation_run is not None and (
            not math.isfinite(now) or now >= self._server_deadline
        ):
            self._server_disabled_epoch = max(self._server_epoch, self._server_disabled_epoch)
            self._clear_motion()
        lease = self._lease
        if lease is None or not lease.alive:
            return
        if (
            not math.isfinite(now)
            or now < lease.refreshed_at
            or (
                self._require_server_heartbeat
                and (lease.epoch != self._server_epoch or now > self._server_deadline)
            )
            or (
                not self._require_server_heartbeat
                and now - lease.refreshed_at > self._disconnect_seconds
            )
        ):
            lease.alive = False
            self._clear_motion()

    def _connected(self, now: float) -> bool:
        lease = self._lease
        return (
            lease is not None
            and lease.alive
            and math.isfinite(now)
            and now >= lease.refreshed_at
            and (
                (
                    self._require_server_heartbeat
                    and lease.epoch == self._server_epoch
                    and now <= self._server_deadline
                )
                or (
                    not self._require_server_heartbeat
                    and now - lease.refreshed_at <= self._disconnect_seconds
                )
            )
        )
