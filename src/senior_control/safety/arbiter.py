"""Persistent estop, final publisher fence and local task navigation arbitration."""

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
    Envelope,
)

ModeName = Literal["estop", "navigation", "idle"]
_ZERO = Velocity(0.0, 0.0)


@dataclass(frozen=True)
class CommandResult:
    accepted: bool
    error: ErrorCode | None = None

    def __post_init__(self) -> None:
        if self.accepted == (self.error is not None):
            raise ValueError("command result conflicts")


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
        self._write(True)

    def clear_confirmed(self) -> Future[None]:
        """Only a trusted local reset confirmation may request durable unlatch."""
        return self._worker.submit(self._write, False)

    def _write(self, latched: bool) -> None:
        revision = self.revision + 1
        fd, name = tempfile.mkstemp(prefix=".estop-", dir=self._path.parent)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(
                    {
                        "version": 1,
                        "robot_id": self._robot_id,
                        "latched": latched,
                        "revision": revision,
                    },
                    stream,
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(name, self._path)
            self._sync_directory(self._path.parent)
            self.latched, self.revision = latched, revision
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
    """Own the local navigation source and software estop fence.

    Manual authority belongs to ManualIngress. Accepted tasks retain their local
    sample TTL through cloud disconnects; a heartbeat never grants motion here.
    """

    def __init__(self, robot_id: str, command_ttl_seconds: float, initial_estop: bool = False):
        if not math.isfinite(command_ttl_seconds) or command_ttl_seconds <= 0:
            raise ValueError("command TTL must be positive")
        self._robot_id = robot_id
        self._command_ttl_seconds = command_ttl_seconds
        self._estop = initial_estop
        self._history: dict[tuple[str, int], int] = {}
        self._navigation_run: str | None = None
        self._navigation_seq = 0
        self._task_epoch: int | None = None
        self._task_fences: dict[str, int] = {}
        self._velocity = _ZERO
        self._deadline = float("-inf")

    def accept(self, command: Envelope, now: float) -> CommandResult:
        if command.robot_id != self._robot_id or not math.isfinite(now):
            return _reject(ErrorCode.VALIDATION)
        if command.type != "safety.estop":
            return _reject(ErrorCode.VALIDATION)
        key = command.session_id, command.connection_epoch
        if command.connection_epoch <= 0 or command.seq <= self._history.get(key, -1):
            return _reject(ErrorCode.CONFLICT)
        self._history[key] = command.seq
        self.confirm_local_estop(True)
        return _ok()

    @property
    def estop_latched(self) -> bool:
        return self._estop

    def confirm_local_estop(self, latched: bool) -> None:
        if latched:
            self.shutdown()
        self._estop = latched

    @property
    def task_navigation_active(self) -> bool:
        return self._navigation_run is not None

    def task_navigation_allowed(self, now: float) -> bool:
        return math.isfinite(now) and not self._estop

    def navigation_source_active(self, execution_id: str, now: float) -> bool:
        return self.task_navigation_allowed(now) and self._navigation_run == execution_id

    def begin_navigation(self, execution_id: str, now: float, *, task_epoch: int) -> bool:
        if (
            not execution_id
            or len(execution_id) > 128
            or self._navigation_run is not None
            or task_epoch <= self._task_fences.get(execution_id, 0)
            or not self.task_navigation_allowed(now)
        ):
            return False
        self._navigation_run, self._task_epoch = execution_id, task_epoch
        self._navigation_seq = 0
        self._velocity, self._deadline = _ZERO, float("-inf")
        return True

    def accept_navigation_sample(
        self, execution_id: str, sequence: int, velocity: Velocity, sampled: float, now: float
    ) -> bool:
        if (
            not self.navigation_source_active(execution_id, now)
            or sequence <= self._navigation_seq
            or not all(math.isfinite(v) for v in (sampled, velocity.vx, velocity.wz))
            or not sampled <= now < sampled + self._command_ttl_seconds
        ):
            return False
        self._navigation_seq = sequence
        self._velocity, self._deadline = velocity, sampled + self._command_ttl_seconds
        return True

    def end_navigation(self, execution_id: str) -> None:
        if self._navigation_run == execution_id:
            self.shutdown()

    def final_command(self, now: float) -> tuple[Velocity, str, int, int, float] | None:
        if (
            self._navigation_run is None
            or self._task_epoch is None
            or not self.task_navigation_allowed(now)
            or self._velocity == _ZERO
            or now >= self._deadline
        ):
            return None
        return (
            self._velocity,
            self._navigation_run,
            self._task_epoch,
            self._navigation_seq,
            self._deadline,
        )

    def mode(self, now: float) -> ModeName:
        if self._estop:
            return "estop"
        return "navigation" if self._navigation_run and math.isfinite(now) else "idle"

    def shutdown(self) -> None:
        if self._navigation_run is not None and self._task_epoch is not None:
            self._task_fences[self._navigation_run] = self._task_epoch
        self._navigation_run, self._task_epoch = None, None
        self._navigation_seq = 0
        self._velocity, self._deadline = _ZERO, float("-inf")
