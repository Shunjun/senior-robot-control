"""Final publisher fence and local task navigation arbitration."""

import math
from dataclasses import dataclass
from typing import Literal

from senior_control.errors import ErrorCode
from senior_control.models import Velocity
from senior_control.protocol import (
    Envelope,
)

ModeName = Literal["navigation", "idle"]
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
    """Own local navigation samples and retire them on explicit stop requests.

    Manual authority belongs to ManualIngress. Accepted tasks retain their local
    sample TTL through cloud disconnects; a heartbeat never grants motion here.
    """

    def __init__(self, robot_id: str, command_ttl_ms: float):
        if not math.isfinite(command_ttl_ms) or command_ttl_ms <= 0:
            raise ValueError("command TTL must be positive")
        self._robot_id = robot_id
        self._command_ttl_seconds = command_ttl_ms / 1000
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
        self.shutdown()
        return _ok()

    @property
    def task_navigation_active(self) -> bool:
        return self._navigation_run is not None

    def task_navigation_allowed(self, now: float) -> bool:
        return math.isfinite(now)

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
        return "navigation" if self._navigation_run and math.isfinite(now) else "idle"

    def shutdown(self) -> None:
        if self._navigation_run is not None and self._task_epoch is not None:
            self._task_fences[self._navigation_run] = self._task_epoch
        self._navigation_run, self._task_epoch = None, None
        self._navigation_seq = 0
        self._velocity, self._deadline = _ZERO, float("-inf")
