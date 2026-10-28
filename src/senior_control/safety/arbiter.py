"""Per-robot lease, heartbeat, and command arbitration.

Motion uses a monotonic deadline of accept time plus the command TTL.
``expires_at`` is not compared with that clock. A latched estop outranks
manual, then recovery, then navigation. Recovery has no remote command yet.
Disconnect and lease expiry stop motion and do not resume navigation.
"""

import math
from dataclasses import dataclass
from typing import Literal

from senior_control.errors import ErrorCode
from senior_control.models import Velocity
from senior_control.protocol import Envelope, SimpleEnvelope, VelocityEnvelope

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


def _ok() -> CommandResult:
    return CommandResult(True, None)


def _reject(error: ErrorCode) -> CommandResult:
    return CommandResult(False, error)


class Arbiter:
    def __init__(
        self,
        robot_id: str,
        heartbeat_seconds: float,
        disconnect_seconds: float,
        command_ttl_seconds: float,
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
        self._disconnect_seconds = disconnect_seconds
        self._command_ttl_seconds = command_ttl_seconds
        self._lease: _Lease | None = None
        self._estop = False
        self._manual = False
        self._recovery = False
        self._navigation = False
        self._velocity = _ZERO
        self._deadline: float | None = None

    def accept(self, command: Envelope, now: float) -> CommandResult:
        self._expire(now)
        if not math.isfinite(now):
            return _reject(ErrorCode.VALIDATION)
        if command.robot_id != self._robot_id:
            return _reject(ErrorCode.SAFETY_REJECTED)
        if not isinstance(command, (VelocityEnvelope, SimpleEnvelope)):
            return _reject(ErrorCode.VALIDATION)
        lease = self._lease
        if lease is None:
            return self._accept_without_lease(command, now)
        if command.session_id != lease.session_id:
            if (
                command.type == "control.acquire"
                and not lease.alive
                and command.connection_epoch > lease.epoch
            ):
                return self._open_lease(command, now)
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
            if command.type in _SAFETY:
                return self._accept_safety(command, now, advance_epoch=False)
            return _reject(ErrorCode.OFFLINE)
        return self._accept_live(command, now)

    def tick(self, now: float) -> Velocity:
        self._expire(now)
        if self._estop or not self._connected(now):
            return _ZERO
        if self._manual and self._deadline is not None and now <= self._deadline:
            return self._velocity
        return _ZERO

    def disconnect(self, connection_epoch: int) -> None:
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
        self, command: VelocityEnvelope | SimpleEnvelope, now: float
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

    def _accept_live(self, command: VelocityEnvelope | SimpleEnvelope, now: float) -> CommandResult:
        kind = command.type
        if kind == "control.acquire":
            self._refresh(command, now)
            return _ok()
        if kind == "control.release":
            self._refresh(command, now)
            self._manual = False
            self._velocity = _ZERO
            self._deadline = None
            return _ok()
        if kind == "drive.stop":
            if self._estop:
                return _reject(ErrorCode.SAFETY_REJECTED)
            self._refresh(command, now)
            self._hold_manual_stop()
            return _ok()
        if kind == "drive.velocity":
            if self._estop:
                return _reject(ErrorCode.SAFETY_REJECTED)
            if not isinstance(command, VelocityEnvelope):
                return _reject(ErrorCode.VALIDATION)
            self._refresh(command, now)
            self._manual = True
            self._navigation = False
            self._recovery = False
            self._velocity = Velocity(command.payload.vx, command.payload.wz)
            self._deadline = now + self._command_ttl_seconds
            return _ok()
        if kind in _SAFETY:
            self._refresh(command, now)
            return self._apply_safety(kind)
        if kind in _NAV_START:
            if self._estop:
                return _reject(ErrorCode.SAFETY_REJECTED)
            if self._manual:
                return _reject(ErrorCode.CONFLICT)
            self._refresh(command, now)
            self._navigation = True
            self._recovery = False
            return _ok()
        if kind == "navigation.cancel":
            self._refresh(command, now)
            self._navigation = False
            return _ok()
        return _reject(ErrorCode.VALIDATION)

    def _accept_safety(
        self,
        command: VelocityEnvelope | SimpleEnvelope,
        _now: float,
        *,
        advance_epoch: bool,
    ) -> CommandResult:
        """Apply estop or reset. `_now` is ignored so a dead lease stays dead."""
        lease = self._lease
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
        self._velocity = _ZERO
        self._deadline = None

    def _expire(self, now: float) -> None:
        lease = self._lease
        if lease is None or not lease.alive:
            return
        if (
            not math.isfinite(now)
            or now < lease.refreshed_at
            or now - lease.refreshed_at > self._disconnect_seconds
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
            and now - lease.refreshed_at <= self._disconnect_seconds
        )
