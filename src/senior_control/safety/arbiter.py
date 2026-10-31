"""Per-robot lease, heartbeat, and command arbitration.

Motion uses a monotonic deadline of the local receive time plus the command TTL.
``expires_at`` is not compared with that clock. A lease that has already died can be
acquired again on the same connection generation. A latched estop outranks
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
        require_server_heartbeat: bool = False,
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
        self._estop = False
        self._manual = False
        self._recovery = False
        self._navigation = False
        self._velocity = _ZERO
        self._deadline: float | None = None

    def accept(
        self, command: Envelope, now: float, *, received_at: float | None = None
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
                return _reject(ErrorCode.OFFLINE)
            if command.connection_epoch < self._server_epoch:
                return _reject(ErrorCode.CONFLICT)
            if command.connection_epoch != self._server_epoch:
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
        return self._accept_live(command, now, received_at)

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
        if self._estop or not self._connected(now):
            return _ZERO
        if (
            (self._manual or self._navigation or self._recovery)
            and self._deadline is not None
            and now <= self._deadline
        ):
            return self._velocity
        return _ZERO

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
    ) -> CommandResult:
        kind = command.type
        if kind == "control.acquire":
            self._refresh(command, now)
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
            self._refresh(command, now)
            self._hold_manual_stop()
            return _ok()
        if kind == "drive.velocity":
            return self._accept_velocity(command, now, received_at)
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
    ) -> CommandResult:
        if self._estop:
            return _reject(ErrorCode.SAFETY_REJECTED)
        if not isinstance(command, VelocityEnvelope):
            return _reject(ErrorCode.VALIDATION)
        received = now if received_at is None else received_at
        if not math.isfinite(received):
            return _reject(ErrorCode.VALIDATION)
        deadline = received + self._command_ttl_seconds
        # A command that already used up its TTL while queued does not start another window.
        if now > deadline:
            return _reject(ErrorCode.TIMEOUT)
        self._refresh(command, now)
        self._manual = True
        self._navigation = False
        self._recovery = False
        self._velocity = Velocity(command.payload.vx, command.payload.wz)
        self._deadline = deadline
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
        self._velocity = _ZERO
        self._deadline = None

    def _expire(self, now: float) -> None:
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
