"""Ownership, heartbeat, and command-lifetime rules for one robot.

Remote ``expires_at`` is not a monotonic deadline. Callers pass ``now``.
"""

from senior_control.models import Velocity
from senior_control.protocol import EmptyPayload, SimpleEnvelope, VelocityEnvelope, VelocityPayload
from senior_control.safety.arbiter import Arbiter

ROBOT = "robot-1"
SESSION = "session-1"


def gate(**overrides: float) -> Arbiter:
    values = {
        "heartbeat_seconds": 0.2,
        "disconnect_seconds": 5.0,
        "command_ttl_seconds": 0.3,
    }
    values.update(overrides)
    return Arbiter(ROBOT, **values)  # type: ignore[arg-type]


def command(
    kind: str,
    *,
    seq: int,
    epoch: int = 1,
    session: str = SESSION,
    robot: str = ROBOT,
    vx: float = 0.0,
    wz: float = 0.0,
    expires_at: float = 0.0,
) -> SimpleEnvelope | VelocityEnvelope:
    common = {
        "v": 1,
        "robot_id": robot,
        "session_id": session,
        "connection_epoch": epoch,
        "request_id": f"req-{epoch}-{seq}",
        "seq": seq,
        "expires_at": expires_at,
    }
    if kind == "drive.velocity":
        return VelocityEnvelope(
            type="drive.velocity",
            payload=VelocityPayload(vx=vx, wz=wz),
            **common,  # type: ignore[arg-type]
        )
    return SimpleEnvelope(type=kind, payload=EmptyPayload(), **common)  # type: ignore[arg-type]


def test_half_open_connection_expires():
    """Breaks if a silent lease keeps the last nonzero velocity past disconnect_seconds."""
    arbiter = gate(heartbeat_seconds=0.2, disconnect_seconds=1.0, command_ttl_seconds=5.0)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("drive.velocity", seq=2, vx=0.35), 0.0).accepted
    assert arbiter.tick(0.2) == Velocity(0.35, 0.0)
    assert arbiter.tick(1.0) == Velocity(0.35, 0.0)
    assert arbiter.tick(1.0 + 1e-9) == Velocity(0.0, 0.0)
    stale = arbiter.accept(command("drive.velocity", seq=3, vx=0.8), 1.1)
    assert stale.accepted is False
    assert arbiter.tick(1.1) == Velocity(0.0, 0.0)
    assert arbiter.accept(command("control.acquire", seq=1, epoch=2), 1.2).accepted
    assert arbiter.tick(1.2) == Velocity(0.0, 0.0)


def test_old_epoch_rejected_after_reconnect():
    """Breaks if a reconnect applies, queues, or is cancelled by an older epoch."""
    arbiter = gate(disconnect_seconds=5.0, command_ttl_seconds=5.0)
    assert arbiter.accept(command("control.acquire", seq=1, epoch=1), 0.0).accepted
    assert arbiter.accept(command("drive.velocity", seq=2, epoch=1, vx=0.4), 0.0).accepted
    assert arbiter.tick(0.1) == Velocity(0.4, 0.0)
    assert arbiter.accept(command("control.acquire", seq=1, epoch=2), 0.2).accepted
    assert arbiter.tick(0.2) == Velocity(0.0, 0.0)
    old = arbiter.accept(command("drive.velocity", seq=3, epoch=1, vx=0.8), 0.3)
    assert old.accepted is False
    assert arbiter.tick(0.3) == Velocity(0.0, 0.0)
    assert arbiter.accept(command("drive.velocity", seq=2, epoch=2, vx=0.2, wz=0.05), 0.3).accepted
    assert arbiter.tick(0.3) == Velocity(0.2, 0.05)
    arbiter.disconnect(1)
    assert arbiter.tick(0.4) == Velocity(0.2, 0.05)
    arbiter.disconnect(2)
    assert arbiter.tick(0.4) == Velocity(0.0, 0.0)
    queued = arbiter.accept(command("drive.velocity", seq=4, epoch=1, vx=0.7), 0.5)
    assert queued.accepted is False
    assert arbiter.tick(0.5) == Velocity(0.0, 0.0)


def test_estop_latches():
    """Breaks if estop clears itself, or if reset on a dead lease produces motion."""
    arbiter = gate(disconnect_seconds=1.0, command_ttl_seconds=4.0)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("drive.velocity", seq=2, vx=0.5), 0.0).accepted
    assert arbiter.tick(0.25) == Velocity(0.5, 0.0)
    assert arbiter.accept(command("safety.estop", seq=3), 0.25).accepted
    assert arbiter.mode(0.25) == "estop"
    assert arbiter.tick(0.25) == Velocity(0.0, 0.0)
    blocked = arbiter.accept(command("drive.velocity", seq=4, vx=0.7), 0.5)
    assert blocked.accepted is False
    assert arbiter.tick(0.5) == Velocity(0.0, 0.0)
    assert arbiter.accept(command("safety.reset", seq=5), 0.5).accepted
    assert arbiter.mode(0.5) == "idle"
    assert arbiter.tick(0.5) == Velocity(0.0, 0.0)
    assert arbiter.accept(command("drive.velocity", seq=6, vx=0.3), 0.5).accepted
    assert arbiter.tick(0.5) == Velocity(0.3, 0.0)

    assert arbiter.accept(command("safety.estop", seq=7), 0.5).accepted
    assert arbiter.tick(1.5 + 1e-9) == Velocity(0.0, 0.0)
    assert arbiter.mode(1.5 + 1e-9) == "estop"
    assert arbiter.accept(command("safety.reset", seq=8), 1.75).accepted
    assert arbiter.mode(1.75) == "idle"
    assert arbiter.tick(1.75) == Velocity(0.0, 0.0)
    revived = arbiter.accept(command("drive.velocity", seq=9, vx=0.6), 1.75)
    assert revived.accepted is False
    assert arbiter.tick(1.75) == Velocity(0.0, 0.0)
    assert arbiter.accept(command("control.acquire", seq=1, epoch=2), 2.0).accepted
    assert arbiter.tick(2.0) == Velocity(0.0, 0.0)
    assert arbiter.accept(command("drive.velocity", seq=2, epoch=2, vx=0.15), 2.0).accepted
    assert arbiter.tick(2.0) == Velocity(0.15, 0.0)


def test_manual_takeover_stops_nav():
    """Breaks if manual release or a rejected nav command resumes navigation."""
    arbiter = gate(disconnect_seconds=5.0, command_ttl_seconds=5.0)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("navigation.start", seq=2), 0.0).accepted
    assert arbiter.mode(0.0) == "navigation"
    assert arbiter.tick(0.0) == Velocity(0.0, 0.0)
    assert arbiter.accept(command("drive.velocity", seq=3, vx=0.4, wz=0.2), 0.1).accepted
    assert arbiter.mode(0.1) == "manual"
    assert arbiter.tick(0.1) == Velocity(0.4, 0.2)
    refused = arbiter.accept(command("navigation.start", seq=4), 0.15)
    assert refused.accepted is False
    assert arbiter.mode(0.15) == "manual"
    assert arbiter.accept(command("control.release", seq=5), 0.2).accepted
    assert arbiter.mode(0.2) == "idle"
    assert arbiter.tick(0.2) == Velocity(0.0, 0.0)
    assert arbiter.accept(command("navigation.resume", seq=6), 0.3).accepted
    assert arbiter.mode(0.3) == "navigation"
    assert arbiter.tick(0.3) == Velocity(0.0, 0.0)


def test_sim_pause_expires_command():
    """Breaks if a frozen remote timestamp keeps a command alive as monotonic time advances."""
    arbiter = gate(disconnect_seconds=5.0, command_ttl_seconds=0.5)
    frozen_remote_time = 42.0
    assert arbiter.accept(
        command("control.acquire", seq=1, expires_at=frozen_remote_time),
        1000.0,
    ).accepted
    accepted = arbiter.accept(
        command("drive.velocity", seq=2, vx=0.25, wz=-0.1, expires_at=frozen_remote_time),
        1000.0,
    )
    assert accepted.accepted
    assert arbiter.tick(1000.0) == Velocity(0.25, -0.1)
    assert arbiter.tick(1000.5) == Velocity(0.25, -0.1)
    assert arbiter.tick(1000.5 + 1e-9) == Velocity(0.0, 0.0)


def test_delayed_same_epoch_command_expired():
    """Breaks if a delayed older sequence on the same epoch receives a new validity window."""
    arbiter = gate(disconnect_seconds=5.0, command_ttl_seconds=0.5)
    assert arbiter.accept(command("control.acquire", seq=1), 10.0).accepted
    assert arbiter.accept(
        command("drive.velocity", seq=2, vx=0.4, expires_at=0.0),
        10.0,
    ).accepted
    delayed = arbiter.accept(
        command("drive.velocity", seq=1, vx=1.0, wz=0.5, expires_at=10_000.0),
        10.25,
    )
    assert delayed.accepted is False
    assert arbiter.tick(10.25) == Velocity(0.4, 0.0)
    assert arbiter.tick(10.5) == Velocity(0.4, 0.0)
    assert arbiter.tick(10.5 + 1e-9) == Velocity(0.0, 0.0)
    replay = arbiter.accept(command("drive.velocity", seq=2, vx=0.4), 11.0)
    assert replay.accepted is False
    assert arbiter.tick(11.0) == Velocity(0.0, 0.0)
    assert arbiter.tick(11.5) == Velocity(0.0, 0.0)


def test_newer_command_uses_accept_time_deadline():
    """Breaks if a newer sequence keeps the previous deadline instead of now + ttl."""
    arbiter = gate(disconnect_seconds=5.0, command_ttl_seconds=0.5)
    assert arbiter.accept(command("control.acquire", seq=1), 10.0).accepted
    assert arbiter.accept(command("drive.velocity", seq=2, vx=0.2), 10.0).accepted
    assert arbiter.accept(command("drive.velocity", seq=3, vx=0.5, wz=-0.2), 10.25).accepted
    assert arbiter.tick(10.75) == Velocity(0.5, -0.2)
    assert arbiter.tick(10.75 + 1e-9) == Velocity(0.0, 0.0)


def test_wrong_robot_session_and_sequence_rejected():
    """Breaks if a bad owner or a non-increasing sequence replaces the live command."""
    arbiter = gate(disconnect_seconds=5.0, command_ttl_seconds=5.0)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("drive.velocity", seq=2, vx=0.3), 0.0).accepted
    wrong_robot = arbiter.accept(command("drive.velocity", seq=3, vx=0.9, robot="other"), 0.1)
    assert wrong_robot.accepted is False
    assert arbiter.tick(0.1) == Velocity(0.3, 0.0)
    wrong_session = arbiter.accept(command("drive.velocity", seq=3, vx=0.9, session="other"), 0.1)
    assert wrong_session.accepted is False
    assert arbiter.tick(0.1) == Velocity(0.3, 0.0)
    stale_seq = arbiter.accept(command("drive.velocity", seq=2, vx=0.9), 0.1)
    assert stale_seq.accepted is False
    assert arbiter.tick(0.1) == Velocity(0.3, 0.0)
    assert arbiter.accept(command("drive.velocity", seq=3, vx=0.6), 0.1).accepted
    assert arbiter.tick(0.1) == Velocity(0.6, 0.0)
