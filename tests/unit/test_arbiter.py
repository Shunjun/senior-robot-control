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
    assert arbiter.accept(command("control.acquire", seq=6), 0.3).accepted
    assert arbiter.accept(command("navigation.resume", seq=7), 0.3).accepted
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


def test_dead_lease_same_generation_can_be_acquired_again():
    """A lost lease does not require a new vehicle epoch before the same generation drives."""
    arbiter = gate(disconnect_seconds=1.0, command_ttl_seconds=0.3)
    assert arbiter.accept(command("control.acquire", seq=1, epoch=3), 0.0).accepted
    assert arbiter.accept(command("drive.velocity", seq=2, epoch=3, vx=0.2), 0.0).accepted
    assert arbiter.tick(1.0 + 1e-9) == Velocity(0.0, 0.0)
    same = arbiter.accept(command("control.acquire", seq=3, epoch=3), 1.2)
    assert same.accepted
    assert arbiter.tick(2.2 + 1e-9) == Velocity(0.0, 0.0)
    other = arbiter.accept(
        command("control.acquire", seq=1, epoch=3, session="session-2"),
        2.3,
    )
    assert other.accepted
    assert arbiter.accept(
        command("drive.velocity", seq=2, epoch=3, session="session-2", vx=0.15),
        2.3,
    ).accepted
    assert arbiter.tick(2.3) == Velocity(0.15, 0.0)
    older = arbiter.accept(command("control.acquire", seq=1, epoch=2, session="session-3"), 2.4)
    assert older.accepted is False


def test_queued_velocity_does_not_receive_a_fresh_command_ttl():
    """A velocity that already waited past the command TTL must not start a new window."""
    arbiter = gate(disconnect_seconds=5.0, command_ttl_seconds=0.3)
    assert arbiter.accept(command("control.acquire", seq=1), 10.0).accepted
    late = arbiter.accept(
        command("drive.velocity", seq=2, vx=0.4),
        10.5,
        received_at=10.0,
    )
    assert late.accepted is False
    assert arbiter.tick(10.5) == Velocity(0.0, 0.0)
    assert arbiter.tick(10.8) == Velocity(0.0, 0.0)
    fresh = arbiter.accept(
        command("drive.velocity", seq=3, vx=0.2),
        11.0,
        received_at=10.9,
    )
    assert fresh.accepted
    assert arbiter.tick(11.15) == Velocity(0.2, 0.0)
    assert arbiter.tick(11.2 + 1e-9) == Velocity(0.0, 0.0)


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


def test_autonomous_velocity_requires_mode_and_expires_without_resuming():
    arbiter = gate(disconnect_seconds=5.0, command_ttl_seconds=0.3)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("navigation.start", seq=2), 0.0).accepted
    assert not arbiter.accept_autonomous("navigation", Velocity(0.2, 0.0), 0.0)
    assert arbiter.tick(0.1) == Velocity(0.0, 0.0)
    assert arbiter.tick(0.31) == Velocity(0.0, 0.0)
    assert arbiter.accept(command("drive.stop", seq=3), 0.32).accepted
    assert not arbiter.accept_autonomous("navigation", Velocity(0.3, 0.0), 0.33)
    assert arbiter.tick(0.33) == Velocity(0.0, 0.0)


def test_release_allows_new_session_and_rejects_old_replay():
    arbiter = gate()
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("control.release", seq=2), 0.1).accepted
    assert arbiter.accept(command("control.acquire", seq=1, session="other"), 0.2).accepted
    assert not arbiter.accept(command("control.acquire", seq=1), 0.3).accepted


def test_recovery_requires_explicit_transition_and_stops_on_disconnect():
    arbiter = gate()
    assert not arbiter.begin_recovery(0.0)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("navigation.start", seq=2), 0.0).accepted
    assert arbiter.begin_recovery(0.1)
    assert not arbiter.accept_autonomous("navigation", Velocity(0.2, 0), 0.1)
    assert not arbiter.accept_autonomous("recovery", Velocity(-0.1, 0), 0.1)
    assert arbiter.tick(0.1) == Velocity(0, 0)
    arbiter.disconnect(1)
    assert arbiter.tick(0.2) == Velocity(0, 0)
    assert not arbiter.accept_autonomous("recovery", Velocity(-0.1, 0), 0.2)


def test_cancel_and_resume_cannot_replay_previous_autonomous_velocity():
    arbiter = gate()
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("navigation.start", seq=2), 0.0).accepted
    assert not arbiter.accept_autonomous("navigation", Velocity(0.2, 0), 0.0)
    assert arbiter.accept(command("navigation.cancel", seq=3), 0.1).accepted
    assert arbiter.accept(command("navigation.resume", seq=4), 0.2).accepted
    assert arbiter.tick(0.2) == Velocity(0, 0)


def test_unstamped_autonomous_command_cannot_drive_after_resume():
    arbiter = gate()
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("navigation.start", seq=2), 0.0).accepted
    assert arbiter.accept(command("navigation.cancel", seq=3), 0.1).accepted
    assert arbiter.accept(command("navigation.resume", seq=4), 0.2).accepted
    assert not arbiter.accept_autonomous("navigation", Velocity(0.2, 0.0), 0.21)
    assert arbiter.tick(0.21) == Velocity(0, 0)


def test_server_heartbeat_keeps_navigation_lease_live_until_heartbeat_expires():
    arbiter = Arbiter(
        ROBOT,
        heartbeat_seconds=0.2,
        disconnect_seconds=1.0,
        command_ttl_seconds=0.3,
        require_server_heartbeat=True,
    )
    assert arbiter.server_heartbeat(1, 0.0, 0.8)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("navigation.start", seq=2), 0.0).accepted
    assert arbiter.server_heartbeat(1, 0.6, 1.4)
    assert arbiter.server_heartbeat(1, 1.2, 2.0)
    assert arbiter.mode(1.3) == "navigation"
    assert arbiter.mode(2.01) == "idle"
    assert not arbiter.server_heartbeat(1, 2.02, 2.82)
    assert not arbiter.accept(command("navigation.resume", seq=3), 2.02).accepted
    assert arbiter.server_heartbeat(2, 2.1, 2.9)
    assert not arbiter.server_heartbeat(1, 2.2, 3.0)
    assert arbiter.mode(2.2) == "idle"
    assert arbiter.accept(command("control.acquire", seq=1, epoch=2), 2.2).accepted
    assert arbiter.mode(2.2) == "idle"


def test_authenticated_other_session_can_estop_without_taking_lease():
    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(1, 0.0, 0.8)
    assert arbiter.accept(command("control.acquire", seq=1, session="page-a"), 0.0).accepted
    assert arbiter.accept(
        command("drive.velocity", seq=2, session="page-a", vx=0.2), 0.1, received_at=0.1
    ).accepted
    assert arbiter.accept(command("safety.estop", seq=1, session="page-b"), 0.2).accepted
    assert arbiter.mode(0.2) == "estop"
    assert arbiter.tick(0.2) == Velocity(0.0, 0.0)
    assert not arbiter.accept(command("safety.reset", seq=2, session="page-b"), 0.3).accepted
    assert not arbiter.accept(command("safety.estop", seq=1, session="page-b"), 0.3).accepted
    assert not arbiter.accept(command("safety.reset", seq=3, session="page-a"), 0.3).accepted
    assert arbiter.mode(0.3) == "estop"
    assert not arbiter.accept(
        command("safety.reset", seq=4, epoch=0, session="page-a"), 0.4
    ).accepted


def test_reset_queued_before_other_session_estop_cannot_clear_it():
    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(1, 0.0, 0.8)
    assert arbiter.accept(command("control.acquire", seq=1, session="page-a"), 0.0).accepted
    queued = command("safety.reset", seq=2, session="page-a")
    assert arbiter.accept(command("safety.estop", seq=1, session="page-b"), 0.1).accepted
    assert not arbiter.accept(queued, 0.2).accepted
    assert arbiter.mode(0.2) == "estop"


def test_buffered_estop_same_epoch_latches_after_heartbeat_deadline():
    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(1, 0.0, 0.8)
    assert arbiter.accept(command("control.acquire", seq=1, session="a"), 0.0).accepted
    assert arbiter.accept(command("safety.estop", seq=1, session="b"), 0.81).accepted
    assert arbiter.mode(0.81) == "estop"
    assert arbiter.server_heartbeat(2, 0.9, 1.7)
    assert not arbiter.accept(command("safety.estop", seq=2, epoch=1, session="b"), 1.0).accepted


def test_final_command_fence_rejects_old_generation_and_deadline():
    from senior_control.safety.arbiter import FinalCommandFence

    fence = FinalCommandFence()
    fence.set_publisher(b"a")
    assert fence.accept(b"a", "boot-a", 1, 0, 5.0, Velocity(0.0, 0.0), 1.0)
    assert fence.accept(b"a", "boot-a", 1, 1, 2.0, Velocity(0.1, 0.0), 1.1)
    assert fence.usable(1.9)
    assert not fence.usable(2.0)
    assert not fence.accept(b"a", "boot-a", 3, 1, 5.0, Velocity(0.1, 0.0), 2.0)
    assert fence.accept(b"a", "boot-a", 3, 0, 5.0, Velocity(0.0, 0.0), 2.0)
    assert not fence.accept(b"a", "boot-a", 1, 2, 5.0, Velocity(0.1, 0.0), 2.1)
    assert not fence.usable(2.1)
    fence.set_publisher(b"b")
    assert not fence.accept(b"a", "boot-a", 3, 0, 5.0, Velocity(0.0, 0.0), 2.1)
    assert not fence.accept(b"b", "boot-b", 1, 1, 5.0, Velocity(0.1, 0.0), 2.1)
    assert fence.accept(b"b", "boot-b", 1, 0, 5.0, Velocity(0.0, 0.0), 2.1)


def test_final_fence_never_reaccepts_retired_publisher_gid():
    from senior_control.safety.arbiter import FinalCommandFence

    fence = FinalCommandFence()
    fence.set_publisher(b"a")
    assert fence.accept(b"a", "boot-a", 1, 0, 2.0, Velocity(0.0, 0.0), 1.0)
    fence.set_publisher(None)
    fence.set_publisher(b"a")
    assert not fence.accept(b"a", "boot-a", 1, 0, 2.0, Velocity(0.0, 0.0), 1.0)


def test_arbiter_exposes_original_deadline_and_owner_for_final_gate():
    arbiter = gate()
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("drive.velocity", seq=2, vx=0.1), 0.1, received_at=0.0).accepted
    final = arbiter.final_command(0.2)
    assert final == (Velocity(0.1, 0.0), SESSION, 1, 2, 0.3)
    assert arbiter.final_command(0.31) is None


def test_production_velocity_requires_original_nonfuture_receipt():
    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(1, 0.0, 0.8)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert not arbiter.accept(command("drive.velocity", seq=2, vx=0.1), 0.1).accepted
    assert not arbiter.accept(
        command("drive.velocity", seq=3, vx=0.1), 0.1, received_at=0.2
    ).accepted
    assert arbiter.accept(command("drive.velocity", seq=4, vx=0.1), 0.1, received_at=0.0).accepted


def test_final_fence_rejects_retired_boot_on_new_publisher():
    from senior_control.safety.arbiter import FinalCommandFence

    fence = FinalCommandFence()
    fence.set_publisher(b"a")
    assert fence.accept(b"a", "boot-a", 1, 0, 2.0, Velocity(0.0, 0.0), 1.0)
    fence.set_publisher(b"b")
    assert not fence.accept(b"b", "boot-a", 1, 0, 2.0, Velocity(0.0, 0.0), 1.0)


def test_final_fence_cannot_relabel_session_within_generation():
    from senior_control.safety.arbiter import FinalCommandFence

    fence = FinalCommandFence()
    fence.set_publisher(b"a")
    assert fence.accept(b"a", "boot", 1, 0, 0.0, Velocity(0.0, 0.0), 1.0)
    assert fence.accept(b"a", "boot", 1, 2, 2.0, Velocity(0.1, 0.0), 1.1, session="page-a", epoch=1)
    assert not fence.accept(
        b"a", "boot", 1, 3, 2.0, Velocity(0.1, 0.0), 1.2, session="page-b", epoch=1
    )
    assert not fence.accept(
        b"a", "boot", 1, 3, 2.0, Velocity(0.1, 0.0), 1.2, session="page-a", epoch=2
    )


def test_estop_is_accepted_before_first_motion_heartbeat():
    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.accept(command("safety.estop", seq=1, epoch=1), 0.1).accepted
    assert arbiter.mode(0.1) == "estop"


def test_production_arbiter_preserves_absolute_source_deadline():
    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.8, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(1, 0.0, 0.9)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(
        command("drive.velocity", seq=2, vx=0.1), 0.1, local_deadline=0.25
    ).accepted
    assert arbiter.final_command(0.15)[4] == 0.25
    assert arbiter.tick(0.26) == Velocity(0.0, 0.0)


def test_shorter_ros_ttl_starts_when_cloud_issues_drive_challenge():
    from senior_control.errors import ErrorCode

    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.1, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(1, 0.0, 0.8)
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    sample = command("drive.velocity", seq=2, vx=0.1)
    assert isinstance(sample, VelocityEnvelope)
    sample.payload.challenge_id = "cloud-token"
    sample.payload.server_deadline = 100.3

    assert arbiter.accept(sample, 0.05, local_deadline=0.3).accepted
    assert arbiter.tick(0.09) == Velocity(0.1, 0.0)
    assert arbiter.tick(0.11) == Velocity(0.0, 0.0)
    late = sample.model_copy(update={"seq": 3})
    assert arbiter.accept(late, 0.22, local_deadline=0.3).error is ErrorCode.TIMEOUT


def test_same_session_reacquire_retires_old_velocity():
    arbiter = gate()
    assert arbiter.accept(command("control.acquire", seq=1), 0.0).accepted
    assert arbiter.accept(command("drive.velocity", seq=2, vx=0.1), 0.01).accepted
    assert arbiter.final_command(0.02) is not None

    assert arbiter.accept(command("control.acquire", seq=3), 0.03).accepted
    assert arbiter.final_command(0.04) is None
    assert arbiter.accept(command("drive.velocity", seq=4, vx=0.2), 0.05).accepted
    assert arbiter.tick(0.06) == Velocity(0.2, 0.0)
