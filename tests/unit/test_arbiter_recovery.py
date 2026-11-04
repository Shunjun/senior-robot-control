from test_arbiter import ROBOT, command

from senior_control.safety.arbiter import Arbiter


def test_late_heartbeat_cannot_discard_current_epoch_estop():
    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(1, 0.0, 0.8)
    assert arbiter.accept(command("control.acquire", seq=1), 0.1).accepted
    assert not arbiter.server_heartbeat(1, 0.9, 1.7)
    assert arbiter.accept(command("safety.estop", seq=2), 0.9).accepted
    assert arbiter.mode(0.9) == "estop"


def test_estop_rejection_distinguishes_stale_from_not_yet_authorized_epoch():
    from senior_control.errors import ErrorCode

    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    assert arbiter.server_heartbeat(2, 0.0, 0.8)
    assert arbiter.accept(command("control.acquire", seq=5, epoch=2), 0.1).accepted
    assert arbiter.accept(command("safety.estop", seq=4, epoch=2), 0.2).error is ErrorCode.CONFLICT
    assert arbiter.accept(command("safety.estop", seq=6, epoch=1), 0.2).error is ErrorCode.CONFLICT
    assert arbiter.accept(command("safety.estop", seq=1, epoch=3), 0.2).error is ErrorCode.OFFLINE
    assert arbiter.mode(0.2) == "idle"


def test_uninitialized_epoch_cannot_latch_estop():
    from senior_control.errors import ErrorCode

    arbiter = Arbiter(ROBOT, 0.2, 1.0, 0.3, require_server_heartbeat=True)
    result = arbiter.accept(command("safety.estop", seq=1, epoch=0), 0.1)
    assert result.error is ErrorCode.OFFLINE
    assert arbiter.mode(0.1) == "idle"
