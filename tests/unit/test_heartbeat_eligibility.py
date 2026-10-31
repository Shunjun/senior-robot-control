from senior_control.agent.connection import HeartbeatLedger


class Clock:
    now = 0.01

    def monotonic(self) -> float:
        return self.now


def test_eligibility_never_outlives_confirmed_ros_deadline() -> None:
    clock = Clock()
    ledger = HeartbeatLedger(clock, 1.0)
    ledger.note("robot-a", deadline=0.8)
    clock.now = 0.8
    assert ledger.eligible("robot-a")
    clock.now = 0.81
    assert not ledger.eligible("robot-a")


def test_heartbeat_is_invalid_if_monotonic_clock_moves_backwards() -> None:
    clock = Clock()
    clock.now = 1.0
    ledger = HeartbeatLedger(clock, 1.0)
    ledger.note("robot-a", deadline=1.8)
    clock.now = 0.9
    assert not ledger.eligible("robot-a")
