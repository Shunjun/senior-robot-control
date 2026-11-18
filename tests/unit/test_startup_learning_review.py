"""Current lifetime clock regressions; old synchronous geometry/cache APIs are retired."""

from learning_test_support import begin_action, learning_runs, startup_for  # noqa: F401


def test_initialization_wait_does_not_consume_learning_budget(tmp_path):
    owner = startup_for(tmp_path / "wait")
    owner.advance(None, now=0, stationary=False)
    owner.advance(None, now=100, stationary=False)
    assert owner.state == "initializing" and owner.started is None
    owner.advance(0.0, now=100.1, stationary=True)
    assert owner.state == "sampling" and not owner.prepare_action()
    begin_action(owner, 100.5)
    assert owner.started == 100.5 and owner.deadline == 103.5


def test_unstarted_action_cannot_validate_or_reopen_after_hold(tmp_path):
    owner = startup_for(tmp_path / "reserved")
    owner.state = "moving"
    owner.advance(0.0, now=10.0, stationary=True)
    assert owner.prepare_action()
    action = owner.session.record.action
    owner.hold_motion("learning_rear_occluded")
    owner.advance(0.0, now=11.0, stationary=True)
    assert owner.state == "interrupted" and owner.started is None
    assert owner.session.record.action == action
    assert owner.session.record.model is None


def test_post_stop_validation_does_not_count_a_frame_observed_while_wheels_move(tmp_path):
    from learning_test_support import TestRun
    from test_learning_compute import opened, recorded, send

    from senior_control.runtime.learning_wire import MotionObservedThrough

    run = TestRun(tmp_path / "stopped-frames")
    try:
        session = opened(run)
        motion = [recorded(i, 10 + i / 10) for i in range(1, 25)]
        validation = [recorded(25 + i, 13.1 + i / 10) for i in range(5)]
        send(run, motion + validation)

        def observed(through, stationary):
            run.rpc(
                MotionObservedThrough(
                    session=session,
                    attempt_id=run.owner.record.attempt_id,
                    segment_id="3" * 32,
                    through_steady_ns=round(through * 1e9),
                    validation_after_steady_ns=13_000_000_000,
                    stationary=stationary,
                )
            )

        observed(13.3, True)
        observed(13.4, False)
        observed(13.5, True)
        assert run.owner.record.capture_summary.state == "open"
        send(run, [recorded(30, 13.6)])
        observed(13.7, True)
        assert run.owner.record.capture_summary.state == "complete"
    finally:
        run.close()
