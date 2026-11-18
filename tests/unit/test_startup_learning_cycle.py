"""The fixed motion exits before deferred model calculation finishes."""

from test_startup_learning_chain import rig  # noqa: F401

from senior_control.models import Velocity


def test_complete_collection_without_reference_points_reports_zero_honestly(rig):  # noqa: F811
    # Use the existing input fixture's real raw path, with no interior echo.
    original_offer = rig.run.offer

    def offer(raw):
        raw.message.ranges[18] = 2.0
        return original_offer(raw)

    rig.run.offer = offer
    start = rig.start()
    while rig.now + 0.05 < start + 3:
        rig.step(rig.now + 0.05, wheel=0.05)
    assert rig.step(start + 3) == Velocity(0, 0)
    for _ in range(10):
        rig.step(rig.now + 0.05)
    record = rig.run.owner.active_record
    assert record is not None and record.phase == "completed" and record.model.points == ()
    assert rig.core.target.source == "idle" and rig.core.machine.state.task_id is None
