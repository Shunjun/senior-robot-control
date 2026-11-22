"""Explicit physical ticks govern learning geometry; host time governs permission."""

from dataclasses import replace

import pytest
from learning_test_support import begin_action, learning_runs, startup_for  # noqa: F401


def test_pause_tick_zero_has_no_motion_and_does_not_extend_three_host_seconds(tmp_path):

    owner = startup_for(tmp_path / "paused")
    begin_action(owner, 10)
    for i in range(31):
        host = 10 + i / 10
        owner.advance(0.05, now=host, stationary=False)
    assert owner.started == 10 and owner.deadline == 13
    assert owner.started == 10 and owner.deadline == 13 and owner.velocity == 0
    assert owner.state == "braking" and owner.state != "completed"
    owner.advance(0, now=20, stationary=True)
    assert owner.deadline == 13 and not owner.prepare_action()


@pytest.mark.parametrize("rate", [0.5, 1, 2])
@pytest.mark.parametrize("body_echo", [True, False])
def test_geometry_classification_uses_original_stamps_at_test_receipt_rates(rate, body_echo):
    from senior_perception.learning_compute import ComputeRequest, compute_samples
    from test_learning_compute import recorded

    frames = []
    for i in range(5):
        source = 10 + i / 10
        frame = recorded(i + 1, source, received=10 + i / (10 * rate))
        ranges = list(frame.ranges)
        ranges[18] = 0.15 if body_echo else 1.0
        frames.append(frame.model_copy(update={"ranges": tuple(ranges)}))
    result = compute_samples(ComputeRequest("a" * 32, frames[0].geometry, tuple(frames)))
    assert len(result.points) == int(body_echo)
    assert [f.capture.stamp_ns / 1e9 for f in frames] == [10 + i / 10 for i in range(5)]


def test_selected_samples_reject_duplicate_identity_and_clock_generation_changes():
    from senior_perception.learning_compute import ComputeRequest, compute_samples
    from test_learning_compute import recorded

    frames = tuple(recorded(i + 1, 10 + i / 10) for i in range(5))
    with pytest.raises(ValueError, match="learning_sample_identity"):
        compute_samples(ComputeRequest("a" * 32, frames[0].geometry, (frames[0],) * 5))
    last = frames[-1]
    changed_time = replace(last.capture.time, clock_generation=2)
    changed = last.model_copy(update={"capture": replace(last.capture, time=changed_time)})
    with pytest.raises(ValueError, match="learning_sample_identity"):
        compute_samples(ComputeRequest("a" * 32, frames[0].geometry, (*frames[:-1], changed)))
