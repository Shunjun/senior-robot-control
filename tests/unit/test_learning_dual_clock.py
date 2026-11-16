"""Explicit physical ticks govern learning geometry; host time governs permission."""

from dataclasses import replace

import pytest
from learning_test_support import begin_action, learning_runs, startup_for  # noqa: F401
from source_time_test_support import capture_time
from test_startup_learning import side_observation
from test_startup_learning_feasibility import BOUNDARY

from senior_control.safety.self_learning import SelfReturnModel


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
def test_real_geometry_classification_is_rate_independent_with_ros_corrections(rate, body_echo):
    model = SelfReturnModel(BOUNDARY)
    for i in range(12):
        tick = i / 10
        host = 10 + tick / rate
        raw = side_observation(0.15 if body_echo else 1.0)
        observation = replace(raw, scan=replace(raw.scan, stamp=1000 if i % 2 else 1))
        model.observe(
            observation,
            capture=capture_time(host, sequence=i + 1, tick=tick, world="world"),
            segment_id="segment",
        )
    assert [m.seconds for m in model.frame_measurements] == [i / 10 for i in range(12)]
    assert model.validate() == int(body_echo)
    if body_echo:
        assert model.filter(side_observation(0.15), provisional=False).rays[180].kind == "self"
        assert model.filter(side_observation(1.0), provisional=False).rays[180].kind == "hit"
        assert model.learned


def test_model_late_capture_replay_and_reset_use_physical_axis_and_producer_identity():
    model = SelfReturnModel(BOUNDARY)
    for i in (3, 1, 2):
        assert model.observe(
            side_observation(0.15),
            capture=capture_time(10 + i, sequence=i, tick=i / 10, world="world"),
            segment_id="segment",
        )
    assert [m.seconds for m in model.frame_measurements] == [0.1, 0.2, 0.3]
    assert not model.observe(
        side_observation(0.15),
        capture=capture_time(11, sequence=1, tick=0.1, world="world"),
        segment_id="segment",
    )
    assert not model.observe(
        side_observation(0.15),
        capture=capture_time(14, sequence=4, tick=0, world="reset"),
        segment_id="segment",
    )
    model.begin_segment("reset")
    assert model.observe(
        side_observation(0.15),
        capture=capture_time(14, sequence=1, tick=0, world="reset", epoch="producer-new"),
        segment_id="reset",
    )


def test_scan_producer_restart_and_world_reset_are_separate_segment_changes(tmp_path):
    owner = startup_for(tmp_path / "source")
    owner.advance(0, now=10, stationary=True)
    model = SelfReturnModel(BOUNDARY)
    for host, producer, world, tick in [
        (10, "producer", "world", 0),
        (10.1, "producer-new", "world", 0.1),
        (10.2, "producer-new", "reset", 0),
    ]:
        before = owner.segment_id
        owner.frame(
            model,
            side_observation(0.15),
            capture=capture_time(host, sequence=1, epoch=producer, world=world, tick=tick),
        )
        if host > 10:
            assert owner.segment_id != before and len(model.frames) == 1
    # A delayed capture from the old producer/world cannot roll the segment back.
    current = owner.segment_id
    owner.frame(
        model,
        side_observation(0.15),
        capture=capture_time(10.1, sequence=2, epoch="producer-new", world="world", tick=0.2),
    )
    assert owner.segment_id == current and len(model.frames) == 1


def test_paused_retention_frontier_does_not_reaccept_trimmed_capture_identity():
    model = SelfReturnModel(BOUNDARY)
    for i in range(1, 9):
        assert model.observe(
            side_observation(0.15),
            capture=capture_time(10 + i / 10, sequence=i, tick=0, world="world"),
            segment_id="paused",
        )
    model.retain_recent(5)
    assert len(model.frames) == 5
    assert not model.observe(
        side_observation(0.15),
        capture=capture_time(10.1, sequence=1, tick=0, world="world"),
        segment_id="paused",
    )
    assert len(model.frames) == 5
