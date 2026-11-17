"""Explicit physical ticks govern learning geometry; host time governs permission."""

from dataclasses import replace

import pytest
from learning_test_support import begin_action, learning_runs, startup_for  # noqa: F401
from sensor_time_test_support import sample_time
from test_startup_learning import side_observation
from test_startup_learning_feasibility import BOUNDARY

from senior_control.safety.self_learning import SelfReturnModel
from senior_control.vision.raw_capture import RawCaptureHeader


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
    model = SelfReturnModel(BOUNDARY)
    for i in range(12):
        tick = i / 10
        host = 10 + tick / rate
        raw = side_observation(0.15 if body_echo else 1.0)
        observation = replace(raw, scan=replace(raw.scan, stamp=tick))
        model.observe(
            observation,
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round((tick) * 1e9),
                    received_ros_ns=round((tick) * 1e9),
                    received_steady_ns=round((host) * 1e9),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=i + 1,
                valid=True,
                reason="",
            ),
            segment_id="segment",
        )
    assert [m / 1e9 for m in model.frame_measurements] == [i / 10 for i in range(12)]
    assert model.validate() == int(body_echo)
    if body_echo:
        assert model.filter(side_observation(0.15), provisional=False).rays[180].kind == "self"
        assert model.filter(side_observation(1.0), provisional=False).rays[180].kind == "hit"
        assert model.learned


def test_model_late_capture_replay_and_clock_generation_preserve_source_identity():
    model = SelfReturnModel(BOUNDARY)
    for i in (3, 1, 2):
        assert model.observe(
            side_observation(0.15),
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round((i / 10) * 1e9),
                    received_ros_ns=round((i / 10) * 1e9),
                    received_steady_ns=round((10 + i) * 1e9),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=i,
                valid=True,
                reason="",
            ),
            segment_id="segment",
        )
    assert [m / 1e9 for m in model.frame_measurements] == [0.1, 0.2, 0.3]
    assert not model.observe(
        side_observation(0.15),
        capture=RawCaptureHeader(
            time=sample_time(
                stamp_ns=round((0.1) * 1e9),
                received_ros_ns=round((0.1) * 1e9),
                received_steady_ns=round((11) * 1e9),
                clock_generation=1,
            ),
            frame_id="laser",
            raw_gid="01",
            bridge_gid="02",
            source_epoch="producer",
            bridge_boot_id="boot",
            capture_generation=1,
            sequence=1,
            valid=True,
            reason="",
        ),
        segment_id="segment",
    )
    assert not model.observe(
        side_observation(0.15),
        capture=RawCaptureHeader(
            time=sample_time(
                stamp_ns=round((0) * 1e9),
                received_ros_ns=round((0) * 1e9),
                received_steady_ns=round((14) * 1e9),
                clock_generation=2,
            ),
            frame_id="laser",
            raw_gid="01",
            bridge_gid="02",
            source_epoch="producer",
            bridge_boot_id="boot",
            capture_generation=2,
            sequence=4,
            valid=True,
            reason="",
        ),
        segment_id="segment",
    )
    model.begin_segment("reset")
    assert model.observe(
        side_observation(0.15),
        capture=RawCaptureHeader(
            time=sample_time(
                stamp_ns=round((0) * 1e9),
                received_ros_ns=round((0) * 1e9),
                received_steady_ns=round((14) * 1e9),
                clock_generation=2,
            ),
            frame_id="laser",
            raw_gid="01",
            bridge_gid="02",
            source_epoch="producer-new",
            bridge_boot_id="boot",
            capture_generation=2,
            sequence=1,
            valid=True,
            reason="",
        ),
        segment_id="reset",
    )


def test_scan_producer_restart_and_clock_rollback_interrupt_segments(tmp_path):
    owner = startup_for(tmp_path / "source")
    owner.advance(0, now=10, stationary=True)
    model = SelfReturnModel(BOUNDARY)
    for host, producer, generation, tick in [
        (10, "producer", 1, 0),
        (10.1, "producer-new", 1, 0.1),
        (10.2, "producer-new", 2, 0),
    ]:
        before = owner.segment_id
        owner.frame(
            model,
            side_observation(0.15),
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round((tick) * 1e9),
                    received_ros_ns=round((tick) * 1e9),
                    received_steady_ns=round((host) * 1e9),
                    clock_generation=generation,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch=producer,
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=1,
                valid=True,
                reason="",
            ),
        )
        if host > 10:
            assert owner.segment_id != before and len(model.frames) == 1
    # A delayed capture from the old producer/world cannot roll the segment back.
    current = owner.segment_id
    owner.frame(
        model,
        side_observation(0.15),
        capture=RawCaptureHeader(
            time=sample_time(
                stamp_ns=round((0.2) * 1e9),
                received_ros_ns=round((0.2) * 1e9),
                received_steady_ns=round((10.1) * 1e9),
                clock_generation=1,
            ),
            frame_id="laser",
            raw_gid="01",
            bridge_gid="02",
            source_epoch="producer-new",
            bridge_boot_id="boot",
            capture_generation=1,
            sequence=2,
            valid=True,
            reason="",
        ),
    )
    assert owner.segment_id == current and len(model.frames) == 1


def test_retention_frontier_does_not_reaccept_trimmed_original_stamp():
    model = SelfReturnModel(BOUNDARY)
    for i in range(1, 9):
        assert model.observe(
            side_observation(0.15),
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round((i / 10) * 1e9),
                    received_ros_ns=round((i / 10) * 1e9),
                    received_steady_ns=round((10 + i / 10) * 1e9),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=i,
                valid=True,
                reason="",
            ),
            segment_id="paused",
        )
    model.retain_recent(5)
    assert len(model.frames) == 5
    assert not model.observe(
        side_observation(0.15),
        capture=RawCaptureHeader(
            time=sample_time(
                stamp_ns=100_000_000,
                received_ros_ns=100_000_000,
                received_steady_ns=round((10.1) * 1e9),
                clock_generation=1,
            ),
            frame_id="laser",
            raw_gid="01",
            bridge_gid="02",
            source_epoch="producer",
            bridge_boot_id="boot",
            capture_generation=1,
            sequence=99,
            valid=True,
            reason="",
        ),
        segment_id="paused",
    )
    assert len(model.frames) == 5
