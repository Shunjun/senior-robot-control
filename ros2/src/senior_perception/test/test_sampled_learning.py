"""Five-frame sampling through the real service, classifier and frozen filter."""

import math

import pytest
from learning_test_support import TestRun, geometry
from senior_perception.learning_compute import ComputeRequest, compute_samples, decode_observation
from senior_perception.learning_service import select_motion_frames
from test_learning_compute import opened, recorded, send
from test_startup_learning_feasibility import BOUNDARY

from senior_control.safety.self_learning import SelfReturnModel


def test_sampling_uses_five_distinct_temporal_quartiles_inside_motion():
    frames = tuple(
        recorded(i, stamp)
        for i, stamp in enumerate((9.9, 10.1, 10.2, 10.85, 11.5, 12.15, 12.85, 12.9, 13.1), 1)
    )
    selected = select_motion_frames(frames, 10_000_000_000, 13_000_000_000)
    assert [frame.capture.sequence for frame in selected] == [2, 4, 5, 6, 8]
    assert all(frame is frames[frame.capture.sequence - 1] for frame in selected)
    with pytest.raises(ValueError, match="learning_samples_insufficient"):
        select_motion_frames(frames[:5], 10_000_000_000, 13_000_000_000)


def test_service_computes_only_five_after_capture_without_changing_action(tmp_path):
    run = TestRun(tmp_path / "sampled-service")
    try:
        opened(run)
        motion = [recorded(i, 10 + i / 10) for i in range(1, 30)]
        send(run, motion)
        action = run.owner.record.action
        run.pump()
        assert run.owner.record.capture_summary.state == "open"
        assert run.service.compute.flight is None
        assert not run.service.compute.requests
        run.service.motion_through = 13_600_000_000
        run.service._stationary = True
        send(run, [recorded(30 + i, 13.1 + i / 10) for i in range(5)])
        assert run.owner.record.capture_summary.state == "complete"
        run.service.compute.held = False
        run.pump()
        run.pump()  # Settle the existing record-then-manifest durable commit sequence.
        assert len(run.service.compute.requests) == 1
        samples = run.service.compute.requests[0].frames
        assert len(samples) == 5
        assert [frame.capture.stamp_ns / 1e9 for frame in samples] == [
            10.1,
            10.8,
            11.5,
            12.2,
            12.9,
        ]
        assert run.owner.active_record is not None
        assert len(run.owner.active_record.model.points) == 1
        assert run.owner.record.capture_summary.compute_complete
        assert run.owner.record.action == action
    finally:
        run.close()


def test_sampled_surface_keeps_attached_extension_and_excludes_separate_wall():
    frames = []
    for i in range(5):
        raw = recorded(i + 1, 10.1 + 0.7 * i)
        # A short sampled surface extends continuously from inside to outside
        # the footprint. A far wall at the next beam is spatially disconnected.
        ranges = tuple(0.15 + 0.008 * beam for beam in range(25))
        ranges += (2.0, 2.0, 2.0, "inf", 1.0 + 0.03 * i)
        frames.append(
            raw.model_copy(
                update={
                    "ranges": ranges,
                    "geometry": geometry(BOUNDARY, count=30, step=0.005, range_max=20.0),
                }
            )
        )
    result = compute_samples(ComputeRequest("a" * 32, frames[0].geometry, tuple(frames)))
    assert len(result.points) == 25
    model = SelfReturnModel(BOUNDARY)
    model.learned = result.points
    assert any(x > 0.3 for x, _ in result.points)
    frozen = model.learned
    filtered = model.filter(decode_observation(frames[0]), provisional=False)
    assert all(ray.kind == "self" for ray in filtered.rays[:25])
    assert all(ray.kind == "hit" for ray in filtered.rays[25:28])
    assert filtered.rays[29].kind == "hit"
    assert math.isinf(float(frames[0].ranges[28]))
    assert model.learned == frozen
