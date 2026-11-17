"""Approved discrete connectivity, stable positions and immutable runtime recovery."""

import pickle
from dataclasses import replace

import pytest
from learning_test_support import TestRun, bind_core, geometry, learning_runs  # noqa: F401
from sensor_time_test_support import sample_time
from test_motion_core_handoff import core  # noqa: F401
from test_startup_learning import side_observation
from test_startup_learning_feasibility import BOUNDARY

from senior_control.config import DeviceProfile
from senior_control.safety.self_filter import RayEvidence
from senior_control.safety.self_learning import SelfReturnModel
from senior_control.vision.raw_capture import RawCaptureHeader


def cloud(points):
    base = side_observation(0.15)
    rays = tuple(
        RayEvidence(i, (0, 0), p, "invalid" if p is None else "hit") for i, p in enumerate(points)
    )
    return replace(base, rays=rays, external_hits=tuple(r for r in rays if r.kind == "hit"))


def learn(frames):
    model = SelfReturnModel(BOUNDARY)
    for i, observation in enumerate(frames, 1):
        assert model.observe(
            observation,
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round((10 + i / 100) * 1e9),
                    received_ros_ns=round((10 + i / 100) * 1e9),
                    received_steady_ns=round((10 + i / 100) * 1e9),
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
            segment_id="moving",
        )
    return model


def test_adjacent_valid_returns_extend_outside_without_connection_distance_threshold():
    points = ((0.0, 0.15), (0.0, 0.2), (0.0, 0.8), (0.0, 2.0))
    model = learn([cloud(points)] * 12)
    assert model.validate() == 4
    assert model.learned == points
    assert all(r.kind == "self" for r in model.filter(cloud(points), provisional=False).rays)


def test_invalid_gap_and_no_interior_seed_keep_disconnected_points_external():
    points = ((0.0, 0.15), None, (0.0, 0.8), (0.0, 2.0))
    model = learn([cloud(points)] * 12)
    assert model.validate() == 1
    result = model.filter(cloud(points), provisional=False)
    assert [r.kind for r in result.rays] == ["self", "invalid", "hit", "hit"]
    outside = learn([cloud(((0.0, 0.8), (0.0, 2.0)))] * 12)
    assert outside.validate() == 0


def test_unstable_connected_bridge_excludes_disconnected_outside_point():
    frames = [
        cloud(((0.0, 0.15), (0.0, 0.3 + (0.03 if i == 5 else 0)), (0.0, 0.8))) for i in range(12)
    ]
    model = learn(frames)
    assert model.validate() == 1
    assert model.learned == ((0.0, 0.15),)


def test_neighbor_direction_and_changed_beam_indices_match_same_surface_position():
    first = cloud(((0.0, 0.15), (0.0, 0.35)))
    shifted = cloud(((0.001, 0.35), (0.001, 0.15)))
    model = learn([first, shifted] * 6)
    assert model.validate() == 2
    assert model.learned == ((0.0, 0.15), (0.0, 0.35))


def test_learning_checks_early_instability_even_after_bounded_capture_window_moves():
    frames = [cloud(((0.0, 0.15), (0.0, 0.4)))] * 100
    frames[1] = cloud(((0.0, 0.15), (0.0, 0.44)))
    model = learn(frames)
    assert len(model.frames) == 64
    assert model.validate() == 1 and model.learned == ((0.0, 0.15),)


@pytest.mark.parametrize(
    "offset,expected", [(-0.01, "self"), (0.01, "self"), (-0.01001, "hit"), (0.01001, "hit")]
)
def test_fixed_one_centimeter_boundary_applies_inside_and_outside_body(offset, expected):
    model = SelfReturnModel(BOUNDARY)
    model.learned = ((0.0, 0.4),)
    assert model.filter(cloud(((0.0, 0.4 + offset),)), provisional=False).rays[0].kind == expected
    assert model.learned == ((0.0, 0.4),)


def test_saved_restored_view_survives_missing_mismatched_occluded_frames_without_writes(
    tmp_path, monkeypatch
):
    run = TestRun(tmp_path / "frozen")
    first = run.client()
    try:
        first.geometry(geometry(BOUNDARY))
        assert first.reserve_action()
        first.complete(((0.0, 0.15), (0.0, 0.4)))
        record = first.record
        first.close()
        restored = run.client()
        assert restored.session is None and not restored.reserve_action()
        assert restored.record.model == record.model and restored.record.action.consumed
        frozen_bytes, revision = run.path.read_bytes(), run.owner.record.revision
        view = pickle.loads(pickle.dumps(restored.view))
        from senior_control.runtime import learning_owner

        def unexpected_write(*args, **kwargs):
            raise AssertionError("runtime filtering must never submit a record write")

        monkeypatch.setattr(learning_owner, "write_record", unexpected_write)
        for _ in range(1000):
            for points in ((None,), ((0.0, 0.1),), ((0.0, 1.0),)):
                result = view.filter(cloud(points))
                assert all(r.kind != "self" for r in result.rays)
            run.pump()
        result = view.filter(cloud(((0.0, 0.15), (0.0, 0.4), (0.0, 0.6))))
        assert [r.kind for r in result.rays] == ["self", "self", "hit"]
        assert run.owner.record.revision == revision and run.path.read_bytes() == frozen_bytes
        assert view.catalog == record.model
        # Geometry reports after completion do not grant write capability or replace results.
        restored.geometry(geometry(BOUNDARY, sensor=(1.0, 0.0, 0.0)))
        assert restored.record == record and run.path.read_bytes() == frozen_bytes
    finally:
        run.close()


@pytest.mark.parametrize("changed_range", [0.1, 1.0, float("nan")])
def test_actual_core_returning_echo_is_filtered_after_occlusion(core, tmp_path, changed_range):  # noqa: F811
    from test_continuous_scan_control import scan

    core.node._profile = lambda: DeviceProfile(footprint=BOUNDARY)
    bind_core(core, tmp_path / "core-frozen")
    first = scan(10.0, 1)
    first.message.ranges[540] = 0.15
    core._sync_learning_model(first.message)
    core._learning_client.complete(((0.0, 0.15),))
    core.startup.state = "completed"
    assert core._filtered_scan(first)[1].rays[540].kind == "self"
    before = core._learning_client.run.path.read_bytes()
    for sequence in range(2, 12):
        current = scan(10 + sequence / 20, sequence)
        current.message.ranges[540] = changed_range
        assert core._filtered_scan(current)[1].rays[540].kind != "self"
    returned = scan(11, 12)
    returned.message.ranges[540] = 0.15
    assert core._filtered_scan(returned)[1].rays[540].kind == "self"
    assert core._learning_client.run.path.read_bytes() == before
