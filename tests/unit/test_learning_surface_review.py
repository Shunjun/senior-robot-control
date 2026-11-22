"""Regressions for dense surfaces, circular scans and stable connectivity."""

import math
from dataclasses import replace

import pytest
from learning_test_support import TestRun, geometry, save_completed
from senior_perception.self_filter import ScanObservation, filter_self
from test_learning_frozen_reference import cloud, learn
from test_startup_learning_feasibility import BOUNDARY

from vehicle_core.config import DeviceProfile
from vehicle_core.values import Pose2D


def dense_surface(count=1000):
    step = (math.pi / 2 - 0.3) / (count - 1)
    scan = ScanObservation(
        tuple(0.15 / math.sin(0.3 + i * step) for i in range(count)),
        0.3,
        step,
        0.05,
        20.0,
        10.0,
        "ros",
        Pose2D(0, 0, 0),
        10.0,
        "base_link",
        10.0,
    )
    return filter_self(scan, DeviceProfile(footprint=BOUNDARY))


@pytest.mark.parametrize("count", [1000, 2048])
def test_dense_long_learned_surface_has_no_stride_holes_and_restores_every_reference(
    tmp_path, count
):
    observation = dense_surface(count)
    model = learn([observation] * 12)
    assert len(model.learned) == len(observation.rays)
    for _ in range(10):
        result = model.filter(observation, provisional=False)
        assert all(ray.kind == "self" for ray in result.rays)
    run = TestRun(tmp_path / "dense-surface")
    try:
        client = run.client()
        client.geometry(geometry(BOUNDARY))
        save_completed(run, model.learned)
        restored = run.client()
        assert restored.record.model.points == model.learned
        from senior_perception.learning_store import attempt_path, read_record

        assert read_record(attempt_path(run.path, restored.record.attempt_id)) == restored.record
        assert all(r.kind == "self" for r in restored.view.filter(observation).rays)
    finally:
        run.close()


@pytest.mark.parametrize("inclusive", [False, True])
def test_complete_scan_connects_surface_across_first_last_samples(inclusive):
    count = 720
    step = math.tau / (count - 1 if inclusive else count)
    # Generate real angle/range endpoints, including a consistent duplicate
    # end sample for inclusive scans. The interior seed lives near the end.
    ranges = [float("nan")] * count
    ranges[0], ranges[1] = 0.205, 0.215
    if inclusive:
        ranges[-2], ranges[-1] = 0.195, 0.205
    else:
        ranges[-1] = 0.195
    template = dense_surface().scan
    obs = filter_self(
        replace(template, ranges=tuple(ranges), angle_min=math.pi / 2, angle_increment=step),
        DeviceProfile(footprint=BOUNDARY),
    )
    model = learn([obs] * 12)
    assert len(model.learned) == (4 if inclusive else 3)
    filtered = model.filter(obs, provisional=False)
    assert filtered.rays[1].kind == "self"
    assert all(r.kind == "self" for r in filtered.rays if r.endpoint is not None)


def test_partial_scan_does_not_join_first_last_samples():
    obs = cloud(((0.0, 0.4), None, (0.0, 0.15)))
    model = learn([obs] * 12)
    assert len(model.learned) == 1 and model.learned == ((0.0, 0.15),)


def test_unstable_bridge_disconnects_stable_outside_piece():
    frames = [cloud(((0.0, 0.15), (0.0, 0.3), (0.0, 0.8)))] * 12
    frames[5] = cloud(((0.0, 0.15), (0.0, 0.33), (0.0, 0.8)))
    model = learn(frames)
    assert len(model.learned) == 1
    assert model.learned == ((0.0, 0.15),)


def test_outside_piece_with_another_stable_route_to_interior_is_retained():
    frames = [cloud(((-0.02, 0.195), (-0.005, 0.21), (0.01, 0.21), (0.015, 0.195)))] * 5
    frames[2] = cloud(((-0.02, 0.195), (-0.005, 0.4), (0.01, 0.21), (0.015, 0.195)))
    model = learn(frames)
    assert len(model.learned) == 3
    assert set(model.learned) == {(-0.02, 0.195), (0.01, 0.21), (0.015, 0.195)}


def wire_float32(value):
    import struct

    return struct.unpack("f", struct.pack("f", value))[0]


def wire_seam_observation(count, step, *, inclusive=False):
    ranges = [float("nan")] * count
    ranges[0], ranges[1] = wire_float32(0.205), wire_float32(0.215)
    if inclusive:
        ranges[-2], ranges[-1] = wire_float32(0.195), wire_float32(0.205)
    else:
        ranges[-1] = wire_float32(0.195)
    scan = replace(
        dense_surface().scan,
        ranges=tuple(ranges),
        angle_min=wire_float32(math.pi / 2),
        angle_increment=step,
        range_min=wire_float32(0.05),
        range_max=wire_float32(20),
    )
    return filter_self(scan, DeviceProfile(footprint=BOUNDARY))


@pytest.mark.parametrize("count", [720, 1080, 1440])
@pytest.mark.parametrize("inclusive", [False, True])
def test_wire_float32_full_turn_preserves_seam_connection(count, inclusive):
    step = wire_float32(math.tau / (count - 1 if inclusive else count))
    observation = wire_seam_observation(count, step, inclusive=inclusive)
    model = learn([observation] * 12)
    assert len(model.learned) == (4 if inclusive else 3)
    result = model.filter(observation, provisional=False)
    assert result.rays[1].kind == "self"
    assert all(r.kind == "self" for r in result.rays if r.endpoint is not None)


@pytest.mark.parametrize("count", [720, 1080, 1440])
@pytest.mark.parametrize("partial", ["270_degrees", "one_missing_sample", "beyond_wire_error"])
def test_wire_float32_partial_fov_does_not_join_seam(count, partial):
    full_step = wire_float32(math.tau / count)
    if partial == "270_degrees":
        step = wire_float32(1.5 * math.pi / (count - 1))
    elif partial == "one_missing_sample":
        count -= 1
        step = full_step
    else:
        # One lower adjacent float32 value is beyond the full-turn step's
        # rounding interval, not a permissible extra angular connection gap.
        import struct

        bits = struct.unpack("I", struct.pack("f", full_step))[0]
        step = struct.unpack("f", struct.pack("I", bits - 1))[0]
    observation = wire_seam_observation(count, step)
    model = learn([observation] * 12)
    assert len(model.learned) == 1
    result = model.filter(observation, provisional=False)
    assert result.rays[1].kind == "hit" and result.rays[-1].kind == "self"
