"""One scan's artificial windows share error; true unknown gaps remain unknown."""

import math

import pytest
from senior_motion_control.constraints import (
    FreePatch,
    NoReturnPolicy,
    PatchKind,
    prepare_perception,
)
from shapely.geometry import Point


def patch(left, right, kind=PatchKind.OBSERVED_FREE):
    return FreePatch(((left, -2), (right, -2), (right, 2), (left, 2)), kind)


def perception(patches, *, error=0.1, policy=NoReturnPolicy.UNKNOWN):
    return prepare_perception(
        tuple(patches),
        ((3, 0),),
        error_m=error,
        stamp_steady=10,
        deadline_steady=11,
        source_epoch="single-scan",
        frame_id="local",
        revision=1,
        forecast_horizon_s=10,
        no_return_policy=policy,
        basis="synthetic one-observation windows",
        max_patches=8,
        max_vertices=64,
        max_hits=8,
    )


@pytest.mark.parametrize("overlap", [0, 0.01, 0.15])
def test_common_error_does_not_cut_an_unknown_seam_between_same_observation_windows(overlap):
    joined = perception((patch(-2, overlap), patch(-overlap, 2)))
    whole = perception((patch(-2, 2),))
    assert joined.free_space.equals(whole.free_space)
    assert joined.free_space.covers(Point(0, 0))
    assert joined.stamp_steady == 10 and joined.deadline_steady == 11
    boundary = 2 - 0.1
    assert joined.free_space.covers(Point(boundary, 0))
    assert joined.free_space.covers(Point(math.nextafter(boundary, -math.inf), 0))
    assert not joined.free_space.covers(Point(math.nextafter(boundary, math.inf), 0))
    assert joined.occupied.intersects(Point(3.1, 0.1))
    assert not joined.occupied.intersects(Point(3.10001, 0))


@pytest.mark.parametrize("gap", [0.001, 0.05, 0.2])
def test_observed_union_never_bridges_a_true_unknown_gap(gap):
    view = perception((patch(-2, -gap / 2), patch(gap / 2, 2)))
    assert not view.free_space.covers(Point(0, 0))
    assert not view.free_space.covers(Point(gap / 2, 0))
    assert view.free_space.covers(Point(-1, 0))
    assert view.free_space.covers(Point(1, 0))


def test_unknown_no_return_window_cannot_join_observed_regions():
    windows = (patch(-2, -0.2), patch(-0.3, 0.3, PatchKind.NO_RETURN), patch(0.2, 2))
    unknown = perception(windows)
    contracted = perception(windows, policy=NoReturnPolicy.VALIDATED_FREE_TO_RANGE)
    assert not unknown.free_space.covers(Point(0, 0))
    assert contracted.free_space.covers(Point(0, 0))
    assert unknown.deadline_steady == contracted.deadline_steady == 11
