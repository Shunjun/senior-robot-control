"""One synthetic S01→S02 episode; thresholds here are NOT deployment evidence.

Run the replay test with pytest --durations=1 as a measurable benchmark entry.
No wall-clock/CPU/RSS acceptance threshold is asserted by this fixture.
"""

import math
from dataclasses import replace

import pytest
from senior_mapping.scan_geometry import GeometryLimits, GeometryProfile, GeometryTracker
from senior_mapping.shared_scan import Beam, QualifiedScan, ScanKey, project_hits
from senior_perception.vision.raw_capture import RawCaptureHeader

from vehicle_core.sample_time import SampleTime
from vehicle_core.values import Pose2D


def synthetic_profile():
    return GeometryProfile(
        profile_id="synthetic-replay-only",
        evidence_digest="0" * 64,
        range_sigma_m=0.001,
        odom_translation_error_m=0.001,
        odom_yaw_error_rad=0.0001,
        max_scan_distortion_m=0.001,
        cluster_gap_m=0.08,
        cluster_gap_per_m=0.01,
        stable_residual_m=0.04,
        motion_residual_m=0.12,
        min_common_fraction=0.8,
        min_independent_frames=3,
        min_span_s=0.2,
        max_gap_s=0.25,
    )


def episode_frame(
    sequence, *, offset=0.0, visible=65, source="source", geometry="mount", wall=False, count=None
):
    """Independent ego/object poses, polar beam projection through exact raw odom.

    Integer capture coordinates and complete source identity survive the replay.
    The isolated front has curvature: a translating line's tangential ambiguity
    cannot stand in for a uniquely associated object surface.
    """
    ego = Pose2D(0.025 * sequence, -0.01 * sequence, 0.002 * sequence)
    points = [
        (3.0 + offset + 0.12 * (i / 64 - 0.5) ** 2, (i / 64 - 0.5) * 0.8) for i in range(visible)
    ]
    if wall:
        points += [(4.5, 1.5 + i * 0.01) for i in range(65)]
    if count is not None:
        points = [(3 + i * 0.00001, -0.4 + i * 0.0004) for i in range(count)]
    local = []
    c, s = math.cos(ego.yaw), math.sin(ego.yaw)
    for x, y in points:
        dx, dy = x - ego.x, y - ego.y
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        angle, distance = math.atan2(ly, lx), math.hypot(lx, ly)
        local.append((angle, distance))
    beams = tuple(
        Beam(
            i,
            (ego.x, ego.y),
            (ego.x + r * math.cos(a + ego.yaw), ego.y + r * math.sin(a + ego.yaw)),
            "hit",
        )
        for i, (a, r) in enumerate(sorted(local))
    )
    ns = 10_000_000_000 + sequence * 100_000_000
    capture = RawCaptureHeader(
        SampleTime(1_700_000_000_000_000_000 + ns, 1_700_000_000_000_000_000 + ns, ns, ns, 1),
        "laser",
        "raw",
        "relay",
        source,
        "bridge",
        1,
        sequence,
        True,
        "",
    )
    return QualifiedScan(
        ScanKey.of("backend", capture), capture, geometry, ego, beams, ns + 500_000_000, "host"
    )


class Replay:
    def __init__(self, limits=None):
        self.now = 0
        self.tracker = GeometryTracker(limits=limits or GeometryLimits(), clock_ns=lambda: self.now)
        self.profile = synthetic_profile()

    def step(self, sequence, *, profile=True, **kwargs):
        scan = episode_frame(sequence, **kwargs)
        before = project_hits(scan, Pose2D(0, 0, 0))
        self.now = scan.capture.time.sample_steady_ns
        result = self.tracker.update(scan, profile=self.profile if profile else None)
        assert project_hits(scan, Pose2D(0, 0, 0)) == before
        assert len(before) == len(scan.beams)
        return result


def stable_prefix(replay, *, wall=False):
    for seq in range(1, 4):
        absent = replay.step(seq, profile=False, wall=wall)
        assert not absent.stable and absent.unresolved_reason == "profile_missing"
    assert not replay.step(4, wall=wall).stable
    assert not replay.step(5, wall=wall).stable
    return replay.step(6, wall=wall)


def test_ego_translation_preserves_isolated_stability():
    replay = Replay()
    result = stable_prefix(replay)
    assert len(result.stable) == 1
    contour = result.stable[0]
    assert contour.first_key == episode_frame(4).key
    assert contour.last_key == episode_frame(6).key
    assert contour.beam_indices == tuple(range(65))
    assert len(contour.odom_surface) == 65  # 32 samples must never truncate support
    assert contour.common_fraction >= replay.profile.min_common_fraction
    assert contour.max_residual_m <= replay.profile.stable_residual_m


def test_motion_revokes_bound_support_and_requires_new_stability_window():
    replay = Replay()
    initial = stable_prefix(replay, wall=True)
    assert len(initial.stable) == 2
    obj, wall = sorted(initial.stable, key=lambda item: item.odom_surface[0][1])
    replay.tracker.bind_support(obj.local_track_id, ("object-support",))
    replay.tracker.bind_support(wall.local_track_id, ("wall-support",))
    revoked = []
    for seq, offset in ((7, 0.2), (8, 0.4), (9, 0.6), (10, 0.8)):
        evidence = replay.step(seq, offset=offset, wall=True)
        revoked.extend(evidence.moved_support_ids)
        assert all("object-support" not in c.support_ids for c in evidence.stable)
    assert revoked == ["object-support"]
    assert not any(c.odom_surface[0][1] < 1 for c in replay.step(11, offset=0.8, wall=True).stable)
    settled = replay.step(12, offset=0.8, wall=True)
    obj = [c for c in settled.stable if c.odom_surface[0][1] < 1]
    assert len(obj) == 1 and obj[0].first_key.sequence == 10
    assert obj[0].support_ids == ()
    replay.tracker.bind_support(obj[0].local_track_id, ("new-object-support",))
    # Same continuous episode: insufficient surface loses association, never revokes wall.
    occluded = replay.step(13, offset=0.8, visible=12, wall=True)
    assert occluded.unresolved_reason is not None
    assert not occluded.moved_support_ids
    assert not any(c.odom_surface[0][1] < 1 for c in occluded.stable)
    assert not any(c.odom_surface[0][1] < 1 for c in replay.step(14, offset=0.8, wall=True).stable)
    assert not any(c.odom_surface[0][1] < 1 for c in replay.step(15, offset=0.8, wall=True).stable)
    assert any(c.odom_surface[0][1] < 1 for c in replay.step(16, offset=0.8, wall=True).stable)
    duplicate = replay.step(16, offset=0.8, wall=True)
    assert not duplicate.stable and not duplicate.moved_support_ids
    # Source gap restarts candidate; owner calls reset for R (R is absent from C1 scan).
    assert not replay.step(18, offset=0.8, wall=True).stable
    assert not replay.step(19, offset=0.8, wall=True).stable
    assert replay.step(20, offset=0.8, wall=True).stable
    replay.tracker.reset()
    assert not replay.step(21, offset=0.8, wall=True).stable
    assert not replay.step(22, offset=0.8, wall=True).stable
    assert replay.step(23, offset=0.8, wall=True).stable
    assert not replay.step(24, offset=0.8, wall=True, source="new-source").stable
    assert not replay.step(
        25, offset=0.8, wall=True, source="new-source", geometry="new-mount"
    ).stable
    scan = episode_frame(26, offset=0.8, wall=True, source="new-source", geometry="new-mount")
    replay.now = scan.valid_until_steady_ns
    expired = replay.tracker.update(scan, profile=replay.profile)
    assert expired.unresolved_reason == "expired" and not expired.stable
    # Continue this episode after expiry, then split the observed surface with
    # one genuinely absent beam. Loss of association cannot withdraw support.
    replay.step(27, offset=0.8, wall=True)
    replay.step(28, offset=0.8, wall=True)
    recovered = replay.step(29, offset=0.8, wall=True)
    obj = next(c for c in recovered.stable if c.odom_surface[0][1] < 1)
    replay.tracker.bind_support(obj.local_track_id, ("post-expiry-support",))
    split = episode_frame(30, offset=0.8, wall=True)
    beams = list(split.beams)
    beams[32] = replace(beams[32], kind="invalid", endpoint_odom=None)
    split = replace(split, beams=tuple(beams))
    before = project_hits(split, Pose2D(0, 0, 0))
    replay.now = split.capture.time.sample_steady_ns
    ambiguous = replay.tracker.update(split, profile=replay.profile)
    assert ambiguous.unresolved_reason is not None and not ambiguous.moved_support_ids
    assert not any(c.odom_surface[0][1] < 1 for c in ambiguous.stable)
    assert project_hits(split, Pose2D(0, 0, 0)) == before and len(before) == 129
    for seq in (31, 32):
        assert not any(
            c.odom_surface[0][1] < 1 for c in replay.step(seq, offset=0.8, wall=True).stable
        )
    assert any(c.odom_surface[0][1] < 1 for c in replay.step(33, offset=0.8, wall=True).stable)
    # A prolonged one-beam occlusion cannot become centroid-only evidence.
    for seq in (34, 35, 36):
        point_only = replay.step(seq, offset=0.8, visible=1, wall=True)
        assert not any(c.odom_surface[0][1] < 1 for c in point_only.stable)
        assert not point_only.moved_support_ids


def test_capacity_never_truncates_current_hits():
    replay = Replay(GeometryLimits(max_clusters=1, max_tracks=1))
    full = replay.step(1, count=2048)
    assert not full.stable
    assert not replay.step(2, count=2048).stable
    full = replay.step(3, count=2048)
    assert len(full.stable[0].odom_surface) == 2048
    limited = replay.step(4, wall=True)
    assert limited.unresolved_reason == "cluster_capacity" and not limited.stable
    crowded = Replay(GeometryLimits(max_candidates=1))
    crowded.step(1, wall=True)
    saturated = crowded.step(2, wall=True)
    assert saturated.unresolved_reason == "candidate_capacity"
    assert not saturated.stable and not saturated.moved_support_ids


def test_profile_rejects_unusable_evidence_and_capacity():
    profile = synthetic_profile()
    for change in (
        {"evidence_digest": "none"},
        {"profile_id": ""},
        {"range_sigma_m": float("nan")},
        {"odom_yaw_error_rad": -1},
        {"min_common_fraction": 0},
        {"min_common_fraction": 1.1},
        {"motion_residual_m": 0.04},
        {"min_independent_frames": True},
        {"min_independent_frames": 1},
        {"max_gap_s": 0},
    ):
        with pytest.raises(ValueError):
            replace(profile, **change)
    with pytest.raises(ValueError):
        GeometryLimits(max_candidates=9)
    replay = Replay(GeometryLimits(max_frames=2))
    assert replay.step(1).unresolved_reason == "profile_exceeds_capacity"
