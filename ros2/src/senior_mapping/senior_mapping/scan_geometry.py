"""Bounded odom-surface evidence, without map state or persistent identities.

QualifiedScan already contains endpoints compensated by the trusted original odom.
Map/anchor corrections never enter this module. The caller must reset on reference
changes or invalid/retired input and serialize update/reset/bind_support calls.
A profile is an explicit evidence claim supplied by the owner, not a deployment
certificate: this module cannot authenticate a digest or measure odom accuracy.
"""

import math
import re
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass

from senior_perception.vision.raw_capture import RawCaptureHeader

from .shared_scan import Beam, QualifiedScan, ScanKey

Point = tuple[float, float]


@dataclass(frozen=True)
class GeometryProfile:
    profile_id: str
    evidence_digest: str
    range_sigma_m: float
    odom_translation_error_m: float
    odom_yaw_error_rad: float
    max_scan_distortion_m: float
    cluster_gap_m: float
    cluster_gap_per_m: float
    stable_residual_m: float
    motion_residual_m: float
    min_common_fraction: float
    min_independent_frames: int
    min_span_s: float
    max_gap_s: float

    def __post_init__(self) -> None:
        if (
            not isinstance(self.profile_id, str)
            or not self.profile_id.strip()
            or len(self.profile_id.encode("utf-8")) > 128
            or not isinstance(self.evidence_digest, str)
            or re.fullmatch(r"[0-9a-f]{64}", self.evidence_digest) is None
        ):
            raise ValueError("profile requires identity and evidence SHA256")
        for name in (
            "range_sigma_m",
            "odom_translation_error_m",
            "odom_yaw_error_rad",
            "max_scan_distortion_m",
            "cluster_gap_m",
            "cluster_gap_per_m",
            "stable_residual_m",
            "motion_residual_m",
            "min_common_fraction",
            "min_span_s",
            "max_gap_s",
        ):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f"invalid geometry profile {name}")
        if (
            not 0 < self.min_common_fraction <= 1
            or not 0 < self.stable_residual_m < self.motion_residual_m
            or self.max_gap_s <= 0
            or self.min_span_s <= 0
            or type(self.min_independent_frames) is not int
            or self.min_independent_frames < 2
        ):
            raise ValueError("invalid geometry separation or independent window")


@dataclass(frozen=True)
class GeometryLimits:
    """Engineering capacities, NOT physical reliability or performance promises."""

    max_frames: int = 8
    max_clusters: int = 256
    max_tracks: int = 256
    max_candidates: int = 8
    samples_per_contour: int = 32

    def __post_init__(self) -> None:
        for name, cap in (
            ("max_frames", 8),
            ("max_clusters", 256),
            ("max_tracks", 256),
            ("max_candidates", 8),
            ("samples_per_contour", 32),
        ):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= cap:
                raise ValueError(f"{name} must be within 1..{cap}")
        if self.samples_per_contour < 2:
            raise ValueError("a surface needs at least two association samples")


@dataclass(frozen=True)
class ContourEvidence:
    local_track_id: int
    beam_indices: tuple[int, ...]
    odom_surface: tuple[Point, ...]  # FULL current contour, not association samples
    first_key: ScanKey
    last_key: ScanKey
    common_fraction: float
    max_residual_m: float
    support_ids: tuple[str, ...]


@dataclass(frozen=True)
class GeometryEvidence:
    key: ScanKey
    evidence_revision: int
    stable: tuple[ContourEvidence, ...]
    moved_support_ids: tuple[str, ...]
    unresolved_reason: str | None


@dataclass(frozen=True)
class _Surface:
    count: int
    samples: tuple[Point, ...]
    center: Point
    diameter: float
    error: float


@dataclass(frozen=True)
class _Contour(_Surface):
    indices: tuple[int, ...]
    points: tuple[Point, ...]


@dataclass(frozen=True)
class _Frame:
    key: ScanKey
    stamp_ns: int
    contour: _Contour
    capture: RawCaptureHeader
    valid_until_steady_ns: int


@dataclass(frozen=True)
class _BoundSurface:
    key: ScanKey
    capture: RawCaptureHeader
    valid_until_steady_ns: int
    surface: _Surface


@dataclass(frozen=True)
class _Residual:
    common: float
    upper: float
    lower: float


@dataclass
class _Track:
    identity: int
    last: _Frame
    stable_window: deque[_Frame]
    motion_window: deque[_Frame]
    supports: tuple[str, ...] = ()
    confirmed: bool = False
    baseline: _BoundSurface | None = None


def _distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _residual(a: _Surface, b: _Surface, tolerance: float) -> _Residual:
    """Translation aligns SHAPE for association only; odom distances test motion.

    Both directions must explain the visible surface. Coverage includes extent
    and point count, so a tiny partial contour cannot inherit a whole support.
    No ICP, category, or centroid-only stability/motion decision is performed.
    """
    if min(len(a.samples), len(b.samples)) < 2 or min(a.diameter, b.diameter) <= 0:
        return _Residual(0.0, math.inf, 0.0)
    dx, dy = a.center[0] - b.center[0], a.center[1] - b.center[1]
    shifted = tuple((x + dx, y + dy) for x, y in b.samples)
    aligned_a = [min(_distance(p, q) for q in shifted) for p in a.samples]
    aligned_b = [min(_distance(p, q) for q in a.samples) for p in shifted]
    common = min(
        sum(d <= tolerance for d in aligned_a) / len(aligned_a),
        sum(d <= tolerance for d in aligned_b) / len(aligned_b),
        min(a.count, b.count) / max(a.count, b.count),
    )
    diameter = max(a.diameter, b.diameter)
    if diameter:
        common = min(common, min(a.diameter, b.diameter) / diameter)
    distances = [min(_distance(p, q) for q in b.samples) for p in a.samples] + [
        min(_distance(p, q) for q in a.samples) for p in b.samples
    ]
    # All common samples must fit the stable band. For motion, both directed
    # means must exceed the motion band after subtracting the error envelope.
    error = a.error + b.error
    lower = (
        min(
            sum(distances[: len(a.samples)]) / len(a.samples),
            sum(distances[len(a.samples) :]) / len(b.samples),
        )
        - error
    )
    return _Residual(common, max(distances) + error, max(0.0, lower))


class GeometryTracker:
    """One bounded current generation; lost/ambiguous tracks are simply forgotten.

    Motion outside the local search aperture is unresolved, never inferred from
    disappearance. No source/reference tombstones or unlimited identity history.
    """

    def __init__(
        self,
        *,
        limits: GeometryLimits = GeometryLimits(),
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.limits = limits
        self._clock_ns = clock_ns
        self._tracks: dict[int, _Track] = {}
        self._previous: QualifiedScan | None = None
        self._profile: GeometryProfile | None = None
        self._next_id = 0
        self._revision = 0

    def reset(self) -> None:
        """Owner calls for R/lifecycle retirement; never revokes fixed supports."""
        self._tracks.clear()
        self._previous = None
        self._profile = None
        # IDs/revisions do not rewind: a late bind cannot attach to a new track.

    @property
    def bound_support_ids(self) -> frozenset[str]:
        """Current local references, including pending motion verification.

        The fixed owner uses this read-only view to avoid assigning a restored
        support to another contour before committing its batch.
        """
        return frozenset(identity for track in self._tracks.values() for identity in track.supports)

    def bind_support(self, local_track_id: int, support_ids: tuple[str, ...]) -> None:
        """Replace references after the sole fixed owner commits support.

        Every nonempty replacement (including partial/repeated binding) retains
        the first binding's surface. Empty binding ends this local lifetime;
        subsequent observations must form a complete new stable track.
        """
        if (
            type(local_track_id) is not int
            or not isinstance(support_ids, tuple)
            or len(support_ids) > 2048
            or any(
                not isinstance(s, str) or not s.strip() or len(s.encode("utf-8")) > 128
                for s in support_ids
            )
        ):
            raise ValueError("invalid support binding")
        requested = set(support_ids)
        if len(requested) != len(support_ids):
            raise ValueError("invalid support binding")
        track = self._tracks.get(local_track_id)
        if track is None or not track.confirmed:
            raise ValueError("binding requires a current stable contour")
        if any(
            requested.intersection(other.supports)
            for other in self._tracks.values()
            if other is not track
        ):
            raise ValueError("support cannot belong to ambiguous local tracks")
        if not support_ids:
            del self._tracks[local_track_id]
            return
        if track.baseline is None:
            frame, contour = track.last, track.last.contour
            track.baseline = _BoundSurface(
                frame.key,
                frame.capture,
                frame.valid_until_steady_ns,
                _Surface(
                    contour.count, contour.samples, contour.center, contour.diameter, contour.error
                ),
            )
        track.supports = support_ids

    def _evidence(
        self,
        scan: QualifiedScan,
        reason: str | None,
        stable: tuple[ContourEvidence, ...] = (),
        moved: tuple[str, ...] = (),
    ) -> GeometryEvidence:
        return GeometryEvidence(scan.key, self._revision, stable, moved, reason)

    def _clusters(self, scan: QualifiedScan, profile: GeometryProfile) -> list[_Contour] | None:
        """One pass in original beam order; an invalid/clear/self beam splits."""
        groups: list[list[Beam]] = []
        group: list[Beam] = []
        previous = None
        for beam in scan.beams:
            if beam.kind != "hit":
                previous = None
                continue
            distance = _distance(beam.origin_odom, beam.endpoint_odom)
            gap = profile.cluster_gap_m + profile.cluster_gap_per_m * distance
            if previous is None or _distance(previous.endpoint_odom, beam.endpoint_odom) > gap:
                group = []
                groups.append(group)
                if len(groups) > self.limits.max_clusters:
                    return None
            group.append(beam)
            previous = beam
        result = []
        for group in groups:
            points = tuple(beam.endpoint_odom for beam in group)
            n = min(len(points), self.limits.samples_per_contour)
            samples = (
                tuple(points[i * (len(points) - 1) // (n - 1)] for i in range(n))
                if n > 1
                else points
            )
            xs, ys = tuple(p[0] for p in points), tuple(p[1] for p in points)
            center = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
            diameter = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            radius = max(_distance(beam.origin_odom, beam.endpoint_odom) for beam in group)
            error = (
                profile.range_sigma_m
                + profile.odom_translation_error_m
                + 2 * radius * math.sin(min(math.pi, profile.odom_yaw_error_rad) / 2)
                + profile.max_scan_distortion_m
            )
            result.append(
                _Contour(
                    len(points),
                    samples,
                    center,
                    diameter,
                    error,
                    tuple(beam.index for beam in group),
                    points,
                )
            )
        return result

    def _associations(self, contours: list[_Contour], profile: GeometryProfile):
        """Local bins bound candidate work before any pairwise surface distances.

        Aperture = maximum local contour diameter + motion separation. It is a
        search budget, not an accepted velocity bound. Saturation returns no
        associations, rather than truncating a list into a spurious unique hit.
        """
        width = max((c.diameter for c in contours), default=0.0)
        width = max(
            width, max((t.last.contour.diameter for t in self._tracks.values()), default=0.0)
        )
        width += profile.motion_residual_m
        bins: dict[tuple[int, int], list[int]] = {}
        for identity, track in self._tracks.items():
            x, y = track.last.contour.center
            bins.setdefault((math.floor(x / width), math.floor(y / width)), []).append(identity)
        edges: dict[int, list[int]] = {}
        reverse: dict[int, list[int]] = {}
        for i, contour in enumerate(contours):
            x, y = (math.floor(v / width) for v in contour.center)
            candidates = []
            for bx in range(x - 1, x + 2):
                for by in range(y - 1, y + 2):
                    for identity in bins.get((bx, by), ()):
                        candidates.append(identity)
                        if len(candidates) > self.limits.max_candidates:
                            return {}, {}, "candidate_capacity"
            for identity in candidates:
                other = self._tracks[identity].last.contour
                if _distance(contour.center, other.center) > width:
                    continue
                metric = _residual(contour, other, profile.stable_residual_m)
                if metric.common >= profile.min_common_fraction:
                    edges.setdefault(i, []).append(identity)
                    reverse.setdefault(identity, []).append(i)
        return edges, reverse, None

    def _new_track(self, frame: _Frame) -> _Track:
        self._next_id += 1
        return _Track(
            self._next_id,
            frame,
            deque((frame,), maxlen=self.limits.max_frames),
            deque((frame,), maxlen=self.limits.max_frames),
        )

    @staticmethod
    def _complete(window: deque[_Frame], profile: GeometryProfile) -> bool:
        return len(window) >= profile.min_independent_frames and window[-1].stamp_ns - window[
            0
        ].stamp_ns >= math.ceil(profile.min_span_s * 1e9)

    def _advance(self, track: _Track, frame: _Frame, profile: GeometryProfile):
        """Stable and motion windows use independent, original capture frames."""
        prior = track.last
        # A rolling window alone cannot constrain cumulative displacement from
        # a still-bound support. Its original compact surface does not roll.
        metrics = [
            _residual(old.contour, frame.contour, profile.stable_residual_m)
            for old in track.stable_window
        ]
        stable = bool(metrics) and all(
            m.common >= profile.min_common_fraction and m.upper <= profile.stable_residual_m
            for m in metrics
        )
        bound_motion = None
        if track.baseline is not None:
            bound_motion = _residual(
                track.baseline.surface, frame.contour, profile.stable_residual_m
            )
            stable = stable and (
                bound_motion.common >= profile.min_common_fraction
                and bound_motion.upper <= profile.stable_residual_m
            )
            metrics.append(bound_motion)
        if stable:
            track.stable_window.append(frame)
            track.motion_window = deque((frame,), maxlen=self.limits.max_frames)
        else:
            track.stable_window = deque((frame,), maxlen=self.limits.max_frames)
            track.confirmed = False
            # A locally stable candidate at a displaced position cannot erase
            # pending verification against the still-bound original surface.
            track.motion_window.append(frame)
        moved: tuple[str, ...] = ()
        if not stable and self._complete(track.motion_window, profile):
            motion = (
                bound_motion
                if bound_motion is not None
                else _residual(
                    track.motion_window[0].contour, frame.contour, profile.stable_residual_m
                )
            )
            if (
                motion.common >= profile.min_common_fraction
                and motion.lower >= profile.motion_residual_m
            ):
                moved, track.supports = track.supports, ()
                track.baseline = None
                track.motion_window = deque((frame,), maxlen=self.limits.max_frames)
                track.stable_window = deque((frame,), maxlen=self.limits.max_frames)
                track.confirmed = False
                if moved:
                    # Late owner callbacks for the withdrawn lifetime cannot
                    # reattach old supports after the fresh stable window.
                    self._next_id += 1
                    track.identity = self._next_id
        track.last = frame
        if stable and self._complete(track.stable_window, profile):
            track.confirmed = True
            return ContourEvidence(
                track.identity,
                frame.contour.indices,
                frame.contour.points,
                track.stable_window[0].key,
                frame.key,
                min(m.common for m in metrics),
                max(m.upper for m in metrics),
                track.supports,
            ), moved
        # Keep the previous sample as motion onset when a stable window breaks.
        # motion_window already ends at prior while stable, so no duplicate add.
        assert prior.key != frame.key
        return None, moved

    def update(self, scan: QualifiedScan, *, profile: GeometryProfile | None) -> GeometryEvidence:
        if profile is None:
            self.reset()
            return self._evidence(scan, "profile_missing")
        if profile.min_independent_frames > self.limits.max_frames or math.ceil(
            profile.min_span_s * 1e9
        ) > (self.limits.max_frames - 1) * math.floor(profile.max_gap_s * 1e9):
            self.reset()
            return self._evidence(scan, "profile_exceeds_capacity")
        if self._clock_ns() >= scan.valid_until_steady_ns:
            self.reset()
            return self._evidence(scan, "expired")
        previous = self._previous
        changed = previous is not None and (
            previous.key.source != scan.key.source
            or previous.host_boot_id != scan.host_boot_id
            or previous.geometry_id != scan.geometry_id
            or profile != self._profile
        )
        if changed:
            self.reset()
            previous = None
        if previous is not None:
            old, new = previous.capture.time, scan.capture.time
            if (
                new.stamp_ns <= old.stamp_ns
                or new.sample_steady_ns <= old.sample_steady_ns
                or new.received_steady_ns <= old.received_steady_ns
            ):
                return self._evidence(scan, "non_independent")
            if new.sample_steady_ns - old.sample_steady_ns > math.floor(
                profile.max_gap_s * 1e9
            ) or new.stamp_ns - old.stamp_ns > math.floor(profile.max_gap_s * 1e9):
                self._tracks.clear()
        self._previous, self._profile = scan, profile
        self._revision += 1
        contours = self._clusters(scan, profile)
        if contours is None:
            self._tracks.clear()
            return self._evidence(scan, "cluster_capacity")
        if len(contours) > self.limits.max_tracks:
            self._tracks.clear()
            return self._evidence(scan, "track_capacity")
        edges, reverse, reason = self._associations(contours, profile)
        if reason:
            self._tracks.clear()
            return self._evidence(scan, reason)
        next_tracks = {}
        stable, moved = [], []
        unresolved = False
        # Mutual unique edges are the conservative greedy matching case. Any
        # competition/split/merge drops identity and binding instead of guessing.
        for i, contour in enumerate(contours):
            frame = _Frame(
                scan.key,
                scan.capture.time.stamp_ns,
                contour,
                scan.capture,
                scan.valid_until_steady_ns,
            )
            candidates = edges.get(i, ())
            if len(candidates) == 1 and len(reverse[candidates[0]]) == 1:
                track = self._tracks[candidates[0]]
                evidence, revocations = self._advance(track, frame, profile)
                if evidence is not None:
                    stable.append(evidence)
                else:
                    unresolved = True
                moved.extend(revocations)
            else:
                track = self._new_track(frame)
                unresolved = True
            next_tracks[track.identity] = track
        self._tracks = next_tracks
        return self._evidence(
            scan,
            "insufficient_or_ambiguous" if unresolved or not contours else None,
            tuple(stable),
            tuple(moved),
        )
