"""Bounded, research-only RGB-D relative motion; never a safety qualification.

No dependency on a map, laser or wheel motion.  Imports of NumPy/OpenCV are
local so installations that do not use this experimental path remain usable.
"""

import importlib
import math
from dataclasses import dataclass, field
from typing import Any

from senior_control.vision.raw_capture import (
    RawCaptureAdmission,
    RawCaptureHeader,
    compatible_captures,
)


def _bounded_text(value: Any, maximum: int, *, nonempty: bool = False) -> bool:
    if type(value) is not str or (nonempty and not value):
        return False
    try:
        return len(value.encode("utf-8")) <= maximum
    except UnicodeError:
        return False


def _number(value: Any) -> bool:
    return type(value) in (int, float)


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(value)
    except (OverflowError, TypeError):
        return False


@dataclass(frozen=True)
class Calibration:
    """Rectified registered optical-z calibration and its external identities."""

    fx: float
    fy: float
    cx: float
    cy: float
    epoch: str
    registration: str
    rectified: bool
    width: int = 0
    height: int = 0
    rgb_encoding: str = ""
    depth_encoding: str = ""
    depth_is_optical_z: bool = False
    distortion_model: str = ""
    roi: tuple[int, int, int, int] = (0, 0, 0, 0)
    binning: tuple[int, int] = (0, 0)

    def __post_init__(self) -> None:
        if (
            not all(_number(v) for v in (self.fx, self.fy, self.cx, self.cy))
            or not all(
                _bounded_text(v, 256)
                for v in (
                    self.epoch,
                    self.registration,
                    self.rgb_encoding,
                    self.depth_encoding,
                    self.distortion_model,
                )
            )
            or type(self.rectified) is not bool
            or type(self.depth_is_optical_z) is not bool
            or type(self.width) is not int
            or type(self.height) is not int
            or type(self.roi) is not tuple
            or len(self.roi) != 4
            or not all(type(v) is int for v in self.roi)
            or type(self.binning) is not tuple
            or len(self.binning) != 2
            or not all(type(v) is int for v in self.binning)
        ):
            raise ValueError("calibration_metadata")


@dataclass(frozen=True)
class Profile:
    """SIM scene declaration and external validation reference, not authorization."""

    name: str
    validation_reference: str
    static_background_declared: bool
    texture_domain: str = ""
    calibration_epoch: str = ""
    max_dynamic_fraction: float = math.nan
    max_occlusion_fraction: float = math.nan
    max_speed_m_s: float = math.nan
    systematic_translation_bound_m: float = math.nan
    systematic_rotation_bound_rad: float = math.nan

    def __post_init__(self) -> None:
        if (
            not all(
                _bounded_text(v, 256)
                for v in (
                    self.name,
                    self.validation_reference,
                    self.texture_domain,
                    self.calibration_epoch,
                )
            )
            or type(self.static_background_declared) is not bool
            or not all(
                _number(v)
                for v in (
                    self.max_dynamic_fraction,
                    self.max_occlusion_fraction,
                    self.max_speed_m_s,
                    self.systematic_translation_bound_m,
                    self.systematic_rotation_bound_rad,
                )
            )
        ):
            raise ValueError("profile_metadata")


@dataclass(frozen=True, eq=False)
class Frame:
    """Snapshot of registered pixels, exclusions and their original capture headers."""

    rgb_header: RawCaptureHeader
    depth_header: RawCaptureHeader
    rgb_info_header: RawCaptureHeader
    depth_info_header: RawCaptureHeader
    image: Any
    depth: Any
    calibration: Calibration
    tf_epoch: str
    dependencies: tuple[str, ...]
    exclusion: Any

    def __post_init__(self) -> None:
        if (
            type(self.dependencies) is not tuple
            or not 1 <= len(self.dependencies) <= 64
            or not all(_bounded_text(v, 256, nonempty=True) for v in self.dependencies)
        ):
            raise ValueError("dependencies")
        if (
            type(self.calibration) is not Calibration
            or not _bounded_text(self.tf_epoch, 256)
            or not all(_immutable_header(h) for h in _headers(self))
        ):
            raise ValueError("frame_metadata")
        import numpy as np

        if any(
            not isinstance(getattr(self, name), np.ndarray)
            or getattr(self, name).dtype != dtype
            or getattr(self, name).ndim != 2
            or getattr(self, name).shape[0] > 480
            or getattr(self, name).shape[1] > 640
            for name, dtype in (("image", np.uint8), ("depth", np.float32), ("exclusion", np.uint8))
        ):
            raise ValueError("frame_budget")
        # Backing bytes are immutable, including through a derived numpy view.
        for name in ("image", "depth", "exclusion"):
            value = np.asarray(getattr(self, name))
            copy = np.frombuffer(value.tobytes(), dtype=value.dtype).reshape(value.shape)
            object.__setattr__(self, name, copy)


@dataclass(frozen=True)
class Result:
    """Fit diagnostics and immutable pair; no hard error or covariance claim."""

    reason: str
    pair: tuple[Frame, Frame] | None = None
    camera_transform: tuple[tuple[float, ...], ...] | None = None
    inliers: int = 0
    matched: int = 0
    median_reprojection_px: float | None = None
    median_depth_residual_m: float | None = None
    hard_error_qualified: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        if (
            not _bounded_text(self.reason, 256)
            or (
                self.pair is not None
                and (
                    type(self.pair) is not tuple
                    or len(self.pair) != 2
                    or any(type(frame) is not Frame for frame in self.pair)
                )
            )
            or (
                self.camera_transform is not None
                and (
                    type(self.camera_transform) is not tuple
                    or len(self.camera_transform) != 4
                    or any(
                        type(row) is not tuple or len(row) != 4 or not all(_number(v) for v in row)
                        for row in self.camera_transform
                    )
                )
            )
            or type(self.inliers) is not int
            or type(self.matched) is not int
            or (
                self.median_reprojection_px is not None and not _number(self.median_reprojection_px)
            )
            or (
                self.median_depth_residual_m is not None
                and not _number(self.median_depth_residual_m)
            )
        ):
            raise ValueError("result_metadata")


def _immutable_header(header: Any) -> bool:
    return (
        type(header) is RawCaptureHeader
        and all(
            type(getattr(header, name)) is int
            for name in ("stamp_ns", "received_ros_ns", "capture_generation", "sequence")
        )
        and _number(header.received_steady)
        and all(
            _bounded_text(getattr(header, name), 256)
            for name in (
                "frame_id",
                "raw_gid",
                "bridge_gid",
                "source_epoch",
                "bridge_boot_id",
                "reason",
            )
        )
        and type(header.valid) is bool
    )


def _identity(frame: Frame) -> tuple[Any, ...]:
    return tuple(
        (
            h.raw_gid,
            h.bridge_gid,
            h.source_epoch,
            h.bridge_boot_id,
            h.capture_generation,
            h.frame_id,
        )
        for h in _headers(frame)
    ) + (
        frame.calibration,
        frame.tf_epoch,
        frame.dependencies,
    )


def _headers(frame: Frame) -> tuple[RawCaptureHeader, ...]:
    return (frame.rgb_header, frame.depth_header, frame.rgb_info_header, frame.depth_info_header)


def _valid(frame: Frame, now_ros_ns: int, now_steady: float) -> bool:
    import numpy as np

    c = frame.calibration
    if (
        not frame.tf_epoch
        or not frame.dependencies
        or not c.epoch
        or not c.registration
        or c.rectified is not True
        or c.rgb_encoding != "mono8"
        or c.depth_encoding != "32FC1"
        or c.depth_is_optical_z is not True
        or c.distortion_model != "rectified"
        or c.roi != (0, 0, c.width, c.height)
        or c.binning != (1, 1)
        or (c.height, c.width) != frame.image.shape
        or not 0.1 * c.width <= c.fx <= 10 * c.width
        or not 0.1 * c.height <= c.fy <= 10 * c.height
        or not 0 <= c.cx < c.width
        or not 0 <= c.cy < c.height
        or not compatible_captures(*_headers(frame))
    ):
        return False
    if (
        frame.image.ndim != 2
        or frame.image.dtype != np.uint8
        or frame.depth.dtype != np.float32
        or frame.depth.shape != frame.image.shape
        or frame.exclusion.dtype != np.uint8
        or frame.exclusion.shape != frame.image.shape
        or not 80 <= frame.image.shape[0] <= 480
        or not 80 <= frame.image.shape[1] <= 640
    ):
        return False
    return _headers_valid(_headers(frame), now_ros_ns, now_steady)


def _headers_valid(
    headers: tuple[RawCaptureHeader, ...], now_ros_ns: int, now_steady: float
) -> bool:
    if (
        type(headers) is not tuple
        or len(headers) != 4
        or not all(_immutable_header(h) for h in headers)
        or not compatible_captures(*headers)
        or not _finite(now_steady)
    ):
        return False
    for h in headers:
        if not _finite(h.received_steady):
            return False
        admission = RawCaptureAdmission(max_age_s=0.3)
        if not admission.offer(
            h, bridge_gid=h.bridge_gid, now_ros_ns=now_ros_ns, now_steady=now_steady
        ):
            return False
        if (
            not h.raw_gid
            or not h.bridge_gid
            or not h.source_epoch
            or h.sequence <= 0
            or not math.isfinite(h.received_steady)
            or not math.isfinite(now_steady)
            or not 0 <= now_ros_ns - h.stamp_ns <= 300_000_000
            or not 0 <= now_ros_ns - h.received_ros_ns <= 300_000_000
            or not 0 <= now_steady - h.received_steady <= 0.3
        ):
            return False
    return True


class IndependentMotion:
    """Two-frame bounded estimator, serial caller only; no background worker."""

    def __init__(self, profile: Profile) -> None:
        self.profile = profile
        self._previous: Frame | None = None
        self._frontier: tuple[RawCaptureHeader, ...] | None = None
        self._retired: tuple[set[tuple[Any, ...]], ...] = tuple(set() for _ in range(4))
        self._retired_raw_gids: tuple[set[str], ...] = tuple(set() for _ in range(4))
        self._retired_boots: tuple[set[str], ...] = tuple(set() for _ in range(4))
        self._retired_bridges: tuple[set[str], ...] = tuple(set() for _ in range(4))
        self._closed = False

    def invalidate(self) -> None:
        """Revoke the retained frame on externally observed source/TF loss."""
        self._previous = None

    @staticmethod
    def _source(header: RawCaptureHeader) -> tuple[Any, ...]:
        return (
            header.raw_gid,
            header.bridge_gid,
            header.source_epoch,
            header.bridge_boot_id,
            header.capture_generation,
            header.frame_id,
        )

    def _advance(self, incoming: tuple[RawCaptureHeader, ...]) -> tuple[bool, bool]:
        """Atomically advance four capture frontiers and retire replaced lineages."""
        if self._closed:
            return False, False
        if self._frontier is None:
            self._frontier = incoming
            return True, False
        replacements: list[tuple[int, tuple[Any, ...]]] = []
        for index, (old, new) in enumerate(zip(self._frontier, incoming, strict=True)):
            old_key, new_key = self._source(old), self._source(new)
            if (
                new.stamp_ns <= old.stamp_ns
                or new.received_ros_ns <= old.received_ros_ns
                or new.received_steady <= old.received_steady
                or new_key in self._retired[index]
                or new.raw_gid in self._retired_raw_gids[index]
                or new.bridge_boot_id in self._retired_boots[index]
                or new.bridge_gid in self._retired_bridges[index]
            ):
                return False, False
            if old_key == new_key:
                if new.sequence <= old.sequence:
                    return False, False
                continue
            # A new lineage must have an unambiguous source/bridge generation change.
            same_boot = new.bridge_boot_id == old.bridge_boot_id
            valid_replacement = (
                same_boot
                and new.bridge_gid == old.bridge_gid
                and (
                    new.capture_generation > old.capture_generation
                    or (
                        new.capture_generation == old.capture_generation
                        and new.raw_gid != old.raw_gid
                        and new.source_epoch != old.source_epoch
                    )
                )
            ) or (not same_boot and new.bridge_gid != old.bridge_gid)
            if not valid_replacement:
                return False, False
            replacements.append((index, old_key))
        if any(len(self._retired[index]) >= 64 for index, _ in replacements):
            self._closed = True
            return False, False
        for index, old_key in replacements:
            self._retired[index].add(old_key)
            old = self._frontier[index]
            new = incoming[index]
            if old.raw_gid != new.raw_gid:
                self._retired_raw_gids[index].add(old.raw_gid)
            if old.bridge_boot_id != new.bridge_boot_id:
                self._retired_boots[index].add(old.bridge_boot_id)
                self._retired_bridges[index].add(old.bridge_gid)
        self._frontier = incoming
        return True, bool(replacements)

    def observe_unusable_headers(
        self, headers: tuple[RawCaptureHeader, ...], *, now_ros_ns: int, now_steady: float
    ) -> bool:
        """Consume verified headers when a bounded Frame cannot be constructed.

        This invalidates the current pair. The same headers cannot later be
        offered as a usable frame; a later capture is required.
        """
        self._previous = None
        if not _headers_valid(headers, now_ros_ns, now_steady):
            return False
        accepted, _ = self._advance(headers)
        return accepted

    def offer(self, frame: Frame, *, now_ros_ns: int, now_steady: float) -> Result:
        """Offer a snapshot; an invalid/replaced pair is revoked immediately."""
        previous = self._previous
        self._previous = None
        if not _headers_valid(_headers(frame), now_ros_ns, now_steady):
            return Result("invalid_frame")
        accepted, replaced = self._advance(_headers(frame))
        if not accepted:
            return Result("source_frontier_invalid")
        if (
            not self.profile.name
            or not self.profile.validation_reference
            or not self.profile.static_background_declared
            or not self.profile.texture_domain
            or self.profile.calibration_epoch != frame.calibration.epoch
            or not all(
                _finite(value) and value > 0
                for value in (
                    self.profile.max_dynamic_fraction,
                    self.profile.max_occlusion_fraction,
                    self.profile.max_speed_m_s,
                    self.profile.systematic_translation_bound_m,
                    self.profile.systematic_rotation_bound_rad,
                )
            )
            or self.profile.max_dynamic_fraction > 1
            or self.profile.max_occlusion_fraction > 1
        ):
            return Result("profile_unavailable")
        if not _valid(frame, now_ros_ns, now_steady):
            return Result("invalid_frame")
        import numpy as np

        if (
            np.count_nonzero(frame.exclusion) / frame.exclusion.size
            > self.profile.max_dynamic_fraction
        ):
            return Result("profile_domain_exceeded")
        if replaced:
            self._previous = frame
            return Result("initializing")
        if previous is None:
            self._previous = frame
            return Result("initializing")
        if not _valid(previous, now_ros_ns, now_steady):
            return Result("expired_previous")
        if (
            _identity(frame) != _identity(previous)
            or any(
                b.sequence <= a.sequence
                for a, b in zip(_headers(previous), _headers(frame), strict=True)
            )
            or not 0 < frame.rgb_header.stamp_ns - previous.rgb_header.stamp_ns <= 200_000_000
        ):
            return Result("pair_revoked")
        self._previous = frame
        result = _fit(previous, frame)
        if result.reason == "numerical_failure":
            self._previous = None
        if result.camera_transform is not None:
            translation = [result.camera_transform[i][3] for i in range(3)]
            elapsed = (frame.rgb_header.stamp_ns - previous.rgb_header.stamp_ns) / 1e9
            if math.sqrt(sum(v * v for v in translation)) / elapsed > self.profile.max_speed_m_s:
                self._previous = None
                return Result("profile_domain_exceeded")
        return result


def _feature_mask(exclusion: Any) -> Any:
    """Conservatively keep ORB's single-scale patch away from exclusions."""
    cv2 = importlib.import_module("cv2")
    import numpy as np

    kernel = np.ones((65, 65), dtype=np.uint8)
    return (cv2.dilate((exclusion != 0).astype(np.uint8), kernel) == 0).astype(np.uint8) * 255


def _pose_quality(
    points: Any, pixels: Any, groups: Any, matrix: Any, rvec: Any, tvec: Any
) -> str | None:
    """Reject weak local pose Jacobians and disagreement between two spatial fits.

    Translation uses u=t/median_depth, so the corresponding Jacobian columns
    are J_t * median_depth. These exploratory filters are not error bounds.
    """
    cv2 = importlib.import_module("cv2")
    import numpy as np

    if not all(np.isfinite(value).all() for value in (points, pixels, matrix, rvec, tvec)):
        return "numerical_failure"
    if np.linalg.norm(rvec) >= math.pi / 2:
        return "ill_conditioned_pose"
    _, jacobian = cv2.projectPoints(points, rvec, tvec, matrix, None)
    jacobian = jacobian[:, :6].copy()
    jacobian[:, 3:6] *= float(np.median(points[:, 2]))
    if not np.isfinite(jacobian).all():
        return "numerical_failure"
    singular = np.linalg.svd(jacobian, compute_uv=False)
    if not np.isfinite(singular).all() or singular[0] <= 0 or singular[-1] / singular[0] < 0.001:
        return "ill_conditioned_pose"
    solutions = []
    for group in (0, 1):
        selection = groups == group
        if np.count_nonzero(selection) < 10:
            return "unstable_pose"
        ok, group_r, group_t = cv2.solvePnP(
            points[selection], pixels[selection], matrix, None, flags=cv2.SOLVEPNP_EPNP
        )
        if not ok or not np.isfinite(group_r).all() or not np.isfinite(group_t).all():
            return "unstable_pose"
        solutions.append((cv2.Rodrigues(group_r)[0], group_t.reshape(3)))
    rotation_delta = solutions[0][0] @ solutions[1][0].T
    angle = float(np.linalg.norm(cv2.Rodrigues(rotation_delta)[0]))
    distance = float(np.linalg.norm(solutions[0][1] - solutions[1][1]))
    if not math.isfinite(angle) or not math.isfinite(distance):
        return "numerical_failure"
    if angle > 0.02 or distance > 0.05:
        return "unstable_pose"
    return None


def _fit(first: Frame, second: Frame) -> Result:
    try:
        cv2 = importlib.import_module("cv2")
        import numpy as np
    except ImportError:
        return Result("dependency_unavailable")
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            return _fit_impl(first, second)
    except (cv2.error, np.linalg.LinAlgError, FloatingPointError, OverflowError):
        return Result("numerical_failure")


def _fit_impl(first: Frame, second: Frame) -> Result:
    cv2 = importlib.import_module("cv2")
    import numpy as np

    cv2.setNumThreads(1)
    cv2.setRNGSeed(103)
    orb = cv2.ORB_create(nfeatures=500, nlevels=1)
    k1, d1 = orb.detectAndCompute(first.image, _feature_mask(first.exclusion))
    k2, d2 = orb.detectAndCompute(second.image, _feature_mask(second.exclusion))
    if d1 is None or d2 is None or len(d1) < 20 or len(d2) < 20:
        return Result("insufficient_texture")
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    forward = matcher.knnMatch(d1, d2, k=2)
    reverse = matcher.knnMatch(d2, d1, k=2)
    back = {
        m.queryIdx: m.trainIdx
        for pair in reverse
        if len(pair) == 2
        for m, n in [pair]
        if m.distance < 0.75 * n.distance
    }
    matches = [
        m
        for pair in forward
        if len(pair) == 2
        for m, n in [pair]
        if m.distance < 0.75 * n.distance and back.get(m.trainIdx) == m.queryIdx
    ]
    c = first.calibration
    matrix = np.array(((c.fx, 0.0, c.cx), (0.0, c.fy, c.cy), (0.0, 0.0, 1.0)))
    p1_list, p2_list, xyz1_list, xyz2_list, cells_list = [], [], [], [], []
    counts: dict[int, int] = {}
    height, width = first.image.shape
    for m in sorted(matches, key=lambda x: x.distance):
        a, b = k1[m.queryIdx].pt, k2[m.trainIdx].pt
        x1, y1, x2, y2 = round(a[0]), round(a[1]), round(b[0]), round(b[1])
        if not (
            2 <= x1 < width - 2
            and 2 <= x2 < width - 2
            and 2 <= y1 < height - 2
            and 2 <= y2 < height - 2
        ):
            continue
        z1, z2 = float(first.depth[y1, x1]), float(second.depth[y2, x2])
        if not (math.isfinite(z1) and math.isfinite(z2) and 0.2 < z1 < 20 and 0.2 < z2 < 20):
            continue
        # Reject boundaries and holes rather than interpolating depth across surfaces.
        if (
            np.any(np.abs(first.depth[y1 - 2 : y1 + 3, x1 - 2 : x1 + 3] - z1) > 0.05)
            or np.any(np.abs(second.depth[y2 - 2 : y2 + 3, x2 - 2 : x2 + 3] - z2) > 0.05)
            or not np.isfinite(first.depth[y1 - 2 : y1 + 3, x1 - 2 : x1 + 3]).all()
            or not np.isfinite(second.depth[y2 - 2 : y2 + 3, x2 - 2 : x2 + 3]).all()
        ):
            continue
        cell = min(3, int(a[0] * 4 / width)) + 4 * min(2, int(a[1] * 3 / height))
        if counts.get(cell, 0) >= 35:
            continue
        counts[cell] = counts.get(cell, 0) + 1
        p1_list.append(a)
        p2_list.append(b)
        cells_list.append(cell)
        xyz1_list.append(((a[0] - c.cx) * z1 / c.fx, (a[1] - c.cy) * z1 / c.fy, z1))
        xyz2_list.append(((b[0] - c.cx) * z2 / c.fx, (b[1] - c.cy) * z2 / c.fy, z2))
    if len(p1_list) < 30 or len(counts) < 6:
        return Result("insufficient_spatial_depth")
    p1 = np.asarray(p1_list)
    p2 = np.asarray(p2_list)
    xyz1 = np.asarray(xyz1_list)
    xyz2 = np.asarray(xyz2_list)
    cells = np.asarray(cells_list)
    if not all(np.isfinite(value).all() for value in (matrix, p1, p2, xyz1, xyz2)):
        return Result("numerical_failure")
    centered = xyz1 - xyz1.mean(axis=0)
    if not np.isfinite(centered).all():
        return Result("numerical_failure")
    singular = np.linalg.svd(centered, compute_uv=False)
    if not np.isfinite(singular).all() or singular[0] <= 0 or singular[-1] / singular[0] < 0.04:
        return Result("degenerate_geometry")
    # Leave out alternating image cells from the fit as an independent spatial check.
    train = (cells % 2) == 0
    if np.count_nonzero(train) < 15 or np.count_nonzero(~train) < 10:
        return Result("insufficient_holdout")
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        xyz1[train],
        p2[train],
        matrix,
        None,
        iterationsCount=100,
        reprojectionError=2.0,
        confidence=0.99,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if ok and (not np.isfinite(rvec).all() or not np.isfinite(tvec).all()):
        return Result("numerical_failure")
    if (
        not ok
        or inliers is None
        or len(inliers) < 15
        or len(inliers) < np.count_nonzero(train) * 0.6
    ):
        return Result("pnp_failed")
    quality_reason = _pose_quality(xyz1, p2, cells % 2, matrix, rvec, tvec)
    if quality_reason is not None:
        return Result(quality_reason)
    rotation = cv2.Rodrigues(rvec)[0]
    if not np.isfinite(rotation).all():
        return Result("numerical_failure")
    predicted = (rotation @ xyz1.T).T + tvec.reshape(3)
    residual = np.linalg.norm(predicted - xyz2, axis=1)
    projected, _ = cv2.projectPoints(xyz1, rvec, tvec, matrix, None)
    reprojection = np.linalg.norm(projected.reshape(-1, 2) - p2, axis=1)
    inverse = (rotation.T @ (xyz2 - tvec.reshape(3)).T).T
    inverse_projection = (matrix @ inverse.T).T
    if (
        not np.isfinite(inverse).all()
        or not np.isfinite(inverse_projection).all()
        or np.any(inverse_projection[:, 2] <= 0)
    ):
        return Result("numerical_failure")
    inverse_pixel = inverse_projection[:, :2] / inverse_projection[:, 2, None]
    reverse_error = np.linalg.norm(inverse_pixel - p1, axis=1)
    if not all(
        np.isfinite(value).all()
        for value in (predicted, residual, projected, reprojection, inverse_pixel, reverse_error)
    ):
        return Result("numerical_failure")
    hold = ~train
    if (
        not np.isfinite(predicted).all()
        or np.any(predicted[:, 2] <= 0)
        or np.any(inverse[:, 2] <= 0)
        or np.quantile(residual[hold], 0.9) > 0.12
        or np.quantile(reprojection[hold], 0.9) > 3.0
        or np.median(reprojection) > 1.0
        or np.quantile(reverse_error[hold], 0.9) > 3.0
    ):
        return Result("holdout_inconsistent")
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rotation, tvec.reshape(3)
    return Result(
        "candidate",
        (first, second),
        tuple(tuple(float(v) for v in row) for row in transform),
        len(inliers),
        len(p1),
        float(np.median(reprojection)),
        float(np.median(residual)),
    )


def camera_to_body_transform(
    camera_transform: Any, body_from_camera_1: Any, body_from_camera_2: Any
) -> tuple[tuple[float, ...], ...]:
    """Convert C2<-C1 to B2<-B1 with independently supplied rigid extrinsics."""
    import numpy as np

    matrices = [
        np.asarray(value, dtype=float)
        for value in (camera_transform, body_from_camera_1, body_from_camera_2)
    ]
    for matrix in matrices:
        if (
            matrix.shape != (4, 4)
            or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], (0, 0, 0, 1), atol=1e-8)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1, atol=1e-6)
        ):
            raise ValueError("invalid_rigid_transform")
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            result = matrices[2] @ matrices[0] @ np.linalg.inv(matrices[1])
    except (np.linalg.LinAlgError, FloatingPointError, OverflowError) as exc:
        raise ValueError("invalid_rigid_transform") from exc
    if not np.isfinite(result).all():
        raise ValueError("invalid_rigid_transform")
    return tuple(tuple(float(v) for v in row) for row in result)
