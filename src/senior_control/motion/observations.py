"""Source values and pure admission checks; SI units, no motion permission.

Covariance covers vx/vy/wz for laser/wheel, ENU east/north for GPS, and
body yaw rate for IMU. Diagonal-only data explicitly leaves correlation unknown.
"""

import math
from dataclasses import dataclass
from enum import Enum


class Source(str, Enum):
    LASER = "laser"
    GPS = "gps"
    IMU = "imu"
    WHEEL = "wheel"


class Frame(str, Enum):
    BODY = "body"
    ENU = "enu"


class SourceStatus(str, Enum):
    VALID = "valid"
    UNAVAILABLE = "unavailable"
    DRIFT = "drift"


class CovarianceKind(str, Enum):
    FULL = "full"
    DIAGONAL_ONLY = "diagonal_only_unknown_correlation"


@dataclass(frozen=True)
class Covariance:
    values: tuple[float, ...]
    kind: CovarianceKind
    basis: str
    correlation_group: str | None


@dataclass(frozen=True)
class Observation:
    source: Source
    frame: Frame
    start_ros: float
    end_ros: float
    received_steady: float
    epoch: str
    sequence: int
    quality: float
    status: SourceStatus
    values: tuple[float, ...]
    covariance: Covariance


@dataclass(frozen=True)
class Heading:
    """Known body yaw in ENU, with caller-supplied calibration/uncertainty basis."""

    yaw_rad: float
    variance_rad2: float
    stamp_ros: float
    received_steady: float
    epoch: str
    basis: str


@dataclass(frozen=True)
class SourceRule:
    source: Source
    max_age_s: float
    max_interval_s: float
    min_quality: float
    max_variance: float

    def __post_init__(self) -> None:
        numbers = (self.max_age_s, self.max_interval_s, self.min_quality, self.max_variance)
        if not all(math.isfinite(x) and x > 0 for x in numbers) or self.min_quality > 1:
            raise ValueError("invalid explicitly supplied source rule")


@dataclass(frozen=True)
class AdmissionConfig:
    rules: tuple[SourceRule, ...]
    recovery_samples: int
    recovery_span_s: float
    normal_cap_mps: float
    degraded_cap_mps: float
    heading_max_age_s: float
    alignment_max_skew_s: float
    heading_max_variance_rad2: float
    clock_pause_s: float
    epoch_history_limit: int

    def __post_init__(self) -> None:
        if len(self.rules) != len(Source) or {r.source for r in self.rules} != set(Source):
            raise ValueError("one rule per source required")
        positive = (
            self.normal_cap_mps,
            self.degraded_cap_mps,
            self.heading_max_age_s,
            self.alignment_max_skew_s,
            self.heading_max_variance_rad2,
            self.clock_pause_s,
        )
        if (
            not all(math.isfinite(x) and x > 0 for x in positive)
            or not math.isfinite(self.recovery_span_s)
            or self.recovery_span_s < 0
            or self.recovery_samples < 1
            or self.epoch_history_limit < 1
            or self.degraded_cap_mps > self.normal_cap_mps
        ):
            raise ValueError("invalid injected admission/recovery policy")

    def rule(self, source: Source) -> SourceRule:
        return next(rule for rule in self.rules if rule.source is source)


@dataclass(frozen=True)
class AdaptedObservation:
    original: Observation
    body_values: tuple[float, ...]
    body_covariance: Covariance | None
    valid_until_steady: float
    heading_context: Heading | None


def covariance_valid(covariance: Covariance, dimension: int, maximum: float) -> bool:
    values = covariance.values
    if not covariance.basis or not all(math.isfinite(x) for x in values):
        return False
    if covariance.kind is CovarianceKind.DIAGONAL_ONLY:
        return len(values) == dimension and all(0 <= x <= maximum for x in values)
    if covariance.kind is not CovarianceKind.FULL or len(values) != dimension * dimension:
        return False
    matrix = [values[i * dimension : (i + 1) * dimension] for i in range(dimension)]
    if any(not 0 <= matrix[i][i] <= maximum for i in range(dimension)):
        return False
    # Scale before checking principal minors: malformed finite cross terms must
    # reject, not overflow during admission. This is a PSD test, not fusion.
    scale = max((matrix[i][i] for i in range(dimension)), default=0.0) or 1.0
    normalized = [[value / scale for value in row] for row in matrix]
    for i in range(dimension):
        for j in range(i):
            if matrix[i][j] != matrix[j][i]:
                return False
            if abs(normalized[i][j]) > math.sqrt(normalized[i][i] * normalized[j][j]):
                return False
    if dimension == 3:
        a, b, c = normalized[0]
        _, d, e = normalized[1]
        _, _, f = normalized[2]
        if a * d * f + 2 * b * c * e - a * e * e - d * c * c - f * b * b < 0:
            return False
    return True


def _number(value: object) -> bool:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        float(value)
    except OverflowError:
        return False
    return True


def metadata_error(
    observation: Observation, heading: Heading | None, *, now_ros: float, now_steady: float
) -> str | None:
    """No owner access: reject malformed values before hashing or ordering them.

    Nonfinite component/covariance *measurements* remain payload failures;
    malformed identity/time metadata never changes owner clocks or watermarks.
    """
    if not isinstance(observation, Observation) or not isinstance(observation.source, Source):
        return "invalid_source"
    if (
        type(observation.epoch) is not str
        or not observation.epoch
        or type(observation.sequence) is not int
        or observation.sequence < 1
    ):
        return "invalid_metadata"
    if not isinstance(observation.frame, Frame) or not isinstance(observation.status, SourceStatus):
        return "invalid_structure"
    stamps = (
        observation.start_ros,
        observation.end_ros,
        observation.received_steady,
        observation.quality,
    )
    if not all(_number(value) and math.isfinite(value) for value in stamps):
        return "invalid_metadata"
    covariance = observation.covariance
    if (
        not isinstance(observation.values, tuple)
        or not all(_number(value) for value in observation.values)
        or not isinstance(covariance, Covariance)
        or not isinstance(covariance.kind, CovarianceKind)
        or not isinstance(covariance.values, tuple)
        or not all(_number(value) for value in covariance.values)
        or not isinstance(covariance.basis, str)
        or (
            covariance.correlation_group is not None
            and not isinstance(covariance.correlation_group, str)
        )
    ):
        return "invalid_structure"
    if observation.source is Source.GPS and heading is not None:
        if not isinstance(heading, Heading) or type(heading.epoch) is not str:
            return "invalid_heading_structure"
        if not isinstance(heading.basis, str) or not all(
            _number(value)
            for value in (
                heading.yaw_rad,
                heading.variance_rad2,
                heading.stamp_ros,
                heading.received_steady,
            )
        ):
            return "invalid_heading_structure"
    if (
        observation.end_ros > now_ros
        or observation.received_steady > now_steady
        or observation.start_ros >= observation.end_ros
    ):
        return "future_or_invalid_interval"
    return None


def is_current(
    observation: AdaptedObservation, rule: SourceRule, *, now_ros: float, now_steady: float
) -> bool:
    """The same ROS-age and fixed steady-deadline test for ingest and assess."""
    return (
        now_steady < observation.valid_until_steady
        and now_ros - observation.original.end_ros <= rule.max_age_s
    )


def validate(
    observation: Observation,
    rule: SourceRule,
    *,
    now_ros: float,
    now_steady: float,
    heading: Heading | None,
    config: AdmissionConfig,
) -> str | None:
    structural = metadata_error(observation, heading, now_ros=now_ros, now_steady=now_steady)
    if structural is not None:
        return structural
    if (
        now_ros - observation.end_ros > rule.max_age_s
        or now_steady - observation.received_steady > rule.max_age_s
        or observation.end_ros - observation.start_ros > rule.max_interval_s
    ):
        return "expired_or_long_interval"
    if observation.status is not SourceStatus.VALID:
        return str(observation.status.value)
    if not rule.min_quality <= observation.quality <= 1:
        return "quality"
    dimension = {Source.LASER: 3, Source.WHEEL: 3, Source.GPS: 2, Source.IMU: 1}[observation.source]
    if len(observation.values) != dimension or not all(
        math.isfinite(x) for x in observation.values
    ):
        return "invalid_components"
    expected_frame = Frame.ENU if observation.source is Source.GPS else Frame.BODY
    if observation.frame is not expected_frame:
        return "frame"
    if not covariance_valid(observation.covariance, dimension, rule.max_variance):
        return "covariance"
    if observation.source is Source.GPS:
        if heading is None:
            return "heading_missing"
        values = (
            heading.yaw_rad,
            heading.variance_rad2,
            heading.stamp_ros,
            heading.received_steady,
        )
        if (
            not all(math.isfinite(x) for x in values)
            or not 0 <= heading.variance_rad2 <= config.heading_max_variance_rad2
            or not heading.epoch
            or not heading.basis
            or heading.stamp_ros > now_ros
            or heading.received_steady > now_steady
            or now_ros - heading.stamp_ros > config.heading_max_age_s
            or now_steady - heading.received_steady > config.heading_max_age_s
            or abs(heading.stamp_ros - observation.end_ros) > config.alignment_max_skew_s
        ):
            return "heading_unqualified"
    return None


def adapt(
    observation: Observation,
    *,
    now_ros: float,
    now_steady: float,
    rule: SourceRule,
    heading: Heading | None,
) -> AdaptedObservation:
    values = observation.values
    covariance: Covariance | None = observation.covariance
    heading_context = None
    if observation.source is Source.GPS:
        assert heading is not None  # Admission has already proved the transform context.
        east, north = values
        c, s = math.cos(heading.yaw_rad), math.sin(heading.yaw_rad)
        values = (c * east + s * north, -s * east + c * north)
        # T3b owns heading/velocity cross-correlation; never assume independence.
        covariance = None
        heading_context = heading
    deadline = min(
        observation.received_steady + rule.max_age_s,
        now_steady + rule.max_age_s - (now_ros - observation.end_ros),
    )
    return AdaptedObservation(observation, values, covariance, deadline, heading_context)
