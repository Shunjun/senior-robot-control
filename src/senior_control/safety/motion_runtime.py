"""Timestamped local motion primitives; none of these use a global pose."""

import math
from collections import deque
from dataclasses import dataclass, replace
from typing import Generic, Literal, TypeVar

from senior_control.config import DeviceProfile
from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.safety.guard import evaluate_motion
from senior_control.safety.motion_estimator import (
    ImuYaw,
    MotionEstimate,
    MotionEstimator,
    RelativeMotion,
    WheelOdom,
    body_twist_delta,
)
from senior_control.safety.self_filter import FilteredObservation, RayEvidence
from senior_control.safety.slip import SlipMonitor


def angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


_DeferredValue = TypeVar("_DeferredValue")


@dataclass(frozen=True)
class DeferredSample(Generic[_DeferredValue]):
    stamp_ns: int
    received_ros_ns: int
    received_steady: float
    value: _DeferredValue


class DeferredSourceQueue(Generic[_DeferredValue]):
    """Bounded, source-ordered future observations retaining first receipt time."""

    def __init__(self, *, max_pending: int) -> None:
        if max_pending < 1:
            raise ValueError("deferred source capacity must be positive")
        self.max_pending = max_pending
        self._pending: deque[DeferredSample[_DeferredValue]] = deque()

    def offer(self, item: DeferredSample[_DeferredValue]) -> bool:
        if (
            item.stamp_ns <= 0
            or item.received_ros_ns <= 0
            or not math.isfinite(item.received_steady)
            or item.received_steady <= 0
            or len(self._pending) >= self.max_pending
            or (self._pending and item.stamp_ns <= self._pending[-1].stamp_ns)
        ):
            return False
        self._pending.append(item)
        return True

    def peek(self) -> DeferredSample[_DeferredValue] | None:
        return self._pending[0] if self._pending else None

    def latest(self) -> DeferredSample[_DeferredValue] | None:
        return self._pending[-1] if self._pending else None

    def consume(self) -> DeferredSample[_DeferredValue] | None:
        return self._pending.popleft() if self._pending else None

    def clear(self) -> None:
        self._pending.clear()

    def status(
        self, *, now_ros_ns: int, now_steady: float, max_age: float, pending_wait: float
    ) -> Literal["empty", "waiting", "ready", "expired", "clock_rollback"]:
        item = self.peek()
        if item is None:
            return "empty"
        if now_ros_ns < item.received_ros_ns:
            return "clock_rollback"
        if (
            not all(math.isfinite(value) for value in (now_steady, max_age, pending_wait))
            or max_age <= 0
            or pending_wait <= 0
            or not 0 <= now_steady - item.received_steady <= min(max_age, pending_wait)
            or (now_ros_ns - item.received_ros_ns) / 1e9 > max_age
        ):
            return "expired"
        if item.stamp_ns > now_ros_ns:
            return "waiting"
        if (now_ros_ns - item.stamp_ns) / 1e9 > max_age:
            return "expired"
        return "ready"


def compose_pose(a: Pose2D, b: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, angle(a.yaw + b.yaw))


def relative_pose(reference: Pose2D, target: Pose2D) -> Pose2D:
    """Pose of target expressed in reference coordinates."""
    c, s = math.cos(reference.yaw), math.sin(reference.yaw)
    dx, dy = target.x - reference.x, target.y - reference.y
    return Pose2D(c * dx + s * dy, -s * dx + c * dy, angle(target.yaw - reference.yaw))


def reframe_observation(
    observation: FilteredObservation,
    measurement_pose: Pose2D,
    reference_pose: Pose2D,
    reference_stamp: float,
) -> FilteredObservation:
    """Express the original evidence in the later independently tracked body frame."""
    if not math.isfinite(reference_stamp) or reference_stamp < observation.scan.stamp:
        raise ValueError("invalid reference stamp")
    transform = relative_pose(reference_pose, measurement_pose)

    def point(p: tuple[float, float]) -> tuple[float, float]:
        pose = compose_pose(transform, Pose2D(p[0], p[1], 0.0))
        return pose.x, pose.y

    rays = tuple(
        RayEvidence(
            ray.index, point(ray.origin), point(ray.endpoint) if ray.endpoint else None, ray.kind
        )
        for ray in observation.rays
    )
    scan = replace(
        observation.scan,
        sensor_to_body=compose_pose(transform, observation.scan.sensor_to_body),
        reference_stamp=reference_stamp,
    )
    return FilteredObservation(
        scan,
        rays,
        tuple(ray for ray in rays if ray.kind == "hit"),
        tuple(ray for ray in rays if ray.kind in ("hit", "clear")),
        tuple(ray for ray in rays if ray.kind == "self"),
        tuple(ray for ray in rays if ray.kind == "invalid"),
        tuple(tuple(point(p) for p in polygon) for polygon in observation.self_masks),
    )


@dataclass(frozen=True)
class TimedPoseSample:
    stamp: float
    pose: Pose2D
    covariance: tuple[float, float, float]
    epoch: str


@dataclass(frozen=True)
class ImuSample:
    stamp: float
    yaw_rate: float
    variance: float


@dataclass(frozen=True)
class IntegratedImu:
    yaw_delta: float
    variance: float


@dataclass(frozen=True)
class CommandInterval:
    start: float
    end: float
    pose_delta: Pose2D
    velocity: Velocity
    lateral_velocity: float
    max_sample_gap: float
    all_zero: bool


@dataclass(frozen=True)
class SafetyObservation:
    observation: FilteredObservation
    added_margin: float


@dataclass(frozen=True)
class StampedObservation:
    observation: FilteredObservation
    received: float
    received_ros: float | None = None
    clock_advance_steady: float | None = None


def reference_steady_deadline(
    stamp: float, admitted_ros: float, admitted_steady: float, max_lag: float
) -> float | None:
    """Fixed lifetime from measurement admission, never from a later dispatch."""
    if not all(math.isfinite(value) for value in (stamp, admitted_ros, admitted_steady, max_lag)):
        return None
    age = admitted_ros - stamp
    if max_lag <= 0 or not 0 <= age <= max_lag:
        return None
    return admitted_steady + max_lag - age


@dataclass(frozen=True)
class ScanInterval:
    start: float
    end: float
    body_delta: Pose2D
    covariance: tuple[float, float, float]
    quality: float


@dataclass(frozen=True)
class ProgressInterval:
    """One independently measured source interval with exact endpoint identities."""

    source_epoch: str
    start_ns: int
    end_ns: int
    scan: ScanInterval
    received_steady: float
    received_ros_ns: int


def progress_interval_fresh(
    item: ProgressInterval,
    *,
    now_steady: float,
    now_ros_ns: int,
    max_age: float,
    pending_wait: float,
) -> bool:
    """Retain each source interval's original steady and ROS expiry."""
    if not all(
        math.isfinite(value) for value in (item.received_steady, now_steady, max_age, pending_wait)
    ):
        return False
    if max_age <= 0 or pending_wait <= 0:
        return False
    return (
        0 <= now_steady - item.received_steady <= min(max_age, pending_wait)
        and 0 < item.received_ros_ns <= now_ros_ns
        and (now_ros_ns - item.received_ros_ns) / 1e9 <= max_age
        and 0 <= (now_ros_ns - item.end_ns) / 1e9 <= max_age
    )


class ProgressQueue:
    """Bounded source-order FIFO; missing continuity stays closed until requalified."""

    def __init__(self, *, max_pending: int) -> None:
        if max_pending < 1:
            raise ValueError("progress queue capacity must be positive")
        self.max_pending = max_pending
        self._pending: deque[ProgressInterval] = deque()
        self._epoch: str | None = None
        self._last_end_ns: int | None = None
        self._unqualified = False

    @property
    def unqualified(self) -> bool:
        return self._unqualified

    def diagnostic_state(self) -> dict[str, str | int | bool | ProgressInterval | None]:
        return {
            "source_epoch": self._epoch,
            "last_end_ns": self._last_end_ns,
            "pending_count": len(self._pending),
            "unqualified": self._unqualified,
            "head": self.peek(),
        }

    def offer(self, item: ProgressInterval) -> bool:
        if self._unqualified:
            return False
        if (
            not item.source_epoch
            or item.start_ns <= 0
            or item.end_ns <= item.start_ns
            or item.scan.start != item.start_ns / 1e9
            or item.scan.end != item.end_ns / 1e9
            or not math.isfinite(item.received_steady)
            or item.received_steady < 0
            or item.received_ros_ns <= 0
            or len(self._pending) >= self.max_pending
            or (self._epoch is not None and item.source_epoch != self._epoch)
            or (self._last_end_ns is not None and item.start_ns != self._last_end_ns)
        ):
            self.invalidate()
            return False
        self._epoch = item.source_epoch
        self._last_end_ns = item.end_ns
        self._pending.append(item)
        return True

    def peek(self) -> ProgressInterval | None:
        return self._pending[0] if self._pending else None

    def consume(self) -> ProgressInterval | None:
        return self._pending.popleft() if self._pending else None

    def invalidate(self) -> None:
        self._pending.clear()
        self._unqualified = True

    def reset_after_requalification(self) -> None:
        """Caller must first establish the explicit stop/requalification contract."""
        self._pending.clear()
        self._epoch = None
        self._last_end_ns = None
        self._unqualified = False


class ProgressRequalification:
    """Arm first, then require fresh zero streams and contiguous stationary evidence."""

    def __init__(self, *, max_gap: float, required_intervals: int) -> None:
        if not math.isfinite(max_gap) or max_gap <= 0 or required_intervals < 1:
            raise ValueError("invalid progress requalification limits")
        self.max_gap = max_gap
        self.required_intervals = required_intervals
        self._armed = False
        self._armed_ros_ns = 0
        self._armed_steady = 0.0
        self._requested_since: tuple[float, int] | None = None
        self._requested_last: tuple[float, int] | None = None
        self._applied_since: tuple[float, int] | None = None
        self._applied_last: tuple[float, int] | None = None
        self._candidate_epoch: str | None = None
        self._candidate_end_ns: int | None = None
        self._candidate_count = 0

    @property
    def armed(self) -> bool:
        return self._armed

    def _clear_candidate(self) -> None:
        self._candidate_epoch = None
        self._candidate_end_ns = None
        self._candidate_count = 0

    def arm(self, *, steady: float, ros_ns: int) -> None:
        if not math.isfinite(steady) or steady < 0 or ros_ns <= 0:
            raise ValueError("invalid requalification request time")
        self._armed = True
        self._armed_steady, self._armed_ros_ns = steady, ros_ns
        self._requested_since = self._requested_last = None
        self._applied_since = self._applied_last = None
        self._clear_candidate()

    def disarm(self) -> None:
        self._armed = False
        self.interrupt()

    def interrupt(self) -> None:
        """Discard zero-stream evidence without granting a new request."""
        self._requested_since = self._requested_last = None
        self._applied_since = self._applied_last = None
        self._clear_candidate()

    def _zero_sample(
        self,
        velocity: Velocity,
        steady: float,
        ros_ns: int,
        since: tuple[float, int] | None,
        last: tuple[float, int] | None,
    ) -> tuple[tuple[float, int] | None, tuple[float, int] | None]:
        if (
            not math.isfinite(steady)
            or steady < self._armed_steady
            or ros_ns < self._armed_ros_ns
            or velocity != Velocity(0.0, 0.0)
        ):
            self._clear_candidate()
            return None, None
        if (
            last is None
            or not 0 <= steady - last[0] <= self.max_gap
            or not 0 <= (ros_ns - last[1]) / 1e9 <= self.max_gap
        ):
            self._clear_candidate()
            since = (steady, ros_ns)
        return since, (steady, ros_ns)

    def requested(self, velocity: Velocity, *, steady: float, ros_ns: int) -> None:
        if self._armed:
            self._requested_since, self._requested_last = self._zero_sample(
                velocity, steady, ros_ns, self._requested_since, self._requested_last
            )

    def applied(self, velocity: Velocity, *, steady: float, ros_ns: int) -> None:
        if self._armed:
            self._applied_since, self._applied_last = self._zero_sample(
                velocity, steady, ros_ns, self._applied_since, self._applied_last
            )

    def observe(
        self,
        epoch: str,
        start_ns: int,
        end_ns: int,
        *,
        stationary: bool,
        steady: float,
        ros_ns: int,
    ) -> bool:
        samples = (
            self._requested_since,
            self._requested_last,
            self._applied_since,
            self._applied_last,
        )
        if (
            not self._armed
            or not stationary
            or not epoch
            or start_ns <= 0
            or end_ns <= start_ns
            or not math.isfinite(steady)
            or any(sample is None for sample in samples)
        ):
            self._clear_candidate()
            return False
        requested_since, requested_last, applied_since, applied_last = samples
        assert requested_since is not None and requested_last is not None
        assert applied_since is not None and applied_last is not None
        if (
            start_ns < max(self._armed_ros_ns, requested_since[1], applied_since[1])
            or not 0 <= steady - requested_last[0] <= self.max_gap
            or not 0 <= steady - applied_last[0] <= self.max_gap
            or not 0 <= (ros_ns - requested_last[1]) / 1e9 <= self.max_gap
            or not 0 <= (ros_ns - applied_last[1]) / 1e9 <= self.max_gap
            or not 0 <= (ros_ns - end_ns) / 1e9 <= self.max_gap
        ):
            self._clear_candidate()
            return False
        if self._candidate_epoch != epoch or self._candidate_end_ns != start_ns:
            self._candidate_count = 0
        self._candidate_epoch, self._candidate_end_ns = epoch, end_ns
        self._candidate_count += 1
        if self._candidate_count < self.required_intervals:
            return False
        self.disarm()
        return True


@dataclass(frozen=True)
class QualifiedInterval:
    estimate: MotionEstimate
    wheel: WheelOdom
    anomalous: bool


@dataclass(frozen=True)
class AlignedInterval:
    estimate: MotionEstimate
    wheel: WheelOdom
    inertial: IntegratedImu


class SourceEpochFence:
    """Remember retired producer identities; exhaustion blocks rather than forgetting."""

    def __init__(self, *, max_retired: int) -> None:
        if max_retired < 1:
            raise ValueError("epoch capacity must be positive")
        self.max_retired = max_retired
        self.active: str | None = None
        self._retired: set[str] = set()
        self._exhausted = False

    def retire_active(self) -> None:
        if self.active is None:
            return
        if len(self._retired) >= self.max_retired:
            self._exhausted = True
        else:
            self._retired.add(self.active)
        self.active = None

    def is_retired(self, epoch: str) -> bool:
        return epoch in self._retired

    def retire_unadmitted(self, epoch: str) -> None:
        """Fence a replaced deferred source without making it the active source."""
        if not epoch or epoch in self._retired:
            return
        if self.active == epoch:
            self.retire_active()
        elif len(self._retired) >= self.max_retired:
            self._exhausted = True
        else:
            self._retired.add(epoch)

    def accept(self, epoch: str) -> bool:
        if self._exhausted or not epoch or epoch in self._retired:
            return False
        if self.active is not None and self.active != epoch:
            self.retire_active()
            if self._exhausted:
                return False
        self.active = epoch
        return True


class SourceStampFrontier:
    """Retain the newest valid or invalid source stamp within one accepted epoch."""

    def __init__(self) -> None:
        self._latest_ns = 0
        self._failure_ns = 0

    def reset(self) -> None:
        self._latest_ns = 0
        self._failure_ns = 0

    def accept(self, stamp_ns: int, *, eligible: bool, receipt_ros_ns: int) -> bool:
        if stamp_ns <= 0:
            if eligible:
                return False
            # An undated failure establishes no generation time. A subsequent
            # qualifying observation must be newer than its receipt instead.
            self._latest_ns = max(self._latest_ns, receipt_ros_ns)
            self._failure_ns = max(self._failure_ns, receipt_ros_ns)
            return True
        if stamp_ns < self._latest_ns or (stamp_ns == self._latest_ns and eligible):
            return False
        self._latest_ns = stamp_ns
        if not eligible:
            self._failure_ns = stamp_ns
        return True

    def interval_after_failure(self, start_ns: int) -> bool:
        """A qualified interval may start at, but not before, the last failure."""
        return 0 < start_ns and start_ns >= self._failure_ns


class StopAcknowledgement:
    """A forced safety zero cannot acknowledge the request that caused the stop."""

    def __init__(self) -> None:
        self.clear()

    def clear(self) -> None:
        self._tripped = False
        self._requested_zero = False
        self._applied_zero = False

    def trip(self) -> None:
        if not self._tripped:
            self._tripped = True
            self._requested_zero = self._applied_zero = False

    def requested(self, velocity: Velocity) -> None:
        if self._tripped:
            self._requested_zero = velocity == Velocity(0, 0)

    def applied(self, velocity: Velocity) -> None:
        if self._tripped:
            self._applied_zero = velocity == Velocity(0, 0)

    @property
    def ready(self) -> bool:
        return self._tripped and self._requested_zero and self._applied_zero


class ProducerTopology:
    """Require one expected ROS graph endpoint; this is not DDS authentication."""

    def __init__(self, name: str, namespace: str) -> None:
        if not name or not namespace:
            raise ValueError("expected producer identity is required")
        self._name = name
        self._namespace = namespace
        self._gid: bytes | None = None

    def observe(self, endpoints: list[tuple[str, str, bytes]]) -> tuple[bool, bool]:
        gid = None
        if len(endpoints) == 1:
            name, namespace, candidate = endpoints[0]
            if name == self._name and namespace == self._namespace and candidate and any(candidate):
                gid = candidate
        changed = gid != self._gid
        self._gid = gid
        return gid is not None, changed


def _valid_limits(max_gap: float, max_samples: int, max_variance: float) -> bool:
    return (
        math.isfinite(max_gap)
        and max_gap > 0
        and max_samples >= 2
        and math.isfinite(max_variance)
        and max_variance > 0
    )


class TimedPoseBuffer:
    def __init__(self, *, max_gap: float, max_samples: int, max_variance: float) -> None:
        if not _valid_limits(max_gap, max_samples, max_variance):
            raise ValueError("invalid pose buffer limits")
        self.max_gap = max_gap
        self.max_variance = max_variance
        self._samples: deque[TimedPoseSample] = deque(maxlen=max_samples)

    def clear(self) -> None:
        self._samples.clear()

    def diagnostic_bounds(self) -> tuple[float | None, float | None, int]:
        if not self._samples:
            return None, None, 0
        return self._samples[0].stamp, self._samples[-1].stamp, len(self._samples)

    def snapshot(self) -> "TimedPoseBuffer":
        """Copy bounded immutable samples before crossing a worker boundary."""
        result = TimedPoseBuffer(
            max_gap=self.max_gap,
            max_samples=self._samples.maxlen or 2,
            max_variance=self.max_variance,
        )
        result._samples.extend(self._samples)
        return result

    def add(self, sample: TimedPoseSample) -> bool:
        if (
            not math.isfinite(sample.stamp)
            or sample.stamp <= 0
            or not sample.epoch
            or len(sample.covariance) != 3
            or any(
                not math.isfinite(v) or not 0 < v <= self.max_variance for v in sample.covariance
            )
            or (
                self._samples
                and (
                    sample.stamp <= self._samples[-1].stamp
                    or sample.epoch != self._samples[-1].epoch
                )
            )
        ):
            self.clear()
            return False
        self._samples.append(sample)
        return True

    @property
    def latest(self) -> TimedPoseSample | None:
        return self._samples[-1] if self._samples else None

    def at(self, stamp: float) -> TimedPoseSample | None:
        if not math.isfinite(stamp):
            return None
        for i, sample in enumerate(self._samples):
            if stamp == sample.stamp:
                return sample
            if stamp < sample.stamp and not i:
                return None
            if stamp < sample.stamp and i:
                prev = self._samples[i - 1]
                gap = sample.stamp - prev.stamp
                if gap > self.max_gap:
                    return None
                fraction = (stamp - prev.stamp) / gap
                pose = Pose2D(
                    prev.pose.x + fraction * (sample.pose.x - prev.pose.x),
                    prev.pose.y + fraction * (sample.pose.y - prev.pose.y),
                    angle(prev.pose.yaw + fraction * angle(sample.pose.yaw - prev.pose.yaw)),
                )
                covariance = tuple(
                    max(a, b) for a, b in zip(prev.covariance, sample.covariance, strict=False)
                )
                return TimedPoseSample(stamp, pose, covariance, sample.epoch)  # type: ignore[arg-type]
        return None

    def interval_uncertainty(self, start: float, end: float) -> tuple[float, float] | None:
        """Conservatively add per-interval standard deviations, without extrapolation."""
        if end < start or self.at(start) is None or self.at(end) is None:
            return None
        linear = yaw = 0.0
        for previous, sample in zip(self._samples, tuple(self._samples)[1:], strict=False):
            if previous.stamp >= end or sample.stamp <= start:
                continue
            if sample.stamp - previous.stamp > self.max_gap:
                return None
            linear += math.sqrt(sample.covariance[0] + sample.covariance[1])
            yaw += math.sqrt(sample.covariance[2])
        return linear, yaw

    def interpolation_horizon(self, stamp: float) -> float | None:
        """Worst-case deviation time from a linear interpolation under a speed bound."""
        if self.at(stamp) is None:
            return None
        for previous, sample in zip(self._samples, tuple(self._samples)[1:], strict=False):
            if stamp == previous.stamp or stamp == sample.stamp:
                return 0.0
            if previous.stamp < stamp < sample.stamp:
                return (
                    2
                    * (stamp - previous.stamp)
                    * (sample.stamp - stamp)
                    / (sample.stamp - previous.stamp)
                )
        return 0.0


def prepare_safety_observation(
    observation: FilteredObservation,
    history: TimedPoseBuffer,
    *,
    now: float,
    max_age: float,
    max_reference_lag: float,
    linear_bound: float,
    yaw_bound: float,
    radius: float,
) -> SafetyObservation | None:
    values = (now, max_age, max_reference_lag, linear_bound, yaw_bound, radius)
    if not all(math.isfinite(v) for v in values) or any(v <= 0 for v in values[1:]):
        return None
    reference = history.latest
    measurement = history.at(observation.scan.stamp)
    if (
        reference is None
        or measurement is None
        or measurement.epoch != reference.epoch
        or not 0 <= now - observation.scan.stamp <= max_age
        or not 0 <= now - reference.stamp <= max_reference_lag
        or observation.scan.stamp > reference.stamp
    ):
        return None
    uncertainty = history.interval_uncertainty(observation.scan.stamp, reference.stamp)
    interpolation = history.interpolation_horizon(observation.scan.stamp)
    if uncertainty is None or interpolation is None:
        return None
    lag = now - reference.stamp
    linear_error, yaw_error = uncertainty
    margin = (linear_bound + radius * yaw_bound) * (lag + interpolation) + 3.0 * (
        linear_error + radius * yaw_error
    )
    return SafetyObservation(
        reframe_observation(observation, measurement.pose, reference.pose, reference.stamp), margin
    )


def select_safety_observation(
    samples: list[StampedObservation],
    history: TimedPoseBuffer,
    *,
    steady: float,
    now: float,
    max_age: float,
    max_reference_lag: float,
    linear_bound: float,
    yaw_bound: float,
    radius: float,
) -> SafetyObservation | None:
    if not math.isfinite(steady):
        return None
    for sample in reversed(samples):
        if not math.isfinite(sample.received) or not 0 <= steady - sample.received <= max_age:
            continue
        prepared = prepare_safety_observation(
            sample.observation,
            history,
            now=now,
            max_age=max_age,
            max_reference_lag=max_reference_lag,
            linear_bound=linear_bound,
            yaw_bound=yaw_bound,
            radius=radius,
        )
        if prepared is not None:
            return prepared
    return None


def newer_scan_veto(
    latest: StampedObservation,
    selected_stamp: float,
    command: Velocity,
    measured: BodyTwist | Velocity,
    profile: DeviceProfile,
    *,
    now: float,
    steady: float,
    linear_bound: float,
    yaw_bound: float,
    radius: float,
    motion_sweep_margin: float = 0.0,
) -> bool:
    """Newer hits veto while their pose is pending; they never add free coverage."""
    # //Compatible: callers without additional uncertainty retain the prior margin.
    if not math.isfinite(motion_sweep_margin) or motion_sweep_margin < 0:
        return True
    stamp = latest.observation.scan.stamp
    if stamp <= selected_stamp:
        return False
    max_age = profile.observation_max_age
    if (
        max_age is None
        or not 0 <= now - stamp <= max_age
        or not 0 <= steady - latest.received <= max_age
        or not latest.observation.free_rays
    ):
        return True
    uncertainty = (linear_bound + radius * yaw_bound) * (now - stamp) + motion_sweep_margin
    decision = evaluate_motion(
        command, measured, [latest.observation], profile, now, uncertainty_margin=uncertainty
    )
    return not decision.allowed and decision.reason not in {
        "no usable observation area",
        "required area contains unknown space",
    }


class TimedImuBuffer:
    def __init__(self, *, max_gap: float, max_samples: int, max_variance: float) -> None:
        if not _valid_limits(max_gap, max_samples, max_variance):
            raise ValueError("invalid IMU buffer limits")
        self.max_gap = max_gap
        self.max_variance = max_variance
        self._samples: deque[ImuSample] = deque(maxlen=max_samples)

    def diagnostic_bounds(self) -> tuple[float | None, float | None, int]:
        if not self._samples:
            return None, None, 0
        return self._samples[0].stamp, self._samples[-1].stamp, len(self._samples)

    def add(self, sample: ImuSample) -> bool:
        if (
            not math.isfinite(sample.stamp)
            or sample.stamp <= 0
            or not math.isfinite(sample.yaw_rate)
            or not math.isfinite(sample.variance)
            or not 0 < sample.variance <= self.max_variance
            or (self._samples and sample.stamp <= self._samples[-1].stamp)
        ):
            self._samples.clear()
            return False
        self._samples.append(sample)
        return True

    def _at(self, stamp: float) -> ImuSample | None:
        for i, sample in enumerate(self._samples):
            if sample.stamp == stamp:
                return sample
            if stamp < sample.stamp and not i:
                return None
            if stamp < sample.stamp and i:
                prev = self._samples[i - 1]
                gap = sample.stamp - prev.stamp
                if gap > self.max_gap:
                    return None
                fraction = (stamp - prev.stamp) / gap
                return ImuSample(
                    stamp,
                    prev.yaw_rate + fraction * (sample.yaw_rate - prev.yaw_rate),
                    max(prev.variance, sample.variance),
                )
        return None

    def integrate(self, start: float, end: float) -> IntegratedImu | None:
        if not math.isfinite(start) or not math.isfinite(end) or end <= start:
            return None
        first, last = self._at(start), self._at(end)
        if first is None or last is None:
            return None
        samples = [first, *(s for s in self._samples if start < s.stamp < end), last]
        yaw, variance = 0.0, 0.0
        for a, b in zip(samples, samples[1:], strict=False):
            dt = b.stamp - a.stamp
            if dt <= 0 or dt > self.max_gap:
                return None
            yaw += 0.5 * (a.yaw_rate + b.yaw_rate) * dt
            # Summed variance is deliberately conservative about correlation.
            variance += max(a.variance, b.variance) * dt
        return IntegratedImu(angle(yaw), variance)


class TimedCommandBuffer:
    def __init__(self, *, max_gap: float, max_samples: int) -> None:
        if not _valid_limits(max_gap, max_samples, 1.0):
            raise ValueError("invalid command history limits")
        self.max_gap = max_gap
        self._samples: deque[tuple[float, Velocity]] = deque(maxlen=max_samples)

    def diagnostic_bounds(self) -> tuple[float | None, float | None, int]:
        if not self._samples:
            return None, None, 0
        return self._samples[0][0], self._samples[-1][0], len(self._samples)

    def add(self, stamp: float, velocity: Velocity) -> bool:
        if (
            not math.isfinite(stamp)
            or stamp <= 0
            or (self._samples and stamp <= self._samples[-1][0])
        ):
            self._samples.clear()
            return False
        self._samples.append((stamp, velocity))
        return True

    def integrated(self, start: float, end: float) -> CommandInterval | None:
        if not math.isfinite(start) or not math.isfinite(end) or end <= start:
            return None
        preceding = [sample for sample in self._samples if sample[0] <= start]
        if not preceding:
            return None
        last_stamp, current = preceding[-1]
        if start - last_stamp > self.max_gap:
            return None
        segments: list[tuple[Velocity, float]] = []
        cursor = start
        largest_gap = start - last_stamp
        for next_stamp, value in self._samples:
            if not start < next_stamp <= end:
                continue
            gap = next_stamp - last_stamp
            if gap > self.max_gap:
                return None
            largest_gap = max(largest_gap, gap)
            segments.append((current, next_stamp - cursor))
            cursor, last_stamp, current = next_stamp, next_stamp, value
        if end - last_stamp > self.max_gap:
            return None
        largest_gap = max(largest_gap, end - last_stamp)
        segments.append((current, end - cursor))
        x_sign = yaw_sign = 0
        total = Pose2D(0.0, 0.0, 0.0)
        total_yaw = 0.0
        for command, duration in segments:
            if duration <= 0:
                continue
            for component, sign in ((command.vx, x_sign), (command.wz, yaw_sign)):
                if component * sign < 0:
                    return None
            if command.vx:
                x_sign = 1 if command.vx > 0 else -1
            if command.wz:
                yaw_sign = 1 if command.wz > 0 else -1
            yaw = command.wz * duration
            total_yaw += yaw
            if abs(total_yaw) >= math.pi:
                return None
            if abs(yaw) < 1e-8:
                sinc, cosc = 1.0 - yaw * yaw / 6.0, yaw / 2.0 - yaw * yaw * yaw / 24.0
            else:
                sinc, cosc = math.sin(yaw) / yaw, 2.0 * math.sin(yaw / 2.0) ** 2 / yaw
            distance = command.vx * duration
            total = compose_pose(total, Pose2D(distance * sinc, distance * cosc, yaw))
        twist = body_twist_delta(total)
        dt = end - start
        return CommandInterval(
            start,
            end,
            total,
            Velocity(twist.x / dt, twist.yaw / dt),
            twist.y / dt,
            largest_gap,
            all(command == Velocity(0.0, 0.0) for command, duration in segments if duration > 0),
        )

    def interval(self, start: float, end: float) -> Velocity | None:
        # //Compatible: callers needing only x/yaw retain the prior return type.
        result = self.integrated(start, end)
        return result.velocity if result is not None else None


def align_interval(
    scan: ScanInterval,
    wheel_history: TimedPoseBuffer,
    imu_history: TimedImuBuffer,
    estimator: MotionEstimator,
) -> AlignedInterval | None:
    """Build fully aligned independent, wheel and inertial observations."""
    if not math.isfinite(scan.start) or not math.isfinite(scan.end) or scan.end <= scan.start:
        return None
    start, end = wheel_history.at(scan.start), wheel_history.at(scan.end)
    inertial = imu_history.integrate(scan.start, scan.end)
    if (
        start is None
        or end is None
        or start.epoch != end.epoch
        or wheel_history.interval_uncertainty(scan.start, scan.end) is None
        or inertial is None
        or not math.isfinite(inertial.variance)
        or inertial.variance <= 0
        or estimator.max_covariance is None
        or inertial.variance > estimator.max_covariance
        or any(not math.isfinite(v) or v <= 0 for v in scan.covariance)
    ):
        return None
    dt = scan.end - scan.start
    wheel = WheelOdom(relative_pose(start.pose, end.pose), scan.end, dt)
    scan_motion = RelativeMotion(
        scan.body_delta, scan.covariance, scan.quality, False, scan.end, "scan_match", False, dt
    )
    estimate = estimator.update(
        wheel, scan_motion, ImuYaw(inertial.yaw_delta, scan.end, dt), None, scan.end
    )
    if estimate.quality <= 0:
        return None
    return AlignedInterval(estimate, wheel, inertial)


def stationary_interval(
    aligned: AlignedInterval,
    command: CommandInterval,
    *,
    max_linear_speed: float,
    max_yaw_rate: float,
) -> bool:
    """Require aligned, observed near-stillness and zero command for the full interval."""
    if (
        not all(math.isfinite(value) and value > 0 for value in (max_linear_speed, max_yaw_rate))
        or not command.all_zero
        or aligned.estimate.dt <= 0
        or aligned.wheel.dt <= 0
    ):
        return False
    body = body_twist_delta(aligned.estimate.pose_delta)
    wheel = body_twist_delta(aligned.wheel.pose_delta)
    return (
        math.hypot(body.x, body.y) / aligned.estimate.dt <= max_linear_speed
        and math.hypot(wheel.x, wheel.y) / aligned.wheel.dt <= max_linear_speed
        and abs(body.yaw) / aligned.estimate.dt <= max_yaw_rate
        and abs(wheel.yaw) / aligned.wheel.dt <= max_yaw_rate
        and abs(aligned.inertial.yaw_delta) / aligned.estimate.dt <= max_yaw_rate
    )


def qualify_interval(
    scan: ScanInterval,
    wheel_history: TimedPoseBuffer,
    imu_history: TimedImuBuffer,
    command: Velocity,
    estimator: MotionEstimator,
    slip: SlipMonitor,
) -> QualifiedInterval | None:
    """Compare only samples covering the same independent scan interval."""
    aligned = align_interval(scan, wheel_history, imu_history, estimator)
    if aligned is None:
        return None
    anomaly = slip.update(command, aligned.wheel, aligned.estimate)
    return QualifiedInterval(aligned.estimate, aligned.wheel, anomaly.anomalous)
