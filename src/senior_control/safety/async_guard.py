"""One owned, spawned geometry worker and immutable authorization tickets."""

import math
import multiprocessing
import queue
import threading
import time
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from senior_control.config import DeviceProfile
from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.safety.depth_geometry import evaluate_depth_candidates
from senior_control.safety.guard import SafetyDecision, evaluate_motion, stopping_distance
from senior_control.safety.motion_runtime import (
    StampedObservation,
    TimedPoseBuffer,
    newer_scan_veto,
    select_safety_observation,
)
from senior_control.safety.self_filter import ScanObservation, filter_self
from senior_control.safety.velocity_uncertainty import IntervalVelocityBounds


@dataclass(frozen=True)
class RawSafetyObservation:
    """Bounded ingress message retained without decoding its rays in the ROS executor."""

    message: Any
    received: float
    received_ros: float
    clock_advance_steady: float
    stamp: float


@dataclass(frozen=True)
class SafetyTicket:
    command_generation: int
    scan_generation: int
    motion_generation: int
    epoch: str
    command: Velocity
    steady_start: float
    steady_deadline: float
    ros_start: float
    ros_deadline: float
    configuration: tuple[object, ...] = ()

    def permits(
        self,
        *,
        command_generation: int,
        scan_generation: int,
        motion_generation: int,
        epoch: str | None,
        command: Velocity,
        steady: float,
        ros: float,
        configuration: tuple[object, ...] = (),
    ) -> bool:
        return (
            self.command_generation == command_generation
            and self.scan_generation == scan_generation
            and self.motion_generation == motion_generation
            and bool(self.epoch)
            and self.epoch == epoch
            and self.command == command
            and self.configuration == configuration
            and all(
                math.isfinite(value)
                for value in (
                    steady,
                    ros,
                    self.steady_start,
                    self.steady_deadline,
                    self.ros_start,
                    self.ros_deadline,
                )
            )
            and self.steady_start <= steady <= self.steady_deadline
            and self.ros_start <= ros <= self.ros_deadline
        )


@dataclass(frozen=True)
class SafetyWork:
    command: Velocity
    measured: BodyTwist
    profile: DeviceProfile
    observations: tuple[StampedObservation | RawSafetyObservation, ...]
    history: TimedPoseBuffer
    evaluation_ros_time: float
    evaluation_steady_time: float
    max_reference_lag: float
    linear_bound: float
    yaw_bound: float
    radius: float
    diagnostics: bool
    interpolate: bool = False
    max_angular_gap: float = 0.0
    no_return_clear: bool = False
    transform_static: bool = False
    self_mask_range_uncertainty: float = 0.0
    body_frame: str = "base_link"
    interval_bounds: IntervalVelocityBounds | None = None
    physical_cap_qualified: bool = False
    # //Compatible: existing laser-only callers do not provide depth work.
    depth_required: bool = False
    depth_message: Any = None
    depth_collision_z: tuple[float, float] | None = None


@dataclass(frozen=True)
class SafetyWorkResult:
    decision: SafetyDecision
    diagnostics: dict[str, object]
    compute_seconds: float
    selected_received: float | None = None
    selected_stamp: float | None = None
    reference_stamp: float | None = None
    uncertainty_margin: float | None = None
    sensor_origin: tuple[float, float, float] | None = None
    self_masked_rays: int = 0
    ambiguous_rays: int = 0


_filtered_cache: OrderedDict[tuple[object, ...], StampedObservation] = OrderedDict()


def _filtered_observations(work: SafetyWork) -> tuple[StampedObservation, ...]:
    """Decode/filter only inside the spawned worker; reuse at most eight scans."""
    result = []
    profile_key = work.profile.model_dump_json()
    for sample in work.observations:
        if isinstance(sample, StampedObservation):
            result.append(sample)
            continue
        msg = sample.message
        key = (
            msg.source_epoch,
            msg.sequence,
            profile_key,
            work.interpolate,
            work.max_angular_gap,
            work.no_return_clear,
            work.transform_static,
            work.self_mask_range_uncertainty,
            work.body_frame,
        )
        filtered = _filtered_cache.get(key)
        if filtered is None:
            scan = ScanObservation(
                tuple(msg.ranges),
                msg.angle_min,
                msg.angle_increment,
                msg.range_min,
                msg.range_max,
                sample.stamp,
                "ros",
                Pose2D(msg.sensor_x, msg.sensor_y, msg.sensor_yaw),
                msg.transform_stamp,
                work.body_frame,
                sample.stamp,
                work.interpolate,
                work.max_angular_gap,
                work.no_return_clear,
                work.transform_static,
            )
            observation = filter_self(
                scan, work.profile, self_mask_range_uncertainty=work.self_mask_range_uncertainty
            )
            if not observation.free_rays:
                raise ValueError("scan has no usable returns")
            filtered = StampedObservation(
                observation, sample.received, sample.received_ros, sample.clock_advance_steady
            )
            _filtered_cache[key] = filtered
            while len(_filtered_cache) > 8:
                _filtered_cache.popitem(last=False)
        else:
            _filtered_cache.move_to_end(key)
        result.append(filtered)
    return tuple(result)


def evaluate_safety_work(work: SafetyWork) -> SafetyWorkResult:
    """Run geometry at the ticket's future deadline, not at a restamped scan."""
    start = time.monotonic()
    details: dict[str, object] = {}
    interval = work.interval_bounds
    if (
        not work.physical_cap_qualified
        or interval is None
        or not all(math.isfinite(value) and value >= 0 for value in (interval.linear, interval.yaw))
        or not all(
            math.isfinite(value) and value > 0 for value in (work.linear_bound, work.yaw_bound)
        )
    ):
        decision = SafetyDecision(False, Velocity(0.0, 0.0), "motion_uncertainty_unqualified")
        return SafetyWorkResult(decision, details, time.monotonic() - start)
    braking = work.profile.braking_deceleration
    lateral_braking = work.profile.lateral_braking_deceleration
    yaw_braking = work.profile.yaw_braking_deceleration
    reaction = work.profile.reaction_time
    if braking is None or lateral_braking is None or yaw_braking is None or reaction is None:
        decision = SafetyDecision(False, Velocity(0.0, 0.0), "motion_braking_unqualified")
        return SafetyWorkResult(decision, details, time.monotonic() - start)
    # The interval-average confidence cannot bound instantaneous speed. Use
    # the complete separately assumed physical cap in each body axis.
    translation = stopping_distance(work.linear_bound, reaction, braking) + stopping_distance(
        work.linear_bound, reaction, lateral_braking
    )
    yaw_stop = stopping_distance(work.yaw_bound, reaction, yaw_braking)
    body_radius = max(math.hypot(x, y) for x, y in work.profile.footprint)
    rotation = 2 * body_radius * math.sin(min(math.pi, yaw_stop) / 2)
    motion_margin = translation + rotation
    details.update(
        interval_linear_upper=interval.linear,
        interval_yaw_upper=interval.yaw,
        assumed_instantaneous_linear_cap=work.linear_bound,
        assumed_instantaneous_yaw_cap=work.yaw_bound,
        motion_uncertainty_sweep_margin=motion_margin,
    )
    if work.depth_required:
        if work.depth_message is None or work.depth_collision_z is None:
            decision = SafetyDecision(False, Velocity(0.0, 0.0), "depth_unqualified")
            return SafetyWorkResult(decision, details, time.monotonic() - start)
        depth = evaluate_depth_candidates(
            work.depth_message,
            work.history,
            now=work.evaluation_ros_time,
            max_age=work.profile.observation_max_age or 0.0,
            max_reference_lag=work.max_reference_lag,
            linear_bound=work.linear_bound,
            yaw_bound=work.yaw_bound,
            radius=work.radius,
            footprint=tuple(work.profile.footprint),
            collision_z=work.depth_collision_z,
            extra_margin=motion_margin + (work.profile.hard_margin or 0.0),
        )
        details["depth_aligned_samples"] = len(depth.aligned_points)
        details["depth_coverage_qualified"] = depth.coverage_qualified
        # No sampled fan or missing depth is a 3-D coverage proof. Until a
        # qualified volume producer is wired, a required-depth profile stops.
        decision = SafetyDecision(False, Velocity(0.0, 0.0), depth.reason)
        return SafetyWorkResult(decision, details, time.monotonic() - start)
    observations = _filtered_observations(work)
    assert work.profile.observation_max_age is not None
    prepared = select_safety_observation(
        list(observations),
        work.history,
        steady=work.evaluation_steady_time,
        now=work.evaluation_ros_time,
        max_age=work.profile.observation_max_age,
        max_reference_lag=work.max_reference_lag,
        linear_bound=work.linear_bound,
        yaw_bound=work.yaw_bound,
        radius=work.radius,
    )
    if prepared is None:
        decision = SafetyDecision(False, Velocity(0.0, 0.0), "motion history unavailable")
        return SafetyWorkResult(decision, details, time.monotonic() - start)
    selected_stamp = prepared.observation.scan.stamp
    selected_received = next(
        item.received
        for item in reversed(observations)
        if item.observation.scan.stamp == selected_stamp
    )
    if newer_scan_veto(
        observations[-1],
        selected_stamp,
        work.command,
        work.measured,
        work.profile,
        now=work.evaluation_ros_time,
        steady=work.evaluation_steady_time,
        linear_bound=work.linear_bound,
        yaw_bound=work.yaw_bound,
        radius=work.radius,
        motion_sweep_margin=motion_margin,
    ):
        decision = SafetyDecision(False, Velocity(0.0, 0.0), "newer scan vetoed motion")
    else:
        decision = evaluate_motion(
            work.command,
            work.measured,
            [prepared.observation],
            work.profile,
            work.evaluation_ros_time,
            uncertainty_margin=prepared.added_margin + motion_margin,
            coverage_diagnostics=details if work.diagnostics else None,
        )
    for key, value in tuple(details.items()):
        if isinstance(value, str) and len(value) > 32768:
            details[key] = value[:32768]
            details[f"{key}_truncated"] = True
    sensor = prepared.observation.scan.sensor_to_body
    return SafetyWorkResult(
        decision,
        details,
        time.monotonic() - start,
        selected_received,
        selected_stamp,
        prepared.observation.scan.reference_stamp,
        prepared.added_margin + motion_margin,
        (sensor.x, sensor.y, sensor.yaw),
        len(observations[-1].observation.occluded_regions),
        sum(ray.kind == "ambiguous_self_overlap" for ray in observations[-1].observation.rays),
    )


@dataclass(frozen=True)
class WorkerReply:
    ticket: SafetyTicket
    value: Any
    error: str | None


def _worker_main(jobs: Any, replies: Any, processor: Callable[[Any], Any]) -> None:
    try:
        while True:
            item = jobs.get()
            if item is None:
                return
            ticket, work = item
            try:
                replies.put(WorkerReply(ticket, processor(work), None))
            except Exception as exc:
                replies.put(WorkerReply(ticket, None, f"{type(exc).__name__}: {exc}"[:512]))
    except KeyboardInterrupt:
        return


class BoundedSafetyWorker:
    """One spawned process; ROS callbacks never perform process or pipe I/O."""

    def __init__(self, processor: Callable[[Any], Any] = evaluate_safety_work) -> None:
        self._context = multiprocessing.get_context("spawn")
        self._processor = processor
        self._inbox: queue.Queue[Any] = queue.Queue(maxsize=1)
        self._outbox: queue.Queue[WorkerReply] = queue.Queue(maxsize=1)
        self._process: Any = None
        self._pending = False
        self._closed = threading.Event()
        self._failed = threading.Event()
        self._supervisor: threading.Thread | None = None

    def _supervise(self) -> None:
        jobs = replies = process = None
        try:
            jobs = self._context.Queue(maxsize=1)
            replies = self._context.Queue(maxsize=1)
            if self._closed.is_set():
                return
            process = self._context.Process(
                target=_worker_main, args=(jobs, replies, self._processor), daemon=True
            )
            self._process = process
            process.start()
            while not self._closed.is_set():
                try:
                    item = self._inbox.get(timeout=0.05)
                except queue.Empty:
                    if not process.is_alive():
                        self._failed.set()
                        return
                    continue
                jobs.put(item)
                while not self._closed.is_set():
                    try:
                        result = replies.get(timeout=0.05)
                        self._outbox.put_nowait(result)
                        break
                    except queue.Empty:
                        if not process.is_alive():
                            self._failed.set()
                            return
        except Exception:
            self._failed.set()
        finally:
            if process is not None and process.pid is not None:
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=0.5)
                if process.is_alive():
                    process.kill()
                    process.join(timeout=0.5)
            for channel in (jobs, replies):
                if channel is not None:
                    channel.cancel_join_thread()
                    channel.close()

    def submit(self, ticket: SafetyTicket, work: Any) -> bool:
        if self._closed.is_set() or self._failed.is_set() or self._pending:
            return False
        try:
            self._inbox.put_nowait((ticket, work))
        except queue.Full:
            return False
        self._pending = True
        if self._supervisor is None:
            self._supervisor = threading.Thread(target=self._supervise, daemon=True)
            try:
                self._supervisor.start()
            except RuntimeError:
                self._failed.set()
                self._pending = False
                return False
        return True

    def poll(self) -> WorkerReply | None:
        if not self._pending:
            return None
        try:
            result = self._outbox.get_nowait()
        except queue.Empty:
            if self._failed.is_set():
                self._pending = False
            return None
        self._pending = False
        return result

    @property
    def pending(self) -> bool:
        return self._pending

    def close(self) -> None:
        self._closed.set()
        process = self._process
        if process is not None and process.pid is not None:
            try:
                process.terminate()
            except (OSError, ValueError):
                pass
