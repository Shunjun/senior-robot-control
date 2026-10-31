"""One owned, spawned geometry worker and immutable authorization tickets."""

import math
import multiprocessing
import queue
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from senior_control.config import DeviceProfile
from senior_control.models import Velocity
from senior_control.safety.guard import SafetyDecision, evaluate_motion
from senior_control.safety.motion_runtime import (
    StampedObservation,
    TimedPoseBuffer,
    newer_scan_veto,
    select_safety_observation,
)


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
    measured: Velocity
    profile: DeviceProfile
    observations: tuple[StampedObservation, ...]
    history: TimedPoseBuffer
    evaluation_ros_time: float
    evaluation_steady_time: float
    max_reference_lag: float
    linear_bound: float
    yaw_bound: float
    radius: float
    diagnostics: bool


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


def evaluate_safety_work(work: SafetyWork) -> SafetyWorkResult:
    """Run geometry at the ticket's future deadline, not at a restamped scan."""
    start = time.monotonic()
    details: dict[str, object] = {}
    assert work.profile.observation_max_age is not None
    prepared = select_safety_observation(
        list(work.observations),
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
        for item in reversed(work.observations)
        if item.observation.scan.stamp == selected_stamp
    )
    if newer_scan_veto(
        work.observations[-1],
        selected_stamp,
        work.command,
        work.measured,
        work.profile,
        now=work.evaluation_ros_time,
        steady=work.evaluation_steady_time,
        linear_bound=work.linear_bound,
        yaw_bound=work.yaw_bound,
        radius=work.radius,
    ):
        decision = SafetyDecision(False, Velocity(0.0, 0.0), "newer scan vetoed motion")
    else:
        decision = evaluate_motion(
            work.command,
            work.measured,
            [prepared.observation],
            work.profile,
            work.evaluation_ros_time,
            uncertainty_margin=prepared.added_margin,
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
        prepared.added_margin,
        (sensor.x, sensor.y, sensor.yaw),
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
