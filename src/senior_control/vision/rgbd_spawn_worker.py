"""Isolated diagnostic estimator. This is not a safety or source admission fence.

Call submit/poll/close on a dedicated coordinator, never a ROS safety callback.
The parent must own capture/TF frontiers across this process and recheck every
original capture and dependency at commit. Queue messages are private child IPC.
"""

import errno
import math
import multiprocessing as mp
import pickle
import signal
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass, field
from multiprocessing.connection import Connection
from queue import Empty, Full, Queue
from threading import Event, Thread
from typing import Any, Protocol, cast

from senior_control.vision.independent_motion import (
    Frame,
    IndependentMotion,
    Profile,
    _immutable_header,
)
from senior_control.vision.raw_capture import RawCaptureHeader
from senior_control.vision.rgbd_frame_builder import CameraInfoSnapshot, ImageLayoutSnapshot
from senior_control.vision.static_tf_binding import Receipt


@dataclass(frozen=True)
class CompactResult:
    generation: int
    sequence: int
    reason: str
    headers: tuple[tuple[RawCaptureHeader, ...], ...]
    camera_transform: tuple[tuple[float, ...], ...] | None
    matched: int
    inliers: int
    median_reprojection_px: float | None
    median_depth_residual_m: float | None
    tf_epochs: tuple[str, ...] = ()
    dependencies: tuple[tuple[str, ...], ...] = ()
    profile_name: str = ""
    calibration_epoch: str = ""
    registration: str = ""
    raw_rgb_encodings: tuple[str, ...] = ()
    preprocessing_versions: tuple[str, ...] = ()
    image_layouts: tuple[tuple[ImageLayoutSnapshot, ...], ...] = ()
    camera_info: tuple[tuple[CameraInfoSnapshot, ...], ...] = ()
    tf_receipts: tuple[Receipt, ...] = ()
    tf_admitted_from: tuple[tuple[int, float], ...] = ()
    hard_error_qualified: bool = field(default=False, init=False)


def _valid_result(result: CompactResult) -> bool:
    try:
        return _valid_result_fields(result)
    except (TypeError, ValueError, OverflowError, UnicodeError, AttributeError):
        return False


def _valid_result_fields(result: CompactResult) -> bool:
    if (
        type(result.reason) is not str
        or not result.reason
        or len(result.reason.encode("utf-8")) > 64
        or type(result.generation) is not int
        or type(result.sequence) is not int
        or type(result.matched) is not int
        or type(result.inliers) is not int
        or not 0 <= result.inliers <= result.matched <= 500
    ):
        return False
    if result.reason != "candidate":
        return not result.headers and result.camera_transform is None
    matrix = result.camera_transform
    return (
        type(result.headers) is tuple
        and len(result.headers) == 2
        and all(
            type(group) is tuple and len(group) == 4 and all(_immutable_header(h) for h in group)
            for group in result.headers
        )
        and type(matrix) is tuple
        and len(matrix) == 4
        and all(
            type(row) is tuple
            and len(row) == 4
            and all(type(v) in (int, float) and math.isfinite(v) for v in row)
            for row in matrix
        )
        and _rigid(matrix)
        and all(
            isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v >= 0
            for v in (result.median_reprojection_px, result.median_depth_residual_m)
        )
    )


def _rigid(matrix: tuple[tuple[float, ...], ...]) -> bool:
    if any(abs(matrix[3][i] - expected) > 1e-8 for i, expected in enumerate((0, 0, 0, 1))):
        return False
    for i in range(3):
        for j in range(3):
            dot = sum(matrix[k][i] * matrix[k][j] for k in range(3))
            if not math.isfinite(dot) or abs(dot - (1 if i == j else 0)) > 1e-6:
                return False
    determinant = (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )
    return math.isfinite(determinant) and abs(determinant - 1) <= 1e-6


@dataclass(frozen=True)
class _Job:
    generation: int
    sequence: int
    frame: Frame
    now_ros_ns: int
    now_steady: float
    ros_deadline_ns: int
    steady_deadline: float


def _run(profile: Profile, work: Any, output: Any) -> None:
    # The parent blocks SIGINT during spawn, including Python startup/imports.
    # Install the child policy before unblocking any pending group SIGINT.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT})
    estimator = IndependentMotion(profile)
    generation = 0
    while True:
        job = work.get()
        if job is None:
            return
        if job.generation != generation:
            estimator.invalidate()
            generation = job.generation
        source = job.frame
        # Deserialization can restore writable arrays; Frame copies to immutable
        # backing bytes again before any estimator operation.
        immutable = Frame(
            source.rgb_header,
            source.depth_header,
            source.rgb_info_header,
            source.depth_info_header,
            source.image,
            source.depth,
            source.calibration,
            source.tf_epoch,
            source.dependencies,
            source.exclusion,
        )
        result = estimator.offer(immutable, now_ros_ns=job.now_ros_ns, now_steady=job.now_steady)
        headers = (
            ()
            if result.pair is None
            else tuple(
                (f.rgb_header, f.depth_header, f.rgb_info_header, f.depth_info_header)
                for f in result.pair
            )
        )
        output.send(
            CompactResult(
                job.generation,
                job.sequence,
                result.reason,
                headers,
                result.camera_transform,
                result.matched,
                result.inliers,
                result.median_reprojection_px,
                result.median_depth_residual_m,
                () if result.pair is None else tuple(f.tf_epoch for f in result.pair),
                () if result.pair is None else tuple(f.dependencies for f in result.pair),
            )
        )


class _QueueInternals(Protocol):
    _reader: Connection
    _writer: Connection
    _thread: Thread | None
    _on_queue_feeder_error: Callable[[Exception, object], None]

    def close(self) -> None: ...


class _QueueCleanup:
    """//Compatible: CPython 3.10/3.12 Queue has no public feeder interruption."""

    def __init__(self, queue: object) -> None:
        self._queue = cast(_QueueInternals, queue)
        self._shutdown = [False]
        original = self._queue._on_queue_feeder_error
        state = self._shutdown

        def on_error(error: Exception, obj: object) -> None:
            if state[0] and getattr(error, "errno", None) == errno.EPIPE:
                return
            original(error, obj)

        self._queue._on_queue_feeder_error = on_error

    def close(self) -> bool:
        self._shutdown[0] = True
        self._queue._reader.close()
        self._queue.close()
        feeder = self._queue._thread
        if feeder is not None:
            feeder.join(timeout=0.5)
            if feeder.is_alive():
                return False
        self._queue._writer.close()
        return True


class SpawnWorker:
    """One active job plus one replaceable pending job, with original deadlines."""

    def __init__(self, profile: Profile, *, generation: int, sequence_floor: int) -> None:
        if (
            type(generation) is not int
            or generation <= 0
            or type(sequence_floor) is not int
            or sequence_floor < 0
        ):
            raise ValueError("invalid_parent_frontier")
        self._context = mp.get_context("spawn")
        self._profile = profile
        self._shutdown_complete = False
        self._start_process()
        self.generation = generation
        self._sequence = sequence_floor
        self._inflight: _Job | None = None
        self._pending: _Job | None = None
        self.dropped = 0
        self._closed = False
        self._unavailable = False
        self._timed_out = False
        self._fresh_after: tuple[int, float] | None = None

    def _start_process(self) -> None:
        self._shutdown_complete = False
        self._work = self._context.Queue(maxsize=1)
        self._queue_cleanup = (_QueueCleanup(self._work),)
        try:
            self._output_reader, output_writer = self._context.Pipe(duplex=False)
        except BaseException:
            with suppress(Exception):
                self._queue_cleanup[0].close()
            raise
        self._results: Queue[object] = Queue(maxsize=1)
        self._output = self._results  # //Compatible: diagnostic test injection.
        self._reader_failed = Event()
        process = None
        reader_thread: Thread | None = None
        try:
            process = self._context.Process(
                target=_run, args=(self._profile, self._work, output_writer), daemon=True
            )
            self._process = process
            # Only the calling thread is masked; the parent's handler and all
            # other threads keep their existing signal policy.
            original_mask = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT})
            try:
                process.start()
            finally:
                signal.pthread_sigmask(signal.SIG_SETMASK, original_mask)
            output_writer.close()
            reader_thread = Thread(
                target=self._read_results,
                args=(self._output_reader, self._results, self._reader_failed),
                daemon=True,
            )
            reader_thread.start()
            self._reader_thread = reader_thread
        except BaseException:
            # Keep the startup error; attempt each independent rollback step.
            with suppress(Exception):
                output_writer.close()
            if process is not None and process.pid is not None:
                with suppress(Exception):
                    if process.is_alive():
                        process.terminate()
                        process.join(timeout=0.5)
                    if process.is_alive():
                        process.kill()
                        process.join(timeout=0.5)
                if reader_thread is not None and reader_thread.ident is not None:
                    with suppress(Exception):
                        reader_thread.join(timeout=0.5)
                with suppress(Exception):
                    if not process.is_alive():
                        process.close()
            with suppress(Exception):
                self._output_reader.close()
            for cleanup in self._queue_cleanup:
                with suppress(Exception):
                    cleanup.close()
            raise

    @staticmethod
    def _read_results(reader: Connection, results: Queue[object], failed: Event) -> None:
        try:
            while True:
                result = reader.recv()
                try:
                    results.put_nowait(result)
                except Full:
                    failed.set()
                    return
        except (
            EOFError,
            OSError,
            ValueError,
            pickle.UnpicklingError,
            AttributeError,
            ImportError,
            IndexError,
        ):
            failed.set()

    def _stop_process(self) -> bool:
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=0.5)
        if self._process.is_alive():
            self._process.kill()
            self._process.join(timeout=0.5)
        stopped = not self._process.is_alive()
        if stopped:
            self._process.join(timeout=0)
            self._reader_thread.join(timeout=0.5)
            if self._reader_thread.is_alive():
                return False
            self._output_reader.close()
            for cleanup in self._queue_cleanup:
                if not cleanup.close():
                    return False
        self._shutdown_complete = stopped
        return stopped

    def _restart(self, now_ros_ns: int, now_steady: float) -> None:
        self.invalidate()
        stopped = self._stop_process()
        self._inflight = None
        if (
            stopped
            and type(now_ros_ns) is int
            and now_ros_ns > 0
            and math.isfinite(now_steady)
            and now_steady > 0
        ):
            self._fresh_after = (now_ros_ns, now_steady)
            try:
                self._start_process()
            except (OSError, RuntimeError):
                self._unavailable = True
                self._closed = True
            else:
                self._timed_out = False
        else:
            self._unavailable = True

    @property
    def shutdown_complete(self) -> bool:
        """True only after process, reader and queue resources all stopped."""
        return self._shutdown_complete

    @property
    def sequence_frontier(self) -> int:
        return self._sequence

    def _dispatch(self) -> None:
        if self._inflight is None and self._pending is not None:
            try:
                self._work.put_nowait(self._pending)
            except Full:
                return
            self._inflight, self._pending = self._pending, None

    def submit(
        self,
        frame: Frame,
        *,
        now_ros_ns: int,
        now_steady: float,
        ros_deadline_ns: int,
        steady_deadline: float,
    ) -> bool:
        if (
            self._closed
            or self._unavailable
            or not self._process.is_alive()
            or type(frame) is not Frame
            or type(now_ros_ns) is not int
            or type(ros_deadline_ns) is not int
            or not math.isfinite(now_steady)
            or not math.isfinite(steady_deadline)
            or not 0 < now_ros_ns <= ros_deadline_ns
            or now_steady > steady_deadline
        ):
            return False
        headers = (
            frame.rgb_header,
            frame.depth_header,
            frame.rgb_info_header,
            frame.depth_info_header,
        )
        if self._fresh_after is not None and any(
            h.stamp_ns <= self._fresh_after[0]
            or h.received_ros_ns <= self._fresh_after[0]
            or h.received_steady <= self._fresh_after[1]
            for h in headers
        ):
            return False
        if any(
            not math.isfinite(h.received_steady)
            or ros_deadline_ns > min(h.stamp_ns, h.received_ros_ns) + 300_000_000
            or steady_deadline > h.received_steady + 0.3
            for h in headers
        ):
            return False
        self._sequence += 1
        if self._pending is not None:
            self.dropped += 1
        self._pending = _Job(
            self.generation,
            self._sequence,
            frame,
            now_ros_ns,
            now_steady,
            ros_deadline_ns,
            steady_deadline,
        )
        self._dispatch()
        return True

    def invalidate(self) -> None:
        self.generation += 1
        if self._pending is not None:
            self.dropped += 1
        self._pending = None

    def poll(self, *, now_ros_ns: int, now_steady: float) -> CompactResult | None:
        if self._closed or self._unavailable:
            return None
        if (
            not self._process.is_alive()
            or self._reader_failed.is_set()
            or not self._reader_thread.is_alive()
        ):
            self._restart(now_ros_ns, now_steady)
            return CompactResult(
                self.generation, self._sequence, "worker_failed", (), None, 0, 0, None, None
            )
        job = self._inflight
        if (
            job is not None
            and not self._timed_out
            and (
                not math.isfinite(now_steady)
                or now_steady > job.steady_deadline
                or not 0 < now_ros_ns <= job.ros_deadline_ns
            )
        ):
            self._timed_out = True
            self._restart(now_ros_ns, now_steady)
            return CompactResult(
                self.generation, job.sequence, "worker_timeout", (), None, 0, 0, None, None
            )
        try:
            result = self._results.get_nowait()
        except Empty:
            self._dispatch()
            return None
        if (
            type(result) is not CompactResult
            or result.hard_error_qualified is not False
            or not _valid_result(result)
        ):
            self.invalidate()
            return CompactResult(
                self.generation,
                self._sequence,
                "malformed_worker_result",
                (),
                None,
                0,
                0,
                None,
                None,
            )
        job = self._inflight
        if (
            job is None
            or result.generation != job.generation
            or result.sequence != job.sequence
            or (
                result.reason == "candidate"
                and result.headers[1]
                != (
                    job.frame.rgb_header,
                    job.frame.depth_header,
                    job.frame.rgb_info_header,
                    job.frame.depth_info_header,
                )
            )
        ):
            self.invalidate()
            return CompactResult(
                self.generation,
                self._sequence,
                "malformed_worker_result",
                (),
                None,
                0,
                0,
                None,
                None,
            )
        self._inflight = None
        self._timed_out = False
        if (
            job.generation != self.generation
            or not math.isfinite(now_steady)
            or not 0 < now_ros_ns <= job.ros_deadline_ns
            or now_steady > job.steady_deadline
        ):
            self._dispatch()
            return None
        if any(
            not 0 <= now_ros_ns - h.stamp_ns <= 300_000_000
            or not 0 <= now_ros_ns - h.received_ros_ns <= 300_000_000
            or not 0 <= now_steady - h.received_steady <= 0.3
            for group in result.headers
            for h in group
        ):
            self.invalidate()
            return CompactResult(
                self.generation,
                result.sequence,
                "capture_expired",
                result.headers,
                None,
                0,
                0,
                None,
                None,
            )
        self._dispatch()
        return result

    def close(self) -> None:
        if self._closed:
            return
        self._closed = self._stop_process()
