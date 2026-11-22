"""Bounded spawn-process resources; no ROS context is inherited across fork."""

from __future__ import annotations

import ctypes
import multiprocessing as mp
import os
import pickle
import signal
import sys
from collections.abc import Callable
from concurrent.futures import Future
from multiprocessing.context import BaseContext
from multiprocessing.process import BaseProcess
from multiprocessing.synchronize import Event
from typing import Generic, Literal, TypeAlias, TypeVar, cast

SnapshotT = TypeVar("SnapshotT")
ResultT = TypeVar("ResultT")
WorkerReply: TypeAlias = tuple[Literal[True], ResultT] | tuple[Literal[False], str]


class LatestSnapshot(Generic[SnapshotT]):
    """One serialized immutable value, never a queue or a blocking busy-lock wait.

    Serialization is outside the lock. A reader copies bytes under a try-lock,
    then deserializes its private copy. Oversize data fails explicitly.
    """

    def __init__(self, context: BaseContext, capacity: int = 8 * 1024 * 1024) -> None:
        self.capacity = capacity
        self._buffer = context.RawArray(ctypes.c_ubyte, capacity)
        self._length = context.RawValue(ctypes.c_uint, 0)
        self._version = context.RawValue(ctypes.c_ulonglong, 0)
        self._lock = context.Lock()

    def put(self, value: SnapshotT) -> bool:
        payload = pickle.dumps(value, protocol=5)
        if len(payload) > self.capacity:
            raise ValueError("snapshot_capacity_exceeded")
        if not self._lock.acquire(False):
            return False
        try:
            memoryview(self._buffer).cast("B")[: len(payload)] = payload
            self._length.value = len(payload)
            self._version.value += 1
        finally:
            self._lock.release()
        return True

    def take(self, after: int) -> tuple[int, SnapshotT] | None:
        if not self._lock.acquire(False):
            return None
        try:
            version = self._version.value
            if version <= after:
                return None
            payload = bytes(memoryview(self._buffer).cast("B")[: self._length.value])
        finally:
            self._lock.release()
        # This private channel only receives values serialized by the paired
        # producer of the same SnapshotT; pickle cannot express that relation.
        return version, cast(SnapshotT, pickle.loads(payload))


def child_lifetime(parent_pid: int) -> None:
    """Rootless Linux parent-death cleanup; check the registration race too."""
    if sys.platform == "linux":
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), "cannot register parent-death signal")
    if os.getppid() != parent_pid:
        raise SystemExit(0)


def stop_process(process: BaseProcess, stop: Event) -> None:
    stop.set()
    process.join(timeout=1.0)
    if process.is_alive():
        process.terminate()
        process.join(timeout=1.0)
    if process.is_alive():
        process.kill()
        process.join(timeout=1.0)
    process.close()


def _refinement_loop(
    requests: LatestSnapshot[Callable[[], ResultT]],
    responses: LatestSnapshot[WorkerReply[ResultT]],
    stop: Event,
    parent_pid: int,
) -> None:
    child_lifetime(parent_pid)
    version = 0
    while not stop.is_set() and os.getppid() == parent_pid:
        row = requests.take(version)
        if row is None:
            stop.wait(0.005)
            continue
        version, task = row
        result: WorkerReply[ResultT]
        try:
            result = (True, task())
        except Exception as exc:
            result = (False, f"{type(exc).__name__}: {exc}")
        while not stop.is_set() and not responses.put(result):
            stop.wait(0.001)


class _ProcessFuture(Future[ResultT]):
    def __init__(
        self,
        owner: ProcessWorker[ResultT],
        version: int,
        pending_task: Callable[[], ResultT] | None,
    ) -> None:
        super().__init__()
        self.owner, self.version = owner, version
        self._pending_task = pending_task

    def done(self) -> bool:
        if not super().done():
            if self._pending_task is not None:
                if not self.owner.process.is_alive():
                    self.set_exception(RuntimeError("geometry_process_exited"))
                    return True
                if not self.owner.requests.put(self._pending_task):
                    return False
                self._pending_task = None
            row = self.owner.responses.take(self.version)
            if row is not None:
                self.owner.response_version, reply = row
                if reply[0]:
                    self.set_result(reply[1])
                else:
                    self.set_exception(RuntimeError(reply[1]))
            elif not self.owner.process.is_alive():
                self.set_exception(RuntimeError("geometry_process_exited"))
        return super().done()


class ProcessWorker(Generic[ResultT]):
    """One dedicated interpreter for one scheduler flight; no executor backlog.

    ResultT is fixed for this worker: task, result slot and Future must agree.
    """

    def __init__(self, *, name: str = "motion-geometry") -> None:
        context = mp.get_context("spawn")
        self.requests: LatestSnapshot[Callable[[], ResultT]] = LatestSnapshot(context)
        self.responses: LatestSnapshot[WorkerReply[ResultT]] = LatestSnapshot(context)
        self.stop = context.Event()
        self.response_version = 0
        self.flight: _ProcessFuture[ResultT] | None = None
        self.process = context.Process(
            target=_refinement_loop,
            args=(self.requests, self.responses, self.stop, os.getpid()),
            name=name,
        )
        self.process.start()
        self._closed = False

    def submit(self, task: Callable[[], ResultT]) -> Future[ResultT]:
        if self.stop.is_set() or not self.process.is_alive():
            raise RuntimeError("geometry_process_unavailable")
        if self.flight is not None and not self.flight.done():
            raise RuntimeError("geometry_process_busy")
        # A reader's try-lock contention is temporary. Retain this one flight
        # and retry on polling; schedulers keep their latest pending snapshot
        # and original submission budget/deadlines without a blocking wait.
        pending_task = None if self.requests.put(task) else task
        self.flight = _ProcessFuture(self, self.response_version, pending_task)
        return self.flight

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.flight is not None:
            self.flight.cancel()
        stop_process(self.process, self.stop)
