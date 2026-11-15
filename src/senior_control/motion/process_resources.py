"""Bounded spawn-process resources; no ROS context is inherited across fork."""

import ctypes
import multiprocessing as mp
import os
import pickle
import signal
import sys
from concurrent.futures import Future


class LatestSnapshot:
    """One serialized immutable value, never a queue or a blocking busy-lock wait.

    Serialization is outside the lock. A reader copies bytes under a try-lock,
    then deserializes its private copy. Oversize data fails explicitly.
    """

    def __init__(self, context, capacity=8 * 1024 * 1024):
        self.capacity = capacity
        self._buffer = context.RawArray("B", capacity)
        self._length = context.RawValue("I", 0)
        self._version = context.RawValue("Q", 0)
        self._lock = context.Lock()

    def put(self, value):
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

    def take(self, after):
        if not self._lock.acquire(False):
            return None
        try:
            version = self._version.value
            if version <= after:
                return None
            payload = bytes(memoryview(self._buffer).cast("B")[: self._length.value])
        finally:
            self._lock.release()
        return version, pickle.loads(payload)


def child_lifetime(parent_pid):
    """Rootless Linux parent-death cleanup; check the registration race too."""
    if sys.platform == "linux":
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), "cannot register parent-death signal")
    if os.getppid() != parent_pid:
        raise SystemExit(0)


def stop_process(process, stop):
    stop.set()
    process.join(timeout=1.0)
    if process.is_alive():
        process.terminate()
        process.join(timeout=1.0)
    if process.is_alive():
        process.kill()
        process.join(timeout=1.0)
    process.close()


def _refinement_loop(requests, responses, stop, parent_pid):
    child_lifetime(parent_pid)
    version = 0
    while not stop.is_set() and os.getppid() == parent_pid:
        row = requests.take(version)
        if row is None:
            stop.wait(0.005)
            continue
        version, task = row
        try:
            result = (True, task())
        except Exception as exc:
            result = (False, f"{type(exc).__name__}: {exc}")
        while not stop.is_set() and not responses.put(result):
            stop.wait(0.001)


class _ProcessFuture(Future):
    def __init__(self, owner, version):
        super().__init__()
        self.owner, self.version = owner, version

    def done(self):
        if not super().done():
            row = self.owner.responses.take(self.version)
            if row is not None:
                self.owner.response_version, (ok, result) = row
                if ok:
                    self.set_result(result)
                else:
                    self.set_exception(RuntimeError(result))
            elif not self.owner.process.is_alive():
                self.set_exception(RuntimeError("geometry_process_exited"))
        return super().done()


class ProcessWorker:
    """One dedicated interpreter for one scheduler flight; no executor backlog."""

    def __init__(self, *, name="motion-geometry"):
        context = mp.get_context("spawn")
        self.requests, self.responses = LatestSnapshot(context), LatestSnapshot(context)
        self.stop = context.Event()
        self.response_version = 0
        self.flight = None
        self.process = context.Process(
            target=_refinement_loop,
            args=(self.requests, self.responses, self.stop, os.getpid()),
            name=name,
        )
        self.process.start()
        self._closed = False

    def submit(self, task):
        if self.stop.is_set() or not self.process.is_alive():
            raise RuntimeError("geometry_process_unavailable")
        if self.flight is not None and not self.flight.done():
            raise RuntimeError("geometry_process_busy")
        if not self.requests.put(task):
            raise RuntimeError("geometry_mailbox_busy")
        self.flight = _ProcessFuture(self, self.response_version)
        return self.flight

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self.flight is not None:
            self.flight.cancel()
        stop_process(self.process, self.stop)
