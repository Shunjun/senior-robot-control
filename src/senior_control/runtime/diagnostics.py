"""Bounded diagnostics, never control policy. Queue mutations only update state.

One reporter handles age, wait, duration and backlog with distinct stream names.
Unknown middleware depths remain None. Logging happens at pump, outside owners'
queue locks; recursive log-writer observations cannot recursively emit warnings.
"""

import asyncio
import logging
import math
import time
import weakref
from collections import OrderedDict, deque
from collections.abc import Callable, Hashable, Mapping
from dataclasses import asdict, dataclass, field, replace
from functools import lru_cache
from threading import RLock
from typing import Any, ClassVar, Generic, Literal, SupportsIndex, TypeVar

from senior_control.sensors.policy import Age, default_policy

_LIMIT = (1 << 63) - 1


@dataclass(frozen=True)
class QueueSnapshot:
    name: str
    pending: int | None
    capacity: int | None
    oldest_wait_ms: float | None = None
    dropped_total: int | None = None
    overwritten_total: int | None = None
    running: int | None = None
    depth_basis: str = "owner_counter"
    kind: str = "pending"
    retained: int | None = None


@dataclass(frozen=True)
class QueueWarningPolicy:
    __pydantic_config__: ClassVar[dict[str, str]] = {"extra": "forbid"}
    count: int = 32
    ratio: float = 0.75
    repeat_s: float = 5.0
    recovery_ratio: float = 0.5
    oldest_wait_ms: float = 300.0

    def __post_init__(self) -> None:
        if (
            type(self.count) is not int
            or self.count < 1
            or not 0 < self.ratio <= 1
            or not 0 <= self.recovery_ratio < 1
            or not all(math.isfinite(v) and v > 0 for v in (self.repeat_s, self.oldest_wait_ms))
        ):
            raise ValueError("invalid queue warning policy")

    def threshold(self, capacity: int | None) -> int:
        return (
            self.count
            if capacity is None
            else max(1, min(self.count, math.floor(self.ratio * capacity)))
        )


@dataclass
class _Stats:
    count: int = 0
    total: float = 0.0
    maximum: float = 0.0
    over_soft: int = 0
    over_hard: int = 0

    def add(self, value: float, soft: bool = False, hard: bool = False) -> None:
        self.count = min(_LIMIT, self.count + 1)
        self.total += value
        self.maximum = max(self.maximum, value)
        self.over_soft = min(_LIMIT, self.over_soft + soft)
        self.over_hard = min(_LIMIT, self.over_hard + hard)

    def report(self) -> dict[str, float | int]:
        return dict(
            count=self.count,
            avg=self.total / self.count if self.count else 0.0,
            max=self.maximum,
            over_soft=self.over_soft,
            over_hard=self.over_hard,
        )


@dataclass
class _QueueState:
    value: QueueSnapshot
    policy: QueueWarningPolicy
    over: bool = False
    announced: bool = False
    last_warning: float = -math.inf
    peak: int | None = None
    suppressed: int = 0
    sampled_at: float = 0.0
    crossing: QueueSnapshot | None = None
    owned: bool = False


class RuntimeDiagnostics:
    def __init__(
        self,
        emit: Callable[[int, dict[str, Any]], None] | None = None,
        *,
        period_s: float | None = None,
        max_streams: int = 128,
        queue_policy: QueueWarningPolicy | None = None,
    ):
        self.period = default_policy().diagnostics_period_s if period_s is None else period_s
        if not math.isfinite(self.period) or self.period <= 0 or max_streams < 1:
            raise ValueError("invalid diagnostics policy")
        self._emit = emit or (
            lambda level, data: logging.getLogger(__name__).log(level, "%s", data)
        )
        self._max = max_streams
        self._policy = queue_policy or QueueWarningPolicy()
        self._lock = RLock()
        self._stats: dict[str, _Stats] = {}
        self._last_keys: dict[str, OrderedDict[Hashable, None]] = {}
        self._sensor_results: dict[str, int] = {}
        self._queues: dict[str, _QueueState] = {}
        self._owners: dict[tuple[str, int], tuple[Any, QueueSnapshot, float]] = {}
        self._last_flush: float | None = None
        self._emitting = False
        self._loop: asyncio.AbstractEventLoop | None = None
        self._timer: asyncio.TimerHandle | None = None
        self.unregistered = 0

    def _record(self, name: str, value: float, soft: bool = False, hard: bool = False) -> None:
        if not math.isfinite(value) or value < 0:
            return
        if name not in self._stats and len(self._stats) >= self._max:
            self.unregistered = min(_LIMIT, self.unregistered + 1)
            return
        self._stats.setdefault(name, _Stats()).add(value, soft, hard)

    def age(
        self, name: str, value: Age, *, key: Hashable | None = None, usage: bool = False
    ) -> None:
        with self._lock:
            stream = ("sensor.use." if usage else "sensor.raw.") + name
            if key is not None and not self._remember(stream, key):
                return
            p = default_policy()
            self._record(
                stream, value.seconds, value.seconds >= p.soft_age_s, value.seconds >= p.hard_age_s
            )

    def _remember(self, stream: str, key: Hashable) -> bool:
        if stream not in self._last_keys and len(self._last_keys) >= self._max:
            self.unregistered = min(_LIMIT, self.unregistered + 1)
            return False
        seen = self._last_keys.setdefault(stream, OrderedDict())
        if key in seen:
            seen.move_to_end(key)
            return False
        seen[key] = None
        if len(seen) > 256:
            seen.popitem(last=False)
        return True

    def sensor_attempt(
        self,
        name: str,
        phase: Literal["raw", "use"],
        value: Age | None,
        outcomes: tuple[str, ...],
        *,
        key: Hashable | None = None,
    ) -> None:
        """One unique raw sample / one attempted sample per estimate, before selection.

        A sample can have different axis outcomes but supplies exactly one age.
        Raw age deduplicates acquisitions; raw outcome counters count all ingress
        attempts, including replay rejection, independently of unique raw count.
        Invalid clocks have counters only. Raw replay dedup survives DEBUG flush
        in a bounded 256-identity/source window, independent of selection outcome.
        """
        with self._lock:
            stream = "sensor." + phase + "." + name
            unique_age = key is None or self._remember(stream, key)
            if value is not None and unique_age:
                p = default_policy()
                self._record(
                    stream,
                    value.seconds,
                    value.seconds >= p.soft_age_s,
                    value.seconds >= p.hard_age_s,
                )
            for outcome in set(outcomes):
                label = phase + "." + name + "." + outcome
                if label not in self._sensor_results and len(self._sensor_results) >= self._max:
                    self.unregistered = min(_LIMIT, self.unregistered + 1)
                    continue
                self._sensor_results[label] = min(_LIMIT, self._sensor_results.get(label, 0) + 1)

    def duration(self, name: str, seconds: float) -> None:
        with self._lock:
            self._record("duration." + name, seconds)

    def queue(
        self,
        value: QueueSnapshot,
        now: float,
        *,
        policy: QueueWarningPolicy | None = None,
        owner: Any = None,
    ) -> None:
        # Bad diagnostic observations must never become a control exception.
        if (
            not math.isfinite(now)
            or not value.name
            or len(value.name) > 128
            or (
                value.oldest_wait_ms is not None
                and (
                    not isinstance(value.oldest_wait_ms, (int, float))
                    or not math.isfinite(value.oldest_wait_ms)
                    or value.oldest_wait_ms < 0
                )
            )
            or any(
                v is not None and (type(v) is not int or v < 0)
                for v in (
                    value.pending,
                    value.capacity,
                    value.dropped_total,
                    value.overwritten_total,
                    value.running,
                    value.retained,
                )
            )
        ):
            return
        self.bind_async()
        with self._lock:
            if owner is not None:
                self._owners = {k: v for k, v in self._owners.items() if v[0]() is not None}
                key = (value.name, id(owner))
                if key not in self._owners and len(self._owners) >= self._max:
                    self.unregistered = min(_LIMIT, self.unregistered + 1)
                    return
                self._owners[key] = (weakref.ref(owner), value, now)
                # Fixed labels aggregate the worst live owner, never last-writer-wins.
                rows = [v for k, v in self._owners.items() if k[0] == value.name]

                def severity(row: tuple[Any, QueueSnapshot, float]) -> float:
                    v = row[1]
                    p = policy or self._policy
                    return max(
                        (v.pending or 0) / p.threshold(v.capacity),
                        (
                            (v.oldest_wait_ms or 0) + max(0.0, now - row[2]) * 1000
                            if v.pending
                            else (v.oldest_wait_ms or 0)
                        )
                        / p.oldest_wait_ms,
                    )

                _, value, observed_at = max(rows, key=severity)
                now = observed_at
            if value.name not in self._queues:
                if len(self._queues) >= self._max:
                    self.unregistered = min(_LIMIT, self.unregistered + 1)
                    return
                self._queues[value.name] = _QueueState(value, policy or self._policy)
            state = self._queues[value.name]
            state.value = value
            state.owned |= owner is not None
            state.sampled_at = now
            if policy is not None:
                state.policy = policy
            threshold = state.policy.threshold(value.capacity)
            count_over = value.pending is not None and value.pending > threshold
            wait_over = (
                value.oldest_wait_ms is not None
                and value.oldest_wait_ms > state.policy.oldest_wait_ms
            )
            if value.kind == "history":
                state.over = False
            elif count_over or wait_over:
                if not state.over and state.crossing is None:
                    state.crossing = value
                elif state.crossing is not None and (value.pending or 0) > (
                    state.crossing.pending or 0
                ):
                    state.crossing = value
                state.over = True
                if value.pending is not None:
                    state.peak = max(state.peak or 0, value.pending)
            elif value.pending is None or value.pending <= math.floor(
                threshold * state.policy.recovery_ratio
            ):
                state.over = False

    def bind_async(self) -> None:
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return
        if self._loop is loop:
            return
        if self._timer is not None:
            self._timer.cancel()
        self._loop = loop

        def tick() -> None:
            if loop.is_closed():
                return
            self.pump(loop.time())
            self._timer = loop.call_later(0.1, tick)

        self._timer = loop.call_later(0.1, tick)

    def snapshot(self, name: str) -> QueueSnapshot:
        with self._lock:
            return self._queues[name].value

    def pump(self, now: float) -> None:
        if not math.isfinite(now):
            return
        events: list[tuple[int, dict[str, Any]]] = []
        with self._lock:
            if self._emitting:
                return
            self._owners = {k: v for k, v in self._owners.items() if v[0]() is not None}
            for state in self._queues.values():
                if state.owned:
                    rows = [
                        row for (name, _), row in self._owners.items() if name == state.value.name
                    ]
                    if rows:

                        def severity(
                            row: tuple[Any, QueueSnapshot, float],
                            queue_policy: QueueWarningPolicy = state.policy,
                        ) -> float:
                            v = row[1]
                            wait = (
                                (v.oldest_wait_ms or 0) + max(0.0, now - row[2]) * 1000
                                if v.pending
                                else 0.0
                            )
                            return max(
                                (v.pending or 0) / queue_policy.threshold(v.capacity),
                                wait / queue_policy.oldest_wait_ms,
                            )

                        _, state.value, state.sampled_at = max(rows, key=severity)
                    else:
                        # The owner was destroyed. No stable API can measure its
                        # discarded pending tasks now; retire observations as unknown.
                        state.value = replace(
                            state.value,
                            pending=None,
                            oldest_wait_ms=None,
                            running=None,
                            depth_basis="owner_released",
                        )
                        state.over = False
                wait: float | None
                if state.value.pending and state.value.oldest_wait_ms is not None:
                    wait = state.value.oldest_wait_ms + max(0.0, now - state.sampled_at) * 1000
                    if wait > state.policy.oldest_wait_ms:
                        state.over = True
                else:
                    wait = state.value.oldest_wait_ms
                threshold = state.policy.threshold(state.value.capacity)
                if state.value.kind != "history":
                    if state.value.pending is not None and state.value.pending > threshold:
                        state.over = True
                    elif (wait is None or wait <= state.policy.oldest_wait_ms) and (
                        state.value.pending is None
                        or state.value.pending
                        <= math.floor(threshold * state.policy.recovery_ratio)
                    ):
                        state.over = False
                due = now - state.last_warning >= state.policy.repeat_s
                if (state.over or state.crossing is not None) and due:
                    warning = state.value if state.over else state.crossing
                    assert warning is not None
                    data = asdict(state.value)
                    data["oldest_wait_ms"] = wait
                    data["trigger_pending"] = warning.pending
                    data["trigger_oldest_wait_ms"] = warning.oldest_wait_ms
                    data["burst_recovered"] = not state.over
                    data.update(
                        event="queue_backlog",
                        queue=state.value.name,
                        threshold=state.policy.threshold(state.value.capacity),
                        peak=state.peak,
                        suppressed=state.suppressed,
                    )
                    events.append((logging.WARNING, data))
                    state.last_warning = now
                    state.crossing = None
                    state.announced = True
                    state.suppressed = 0
                elif state.over:
                    state.suppressed = min(_LIMIT, state.suppressed + 1)
                elif state.announced and due:
                    events.append(
                        (
                            logging.INFO,
                            dict(
                                event="queue_recovered",
                                queue=state.value.name,
                                **asdict(state.value),
                            ),
                        )
                    )
                    state.announced = False
                    state.last_warning = now
                    state.peak = None
            if self._last_flush is None:
                self._last_flush = now
            elif now - self._last_flush >= self.period:
                events.append(
                    (
                        logging.DEBUG,
                        dict(
                            event="runtime_statistics",
                            streams={name: s.report() for name, s in self._stats.items()},
                            sensor_results=dict(self._sensor_results),
                            unregistered=self.unregistered,
                            queues={
                                name: asdict(state.value) for name, state in self._queues.items()
                            },
                        ),
                    )
                )
                self._stats.clear()
                self._sensor_results.clear()
                self._last_flush = now
            self._emitting = True
        try:
            for level, data in events:
                try:
                    self._emit(level, data)
                except Exception:
                    # Diagnostics never change motion or caller lifecycle.
                    pass
        finally:
            with self._lock:
                self._emitting = False


@dataclass(frozen=True)
class DiagnosticsSettings:
    __pydantic_config__: ClassVar[dict[str, str]] = {"extra": "forbid"}
    queue: QueueWarningPolicy = QueueWarningPolicy()
    period_s: float = field(default_factory=lambda: default_policy().diagnostics_period_s)

    def __post_init__(self) -> None:
        if not math.isfinite(self.period_s) or self.period_s <= 0:
            raise ValueError("invalid diagnostics period")


def diagnostics_from_environment(environment: Mapping[str, str]) -> DiagnosticsSettings:
    """Same nested setting names as CommonSettings; no unrelated env values read."""
    from pydantic import TypeAdapter

    values: dict[str, Any] = {}
    prefix = "SENIOR_DIAGNOSTICS__"
    for name, value in environment.items():
        if not name.startswith(prefix):
            continue
        parts = name[len(prefix) :].lower().split("__")
        cursor = values
        for part in parts[:-1]:
            existing = cursor.setdefault(part, {})
            if not isinstance(existing, dict):
                raise ValueError("conflicting diagnostics environment setting")
            cursor = existing
        cursor[parts[-1]] = value
    return TypeAdapter(DiagnosticsSettings).validate_python(values)


def configure_diagnostics(settings: DiagnosticsSettings) -> RuntimeDiagnostics:
    d = process_diagnostics()
    if not math.isfinite(settings.period_s) or settings.period_s <= 0:
        raise ValueError("invalid diagnostics period")
    with d._lock:
        previous = d._policy
        d.period = settings.period_s
        d._policy = settings.queue
        for state in d._queues.values():
            if state.policy is previous:
                state.policy = settings.queue
    return d


@lru_cache(maxsize=1)
def process_diagnostics() -> RuntimeDiagnostics:
    return RuntimeDiagnostics()


class PendingCounter:
    """Instrument existing owners; never enqueue, reject or reorder their work.

    Tokens are internal bookkeeping only, not unbounded log/metric labels. Public
    pending/running counters avoid private asyncio/Executor queue introspection.
    """

    def __init__(
        self,
        name: str,
        capacity: int | None = None,
        *,
        diagnostics: RuntimeDiagnostics | None = None,
    ):
        self.name, self.capacity = name, capacity
        self.diagnostics = diagnostics or process_diagnostics()
        self._lock = RLock()
        self._waiting: dict[Hashable, float] = {}
        self._running: set[Hashable] = set()
        self._lost_metadata = False
        self._dropped = 0
        self._overwritten = 0

    def enqueue(self, key: Hashable, now: float) -> None:
        with self._lock:
            if len(self._waiting) >= 4096:
                self._lost_metadata = True
            else:
                self._waiting[key] = now
            self._publish(now)

    def start(self, key: Hashable, now: float) -> None:
        with self._lock:
            at = self._waiting.pop(key, None)
            if at is not None:
                self.diagnostics.duration("queue_wait." + self.name, max(0.0, now - at))
            if len(self._running) < 4096:
                self._running.add(key)
            else:
                self._lost_metadata = True
            self._publish(now)

    def finish(self, key: Hashable, now: float) -> None:
        with self._lock:
            self._running.discard(key)
            self._waiting.pop(key, None)
            self._publish(now)

    def discard(self, key: Hashable, now: float, *, overwritten: bool = False) -> None:
        with self._lock:
            existed = key in self._waiting
            self._waiting.pop(key, None)
            if existed:
                if overwritten:
                    self._overwritten = min(_LIMIT, self._overwritten + 1)
                else:
                    self._dropped = min(_LIMIT, self._dropped + 1)
            self._publish(now)

    def reject(self, now: float, count: int = 1) -> None:
        with self._lock:
            self._dropped = min(_LIMIT, self._dropped + count)
            self._publish(now)

    def reset(self, now: float) -> None:
        with self._lock:
            self._dropped = min(_LIMIT, self._dropped + len(self._waiting))
            self._waiting.clear()
            self._running.clear()
            self._publish(now)

    def snapshot(self, now: float) -> QueueSnapshot:
        with self._lock:
            return QueueSnapshot(
                self.name,
                None if self._lost_metadata else len(self._waiting),
                self.capacity,
                max(0.0, now - min(self._waiting.values())) * 1000 if self._waiting else 0.0,
                self._dropped,
                self._overwritten,
                len(self._running),
                "counter_capacity_exceeded" if self._lost_metadata else "owner_counter",
            )

    def _publish(self, now: float) -> None:
        self.diagnostics.queue(self.snapshot(now), now, owner=self)


T = TypeVar("T")


class DiagnosticDeque(deque[T], Generic[T]):
    """Existing deque semantics plus owner-side pending diagnostics.

    This is for work waiting to be consumed, never history buffers. Callers keep
    their existing locks. No reporter callback traverses the owner's container.
    """

    def __init__(self, name: str, capacity: int | None = None, *, maxlen: int | None = None):
        super().__init__(maxlen=maxlen)
        self.name = name
        self.capacity = capacity if capacity is not None else maxlen
        self._times: deque[float] = deque(maxlen=maxlen)
        self._drops = 0
        self._overwrites = 0
        self._report()

    def _report(self) -> None:
        now = time.monotonic()
        process_diagnostics().queue(
            QueueSnapshot(
                self.name,
                len(self),
                self.capacity,
                (now - self._times[0]) * 1000 if self._times else 0.0,
                self._drops,
                self._overwrites,
            ),
            now,
            owner=self,
        )

    def append(self, value: T) -> None:
        if self.maxlen is not None and len(self) == self.maxlen:
            self._overwrites = min(_LIMIT, self._overwrites + 1)
        super().append(value)
        self._times.append(time.monotonic())
        self._report()

    def insert(self, index: int, value: T) -> None:
        super().insert(index, value)
        self._times.insert(index, time.monotonic())
        self._report()

    def popleft(self) -> T:
        value = super().popleft()
        self._times.popleft()
        self._report()
        return value

    def pop(self) -> T:  # type: ignore[override]  # deque takes no index unlike MutableSequence.
        value = super().pop()
        self._times.pop()
        self._report()
        return value

    def __delitem__(self, index: SupportsIndex) -> None:  # type: ignore[override]
        self.discard_index(index)

    def discard_index(self, index: SupportsIndex, *, overwritten: bool = False) -> None:
        super().__delitem__(index)
        del self._times[index]
        if overwritten:
            self._overwrites = min(_LIMIT, self._overwrites + 1)
        else:
            self._drops = min(_LIMIT, self._drops + 1)
        self._report()

    def reject(self, count: int = 1) -> None:
        self._drops = min(_LIMIT, self._drops + count)
        self._report()

    def clear(self) -> None:
        self._drops = min(_LIMIT, self._drops + len(self))
        super().clear()
        self._times.clear()
        self._report()

    def remove(self, value: T) -> None:
        del self[self.index(value)]


class DiagnosticAsyncQueue(asyncio.Queue[T], Generic[T]):
    """Public Queue operations, with truthful waiting depth and drop accounting."""

    def __init__(self, name: str, maxsize: int = 0):
        super().__init__(maxsize=maxsize)
        self.name = name
        self._enqueued: deque[float] = deque()
        self._drops = 0

    def _report(self) -> None:
        now = time.monotonic()
        process_diagnostics().queue(
            QueueSnapshot(
                self.name,
                self.qsize(),
                self.maxsize or None,
                (now - self._enqueued[0]) * 1000 if self._enqueued else 0.0,
                self._drops,
            ),
            now,
            owner=self,
        )

    def put_nowait(self, item: T) -> None:
        try:
            super().put_nowait(item)
        except asyncio.QueueFull:
            self._drops = min(_LIMIT, self._drops + 1)
            self._report()
            raise
        self._enqueued.append(time.monotonic())
        self._report()

    def _take(self, *, discard: bool) -> T:
        item = super().get_nowait()
        enqueued = self._enqueued.popleft()
        if discard:
            self._drops = min(_LIMIT, self._drops + 1)
        stream = "queue_drop_wait." if discard else "queue_wait."
        process_diagnostics().duration(stream + self.name, time.monotonic() - enqueued)
        self._report()
        return item

    def get_nowait(self) -> T:
        return self._take(discard=False)

    def discard_nowait(self) -> T:
        return self._take(discard=True)


K = TypeVar("K", bound=Hashable)
V = TypeVar("V")


class DiagnosticPending(dict[K, V], Generic[K, V]):
    """Owner's keyed waiting tasks; replacement, cancellation and clear stay bounded."""

    def __init__(self, name: str, capacity: int | None = None):
        super().__init__()
        self.name, self.capacity = name, capacity
        self._at: dict[K, float] = {}
        self._drops = 0
        self._overwrites = 0

    def _report(self) -> None:
        now = time.monotonic()
        process_diagnostics().queue(
            QueueSnapshot(
                self.name,
                len(self),
                self.capacity,
                max(0.0, now - min(self._at.values())) * 1000 if self._at else 0.0,
                self._drops,
                self._overwrites,
            ),
            now,
            owner=self,
        )

    def __setitem__(self, key: K, value: V) -> None:
        self._overwrites = min(_LIMIT, self._overwrites + int(key in self))
        super().__setitem__(key, value)
        self._at[key] = time.monotonic()
        self._report()

    def __delitem__(self, key: K) -> None:
        super().__delitem__(key)
        self._at.pop(key, None)
        self._drops = min(_LIMIT, self._drops + 1)
        self._report()

    def pop(self, key: K, *default: Any) -> Any:
        result = super().pop(key, *default)
        at = self._at.pop(key, None)
        if at is not None:
            process_diagnostics().duration("queue_wait." + self.name, time.monotonic() - at)
        self._report()
        return result

    def clear(self) -> None:
        self._drops = min(_LIMIT, self._drops + len(self))
        super().clear()
        self._at.clear()
        self._report()


class DiagnosticLock(asyncio.Lock):
    """Count public lock acquisition waiters without inspecting asyncio internals."""

    def __init__(self, name: str):
        super().__init__()
        self._diagnostic = PendingCounter(name)
        self._held_token: object | None = None

    async def acquire(self) -> Literal[True]:
        token = object()
        self._diagnostic.enqueue(token, time.monotonic())
        try:
            result = await super().acquire()
        except BaseException:
            self._diagnostic.finish(token, time.monotonic())
            raise
        self._held_token = token
        self._diagnostic.start(token, time.monotonic())
        return result

    def release(self) -> None:
        super().release()
        token, self._held_token = self._held_token, None
        if token is not None:
            self._diagnostic.finish(token, time.monotonic())
