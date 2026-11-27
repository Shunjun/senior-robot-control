"""One owner for clock/epoch admission and distinct-sample recovery history."""

import math
from collections import deque
from dataclasses import dataclass, field

from senior_motion_estimation.admission_events import AdmissionEvent, SourceSnapshot
from senior_motion_estimation.observations import (
    AdaptedObservation,
    AdmissionConfig,
    Heading,
    Source,
    SourceRule,
    adapt,
    is_current,
    metadata_error,
    validate,
)
from senior_motion_estimation.sample_types import MotionSample, physical_support
from senior_motion_estimation.source_modes import SourceView
from vehicle_core.sample_time import ClockTracker
from vehicle_core.source_order import SourceOrder, source_context


@dataclass(frozen=True)
class AdmissionDecision:
    accepted: bool
    reason: str


@dataclass
class _State:
    epoch: str | None = None
    order: SourceOrder = field(default_factory=SourceOrder)
    sequence: int = 0
    end_steady: float = -math.inf
    received_steady: float = -math.inf
    stamp_ns: int | None = None
    observation: AdaptedObservation | None = None
    count: int = 0
    first_measurement_steady: float = 0.0
    first_steady: float = 0.0
    ready: bool = False

    def interrupt(self) -> None:
        self.observation = None
        self.count = 0
        self.ready = False


class AdmissionOwner:
    def __init__(self, config: AdmissionConfig, *, event_capacity: int | None = None):
        # //Compatible: legacy pure callers need no journal; T3b must inject capacity.
        if event_capacity is not None and event_capacity < 1:
            raise ValueError("event_capacity must be positive")
        self._events: deque[AdmissionEvent] = deque(maxlen=event_capacity)
        self._event_capacity = event_capacity
        self._cursor = 0
        self.config = config
        self._states = {source: _State() for source in Source}
        self._now_steady: float | None = None
        self._tracker = ClockTracker()
        self._heading_epoch: str | None = None
        self._heading_stamp = -math.inf
        self._heading_received = -math.inf

    def _clock(self, now_ros: float, now_steady: float) -> bool:
        finite = math.isfinite(now_ros) and math.isfinite(now_steady)
        rollback = (
            finite
            and self._tracker.observe(
                ros_ns=round(now_ros * 1e9), steady_ns=round(now_steady * 1e9)
            ).discontinuity
        )
        if not finite or rollback:
            for state in self._states.values():
                state.interrupt()
                if rollback and state.epoch is not None:
                    state.epoch = None
                    state.sequence = 0
                    state.end_steady = -math.inf
                    state.received_steady = -math.inf
        if rollback and self._heading_epoch is not None:
            self._heading_epoch = None
            self._heading_stamp = -math.inf
            self._heading_received = -math.inf
        if finite:
            self._now_steady = now_steady
        return finite and not rollback

    @staticmethod
    def _source_order(observation: MotionSample, state: _State) -> str | None:
        # MotionSample is constructed by the native adapter/session owner, not
        # decoded from arbitrary DDS body identities at this algorithm boundary.
        boot, generation = source_context(observation.epoch)
        sample_time = getattr(observation, "time", None)
        if sample_time is None:
            sample_time = observation.last_time
        # Adapter-owned structured lifetimes carry immutable native first receipts.
        accepted = (
            state.order.accept_capture(
                boot,
                generation,
                physical_support(observation).end_ns,
                sample_time.received_steady_ns,
            )
            if observation.epoch != boot
            else state.order.accept(boot, generation, physical_support(observation).end_ns)
        )
        if not accepted:
            return state.order.reason
        return None

    def confirm_source_boot(self, source: Source, boot_id: str) -> None:
        """Explicit receiving-session replacement, never an arbitrary sample epoch."""
        state = self._states[source]
        if state.order.boot_id != boot_id:
            state.order.confirm_boot(boot_id)
            state.interrupt()

    @staticmethod
    def _reject_evidence(
        observation: MotionSample, state: _State, *, now_ros: float, now_steady: float
    ) -> None:
        if observation.epoch != state.epoch:
            return
        # Structure and ordering are already valid. Fresh negative evidence
        # fences older packets; future timestamps never poison the watermark.
        if (
            state.end_steady < observation.end_steady <= now_steady
            and state.received_steady <= observation.received_steady <= now_steady
        ):
            state.sequence = observation.sequence
            state.end_steady = observation.end_steady
            state.stamp_ns = physical_support(observation).end_ns
            state.received_steady = observation.received_steady
        state.interrupt()

    def _admit_heading(self, heading: Heading, state: _State) -> str | None:
        if heading.received_steady < self._heading_received or (
            heading.epoch == self._heading_epoch and heading.time.stamp_ns < self._heading_stamp
        ):
            return "heading_reordering"
        if self._heading_epoch is not None and heading.epoch != self._heading_epoch:
            state.interrupt()
        self._heading_epoch = heading.epoch
        self._heading_stamp = heading.time.stamp_ns
        self._heading_received = heading.received_steady
        return None

    def _admit_epoch(self, observation: MotionSample, state: _State) -> str | None:
        if observation.epoch == state.epoch:
            return None
        state.interrupt()
        state.epoch = observation.epoch
        state.sequence = 0
        state.end_steady = -math.inf
        state.stamp_ns = None
        state.received_steady = -math.inf
        return None

    def _expire(self, state: _State, *, now_ros: float, now_steady: float) -> None:
        observation = state.observation
        if observation is not None and not is_current(
            observation,
            self.config.rule(observation.original.source),
            now_ros=now_ros,
            now_steady=now_steady,
        ):
            state.interrupt()

    def _accept(
        self,
        observation: MotionSample,
        state: _State,
        rule: SourceRule,
        *,
        now_ros: float,
        now_steady: float,
        heading: Heading | None,
    ) -> None:
        self._expire(state, now_ros=now_ros, now_steady=now_steady)
        if state.count == 0:
            state.first_measurement_steady = observation.end_steady
            state.first_steady = observation.received_steady
        state.count += 1
        state.sequence, state.end_steady = observation.sequence, observation.end_steady
        state.received_steady = observation.received_steady
        state.stamp_ns = physical_support(observation).end_ns
        state.observation = adapt(
            observation, now_ros=now_ros, now_steady=now_steady, rule=rule, heading=heading
        )
        span = self.config.recovery_span_ms / 1000
        state.ready = (
            state.count >= self.config.recovery_samples
            and observation.end_steady
            - state.first_measurement_steady
            + math.ulp(observation.end_steady)
            >= span
            and observation.received_steady
            - state.first_steady
            + math.ulp(observation.received_steady)
            >= span
        )

    def _ingest(
        self,
        observation: MotionSample,
        *,
        now_ros: float,
        now_steady: float,
        heading: Heading | None = None,
    ) -> AdmissionDecision:
        structural = metadata_error(observation, heading, now_ros=now_ros, now_steady=now_steady)
        if structural is not None:
            return AdmissionDecision(False, structural)
        if not self._clock(now_ros, now_steady):
            return AdmissionDecision(False, "clock_discontinuity")
        state = self._states[observation.source]
        ordering = self._source_order(observation, state)
        if ordering is not None:
            return AdmissionDecision(False, ordering)
        if state.epoch is not None and observation.epoch != state.epoch:
            reason = self._admit_epoch(observation, state)
            if reason is not None:
                return AdmissionDecision(False, reason)
        rule = self.config.rule(observation.source)
        reason = validate(
            observation,
            rule,
            now_ros=now_ros,
            now_steady=now_steady,
            heading=heading,
            config=self.config,
        )
        if reason is not None:
            self._reject_evidence(observation, state, now_ros=now_ros, now_steady=now_steady)
            return AdmissionDecision(False, reason)
        if observation.source is Source.GPS and heading is not None:
            reason = self._admit_heading(heading, state)
            if reason is not None:
                return AdmissionDecision(False, reason)
        reason = self._admit_epoch(observation, state)
        if reason is not None:
            return AdmissionDecision(False, reason)
        self._accept(
            observation, state, rule, now_ros=now_ros, now_steady=now_steady, heading=heading
        )
        return AdmissionDecision(True, "admitted")

    def _markers(self) -> dict[Source, tuple[AdaptedObservation | None, str | None, int]]:
        return {
            source: (state.observation, state.epoch, state.count)
            for source, state in self._states.items()
        }

    def _record(
        self, before: dict[Source, tuple[AdaptedObservation | None, str | None, int]]
    ) -> None:
        if self._event_capacity is None:
            return
        for source, state in self._states.items():
            previous, epoch, count = before[source]
            current = state.observation
            if current is previous:
                continue
            reset = previous is not None and (
                current is None or epoch != state.epoch or state.count <= count
            )
            if reset:
                self._cursor += 1
                self._events.append(AdmissionEvent(self._cursor, source, None))
            if current is not None:
                self._cursor += 1
                self._events.append(AdmissionEvent(self._cursor, source, current))

    def ingest(
        self,
        observation: MotionSample,
        *,
        now_ros: float,
        now_steady: float,
        heading: Heading | None = None,
    ) -> AdmissionDecision:
        structural = metadata_error(observation, heading, now_ros=now_ros, now_steady=now_steady)
        if structural is not None:
            return AdmissionDecision(False, structural)
        before = self._markers()
        decision = self._ingest(
            observation, now_ros=now_ros, now_steady=now_steady, heading=heading
        )
        self._record(before)
        return decision

    def interrupt_source(self, source: Source) -> None:
        """Revoke a native writer/channel without creating a replacement observation.

        Preserve the sequence/time frontier; recovery still needs distinct real
        samples. Export the interruption through this owner's existing journal.
        """
        before = self._markers()
        self._states[source].interrupt()
        self._record(before)

    def _views(self, *, now_ros: float, now_steady: float) -> dict[Source, SourceView]:
        before = self._markers()
        views = {}
        if self._clock(now_ros, now_steady):
            for source, state in self._states.items():
                self._expire(state, now_ros=now_ros, now_steady=now_steady)
                if state.observation is not None:
                    views[source] = SourceView(state.observation, state.ready)
        self._record(before)
        return views

    def snapshot(self, *, after_cursor: int, now_ros: float, now_steady: float) -> SourceSnapshot:
        if self._event_capacity is None or after_cursor < 0 or after_cursor > self._cursor:
            raise ValueError("T3b snapshot needs configured journal and valid cursor")
        views = self._views(now_ros=now_ros, now_steady=now_steady)
        gap = bool(self._events and after_cursor < self._events[0].cursor - 1)
        events = tuple(event for event in self._events if event.cursor > after_cursor)
        return SourceSnapshot(
            self._cursor, events, gap, tuple(views.items()), now_ros, now_steady, self.config.rules
        )
