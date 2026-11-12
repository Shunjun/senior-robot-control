"""One owner for clock/epoch admission and distinct-sample recovery history."""

import math
from dataclasses import dataclass, field

from senior_control.motion.observations import (
    AdaptedObservation,
    AdmissionConfig,
    Heading,
    Observation,
    Source,
    SourceRule,
    adapt,
    is_current,
    metadata_error,
    validate,
)
from senior_control.motion.source_modes import MotionAssessment, SourceView, select_reference


@dataclass(frozen=True)
class AdmissionDecision:
    accepted: bool
    reason: str


@dataclass
class _State:
    epoch: str | None = None
    retired: set[str] = field(default_factory=set)
    sequence: int = 0
    end_ros: float = -math.inf
    received_steady: float = -math.inf
    observation: AdaptedObservation | None = None
    count: int = 0
    first_ros: float = 0.0
    first_steady: float = 0.0
    ready: bool = False

    def interrupt(self) -> None:
        self.observation = None
        self.count = 0
        self.ready = False


class AdmissionOwner:
    def __init__(self, config: AdmissionConfig):
        self.config = config
        self._states = {source: _State() for source in Source}
        self._now_ros: float | None = None
        self._now_steady: float | None = None
        self._last_advance_steady: float | None = None
        self._heading_epoch: str | None = None
        self._retired_headings: set[str] = set()
        self._heading_stamp = -math.inf
        self._heading_received = -math.inf

    def _clock(self, now_ros: float, now_steady: float) -> bool:
        finite = math.isfinite(now_ros) and math.isfinite(now_steady)
        rollback = (
            finite
            and self._now_ros is not None
            and self._now_steady is not None
            and (now_ros < self._now_ros or now_steady < self._now_steady)
        )
        paused = (
            self._now_ros == now_ros
            and self._last_advance_steady is not None
            and now_steady - self._last_advance_steady > self.config.clock_pause_s
        )
        if not finite or rollback or paused:
            for state in self._states.values():
                state.interrupt()
                if rollback and state.epoch is not None:
                    state.retired.add(state.epoch)
                    state.epoch = None
                    state.sequence = 0
                    state.end_ros = -math.inf
                    state.received_steady = -math.inf
        if rollback and self._heading_epoch is not None:
            self._retired_headings.add(self._heading_epoch)
            self._heading_epoch = None
            self._heading_stamp = -math.inf
            self._heading_received = -math.inf
        if finite:
            if self._now_ros is None or now_ros != self._now_ros:
                self._last_advance_steady = now_steady
            self._now_ros, self._now_steady = now_ros, now_steady
        return finite and not rollback and not paused

    @staticmethod
    def _source_order(observation: Observation, state: _State) -> str | None:
        if observation.epoch in state.retired:
            return "retired_epoch"
        if observation.epoch == state.epoch and (
            observation.sequence <= state.sequence
            or observation.end_ros <= state.end_ros
            or observation.received_steady < state.received_steady
        ):
            return "replay_or_reordering"
        if (
            state.epoch is not None
            and observation.epoch != state.epoch
            and observation.received_steady < state.received_steady
        ):
            return "delayed_new_epoch"
        return None

    @staticmethod
    def _reject_evidence(
        observation: Observation, state: _State, *, now_ros: float, now_steady: float
    ) -> None:
        if observation.epoch != state.epoch:
            return
        # Structure and ordering are already valid. Fresh negative evidence
        # fences older packets; future timestamps never poison the watermark.
        if (
            state.end_ros < observation.end_ros <= now_ros
            and state.received_steady <= observation.received_steady <= now_steady
        ):
            state.sequence = observation.sequence
            state.end_ros = observation.end_ros
            state.received_steady = observation.received_steady
        state.interrupt()

    def _admit_heading(self, heading: Heading, state: _State) -> str | None:
        if heading.received_steady < self._heading_received or (
            heading.epoch == self._heading_epoch and heading.stamp_ros < self._heading_stamp
        ):
            return "heading_reordering"
        if heading.epoch in self._retired_headings:
            return "retired_heading"
        if self._heading_epoch is not None and heading.epoch != self._heading_epoch:
            if len(self._retired_headings) >= self.config.epoch_history_limit:
                state.interrupt()
                return "heading_history_exhausted"
            self._retired_headings.add(self._heading_epoch)
            state.interrupt()
        self._heading_epoch = heading.epoch
        self._heading_stamp = heading.stamp_ros
        self._heading_received = heading.received_steady
        return None

    def _admit_epoch(self, observation: Observation, state: _State) -> str | None:
        if observation.epoch == state.epoch:
            return None
        if len(state.retired) >= self.config.epoch_history_limit:
            state.interrupt()
            return "epoch_history_exhausted"
        if state.epoch is not None:
            state.retired.add(state.epoch)
        state.interrupt()
        state.epoch = observation.epoch
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
        observation: Observation,
        state: _State,
        rule: SourceRule,
        *,
        now_ros: float,
        now_steady: float,
        heading: Heading | None,
    ) -> None:
        self._expire(state, now_ros=now_ros, now_steady=now_steady)
        if state.count == 0:
            state.first_ros = observation.end_ros
            state.first_steady = observation.received_steady
        state.count += 1
        state.sequence, state.end_ros = observation.sequence, observation.end_ros
        state.received_steady = observation.received_steady
        state.observation = adapt(
            observation, now_ros=now_ros, now_steady=now_steady, rule=rule, heading=heading
        )
        span = self.config.recovery_span_s
        state.ready = (
            state.count >= self.config.recovery_samples
            and observation.end_ros - state.first_ros + math.ulp(observation.end_ros) >= span
            and observation.received_steady
            - state.first_steady
            + math.ulp(observation.received_steady)
            >= span
        )

    def ingest(
        self,
        observation: Observation,
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

    def assess(self, *, now_ros: float, now_steady: float) -> MotionAssessment:
        clocks_valid = self._clock(now_ros, now_steady)
        views = {}
        if clocks_valid:
            for source, state in self._states.items():
                self._expire(state, now_ros=now_ros, now_steady=now_steady)
                if state.observation is not None:
                    views[source] = SourceView(state.observation, state.ready)
        return select_reference(views, self.config, now_steady=now_steady)
