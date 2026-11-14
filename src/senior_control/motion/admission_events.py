"""Bounded accepted-event journal exported by the existing admission owner."""

from dataclasses import dataclass

from senior_control.motion.observations import AdaptedObservation, Source, SourceRule
from senior_control.motion.source_modes import SourceView


@dataclass(frozen=True)
class AdmissionEvent:
    cursor: int
    source: Source
    sample: AdaptedObservation | None  # None is an interruption/cache reset


@dataclass(frozen=True)
class SourceSnapshot:
    cursor: int
    events: tuple[AdmissionEvent, ...]
    journal_gap: bool
    sources: tuple[tuple[Source, SourceView], ...]
    now_ros: float
    now_steady: float
    rules: tuple[SourceRule, ...]
