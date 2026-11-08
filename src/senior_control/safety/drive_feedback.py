"""Correlate an admitted drive sample with the first actual safety output."""

from dataclasses import dataclass

from senior_control.models import Velocity
from senior_control.protocol import DriveExecutionFeedback, DriveFeedbackKey


@dataclass(frozen=True)
class _Sample:
    key: DriveFeedbackKey
    requested: Velocity
    deadline: float


class OutputFeedback:
    def __init__(self) -> None:
        self._sample: _Sample | None = None
        self._reported = False

    @property
    def current(self) -> DriveFeedbackKey | None:
        return self._sample.key if self._sample is not None else None

    def note(
        self, key: DriveFeedbackKey | None, requested: Velocity, deadline: float
    ) -> DriveExecutionFeedback | None:
        previous = self._sample
        if key is not None and previous is not None and key == previous.key:
            return None
        superseded = (
            DriveExecutionFeedback(**previous.key.model_dump(), status="superseded")
            if previous is not None and not self._reported
            else None
        )
        self._sample = _Sample(key, requested, deadline) if key is not None else None
        self._reported = False
        return superseded

    def published(
        self, velocity: Velocity, now: float, reason: str
    ) -> DriveExecutionFeedback | None:
        sample = self._sample
        if sample is None or self._reported:
            return None
        if reason == "evaluation_pending" and now < sample.deadline:
            return None
        self._reported = True
        key = sample.key.model_dump()
        if now >= sample.deadline:
            return DriveExecutionFeedback(**key, status="expired")
        if velocity == Velocity(0.0, 0.0) and velocity != sample.requested:
            return DriveExecutionFeedback(**key, status="blocked", reason=reason[:128])
        return DriveExecutionFeedback(
            **key, status="applied", vx=velocity.vx, wz=velocity.wz,
            limited=velocity != sample.requested,
        )
