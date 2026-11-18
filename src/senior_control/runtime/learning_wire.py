"""Small typed local protocol; no runtime/healthy/allowed-action entrance gate."""

from typing import Annotated, Literal, TypeAlias

from pydantic import Field, TypeAdapter, model_validator

from senior_control.motion.learning_record import (
    FrozenFact,
    LearningGeometry,
    LearningRecord,
    Nonnegative,
    Phase,
    ProgramManifest,
    RunId,
)
from senior_control.protocol import LearningStartPayload, LearningStatusPayload
from senior_control.vision.raw_capture import RawCaptureHeader


class Attach(FrozenFact):
    active_revision: Nonnegative = 0
    type: Literal["attach"] = "attach"


class State(FrozenFact):
    record_revision: int = Field(default=-1, ge=-1, strict=True)
    active_revision: Nonnegative = 0
    type: Literal["state"] = "state"


class SessionRequest(FrozenFact):
    active_revision: Nonnegative = 0
    record_revision: int = Field(default=-1, ge=-1, strict=True)
    session: RunId


class Reserve(SessionRequest):
    type: Literal["reserve"] = "reserve"


class ActionReady(SessionRequest):
    type: Literal["action_ready"] = "action_ready"
    revision: Nonnegative


class ActionReport(SessionRequest):
    type: Literal["action"] = "action"
    start_ns: Nonnegative
    interval_ns: Nonnegative
    finished: bool = Field(strict=True)


class GeometryReport(SessionRequest):
    type: Literal["geometry"] = "geometry"
    geometry: LearningGeometry


class PhaseReport(SessionRequest):
    type: Literal["phase"] = "phase"
    phase: Phase
    reason: str = Field(max_length=256)
    segment_id: RunId | None = None


class RecordedFrame(FrozenFact):
    capture: RawCaptureHeader
    recorded_steady_ns: Nonnegative
    index: int = Field(strict=True, ge=1)
    geometry: LearningGeometry
    transform_stamp: float = Field(allow_inf_nan=False)
    ranges: tuple[float | Literal["inf", "-inf", "nan"], ...] = Field(min_length=1, max_length=2048)

    @model_validator(mode="after")
    def original_shape(self):
        if len(self.ranges) != self.geometry.count or not self.capture.valid:
            raise ValueError("learning_frame_geometry")
        return self


class FrameBatch(FrozenFact):
    type: Literal["frames"] = "frames"
    attempt_id: RunId
    stream_id: RunId
    frames: tuple[RecordedFrame, ...] = Field(max_length=4)
    gap: bool = False
    through_steady_ns: Nonnegative


class MotionObservedThrough(SessionRequest):
    type: Literal["motion_observed"] = "motion_observed"
    attempt_id: RunId
    segment_id: RunId
    through_steady_ns: Nonnegative
    validation_after_steady_ns: Nonnegative | None = None
    first_takeover_ns: Nonnegative | None = None


class BeginAttempt(FrozenFact):
    type: Literal["begin"] = "begin"
    start_request_id: str = Field(min_length=1, max_length=128)
    payload: LearningStartPayload
    fingerprint: str = Field(min_length=64, max_length=64)
    owner_boot_id: RunId
    generation: Nonnegative
    admitted: bool
    deadline_steady: float = Field(allow_inf_nan=False, ge=0)


class QueryAttempt(FrozenFact):
    type: Literal["query"] = "query"
    payload: LearningStatusPayload = LearningStatusPayload()


class AcceptAttempt(SessionRequest):
    type: Literal["accept_attempt"] = "accept_attempt"
    attempt_id: RunId
    accepted: bool
    reason: str = Field(max_length=256)


Request: TypeAlias = Annotated[
    Attach
    | State
    | Reserve
    | ActionReady
    | ActionReport
    | GeometryReport
    | PhaseReport
    | FrameBatch
    | MotionObservedThrough
    | BeginAttempt
    | QueryAttempt
    | AcceptAttempt,
    Field(discriminator="type"),
]
REQUEST = TypeAdapter[Request](Request)


class Reply(FrozenFact):
    ok: bool
    error: str | None = None
    record: LearningRecord | None = None
    manifest: ProgramManifest | None = None
    active_record: LearningRecord | None = None
    proposal: BeginAttempt | None = None
    attempt_id: RunId | None = None
    data: dict[str, object] | None = None
    through_index: Nonnegative = 0
    accepted_frames: Nonnegative = 0
    session: RunId | None = None
    revision: int | None = None
    ready: bool = False
    durable_revision: int = -1
