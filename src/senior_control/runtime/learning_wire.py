"""Small typed local protocol; no runtime/healthy/allowed-action entrance gate."""

from typing import Annotated, Literal, TypeAlias

from pydantic import Field, TypeAdapter

from senior_control.motion.learning_record import (
    MAX_ANCHORS,
    FrozenFact,
    LearningGeometry,
    LearningRecord,
    Nonnegative,
    Phase,
    Point,
    RunId,
)


class Attach(FrozenFact):
    type: Literal["attach"] = "attach"


class State(FrozenFact):
    type: Literal["state"] = "state"


class SessionRequest(FrozenFact):
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


class Complete(SessionRequest):
    type: Literal["complete"] = "complete"
    points: tuple[Point, ...] = Field(max_length=MAX_ANCHORS)


Request: TypeAlias = Annotated[
    Attach | State | Reserve | ActionReady | ActionReport | GeometryReport | PhaseReport | Complete,
    Field(discriminator="type"),
]
REQUEST = TypeAdapter[Request](Request)


class Reply(FrozenFact):
    ok: bool
    error: str | None = None
    record: LearningRecord | None = None
    session: RunId | None = None
    revision: int | None = None
    ready: bool = False
    durable_revision: int = -1
