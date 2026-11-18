"""Immutable, bounded facts for one program run's learning record."""

import hashlib
import json
import time
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

MAX_ANCHORS = 2048
MAX_RECORD_BYTES = 262144
RUN_ID = r"^[0-9a-f]{32}$"
RunId = Annotated[str, Field(pattern=RUN_ID)]
Nonnegative = Annotated[int, Field(strict=True, ge=0)]
Coordinate = Annotated[float, Field(allow_inf_nan=False)]
Point = tuple[Coordinate, Coordinate]
Phase = Literal[
    "not_started", "sampling", "moving", "validating", "completed", "unconfirmed", "failed"
]


class FrozenFact(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", allow_inf_nan=False, strict=True)


class LearningGeometry(FrozenFact):
    body_frame: str = Field(min_length=1, max_length=256)
    sensor_frame: str = Field(min_length=1, max_length=256)
    source_kind: Literal["scan"] = "scan"
    physical_polygon: tuple[Point, ...] = Field(min_length=3)
    sensor_to_body: tuple[Coordinate, Coordinate, Coordinate]
    angle_min: Coordinate
    angle_increment: Coordinate = Field(gt=0)
    count: int = Field(strict=True, gt=0)
    range_min: Coordinate = Field(ge=0)
    range_max: Coordinate = Field(gt=0)
    classifier: Literal["body-return-1cm-v3"] = "body-return-1cm-v3"

    @model_validator(mode="after")
    def valid_range(self) -> "LearningGeometry":
        from shapely.geometry import Polygon

        body = Polygon(self.physical_polygon)
        if not body.is_valid or body.area <= 0:
            raise ValueError("learning_geometry_polygon")
        if self.range_max <= self.range_min:
            raise ValueError("learning_geometry_range")
        return self

    @property
    def digest(self) -> str:
        payload = json.dumps(
            self.model_dump(), sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        return hashlib.sha256(payload.encode()).hexdigest()


class LearningAction(FrozenFact):
    consumed: bool = Field(default=False, strict=True)
    stage: Literal[
        "not_started",
        "reserved_may_have_output",
        "running",
        "window_finished",
        "withdrawn",
        "unknown",
    ] = "not_started"
    speed_mps: Coordinate = 0.05
    duration_ns: Literal[3000000000] = 3000000000
    first_submitted_steady_ns: Nonnegative | None = None
    deadline_steady_ns: Nonnegative | None = None
    reported_nonzero_interval_ns: Nonnegative = Field(default=0, le=3000000000)
    execution_measurement: Literal["local_submission_not_physical_ack"] = (
        "local_submission_not_physical_ack"
    )

    @model_validator(mode="after")
    def coherent(self) -> "LearningAction":
        if self.speed_mps != 0.05:
            raise ValueError("learning_action_speed_policy")
        if self.consumed != (self.stage != "not_started"):
            raise ValueError("learning_action_consumption")
        start, end = self.first_submitted_steady_ns, self.deadline_steady_ns
        if (start is None) != (end is None):
            raise ValueError("learning_action_interval")
        if start is not None and end != start + self.duration_ns:
            raise ValueError("learning_action_deadline")
        if self.stage in {"running", "window_finished"} and start is None:
            raise ValueError("learning_action_missing_start")
        if not self.consumed and self.reported_nonzero_interval_ns:
            raise ValueError("learning_action_unconsumed_duration")
        return self


class LearningAnchor(FrozenFact):
    id: int = Field(strict=True, ge=0, lt=MAX_ANCHORS)
    point: Point


class LearnedCatalog(FrozenFact):
    model_id: RunId
    anchors: tuple[LearningAnchor, ...] = Field(max_length=MAX_ANCHORS)

    @model_validator(mode="after")
    def valid_ids(self) -> "LearnedCatalog":
        ids = {anchor.id for anchor in self.anchors}
        if len(ids) != len(self.anchors):
            raise ValueError("learning_anchor_ids")
        return self

    @property
    def points(self) -> tuple[tuple[float, float], ...]:
        return tuple(anchor.point for anchor in self.anchors)


class CaptureSummary(FrozenFact):
    state: Literal["open", "complete", "incomplete"] = "open"
    stream_id: RunId | None = None
    through_index: Nonnegative = 0
    validation_after_steady_ns: Nonnegative | None = None
    closed_steady_ns: Nonnegative | None = None
    accepted_frames: Nonnegative = 0
    compute_complete: bool = False


class ActiveReference(FrozenFact):
    attempt_id: RunId
    model_id: RunId
    record_revision: Nonnegative


class ProgramManifest(FrozenFact):
    schema_version: Literal[2] = 2
    run_id: RunId
    owner_epoch: RunId
    revision: Nonnegative = 0
    latest_attempt_id: RunId
    active_revision: Nonnegative = 0
    active_ref: ActiveReference | None = None

    def to_bytes(self) -> bytes:
        return self.model_dump_json().encode()


def attempt_identity(run_id: str, request_id: str | None = None) -> str:
    trigger = "startup" if request_id is None else "manual:" + request_id
    return hashlib.sha256((run_id + ":" + trigger).encode()).hexdigest()[:32]


class LearningRecord(FrozenFact):
    schema_version: Literal[2] = 2
    attempt_id: RunId
    created_steady_ns: Nonnegative = Field(default_factory=time.monotonic_ns)
    trigger: Literal["startup", "manual"] = "startup"
    start_request_id: str | None = Field(default=None, min_length=1, max_length=128)
    request_fingerprint: str | None = Field(default=None, max_length=64)
    capture_summary: CaptureSummary = CaptureSummary()
    run_id: RunId
    clock_domain: str = Field(min_length=1, max_length=256)
    owner_epoch: RunId
    revision: Nonnegative = 0
    phase: Phase = "not_started"
    reason: str = Field(default="new_run", max_length=256)
    geometry: LearningGeometry | None = None
    action: LearningAction = LearningAction()
    model: LearnedCatalog | None = None
    segment_id: RunId | None = None
    committed_steady_ns: Nonnegative = 0

    @model_validator(mode="after")
    def valid_model(self) -> "LearningRecord":
        if self.model is not None and self.geometry is None:
            raise ValueError("learning_model_without_geometry")
        if self.phase == "completed" and self.model is None:
            raise ValueError("learning_completed_without_model")
        return self

    def to_bytes(self) -> bytes:
        data = json.dumps(
            self.model_dump(mode="json"), sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
        if len(data) > MAX_RECORD_BYTES:
            raise ValueError("learning_record_size")
        return data


def _unique_object(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("learning_record_duplicate_key")
        result[key] = value
    return result


def _invalid_constant(value: str) -> object:
    raise ValueError("learning_record_nonfinite_" + value)


def parse_record(data: bytes) -> LearningRecord:
    if len(data) > MAX_RECORD_BYTES:
        raise ValueError("learning_record_size")
    json.loads(data, object_pairs_hook=_unique_object, parse_constant=_invalid_constant)
    # JSON decoding keeps strict primitive types while accepting JSON arrays for
    # immutable tuple fields; the first pass rejects duplicate keys explicitly.
    return LearningRecord.model_validate_json(data)
