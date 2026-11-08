"""Versioned, bounded remote control wire format."""

from typing import Annotated, Any, Literal, get_args

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    TypeAdapter,
    ValidationError,
    model_serializer,
    model_validator,
)

from senior_control.errors import ErrorCode
from senior_control.navigation import RouteProgress

DRIVE_CHALLENGE_SECONDS = 0.3
DRIVE_TRANSPORT_SECONDS = 0.5
DRIVE_HOLD_FLOOR_SECONDS = 0.2
DEVICE_REVOKED_CLOSE_CODE = 4403
_MAX_SAFE_WEB_INTEGER = 2**53 - 1


class StrictModel(BaseModel):
    model_config = ConfigDict(
        extra="forbid",
        allow_inf_nan=False,
        strict=True,
        revalidate_instances="always",
        hide_input_in_errors=True,
    )


class DriveClockConfirmation(StrictModel):
    nonce: str = Field(min_length=1, max_length=128)
    t4: float


class DriveClockProbe(StrictModel):
    type: Literal["drive.clock"]
    nonce: str = Field(min_length=1, max_length=128)
    t1: float
    confirmation: DriveClockConfirmation | None = None


class HeartbeatRequest(StrictModel):
    v: Literal[1]
    type: Literal["heartbeat.request"]
    robot_id: str = Field(min_length=1, max_length=128)
    connection_epoch: int = Field(ge=1)
    nonce: str = Field(min_length=1, max_length=128)
    sent_at: float
    drive_t1: float | None = None
    drive_confirmation: DriveClockConfirmation | None = None


class EmptyPayload(StrictModel):
    gesture: int | None = Field(default=None, ge=1, le=_MAX_SAFE_WEB_INTEGER)


class VelocityPayload(StrictModel):
    vx: float
    wz: float
    challenge_id: str | None = Field(default=None, min_length=1, max_length=128)
    server_deadline: float | None = None
    gesture: int | None = Field(default=None, ge=1, le=_MAX_SAFE_WEB_INTEGER)
    sampled_at: float | None = None
    server_issued_at: float | None = None

    @model_validator(mode="after")
    def validate_sample(self) -> "VelocityPayload":
        if (self.gesture is None) != (self.sampled_at is None):
            raise ValueError("sample requires both gesture and source time")
        if self.gesture is not None:
            if self.challenge_id is not None:
                raise ValueError("timed sample cannot use a legacy challenge")
            if (self.server_issued_at is None) != (self.server_deadline is None):
                raise ValueError("sample grant requires both issuance and deadline")
        elif self.server_issued_at is not None:
            raise ValueError("sample grant requires a timed sample")
        return self


class EnvelopeBase(StrictModel):
    v: Literal[1]
    robot_id: str = Field(min_length=1, max_length=128)
    session_id: str = Field(min_length=1, max_length=128)
    connection_epoch: int = Field(ge=0)
    request_id: str = Field(min_length=1, max_length=128)
    seq: int = Field(ge=0, le=2**63 - 1)
    expires_at: float


class DriveFeedbackKey(StrictModel):
    robot_id: str = Field(min_length=1, max_length=128)
    session_id: str = Field(min_length=1, max_length=128)
    connection_epoch: int = Field(ge=1)
    request_id: str = Field(min_length=1, max_length=128)
    seq: int = Field(ge=1, le=_MAX_SAFE_WEB_INTEGER)
    gesture: int = Field(ge=1, le=_MAX_SAFE_WEB_INTEGER)


class DriveExecutionFeedback(DriveFeedbackKey):
    type: Literal["drive.feedback"] = "drive.feedback"
    status: Literal["applied", "blocked", "superseded", "expired"]
    vx: float | None = None
    wz: float | None = None
    limited: bool | None = None
    reason: str | None = Field(default=None, min_length=1, max_length=128)

    @classmethod
    def superseded(cls, envelope: "VelocityEnvelope") -> "DriveExecutionFeedback":
        gesture = envelope.payload.gesture
        if gesture is None:
            raise ValueError("legacy samples do not carry feedback correlation")
        return cls(
            robot_id=envelope.robot_id,
            session_id=envelope.session_id,
            connection_epoch=envelope.connection_epoch,
            request_id=envelope.request_id,
            seq=envelope.seq,
            gesture=gesture,
            status="superseded",
        )

    @model_validator(mode="after")
    def validate_output(self) -> "DriveExecutionFeedback":
        fields = self.vx, self.wz, self.limited
        if self.status == "applied":
            if any(value is None for value in fields) or self.reason is not None:
                raise ValueError("applied feedback requires actual output")
        elif any(value is not None for value in fields):
            raise ValueError("only applied feedback may include actual output")
        return self


class VelocityEnvelope(EnvelopeBase):
    type: Literal["drive.velocity"]
    payload: VelocityPayload

    @model_validator(mode="after")
    def validate_timed_sequence(self) -> "VelocityEnvelope":
        if self.payload.gesture is not None and not 1 <= self.seq <= _MAX_SAFE_WEB_INTEGER:
            raise ValueError("timed sample sequence must be a positive browser-safe integer")
        return self


class SimpleEnvelope(EnvelopeBase):
    type: Literal[
        "control.acquire",
        "drive.challenge",
        "control.release",
        "drive.stop",
        "safety.estop",
        "safety.reset",
        "navigation.start",
        "navigation.cancel",
        "navigation.resume",
    ]
    payload: EmptyPayload

    @model_validator(mode="after")
    def validate_gesture(self) -> "SimpleEnvelope":
        if self.payload.gesture is not None and self.type != "drive.stop":
            raise ValueError("gesture is only supported for drive.stop")
        return self


OPERATION_TYPES = (
    "task.create",
    "task.update",
    "task.delete",
    "task.enable",
    "home.set",
    "navigation.goal",
    "navigation.return_home",
    "waypoint.create",
    "map.export",
    "map.import",
    "map.clear",
    "operation.query",
)


class OperationPayload(StrictModel):
    """Device-owned edit. The robot dedupes by ``operation_id`` and guards by revision."""

    operation_id: str = Field(min_length=1, max_length=128)
    expected_revision: int | None = Field(default=None, ge=0)
    body: dict[str, Any] = Field(default_factory=dict)


class OperationEnvelope(EnvelopeBase):
    type: Literal[
        "task.create",
        "task.update",
        "task.delete",
        "task.enable",
        "home.set",
        "navigation.goal",
        "navigation.return_home",
        "waypoint.create",
        "map.export",
        "map.import",
        "map.clear",
        "operation.query",
    ]
    payload: OperationPayload


ShortText = Annotated[str, Field(min_length=1, max_length=128)]


class MapRefPayload(StrictModel):
    map_id: ShortText
    reference_revision: int = Field(ge=0)


WorkspaceRequest = Literal["task.get", "waypoint.list", "map.manifest", "map.tile"]
WORKSPACE_REQUEST_TYPES = frozenset(get_args(WorkspaceRequest))


class WorkspacePayload(StrictModel):
    map_ref: MapRefPayload
    task_id: ShortText | None = None
    cursor: ShortText | None = None
    grid_id: ShortText | None = None
    tile_x: int | None = Field(default=None, ge=0, le=2047)
    tile_y: int | None = Field(default=None, ge=0, le=2047)

    @model_serializer(mode="wrap")
    def serialize(self, handler: Any) -> dict[str, Any]:
        result: dict[str, Any] = handler(self)
        return {field: value for field, value in result.items() if value is not None}


class WorkspaceEnvelope(EnvelopeBase):
    type: WorkspaceRequest
    payload: WorkspacePayload

    @model_validator(mode="after")
    def matching_fields(self) -> "WorkspaceEnvelope":
        fields = self.payload.model_fields_set - {"map_ref"}
        match self.type:
            case "task.get":
                valid = fields == {"task_id"} and self.payload.task_id is not None
            case "waypoint.list":
                valid = fields <= {"cursor"}
            case "map.manifest":
                valid = not fields
            case "map.tile":
                valid = fields == {"grid_id", "tile_x", "tile_y"} and all(
                    getattr(self.payload, field) is not None for field in fields
                )
        if not valid:
            raise ValueError("invalid workspace read parameters")
        return self


class HomePayload(StrictModel):
    waypoint_id: ShortText
    map_ref: MapRefPayload
    status: ShortText


class TaskSummaryPayload(StrictModel):
    task_id: ShortText
    revision: int = Field(ge=0)
    enabled: bool


class SnapshotPayload(StrictModel):
    """The robot's own view of itself, pushed on connect and after each change."""

    connected: bool
    localized: bool
    map_ref: MapRefPayload | None
    home: HomePayload | None
    task_revision: int = Field(ge=0)
    runnable_task_ids: list[ShortText] = Field(max_length=200)
    execution_status: str = Field(min_length=1, max_length=64)
    collected_at: float
    # Set only by the cloud when it sends its cached copy to a browser; an agent may omit both.
    stale: bool = False
    last_synced_at: float | None = None
    tasks: list[TaskSummaryPayload] = Field(default_factory=list, max_length=200)
    # home_edit.revision. 0 is the store's answer when that row does not exist yet.
    home_revision: int = Field(default=0, ge=0)
    map_generation: ShortText | None = None
    route_progress: RouteProgress | None = None

    @model_serializer(mode="wrap")
    def serialize(self, handler: Any) -> dict[str, Any]:
        result: dict[str, Any] = handler(self)
        # //Compatible: omit generation in snapshots without a qualified source.
        if self.map_generation is None:
            result.pop("map_generation", None)
        if self.route_progress is None:
            result.pop("route_progress", None)
        return result


class SnapshotEnvelope(EnvelopeBase):
    type: Literal["snapshot"]
    payload: SnapshotPayload


class ResponsePayload(StrictModel):
    request_type: Literal[
        "task.create",
        "task.update",
        "task.delete",
        "task.enable",
        "home.set",
        "navigation.goal",
        "navigation.return_home",
        "waypoint.create",
        "map.export",
        "map.import",
        "map.clear",
        "operation.query",
        "control.acquire",
        "drive.challenge",
        "control.release",
        "drive.velocity",
        "drive.stop",
        "safety.estop",
        "safety.reset",
        "navigation.start",
        "navigation.cancel",
        "navigation.resume",
        "video.offer",
        "video.answer",
        "video.candidate",
        "video.close",
        "task.get",
        "waypoint.list",
        "map.manifest",
        "map.tile",
    ]
    ok: bool
    error: ErrorCode | None = None
    operation_id: str | None = None
    challenge_id: str | None = Field(default=None, min_length=1, max_length=128)
    server_deadline: float | None = None
    data: dict[str, Any] | None = None
    status: (
        Literal["accepted", "dispatching", "running", "succeeded", "failed", "needs_review"] | None
    ) = None

    @model_validator(mode="after")
    def consistent_result(self) -> "ResponsePayload":
        if self.ok == (self.error is not None):
            raise ValueError("response success and error conflict")
        if (self.data is not None) != (self.ok and self.request_type in WORKSPACE_REQUEST_TYPES):
            raise ValueError("workspace response data does not match its request")
        return self

    @model_serializer(mode="wrap")
    def serialize(self, handler: Any) -> dict[str, Any]:
        result: dict[str, Any] = handler(self)
        # //Compatible: strict v1 peers accept the original response fields.
        if self.data is None:
            result.pop("data", None)
        return result


class ResponseEnvelope(EnvelopeBase):
    type: Literal["response"]
    payload: ResponsePayload


MAX_SDP_CHARS = 16384
MAX_CANDIDATE_CHARS = 512


class SdpPayload(StrictModel):
    sdp: str = Field(min_length=1, max_length=MAX_SDP_CHARS)


class CandidatePayload(StrictModel):
    candidate: str = Field(min_length=1, max_length=MAX_CANDIDATE_CHARS, pattern=r"^candidate:")


class VideoSdpEnvelope(EnvelopeBase):
    """Video signaling. The controller sends the offer; the vehicle sends the answer."""

    type: Literal["video.offer", "video.answer"]
    payload: SdpPayload


class VideoCandidateEnvelope(EnvelopeBase):
    """An ICE candidate, in either direction."""

    type: Literal["video.candidate"]
    payload: CandidatePayload


class VideoCloseEnvelope(EnvelopeBase):
    """Ends the video of one signaling id. The relay sends it when the viewer goes away."""

    type: Literal["video.close"]
    payload: EmptyPayload


VideoEnvelope = VideoSdpEnvelope | VideoCandidateEnvelope | VideoCloseEnvelope

Envelope = Annotated[
    VelocityEnvelope
    | SimpleEnvelope
    | OperationEnvelope
    | WorkspaceEnvelope
    | SnapshotEnvelope
    | ResponseEnvelope,
    Field(discriminator="type"),
]
Frame = Annotated[
    VelocityEnvelope
    | SimpleEnvelope
    | OperationEnvelope
    | WorkspaceEnvelope
    | SnapshotEnvelope
    | ResponseEnvelope
    | VideoSdpEnvelope
    | VideoCandidateEnvelope
    | VideoCloseEnvelope,
    Field(discriminator="type"),
]
_adapter = TypeAdapter(Envelope)
_frame_adapter = TypeAdapter(Frame)
MAX_MESSAGE_BYTES = 65536


def validate_envelope(raw: bytes, expected_robot_id: str) -> Envelope:
    """Validate a bounded message for the authenticated robot."""
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    try:
        envelope = _adapter.validate_json(raw)
    except (ValidationError, ValueError) as exc:
        raise ValueError("invalid envelope") from exc
    if envelope.robot_id != expected_robot_id:
        raise ValueError("wrong robot")
    return envelope


def validate_frame(raw: bytes, expected_robot_id: str) -> Envelope | VideoEnvelope:
    """Like `validate_envelope`, and also accepts video signaling for the relay to forward."""
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    try:
        frame = _frame_adapter.validate_json(raw)
    except (ValidationError, ValueError) as exc:
        raise ValueError("invalid envelope") from exc
    if frame.robot_id != expected_robot_id:
        raise ValueError("wrong robot")
    return frame


def serialize_envelope(envelope: Envelope | VideoEnvelope) -> bytes:
    """Serialize a validated envelope with the same size bound as input."""
    try:
        validated = _frame_adapter.validate_python(envelope)
    except ValidationError as exc:
        raise ValueError("invalid envelope") from exc
    # //Compatible: omit new optional timing fields from legacy wire payloads.
    excluded: dict[str, set[str]] = {}
    if isinstance(validated, VelocityEnvelope) and validated.payload.gesture is None:
        excluded["payload"] = {"gesture", "sampled_at", "server_issued_at"}
    if isinstance(validated, SimpleEnvelope) and validated.payload.gesture is None:
        excluded["payload"] = {"gesture"}
    raw = _frame_adapter.dump_json(validated, exclude=excluded)
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    return raw
