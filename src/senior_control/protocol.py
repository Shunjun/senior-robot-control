"""Versioned, bounded remote control wire format."""

from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, ValidationError, model_validator

from senior_control.errors import ErrorCode


class StrictModel(BaseModel):
    model_config = ConfigDict(
        extra="forbid",
        allow_inf_nan=False,
        strict=True,
        revalidate_instances="always",
        hide_input_in_errors=True,
    )


class EmptyPayload(StrictModel):
    pass


class VelocityPayload(StrictModel):
    vx: float
    wz: float


class EnvelopeBase(StrictModel):
    v: Literal[1]
    robot_id: str = Field(min_length=1, max_length=128)
    session_id: str = Field(min_length=1, max_length=128)
    connection_epoch: int = Field(ge=0)
    request_id: str = Field(min_length=1, max_length=128)
    seq: int = Field(ge=0)
    expires_at: float


class VelocityEnvelope(EnvelopeBase):
    type: Literal["drive.velocity"]
    payload: VelocityPayload


class SimpleEnvelope(EnvelopeBase):
    type: Literal[
        "control.acquire",
        "control.release",
        "drive.stop",
        "safety.estop",
        "safety.reset",
        "navigation.start",
        "navigation.cancel",
        "navigation.resume",
    ]
    payload: EmptyPayload


OPERATION_TYPES = (
    "task.create",
    "task.update",
    "task.delete",
    "task.enable",
    "home.set",
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


class HomePayload(StrictModel):
    waypoint_id: ShortText
    map_ref: MapRefPayload
    status: ShortText


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
        "map.export",
        "map.import",
        "map.clear",
        "operation.query",
        "control.acquire",
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
    ]
    ok: bool
    error: ErrorCode | None = None
    operation_id: str | None = None
    status: Literal[
        "accepted", "dispatching", "running", "succeeded", "failed", "needs_review"
    ] | None = None

    @model_validator(mode="after")
    def consistent_result(self) -> "ResponsePayload":
        if self.ok == (self.error is not None):
            raise ValueError("response success and error conflict")
        return self


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
    VelocityEnvelope | SimpleEnvelope | OperationEnvelope | SnapshotEnvelope | ResponseEnvelope,
    Field(discriminator="type"),
]
Frame = Annotated[
    VelocityEnvelope
    | SimpleEnvelope
    | OperationEnvelope
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
    raw = _frame_adapter.dump_json(validated)
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    return raw
