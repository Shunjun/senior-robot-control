"""Versioned, bounded remote control wire format."""

from typing import Annotated, Literal

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


class ResponsePayload(StrictModel):
    request_type: Literal[
        "control.acquire",
        "control.release",
        "drive.velocity",
        "drive.stop",
        "safety.estop",
        "safety.reset",
        "navigation.start",
        "navigation.cancel",
        "navigation.resume",
    ]
    ok: bool
    error: ErrorCode | None = None
    operation_id: str | None = None

    @model_validator(mode="after")
    def consistent_result(self) -> "ResponsePayload":
        if self.ok == (self.error is not None):
            raise ValueError("response success and error conflict")
        return self


class ResponseEnvelope(EnvelopeBase):
    type: Literal["response"]
    payload: ResponsePayload


Envelope = Annotated[
    VelocityEnvelope | SimpleEnvelope | ResponseEnvelope, Field(discriminator="type")
]
_adapter = TypeAdapter(Envelope)
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


def serialize_envelope(envelope: Envelope) -> bytes:
    """Serialize a validated envelope with the same size bound as input."""
    try:
        validated = _adapter.validate_python(envelope)
    except ValidationError as exc:
        raise ValueError("invalid envelope") from exc
    raw = _adapter.dump_json(validated)
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    return raw
