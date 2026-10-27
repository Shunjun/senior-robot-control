"""Versioned, bounded remote control wire format."""

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, ValidationError


class StrictModel(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False, strict=True)


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
        "control.acquire", "control.release", "drive.stop", "safety.estop", "safety.reset"
    ]
    payload: EmptyPayload


Envelope = Annotated[VelocityEnvelope | SimpleEnvelope, Field(discriminator="type")]
_adapter = TypeAdapter(Envelope)
MAX_MESSAGE_BYTES = 65536


def validate_envelope(raw: bytes, expected_robot_id: str) -> EnvelopeBase:
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
