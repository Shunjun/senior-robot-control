import json

import pytest

from senior_control.protocol import validate_envelope


def message(**changes):
    data = {
        "v": 1,
        "type": "drive.velocity",
        "robot_id": "r1",
        "session_id": "s",
        "connection_epoch": 1,
        "request_id": "q",
        "seq": 1,
        "expires_at": 123.0,
        "payload": {"vx": 0.2, "wz": 0.1},
    }
    data.update(changes)
    return json.dumps(data).encode()


def test_valid_velocity():
    assert validate_envelope(message(), "r1").payload.vx == 0.2


@pytest.mark.parametrize(
    "changes",
    [
        {"v": 2},
        {"robot_id": "other"},
        {"type": "unknown"},
        {"payload": {"vx": float("nan"), "wz": 0}},
        {"payload": {"vx": 0, "wz": float("inf")}},
        {"payload": {"vx": 0, "wz": 0, "vy": 1}},
    ],
)
def test_reject_invalid_envelope(changes):
    with pytest.raises(ValueError):
        validate_envelope(message(**changes), "r1")


def test_reject_oversized_message():
    with pytest.raises(ValueError):
        validate_envelope(message(session_id="x" * 100_000), "r1")


def test_response_round_trip_and_typed_payload():
    from senior_control.protocol import ResponseEnvelope, serialize_envelope

    raw = message(
        type="response",
        payload={"request_type": "drive.velocity", "ok": False, "error": "safety_rejected"},
    )
    result = validate_envelope(raw, "r1")
    assert isinstance(result, ResponseEnvelope)
    assert result.payload.error == "safety_rejected"
    assert validate_envelope(serialize_envelope(result), "r1") == result


def test_response_cannot_have_success_and_error():
    with pytest.raises(ValueError):
        validate_envelope(
            message(
                type="response",
                payload={"request_type": "drive.stop", "ok": True, "error": "timeout"},
            ),
            "r1",
        )


def test_shared_values_reject_invalid_values():
    from senior_control.models import MapRef, OperationResult, Pose2D, Velocity

    for factory in (
        lambda: Velocity(float("nan"), 0),
        lambda: Pose2D(0, float("inf"), 0),
        lambda: MapRef("", -1),
        lambda: OperationResult("id", "invented"),
    ):
        with pytest.raises(ValueError):
            factory()


def test_serialization_rejects_mutated_and_constructed_nonfinite_payloads():
    from senior_control.protocol import VelocityPayload, serialize_envelope

    envelope = validate_envelope(message(), "r1")
    assert envelope.type == "drive.velocity"
    envelope.payload.vx = float("nan")
    with pytest.raises(ValueError):
        serialize_envelope(envelope)
    envelope.payload = VelocityPayload.model_construct(vx=float("inf"), wz=0.0)
    with pytest.raises(ValueError):
        serialize_envelope(envelope)
