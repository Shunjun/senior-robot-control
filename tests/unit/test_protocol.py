import json

import pytest
from senior_contracts.wire import validate_envelope


def message(**changes):
    data = {
        "v": 1,
        "type": "drive.intent",
        "robot_id": "r1",
        "session_id": "s",
        "connection_epoch": 1,
        "request_id": "q",
        "seq": 1,
        "expires_at": 123.0,
        "payload": {
            "authority_id": "authority",
            "gesture": 1,
            "sequence": 1,
            "linear": 0.2,
            "turn": 0.1,
            "gear": "low",
        },
    }
    data.update(changes)
    return json.dumps(data).encode()


def test_valid_velocity():
    from senior_contracts.wire import validate_envelope as shared_validate

    assert shared_validate is validate_envelope
    assert validate_envelope(message(), "r1").payload.linear == 0.2


def test_receipt_clock_is_private_and_does_not_extend_the_wire_envelope():
    from senior_contracts.wire import EnvelopeBase, serialize_envelope

    envelope = validate_envelope(message(), "r1")
    envelope._received_steady = 42.0
    assert list(EnvelopeBase.model_fields) == [
        "v",
        "robot_id",
        "session_id",
        "connection_epoch",
        "request_id",
        "seq",
        "expires_at",
    ]
    assert "_received_steady" not in json.loads(serialize_envelope(envelope))
    with pytest.raises(ValueError):
        validate_envelope(message(_received_steady=42.0), "r1")


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
    from senior_contracts.wire import ResponseEnvelope, serialize_envelope

    raw = message(
        type="response",
        payload={"request_type": "control.acquire", "ok": False, "error": "safety_rejected"},
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
    from vehicle_core.values import MapRef, OperationResult, Pose2D, Velocity

    for factory in (
        lambda: Velocity(float("nan"), 0),
        lambda: Pose2D(0, float("inf"), 0),
        lambda: MapRef("", -1),
        lambda: OperationResult("id", "invented"),
    ):
        with pytest.raises(ValueError):
            factory()


def video_message(kind="video.offer", payload=None, **changes):
    return message(
        type=kind,
        payload={"sdp": "v=0\r\n"} if payload is None else payload,
        **changes,
    )


def test_video_frames_validate_and_round_trip():
    from senior_contracts.wire import serialize_envelope, validate_frame

    offer = validate_frame(video_message(), "r1")
    assert offer.type == "video.offer" and offer.payload.sdp == "v=0\r\n"
    assert validate_frame(serialize_envelope(offer), "r1") == offer
    candidate = validate_frame(
        video_message("video.candidate", {"candidate": "candidate:1 1 udp 1 10.0.0.1 9 typ host"}),
        "r1",
    )
    assert candidate.payload.candidate.startswith("candidate:")
    assert validate_frame(message(), "r1").payload.linear == 0.2  # commands still validate


@pytest.mark.parametrize(
    "raw",
    [
        video_message(payload={"sdp": ""}),
        video_message(payload={"sdp": "x" * 17000}),
        video_message(payload={"sdp": "v=0", "extra": 1}),
        video_message("video.candidate", {"candidate": "not-a-candidate"}),
        video_message("video.candidate", {"candidate": "candidate:" + "x" * 600}),
        video_message(robot_id="other"),
    ],
)
def test_reject_invalid_video_frames(raw):
    from senior_contracts.wire import validate_frame

    with pytest.raises(ValueError):
        validate_frame(raw, "r1")


def test_commands_do_not_accept_video_types():
    with pytest.raises(ValueError):
        validate_envelope(video_message(), "r1")


def test_video_close_has_no_payload_and_round_trips():
    from senior_contracts.wire import serialize_envelope, validate_frame

    closing = validate_frame(video_message("video.close", {}), "r1")
    assert closing.type == "video.close"
    assert validate_frame(serialize_envelope(closing), "r1") == closing
    with pytest.raises(ValueError):
        validate_frame(video_message("video.close", {"sdp": "v=0"}), "r1")
    with pytest.raises(ValueError):
        validate_envelope(video_message("video.close", {}), "r1")


def test_retired_intent_receipt_is_rejected():
    with pytest.raises(ValueError):
        validate_envelope(
            message(type="response", payload={"request_type": "drive.intent", "ok": True}), "r1"
        )


def test_learning_contract_keeps_start_identity_separate_from_status_query():
    from senior_contracts.wire import LearningStartEnvelope, LearningStatusEnvelope, ResponsePayload

    base = dict(
        v=1,
        robot_id="robot",
        session_id="operator",
        connection_epoch=1,
        request_id="start-1",
        seq=1,
        expires_at=10.0,
    )
    start = LearningStartEnvelope(
        **base,
        type="learning.start",
        payload={"expected_active_revision": 0, "expected_authority_id": None},
    )
    query = LearningStatusEnvelope(
        **{**base, "request_id": "query-1", "seq": 2},
        type="learning.status",
        payload={"start_request_id": start.request_id},
    )
    assert query.payload.start_request_id == "start-1" and query.request_id != "start-1"
    receipt = ResponsePayload(
        request_type="learning.start",
        ok=True,
        data={"start_request_id": "start-1", "attempt_id": "a" * 32, "state": "busy"},
    )
    assert receipt.data["state"] == "busy"
    with pytest.raises(ValueError):
        LearningStatusEnvelope(
            **base,
            type="learning.status",
            payload={"start_request_id": "start-1", "attempt_id": "a" * 32},
        )
