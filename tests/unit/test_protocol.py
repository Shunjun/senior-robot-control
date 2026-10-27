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
