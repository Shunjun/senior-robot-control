"""Retired remote wire is rejected; diagnostic capture consumes the current producer."""

import json

import pytest

from senior_control.protocol import validate_frame
from tools.vehicle_motion_capture import TOPICS, ros_sample


@pytest.mark.parametrize(
    "kind",
    [
        "drive.challenge",
        "drive.clock",
        "drive.velocity",
        "safety.reset",
        "navigation.start",
        "navigation.resume",
    ],
)
def test_retired_control_protocol_is_not_admitted(kind):
    raw = json.dumps(
        {
            "v": 1,
            "type": kind,
            "robot_id": "robot",
            "session_id": "page",
            "connection_epoch": 1,
            "request_id": "old",
            "seq": 1,
            "expires_at": 100.0,
            "payload": {},
        }
    ).encode()
    with pytest.raises(ValueError):
        validate_frame(raw, "robot")


def test_capture_preserves_current_reference_interval_and_original_wire():
    value = {
        "sampled_ros": 10.0,
        "boot_id": "fusion",
        "sequence": 3,
        "reference": {
            "start_ros": 9.9,
            "average": {"vx": 0.2, "wz": 0.1},
            "mode": "normal",
            "current_reference": {"reason": "available"},
        },
    }
    message = {"data": json.dumps(value)}
    sample = ros_sample("motion", message, 20.0)
    assert TOPICS["motion"] == ("/motion/core_reference", "std_msgs/msg/String")
    assert sample["stamp_ros"] == 10.0
    assert sample["data"]["interval_start_ros"] == 9.9
    assert sample["data"]["vx"] == 0.2
    assert sample["data"]["source_epoch"] == "fusion"
    assert sample["raw"] == message
    assert ros_sample("motion", {"data": "bad"}, 20.0)["data"]["valid"] is False
