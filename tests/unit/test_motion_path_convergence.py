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
    first = {
        "stamp_ns": 9_900_000_000,
        "received_ros_ns": 9_920_000_000,
        "received_steady_ns": 19_920_000_000,
        "sample_steady_ns": 19_900_000_000,
        "clock_generation": 1,
    }
    last = {
        "stamp_ns": 10_000_000_000,
        "received_ros_ns": 10_020_000_000,
        "received_steady_ns": 20_020_000_000,
        "sample_steady_ns": 20_000_000_000,
        "clock_generation": 1,
    }
    value = {
        "boot_id": "fusion",
        "sequence": 3,
        "reference": {
            "first_time": first,
            "last_time": last,
            "average": {"vx": 0.2, "wz": 0.1},
            "mode": "normal",
            "current_reference": {"reason": "available"},
        },
    }
    message = {"data": json.dumps(value)}
    sample = ros_sample(
        "motion",
        message,
        20_100_000_000,
        received_ros_ns=10_100_000_000,
        clock_generation=1,
    )
    assert TOPICS["motion"] == ("/motion/core_reference", "std_msgs/msg/String")
    assert sample["time"] == sample["last_time"] == last
    assert sample["first_time"] == first
    assert sample["received_steady_ns"] == 20_100_000_000
    assert sample["data"]["vx"] == 0.2
    assert sample["data"]["source_epoch"] == "fusion"
    assert sample["raw"] == message
    assert (
        ros_sample(
            "motion",
            {"data": "bad"},
            20_100_000_000,
            received_ros_ns=10_100_000_000,
            clock_generation=1,
        )["data"]["valid"]
        is False
    )
