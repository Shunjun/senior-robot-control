"""Explicit standard ROS first-receipt records for synthetic test inputs."""

from types import SimpleNamespace
from typing import Any

from senior_control.sensors.sample_time import SampleTime, map_sample


def sample_time(
    *, stamp_ns: int, received_ros_ns: int, received_steady_ns: int, clock_generation: int = 1
) -> SampleTime:
    """Exercise the real mapper; callers must provide all three clock coordinates."""
    return map_sample(
        stamp_ns=stamp_ns,
        received_ros_ns=received_ros_ns,
        received_steady_ns=received_steady_ns,
        clock_generation=clock_generation,
    )


def capture_fields(*, time: SampleTime) -> dict[str, Any]:
    """New RawCaptureHeader time fields, independent of generated ROS classes."""

    def stamp(ns: int) -> SimpleNamespace:
        sec, nanosec = divmod(ns, 1_000_000_000)
        return SimpleNamespace(sec=sec, nanosec=nanosec)

    return {
        "stamp": stamp(time.stamp_ns),
        "received_ros": stamp(time.received_ros_ns),
        "received_steady_ns": time.received_steady_ns,
        "sample_steady_ns": time.sample_steady_ns,
        "clock_generation": time.clock_generation,
    }
