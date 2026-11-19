"""Native adapter tests: no ROS process or motion stimulus."""

from types import SimpleNamespace as N

from control_process_test_support import generic_motion_profile
from senior_motion_estimation.motion_observation_adapter import (
    NativeObservations,
    strict_reference_json,
)
from senior_motion_estimation.observations import Source
from sensor_time_test_support import sample_time


def adapter():
    p = generic_motion_profile()
    return NativeObservations(p.admission(), p.temporal(), p.fusion())


def test_real_context_cannot_select_candidate_and_unknown_covariance_not_invented():
    a = adapter()
    wheel = N(
        child_frame_id="base_link",
        header=N(stamp=N(sec=1, nanosec=0), frame_id="odom"),
        twist=N(twist=N(linear=N(x=0.2, y=0), angular=N(z=0))),
    )
    assert a.wheel(
        wheel,
        b"wheel",
        1.0,
        10.0,
        timing=sample_time(
            stamp_ns=round((1.0) * 1e9),
            received_ros_ns=round((1.0) * 1e9),
            received_steady_ns=round((10.0) * 1e9),
            clock_generation=1,
        ),
        generation=1,
        sequence=25,
    )
    s = a.snapshot(1.0, 10.0)
    assert s["sources"][0]["sample"]["covariance"] is None
    assert "NaN" not in strict_reference_json(s)


def test_source_identity_contains_both_producer_and_dds_and_replays_cannot_refresh():
    a = adapter()
    e1, _ = a.identity(Source.LASER, b"gid", "producer1", 1.0)
    e2, _ = a.identity(Source.LASER, b"gid", "producer2", 1.1)
    assert e1 != e2 and "producer1" in e1 and b"gid".hex() in e1
    wheel = N(
        child_frame_id="base_link",
        header=N(stamp=N(sec=1, nanosec=0), frame_id="odom"),
        twist=N(twist=N(linear=N(x=0.2, y=0), angular=N(z=0))),
    )
    assert a.wheel(
        wheel,
        b"wheel",
        1.0,
        10.0,
        timing=sample_time(
            stamp_ns=round((1.0) * 1e9),
            received_ros_ns=round((1.0) * 1e9),
            received_steady_ns=round((10.0) * 1e9),
            clock_generation=1,
        ),
        generation=1,
        sequence=41,
    )
    assert not a.wheel(
        wheel,
        b"wheel",
        1.1,
        10.1,
        timing=sample_time(
            stamp_ns=round((1.0) * 1e9),
            received_ros_ns=round((1.0) * 1e9),
            received_steady_ns=round((10.0) * 1e9),
            clock_generation=1,
        ),
        generation=1,
        sequence=42,
    )
    assert a.snapshot(2.0, 11.0)["sources"] == []


def test_missing_gid_never_uses_topology_as_sample_identity():
    from senior_motion_estimation.motion_observation_adapter import observation_origin

    endpoints = [N(node_name="drive", node_namespace="/", endpoint_gid=b"wheel")]
    assert observation_origin({}, endpoints, "drive") is None
    assert observation_origin({"publisher_gid": b"other"}, endpoints, "drive") is None
    identity = observation_origin({"publisher_gid": b"wheel"}, endpoints, "drive")
    assert identity.sample_gid == identity.topology_gid == b"wheel"
    endpoints.append(N(node_name="drive", node_namespace="/", endpoint_gid=b"other"))
    assert observation_origin({"publisher_gid": b"wheel"}, endpoints, "drive") is None
