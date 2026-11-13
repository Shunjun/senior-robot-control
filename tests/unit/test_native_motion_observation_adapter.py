"""Native adapter tests: no ROS process or motion stimulus."""

from types import SimpleNamespace as N

import pytest
from senior_robot.motion_observation_adapter import NativeObservations, strict_reference_json
from senior_robot.simulation_motion_profile import PROFILE_ID, select_simulation_profile

from senior_control.motion.observations import Source


def adapter():
    p = select_simulation_profile(
        PROFILE_ID, simulated_clock=True, localhost_only="1", domain_id="173"
    )
    return NativeObservations(p.admission(), p.temporal(), p.fusion())


def test_real_context_cannot_select_candidate_and_unknown_covariance_not_invented():
    with pytest.raises(ValueError):
        select_simulation_profile(
            PROFILE_ID, simulated_clock=False, localhost_only="1", domain_id="173"
        )
    a = adapter()
    wheel = N(
        child_frame_id="base_link",
        header=N(stamp=N(sec=1, nanosec=0), frame_id="odom"),
        twist=N(twist=N(linear=N(x=0.2, y=0), angular=N(z=0))),
    )
    assert a.wheel(wheel, b"wheel", 1.0, 10.0)
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
    assert a.wheel(wheel, b"wheel", 1.0, 10.0)
    assert not a.wheel(wheel, b"wheel", 1.1, 10.1)
    assert a.snapshot(2.0, 11.0)["sources"] == []


def test_humble_missing_gid_accepts_only_expected_unique_simulation_publisher(monkeypatch):
    import sys
    from types import ModuleType

    from senior_robot.motion_observation_adapter import NativeObservationRosBridge

    lifecycle = ModuleType("senior_robot.motion_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, "senior_robot.motion_lifecycle", lifecycle)
    tf = ModuleType("tf2_ros")
    tf.TransformException = type("TransformException", (Exception,), {})
    monkeypatch.setitem(sys.modules, "tf2_ros", tf)
    endpoints = [N(node_name="drive", node_namespace="/", endpoint_gid=b"wheel")]
    bridge = object.__new__(NativeObservationRosBridge)
    bridge.allow_topology_only = True
    bridge.node = N(
        get_publishers_info_by_topic=lambda _: endpoints,
        get_clock=lambda: N(now=lambda: N(nanoseconds=1_000_000_000)),
    )
    bridge._warn_at = 0
    accepted = []
    callback = bridge._callback(lambda *args: accepted.append(args), "/wheel/odom")
    callback(N(), {"source_timestamp": 123})
    assert len(accepted) == 1
    identity = accepted[0][1]
    assert identity.sample_gid is None and identity.topology_gid == b"wheel"
    a = adapter()
    epoch, _ = a.identity(Source.WHEEL, identity, "", 1.0)
    assert "simulation_topology_only" in epoch and '"sample_gid": null' in epoch
    callback(N(), {"publisher_gid": b"other"})
    assert len(accepted) == 1
    endpoints.append(N(node_name="drive", node_namespace="/", endpoint_gid=b"other"))
    callback(N(), {})
    assert len(accepted) == 1
    endpoints[:] = [N(node_name="unexpected", node_namespace="/", endpoint_gid=b"wheel")]
    callback(N(), {})
    assert len(accepted) == 1
    endpoints[0].node_name = "drive"
    bridge.allow_topology_only = False
    callback(N(), {})
    assert len(accepted) == 1
    callback(N(), {"publisher_gid": b"wheel"})
    assert len(accepted) == 2
