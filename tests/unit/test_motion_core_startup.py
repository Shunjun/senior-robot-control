"""Actual raw-scan output does not require independent-motion qualification."""

import math
from types import SimpleNamespace

from senior_motion_control.contracts import MotionEventKind, MotionTarget
from senior_perception.raw_safety_observation import RawSafetyObservation
from test_motion_core_handoff import core  # noqa: F401

from vehicle_core.values import Velocity


def test_manual_raw_scan_start_missing_scan_stop_and_recovery(core):  # noqa: F811
    assert core.sensors.independent is None
    scan = SimpleNamespace(
        ranges=[math.inf, math.inf],
        angle_min=0.0,
        angle_increment=math.pi,
        range_min=0.05,
        range_max=10.0,
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_yaw=0.0,
    )
    raw = RawSafetyObservation(scan, 10.0, 10.0, 10.0, 10.0)
    core.node._scans.append(raw)
    core._event(MotionEventKind.MANUAL_START, 10.0, "authority")
    core.target = MotionTarget(Velocity(0.2, 0), "manual", "authority:1", 10.2)
    core._last_tick = 9.95
    core.tick()
    assert core.node.emitted[-1][0].vx > 0
    core.node._scans.clear()
    core.tick()
    assert core.node.emitted[-1][0] == Velocity(0, 0)
    assert core.target.source == "manual" and core.machine.state.fault_revision == 0
    core.node._scans.append(raw)
    core._last_tick = 9.95
    core.tick()
    assert core.node.emitted[-1][0].vx > 0
    assert core.sensors.independent is None
