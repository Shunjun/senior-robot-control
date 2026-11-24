"""Learning switches do not disable the current raw-scan obstacle limit."""

import math
from types import SimpleNamespace

from senior_motion_control.contracts import MotionTarget
from senior_perception.raw_safety_observation import RawSafetyObservation
from test_motion_core_handoff import core  # noqa: F401

from vehicle_core.values import Velocity


def test_false_switches_still_block_real_obstacle_in_actual_core(core):  # noqa: F811
    core.profile = core.profile.model_copy(
        update={
            "startup_learning_front_occlusion_stop": False,
            "startup_learning_rear_occlusion_stop": False,
        }
    )
    core.collision.profile = core.profile
    assert not core.profile.startup_learning_front_occlusion_stop
    assert not core.profile.startup_learning_rear_occlusion_stop
    scan = SimpleNamespace(
        ranges=[0.3, math.inf],
        angle_min=0.0,
        angle_increment=math.pi,
        range_min=0.05,
        range_max=10.0,
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_yaw=0.0,
    )
    core.node._scans.append(RawSafetyObservation(scan, 10.0, 10.0, 10.0, 10.0))
    core.target = MotionTarget(Velocity(0.2, 0), "manual", "authority", 10.2)
    core.tick()
    assert core.node.emitted[-1][0] == Velocity(0, 0)
    assert core.target.source == "manual"
    assert core.machine.state.fault_revision == 0
