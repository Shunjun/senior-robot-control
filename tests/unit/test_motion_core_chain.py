"""Synthetic ROS-shaped observations through actual T3/T4/T5; fake transport only."""

import json
import math
from concurrent.futures import Future
from types import SimpleNamespace as N

from control_process_test_support import capture_inputs, publish_inputs
from senior_robot.motion_observation_adapter import NativeObservations, strict_reference_json
from test_motion_core_handoff import barrier, core  # noqa: F401 -- shared fake ROS fixture

from senior_control.config import DeviceProfile
from senior_control.models import Velocity
from senior_control.motion.constraint_worker import ConstraintScheduler


class InlineWorker:
    def submit(self, task):
        f = Future()
        f.set_result(task())
        return f


def timestamp(t):
    ns = round(t * 1e9)
    return N(sec=ns // 10**9, nanosec=ns % 10**9)


def feed(adapter, t, vx=0.0, wz=0.0):
    header = N(stamp=timestamp(t), frame_id="odom")
    adapter.wheel(
        N(
            header=header,
            child_frame_id="base_link",
            twist=N(twist=N(linear=N(x=vx, y=0), angular=N(z=wz))),
            pose=N(pose=N(position=N(x=0.0, y=0.0), orientation=N(x=0.0, y=0.0, z=0.0, w=1.0))),
        ),
        b"wheel",
        t,
        t,
    )
    adapter.imu(
        N(header=N(stamp=timestamp(t), frame_id="base_link"), angular_velocity=N(z=wz)),
        b"imu",
        t,
        t,
    )
    captures = {}
    if adapter.sample_sink is not None:

        def capture(stamp):
            return N(
                valid=True,
                raw_gid="ab" * 24,
                source_epoch="raw-scan",
                bridge_boot_id="capture-boot",
                capture_generation=1,
                sequence=round(stamp * 1e9),
                stamp=timestamp(stamp),
                received_ros=timestamp(stamp),
                received_steady=stamp,
                frame_id="laser_link",
            )

        captures = {"first_capture": capture(t - 0.05), "second_capture": capture(t)}
    adapter.laser(
        N(
            header=N(stamp=timestamp(t), frame_id="base_link"),
            interval_start=timestamp(t - 0.05),
            source_epoch="matcher",
            valid=True,
            body_delta=N(x=vx * 0.05, y=0.0, theta=wz * 0.05),
            covariance=[0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001],
            response=0.9,
            **captures,
        ),
        b"laser",
        t,
        t,
    )


def test_native_recovery_then_intent_and_local_release_are_actually_consumed(core, monkeypatch):  # noqa: F811
    p = core.profile
    a = NativeObservations(p.admission(), p.temporal(), p.fusion())
    core.scheduler = ConstraintScheduler(core.engine, InlineWorker(), worker_budget_s=0.25)
    node = core.node
    node._depth_policy = "laser_only"
    node._scan_topology_ready = lambda: True
    node._profile = lambda: DeviceProfile(self_mask_calibrated=True)
    node.get_publishers_info_by_topic = lambda _: [
        N(node_name="motion_fusion", node_namespace="/", endpoint_gid=b"fusion")
    ]
    now = [10.0]
    monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: now[0])

    def tick(t):
        now[0] = t
        feed(a, t)
        capture_inputs(core, N(data=strict_reference_json(a.snapshot(t, t))))
        node._scans = [
            N(
                received=t,
                received_ros=t,
                stamp=t,
                message=N(
                    source_epoch="scan",
                    frame_id="laser_link",
                    sequence=round(t * 100),
                    ranges=[10.0] * 720,
                    angle_min=-math.pi,
                    angle_increment=2 * math.pi / 719,
                    range_min=0.08,
                    range_max=20.0,
                    sensor_x=0.15699,
                    sensor_y=0.00201,
                    sensor_yaw=0.0,
                    transform_stamp=t,
                ),
            )
        ]
        publish_inputs(core)
        core.prefetch_scan()
        core.tick()
        return json.loads(node.publishers["/motion/runtime"].messages[-1].data)

    for i in range(70):
        state = tick(10.0 + i * 0.025)
    assert state["healthy"], (state["reason"], state)
    assert state["stationary"], state
    assert "acquire" in state["allowed_actions"], state
    assert core.command(barrier("prepared"))
    msg = barrier("intent")
    msg.sequence = 1
    msg.gesture = 1
    msg.vx = 0.2
    msg.manual_linear = 1.0
    msg.manual_linear_limit = 0.2
    msg.degraded_linear_limit = 0.3
    msg.steady_deadline = 13.0
    msg.received_steady = now[0]
    assert core.command(msg)
    for i in range(10):
        state = tick(11.75 + i * 0.025)
    assert core.node.emitted[-1][0].vx > 0, (state, core.node.emitted[-10:])
    before = core.controller.trajectory.velocity.vx
    assert core.command(barrier("release", generation=3))
    tick(12.0)
    # Residual positive acceleration is released with bounded jerk, so the first
    # braking tick may increase speed; it must not jump to zero or reverse.
    assert 0 < core.node.emitted[-1][0].vx < before + p.accel * 0.025
    for i in range(1, 45):
        tick(12.0 + i * 0.025)
    assert core.node.emitted[-1][0] == Velocity(0, 0)
    assert core.target.source == "idle"
    assert state["body_reference"] is not None
    assert core.sensors.latest.vx.error is None  # No invented physical error bound.


def test_disabled_learning_prefetch_submits_latest_scan_before_control_tick(core, monkeypatch):  # noqa: F811
    from control_process_test_support import LocalWorker
    from senior_robot.motion_core_perception import ScanGeometry
    from test_continuous_scan_control import scan

    assert core.startup.state == "disabled" and core._learning_geometry is None
    core.scan_geometry.worker.close()
    core.scan_geometry = ScanGeometry(LocalWorker())
    core.node._profile = lambda: DeviceProfile(self_mask_calibrated=True)
    core.node._scans = [scan(10, 1)]
    core.prefetch_scan()
    assert core.scan_geometry.flight is not None and core.scan_geometry.flight.done()
    assert core.scan_geometry.flight_key[:2] == ("scan", 1)
    assert core._learning_geometry is None
