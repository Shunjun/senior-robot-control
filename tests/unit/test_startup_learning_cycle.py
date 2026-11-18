"""Synthetic raw measurements and fake transport through the real unified/core chain."""

import math
from types import SimpleNamespace as N

import pytest
from control_process_test_support import publish_inputs
from learning_test_support import (  # noqa: F401
    begin_action,
    bind_core,
    learning_point,
    learning_runs,
    startup_for,
    write_geometry_record,
)
from sensor_time_test_support import capture_fields, sample_time
from test_motion_core_chain import InlineWorker
from test_motion_core_handoff import core  # noqa: F401

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
from senior_control.motion.constraint_worker import ConstraintScheduler
from senior_control.motion.estimator import MotionObservation, Origin
from senior_control.motion.observations import Source
from senior_control.sensors.sample_time import StampInterval
from senior_control.vision.raw_capture import RawCaptureHeader


@pytest.mark.parametrize(
    "delay_pattern,late_device,motion_reordering,expected",
    [
        ((0.0,), False, False, "completed"),
        ((0.0,), True, False, "completed"),
        ((0.075, 0.025, 0.125, 0.025), False, False, "completed"),
        ((0.075, 0.025, 0.125, 0.025), True, False, "completed"),
        ((0.075, 0.025, 0.125, 0.025), False, True, "completed"),
    ],
)
def test_full_core_cycle_with_delayed_and_reordered_raw_scans(
    core,  # noqa: F811
    tmp_path,
    monkeypatch,
    delay_pattern,
    late_device,
    motion_reordering,
    expected,
):
    core.scheduler = ConstraintScheduler(core.engine, InlineWorker(), worker_budget_ms=250)
    bind_core(core, tmp_path / "claim")
    core.node._depth_policy = "laser_only"
    core.node._scan_topology_ready = lambda: True
    core.node._profile = lambda: DeviceProfile(
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)]
    )
    now = [10.0]
    monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: now[0])
    losses = []  # Final output reasons are retained without a retired admission hook.
    phases = set()
    captures = []
    stopped_stamp = None
    outputs = []
    positions = [(50.0, 0.0)]
    # Real core initialization may wait longer than the learning budget.
    for t in (10.0, 50.0):
        now[0] = t
        publish_inputs(core)
        core.prefetch_scan()
        core.tick()
    assert core.startup.state == "initializing" and core.startup.started is None
    for i in range(1, 301):
        t = now[0] = 50 + i * 0.05
        # Explicit synthetic zero-order plant: the preceding actual core output
        # supplies the next measured speed. No permit or obstacle is patched.
        v = core.node.emitted[-1][0].vx if core.node.emitted else 0.0
        positions.append((t, positions[-1][1] + v * 0.05))
        core.node._reception_test.sensors.ingest(
            learning_point(
                Source.WHEEL, round(t * 1e9), round(t * 1e9), round(t * 1e9), (v, 0.0), i
            ),
            t,
            t,
            t,
        )
        core.node._reception_test.sensors.ingest(
            learning_point(Source.IMU, round(t * 1e9), round(t * 1e9), round(t * 1e9), (0.0,), i),
            t,
            t,
            t,
        )
        core.node._reception_test.sensors.ingest_observation(
            MotionObservation(
                "laser",
                (v, 0.0, 0.0),
                t,
                (
                    Origin(
                        "laser",
                        "native",
                        i,
                        sample_time(
                            stamp_ns=round((t) * 1e9),
                            received_ros_ns=round((t) * 1e9),
                            received_steady_ns=round((t) * 1e9),
                        ),
                    ),
                ),
                quality=1.0,
                independent=(True, True, True),
                measured_steady=t,
                measurement_interval=StampInterval(
                    RawCaptureHeader(
                        time=sample_time(
                            stamp_ns=round((t - 0.05) * 1e9),
                            received_ros_ns=round((t - 0.05) * 1e9),
                            received_steady_ns=round((t - 0.05) * 1e9),
                            clock_generation=1,
                        ),
                        frame_id="laser",
                        raw_gid="01",
                        bridge_gid="02",
                        source_epoch="producer",
                        bridge_boot_id="boot",
                        capture_generation=1,
                        sequence=1,
                        valid=True,
                        reason="",
                    ).time.stamp_ns,
                    RawCaptureHeader(
                        time=sample_time(
                            stamp_ns=round((t) * 1e9),
                            received_ros_ns=round((t) * 1e9),
                            received_steady_ns=round((t) * 1e9),
                            clock_generation=1,
                        ),
                        frame_id="laser",
                        raw_gid="01",
                        bridge_gid="02",
                        source_epoch="producer",
                        bridge_boot_id="boot",
                        capture_generation=1,
                        sequence=1,
                        valid=True,
                        reason="",
                    ).time.stamp_ns,
                ),
                measurement_delta=Pose2D(v * 0.05, 0, 0),
                stamp_ns=RawCaptureHeader(
                    time=sample_time(
                        stamp_ns=round((t) * 1e9),
                        received_ros_ns=round((t) * 1e9),
                        received_steady_ns=round((t) * 1e9),
                        clock_generation=1,
                    ),
                    frame_id="laser",
                    raw_gid="01",
                    bridge_gid="02",
                    source_epoch="producer",
                    bridge_boot_id="boot",
                    capture_generation=1,
                    sequence=1,
                    valid=True,
                    reason="",
                ).stamp_ns,
            ),
            t,
            t,
        )
        delay = delay_pattern[i % len(delay_pattern)] if i > 10 else 0.0
        if (
            core.startup.state in {"moving", "sampling"}
            and len(delay_pattern) > 1
            and not motion_reordering
        ):
            delay = 0.025
        stamp = t - delay
        ranges = [10.0] * 720
        if late_device and core.startup.state == "validating":
            ranges[0] = 0.15  # new equipment appears after the move: unverified
        core.node._scans = [
            N(
                received=t,
                received_ros=t,
                stamp=stamp,
                message=N(
                    source_epoch="scan",
                    frame_id="laser_link",
                    sequence=max(1, round(stamp * 1000)),
                    ranges=ranges,
                    angle_min=-math.pi,
                    angle_increment=2 * math.pi / 719,
                    range_min=0.08,
                    range_max=20.0,
                    sensor_x=0.0,
                    sensor_y=0.0,
                    sensor_yaw=0.0,
                    transform_stamp=stamp,
                    raw_capture=N(
                        frame_id="laser_link",
                        raw_gid="01",
                        bridge_gid="02",
                        source_epoch="scan",
                        bridge_boot_id="boot",
                        capture_generation=1,
                        sequence=max(1, round(stamp * 1000)),
                        valid=True,
                        reason="",
                        **capture_fields(
                            time=sample_time(
                                stamp_ns=round((stamp) * 1e9),
                                received_ros_ns=round((stamp) * 1e9),
                                received_steady_ns=round((stamp) * 1e9),
                            )
                        ),
                    ),
                ),
                measurement_steady=stamp,
            )
        ]
        publish_inputs(core)
        core.prefetch_scan()
        core.tick()
        phases.add(core.startup.state)
        captures.append(stamp)
        outputs.append(core.node.emitted[-1][0].vx)
        if core.startup.state == "validating" and stopped_stamp is None:
            stopped_stamp = core.startup.validation_after_steady
        if not core.startup.active:
            break
    assert core.startup.state == expected, (core.startup.summary(), losses, phases)
    assert core.startup.started > 50.0
    assert {"sampling", "moving", "validating"} <= phases
    assert max(outputs) > outputs[-2]  # actual controller deceleration to zero
    assert any(v > 0 for v in outputs) and outputs[-1] == 0
    assert core.startup.deadline - core.startup.started == 3.0
    assert all(
        v == 0
        for t, v in zip([50 + (i + 1) * 0.05 for i in range(len(outputs))], outputs, strict=True)
        if t >= core.startup.deadline
    )
    assert core.startup.validation_samples >= 5
    assert stopped_stamp is not None
    assert [m / 1e9 for m in core.self_returns.frame_measurements] == sorted(
        set([m / 1e9 for m in core.self_returns.frame_measurements])
    )
    # Completion can occur while consuming the exit geometry snapshot. The
    # next fixed control slot retires its lifecycle identity and stays zero.
    now[0] += 0.05
    publish_inputs(core)
    core.tick()
    assert core.machine.state.task_id is None
    assert core.target.source == "idle" and core.node.emitted[-1][0].vx == 0
    if len(delay_pattern) > 1:
        assert any(a > b for a, b in zip(captures, captures[1:], strict=False))

    assert all(
        core.startup.started <= m / 1e9 < core.startup.deadline
        for m in core.self_returns.frame_measurements
    )
    assert core._learning_client.record.model is not None
    if late_device:
        # Equipment first seen after collection cannot enter the frozen model.
        assert core._learning_client.record.model.points == ()
        assert core._latest_masked.rays[0].kind == "hit"
