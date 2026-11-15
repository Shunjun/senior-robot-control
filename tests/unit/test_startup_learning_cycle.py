"""Synthetic raw measurements and fake transport through the real unified/core chain."""

import math
from types import SimpleNamespace as N

import pytest
from control_process_test_support import publish_inputs
from test_motion_core_chain import InlineWorker
from test_motion_core_handoff import core  # noqa: F401
from test_unified_motion_runtime import point

from senior_control.config import DeviceProfile
from senior_control.motion.constraint_worker import ConstraintScheduler
from senior_control.motion.estimator import MotionObservation, Origin
from senior_control.motion.observations import Source
from senior_control.motion.startup_learning import StartupLearning


@pytest.mark.parametrize(
    "delay_pattern,late_device,motion_reordering,expected",
    [
        ((0.0,), False, False, "completed"),
        ((0.0,), True, False, "ambiguous"),
        ((0.075, 0.025, 0.125, 0.025), False, False, "completed"),
        ((0.075, 0.025, 0.125, 0.025), True, False, "ambiguous"),
        ((0.075, 0.025, 0.125, 0.025), False, True, "interrupted"),
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
    core.scheduler = ConstraintScheduler(core.engine, InlineWorker(), worker_budget_s=0.25)
    core.startup = StartupLearning(tmp_path / "claim")
    core.node._depth_policy = "laser_only"
    core.node._scan_topology_ready = lambda: True
    core.node._profile = lambda: DeviceProfile(
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)]
    )
    now = [10.0]
    monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: now[0])
    losses = []
    original_permission = core._startup_permission_lost

    def permission(value, t=None):
        result = original_permission(value, t)
        if result:
            losses.append((t, None if value.brake is None else value.brake.reason))
        return result

    core._startup_permission_lost = permission
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
        core.node._reception_test.sensors.ingest(point(Source.WHEEL, t, t, (v, 0.0), i), t, t, t)
        core.node._reception_test.sensors.ingest(point(Source.IMU, t, t, (0.0,), i), t, t, t)
        core.node._reception_test.sensors.ingest_observation(
            MotionObservation(
                "laser",
                (v, 0.0, 0.0),
                t,
                (Origin("laser", "native", i, t, t, t),),
                quality=1.0,
                independent=(True, True, True),
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
                    sequence=i,
                    ranges=ranges,
                    angle_min=-math.pi,
                    angle_increment=2 * math.pi / 719,
                    range_min=0.08,
                    range_max=20.0,
                    sensor_x=0.0,
                    sensor_y=0.0,
                    sensor_yaw=0.0,
                    transform_stamp=stamp,
                ),
            )
        ]
        publish_inputs(core)
        core.prefetch_scan()
        core.tick()
        phases.add(core.startup.state)
        captures.append(stamp)
        outputs.append(core.node.emitted[-1][0].vx)
        if core.startup.state == "validating" and stopped_stamp is None:
            stopped_stamp = core.startup.validation_after_ros
        if not core.startup.active:
            break
    assert core.startup.state == expected, (core.startup.summary(), losses, phases)
    assert core.startup.started > 50.0
    if expected == "interrupted":
        assert losses and losses[-1][1] == "sweep_unknown"
        assert outputs[-1] == 0 and core.startup.distance < 0.148
        return  # Preserved conservative negative; no profile/error/free-space tuning.
    assert {"sampling", "moving", "validating"} <= phases
    assert max(outputs) > outputs[-2]  # actual controller deceleration to zero
    assert any(v > 0 for v in outputs) and outputs[-1] == 0
    assert 0.148 <= core.startup.distance < 0.17
    assert core.startup.validation_samples == 5
    assert stopped_stamp is not None
    assert core.self_returns.frame_stamps == sorted(set(core.self_returns.frame_stamps))
    # Completion can occur while consuming the exit geometry snapshot. The
    # next fixed control slot retires its lifecycle identity and stays zero.
    now[0] += 0.05
    publish_inputs(core)
    core.tick()
    assert core.machine.state.task_id is None
    assert core.target.source == "idle" and core.node.emitted[-1][0].vx == 0
    if len(delay_pattern) > 1:
        assert any(a > b for a, b in zip(captures, captures[1:], strict=False))

    for (pose, _), stamp in zip(
        core.self_returns.frames, core.self_returns.frame_stamps, strict=True
    ):
        lo = max(p for p in positions if p[0] <= stamp)
        hi = min(p for p in positions if p[0] >= stamp)
        expected_x = (
            lo[1] if lo[0] == hi[0] else lo[1] + (hi[1] - lo[1]) * (stamp - lo[0]) / (hi[0] - lo[0])
        )
        assert pose.x == pytest.approx(expected_x, abs=1e-8)
