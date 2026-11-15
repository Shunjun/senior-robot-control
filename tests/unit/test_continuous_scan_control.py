"""Synthetic continuous scans through real core perception, constraints and controller."""

import math
from concurrent.futures import Future
from types import SimpleNamespace as NS

import pytest
from senior_robot.motion_core_output import MotionCoreOutput
from senior_robot.motion_core_perception import ScanGeometry
from senior_robot.raw_safety_observation import RawSafetyObservation
from senior_robot.simulation_motion_profile import SimulationMotionProfile

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D, Velocity
from senior_control.motion.constraints import (
    ConstraintConfig,
    ConstraintEngine,
    ConstraintSnapshot,
    Footprint,
    RiskEnvelope,
)
from senior_control.motion.contracts import BodyMotion, MotionTarget, ResponseBounds
from senior_control.motion.controller import Feedback, MotionController
from senior_control.motion.source_modes import SourceMode
from senior_control.safety.space_estimator import SpaceEstimation


class Delayed(Future):
    def __init__(self, task, finish, clock):
        super().__init__()
        self.task, self.finish, self.clock = task, finish, clock

    def done(self):
        if not super().done() and self.clock[0] + 1e-10 >= self.finish:
            self.set_result(self.task())
        return super().done()


class Worker:
    def __init__(self, clock, delay):
        self.clock, self.delay, self.submitted = clock, delay, 0

    def submit(self, task):
        self.submitted += 1
        return Delayed(task, self.clock[0] + self.delay, self.clock)

    def close(self):
        pass


class Pose:
    frame, frame_generation = "odom", 0

    def at(self, *args, **kwargs):
        return NS(pose=Pose2D(0, 0, 0), error_m=0.0, prediction_error=NS(software_position_m=0.0))

    def relative_error(self, *args):
        return NS(position_m=0.0, yaw_rad=0.0, software_position_m=0.0, software_yaw_rad=0.0)


class Diagnostics:
    def sensor_attempt(self, *args, **kwargs):
        pass


def make(clock, delay):
    profile = SimulationMotionProfile()
    core = MotionCoreOutput.__new__(MotionCoreOutput)
    core.profile = profile
    core.node = NS(
        _scans=[],
        _depth_policy="laser_only",
        _scan_topology_ready=lambda: True,
        _profile=lambda: DeviceProfile(
            footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)]
        ),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=round(clock[0] * 1e9))),
        get_parameter=lambda name: NS(value="base_link"),
    )
    core.sensors = NS(latest=NS(generation=0), pose_estimator=Pose())
    core.startup = NS(state="disabled", provisional=False)
    core.self_returns = None
    core._learning_geometry = None
    core._filter_signature = None
    core._perception = None
    core._perception_key = None
    core._perception_pose = None
    core.space_estimator = SpaceEstimation(
        frame="odom", frame_epoch="0:0", software_pose_error_m=0.03
    )
    core.diagnostics = Diagnostics()
    core.scan_geometry = ScanGeometry(Worker(clock, delay))
    return core, profile


def scan(t, seq, obstacle=False):
    ranges = [4.0 + 0.002 * math.sin(seq + i) for i in range(720)]
    if obstacle:
        ranges[360] = 0.38
    m = NS(
        source_epoch="scan",
        sequence=seq,
        ranges=ranges,
        angle_min=-math.pi,
        angle_increment=2 * math.pi / 720,
        range_min=0.06,
        range_max=8.0,
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_yaw=0.0,
        transform_stamp=0.0,
        frame_id="laser",
    )
    return RawSafetyObservation(m, t, t, t, t)


def context(profile, perception, pose, t):
    response = ResponseBounds(0.3, 0.3, 1.0, 0.5, 0.5, 2.0, 0.2, "p")
    risk = RiskEnvelope(
        "p",
        "synthetic probe",
        "motion",
        (SourceMode.WHEEL_IMU_LIMITED,),
        "p",
        12.0,
        (-0.05, 0.05),
        (-0.02, 0.02),
        Velocity(0.3, 0.5),
        0.0,
        0.0,
        0.0,
        0.3,
        2.0,
        True,
        brake_drive_overshoot=Velocity(0.1, 0.1),
        brake_tracking_basis="synthetic probe",
    )
    return ConstraintSnapshot(
        MotionTarget(Velocity(0.1, 0), "manual", "manual", t + 0.3),
        BodyMotion(None, None, t, t, t, t + 1.0, "motion", True, "synthetic"),
        SourceMode.WHEEL_IMU_LIMITED,
        response,
        Footprint(0.674, 0.552),
        perception,
        pose,
        "scan_reference",
        "p",
        "p",
        risk,
    )


def run_scan_trace(hz, delay, *, real_pose=False):
    base = 10.0
    clock = [base]
    core, profile = make(clock, delay)
    reception = None
    if real_pose:
        import multiprocessing as mp
        from types import SimpleNamespace

        from senior_robot.control_inputs import ControlInputs
        from senior_robot.motion_observation_adapter import NativeObservations
        from senior_robot.motion_sensor_runtime import MotionSensorRuntime

        from senior_control.motion.process_resources import LatestSnapshot
        from senior_control.runtime.diagnostics import RuntimeDiagnostics

        source = MotionSensorRuntime(profile, RuntimeDiagnostics(lambda *_: None))
        core.sensors = MotionSensorRuntime(profile, RuntimeDiagnostics(lambda *_: None))
        channel = ControlInputs.__new__(ControlInputs)
        channel.slot, channel.version = LatestSnapshot(mp.get_context("spawn")), 0
        reception = SimpleNamespace(
            source=source,
            channel=channel,
            observations=NativeObservations(
                profile.admission(), profile.temporal(), profile.fusion(), sample_sink=source.ingest
            ),
        )
    engine = ConstraintEngine(ConstraintConfig((1.0, 0.75, 0.5, 0.25), 0.15, 0.05, 256, 0.005))
    controller = MotionController(engine, profile.controller(), Velocity(0, 0), Velocity(0, 0))
    permits = outputs = 0
    first = None
    deadlines = []
    alignment_errors = []
    sequence = -1
    for tick in range(41):
        t = base + round(tick * 0.05, 6)
        clock[0] = t
        newseq = 1 + math.floor((t - base) * hz + 1e-8)
        if newseq != sequence:
            sequence = newseq
            core.node._scans[:] = [scan(t, sequence)]
        if reception is not None:
            from test_motion_core_chain import feed

            feed(reception.observations, t)
            reception.source.poses = list(reception.observations.poses)
            reception.source.ingest_poses(reception.source.poses)
            reference = reception.source.estimate(t, t)
            state = {
                k: v
                for k, v in vars(reception.source).items()
                if k not in {"profile", "diagnostics"}
            }
            assert reception.channel.slot.put(
                (state, tuple(core.node._scans), True, sequence, reference)
            )
            reception.channel.update(core.node, core.sensors)
        try:
            perception, pose = core._observation(t)
        except ValueError as exc:
            assert str(exc) == "scan_geometry_pending", exc
            continue
        alignment_errors.append(core._space_estimate.software_alignment_error_m)
        ctx = context(profile, perception, pose, t)
        permission = engine.refine(ctx, t)
        if permission.permit:
            permits += 1
            first = first or t
        wheel = Feedback(
            Velocity(0.0, 0.0), t, t + 1.0, "wheel", tick, True, "synthetic", unified=True
        )
        imu = Feedback(
            Velocity(0.0, 0.0),
            t,
            t + 1.0,
            "imu",
            tick,
            True,
            "synthetic",
            observed_axes=(False, True),
        )
        decision = controller.tick(
            ctx.target.velocity, ctx, permission, wheel, wheel, t, imu=imu, still_wheel=wheel
        )
        outputs += bool(decision.verified and decision.command and decision.command.vx > 0)
        deadlines.append((t, perception.stamp_steady, perception.deadline_steady))
    assert first <= base + 0.25
    assert outputs >= 20
    assert core.scan_geometry.worker.submitted <= 22
    assert permits and outputs
    if real_pose:
        assert max(alignment_errors) >= 0.06 - 1e-9  # Original cross-anchor error is retained.
    assert all(deadline <= stamp + 1.000001 for _, stamp, deadline in deadlines)
    # Latest obstacle must be reflected while a newer slow job is still running.
    clock[0] = base + 2.01
    core.node._scans[:] = [scan(clock[0], sequence + 1, True)]
    perception, pose = core._observation(clock[0])
    from shapely.geometry import Point

    assert perception.occupied.intersects(Point(0.38, 0))
    assert not perception.free_space.covers(Point(1.0, 0))


@pytest.mark.parametrize("hz", [10, 20])
@pytest.mark.parametrize("delay", [0.08, 0.12])
def test_continuous_scan_and_slow_geometry_eventually_drive_then_tighten_obstacle(hz, delay):
    run_scan_trace(hz, delay)


@pytest.mark.parametrize("hz", [10, 20])
@pytest.mark.parametrize("delay", [0.08, 0.12])
def test_slow_scan_with_real_wheel_pose_anchors_and_mailbox_eventually_drives(hz, delay):
    run_scan_trace(hz, delay, real_pose=True)
