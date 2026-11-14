"""Captured startup occupancy witness; real T4/T5, ROS I/O doubles only.

Break caught: unknown startup motion must deny execution without permanently
latching an execution fault, and that exception must never reopen after entry
into normal operation. Raw scan was not captured; recovery free space below
is explicitly synthetic, not an additional physical acceptance claim.
"""

import copy
import json
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest
from motion_reference_test_support import ReadySubscription
from senior_robot import motion_core_output as module
from senior_robot.simulation_motion_profile import SimulationMotionProfile
from test_navigation_command_clock import navigation  # noqa: F401

from senior_control.agent.manual_control import MotionRuntimeReceiver
from senior_control.models import Pose2D, Velocity
from senior_control.motion.constraints import (
    FreePatch,
    NoReturnPolicy,
    PatchKind,
    prepare_perception,
)
from senior_control.motion.contracts import BrakeKind, MotionEventKind
from senior_control.motion.state_machine import RecoveryObservation
from senior_control.motion.trajectory import Trajectory
from senior_control.protocol import MotionRuntimeState


class Node:
    def __init__(self, clock):
        self.clock = clock
        self.messages = []
        self.output = []

    def create_publisher(self, *_args):
        return N(publish=self.messages.append)

    def create_subscription(self, message_type, _topic, callback, _qos):
        return ReadySubscription(message_type, callback)

    def get_clock(self):
        return N(now=lambda: N(nanoseconds=round(self.clock.ros * 1e9)))

    def get_parameter(self, _name):
        return N(value="route-sim")

    def _emit_motion_core(self, velocity, reason):
        self.output.append((velocity, reason))


class Replay:
    def __init__(self, capture, clock):
        self.capture, self.clock = capture, clock
        self.node = Node(clock)
        self.core = module.MotionCoreOutput(self.node, SimulationMotionProfile())
        self.core.controller._last_now = clock.steady - 0.05
        self.free = False
        self.decisions = []
        actual_tick = self.core.controller.propose_tick

        def tick(*args, **kwargs):
            result = actual_tick(*args, **kwargs)
            self.decisions.append(result.decision)
            return result

        self.core.controller.propose_tick = tick
        self.core._observation = self.observation

    def observation(self, now):
        patches = (
            (
                FreePatch(
                    ((-0.8, -0.8), (0.8, -0.8), (0.8, 0.8), (-0.8, 0.8)), PatchKind.OBSERVED_FREE
                ),
            )
            if self.free
            else ()
        )
        perception = prepare_perception(
            patches,
            tuple(tuple(p) for p in self.capture["hit_witnesses"]),
            error_m=0.02,
            stamp_steady=now - 0.01,
            deadline_steady=now + 0.79,
            source_epoch="captured-wall-subset",
            frame_id="scan_reference",
            revision=1,
            forecast_horizon_s=12,
            no_return_policy=NoReturnPolicy.UNKNOWN,
            basis="captured static occupancy subset; recovery free patch synthetic",
            max_patches=2048,
            max_vertices=8192,
            max_hits=2048,
        )
        return perception, Pose2D(0, 0, 0)

    def step(self, packet=None, *, ros=None):
        packet = self.capture["reference"] if packet is None else packet
        if ros is None:
            self.clock.steady = packet["sampled_steady"]
            self.clock.ros = packet["sampled_ros"]
        else:
            self.clock.ros = ros
        self.core._reference = copy.deepcopy(packet)
        self.core.tick()
        return MotionRuntimeState.model_validate_json(self.node.messages[-1].data)

    def recover(self):
        self.free = True
        return [self.step(packet) for packet in self.capture["recovery_references"]]

    def blocked_again(self):
        self.clock.steady += 0.05
        packet = copy.deepcopy(self.capture["recovery_references"][-1])
        packet["reference"]["end_ros"] = packet["sampled_ros"] - 0.16
        packet["reference"]["start_ros"] = packet["reference"]["end_ros"] - 0.1
        return self.step(packet, ros=packet["sampled_ros"])


@pytest.fixture
def replay(monkeypatch):
    msg = ModuleType("std_msgs.msg")
    msg.String = lambda: N(data="")
    monkeypatch.setitem(sys.modules, "std_msgs.msg", msg)
    lifecycle = ModuleType("senior_robot.motion_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    capture = json.loads(Path("tests/fixtures/navigation_startup_reference.json").read_text())
    clock = N(steady=capture["fault_steady"], ros=capture["blocked_ros"])
    monkeypatch.setattr(module, "time", N(monotonic=lambda: clock.steady))
    value = Replay(capture, clock)
    yield value
    value.core.close()


def test_captured_startup_unknown_stop_denies_motion_without_fault(replay):
    state = replay.step(ros=1.46)
    assert replay.decisions[-1].reason == "stop_envelope_blocked"
    assert replay.decisions[-1].emergency and not replay.decisions[-1].verified
    assert replay.decisions[-1].command is None
    assert replay.core.scheduler.current.permit is None
    assert replay.core.scheduler.current.brake.kind is BrakeKind.EMERGENCY
    assert not replay.core.scheduler.current.brake.stop_verified
    assert replay.node.output[-1][0] == Velocity(0, 0)
    assert not state.stationary and state.body_reference is None
    assert state.mode == "feedback_stop"
    assert state.fault_revision == 0 and "acquire" not in state.allowed_actions


def test_captured_startup_recovers_to_real_navigation_admission(replay, navigation):  # noqa: F811
    replay.step(ros=1.46)
    states = replay.recover()
    first_body_stamp = next(
        row["sample"]["received_steady"]
        for row in replay.capture["recovery_references"][0]["sources"]
        if row["source"] == "laser"
    )
    early = [s for s in states if s.sampled_steady < first_body_stamp + 0.5]
    assert early and all(not s.stationary and "acquire" not in s.allowed_actions for s in early)
    state = states[-1]
    assert state.mode == "idle" and state.stationary and state.healthy
    assert state.fault_revision == 0 and state.body_reference is not None
    assert "acquire" in state.allowed_actions
    assert all(velocity == Velocity(0, 0) for velocity, _reason in replay.node.output)
    receiver = MotionRuntimeReceiver()
    assert receiver.accept(state, b"real-core-test-output", replay.clock.steady)
    navigation.node._motion_runtime = receiver
    navigation.node._monotonic_clock = N(monotonic=lambda: replay.clock.steady)
    navigation.node._arbiter = N(estop_latched=False)
    navigation.node._manual_target = None
    assert navigation.node.task_motion_allowed(continuing=False)


def test_independent_stillness_permanently_ends_startup_exception(replay):
    replay.step(ros=1.46)
    assert replay.recover()[-1].stationary
    _, body, wheel = replay.core._inputs(replay.clock.steady)
    assert replay.core.controller.reset_at_stillness(body, wheel, replay.clock.steady)
    replay.core._event(MotionEventKind.FEEDBACK_LOST, replay.clock.steady)
    state = replay.blocked_again()
    assert state.mode == "execution_fault" and state.fault_revision == 1
    assert replay.decisions[-1].reason == "stop_envelope_blocked"


@pytest.mark.parametrize("source", ["manual", "navigation"])
@pytest.mark.parametrize(
    "velocity,expected_mode,expected_fault",
    [
        (Velocity(0.05, 0), "execution_fault", 1),
        (Velocity(0, 0.05), "execution_fault", 1),
        (Velocity(0, 0), "feedback_stop", 0),
    ],
)
def test_source_intent_then_release_preserves_one_way_startup(
    replay, source, velocity, expected_mode, expected_fault
):
    core, now = replay.core, replay.clock.steady
    # Use captured current body/wheel evidence for the real 9555 ingress check,
    # without claiming independent stillness or bypassing conditional recovery.
    replay.clock.ros = 1.44
    core._reference = copy.deepcopy(replay.capture["reference"])
    _, body, wheel = core._inputs(now)
    assert body is not None and wheel is not None
    core.machine.observe(
        RecoveryObservation(
            body.epoch,
            body.sequence,
            body.stamp_steady,
            body.deadline_steady,
            body.fresh(now) and wheel.fresh(now),
            False,
        ),
        now,
    )
    intent = N(
        owner_boot_id="producer",
        generation=1,
        sequence=1,
        source_kind=source,
        source_id="source",
        source_epoch=1,
        received_steady=now,
        steady_deadline=now + 0.3,
        vx=velocity.vx,
        wz=velocity.wz,
        close_reason="intent",
    )
    assert core.command(intent)
    close = N(
        robot_id="route-sim",
        owner_boot_id="producer",
        session_id="session",
        generation=2,
        sequence=0,
        connection_epoch=1,
        source_kind=source,
        source_id="source",
        source_epoch=1,
        received_steady=now,
        steady_deadline=0.0,
        vx=0.0,
        wz=0.0,
        close_reason="release",
        request_id="release",
        gesture=0,
    )
    assert core.command(close)
    assert core.target.source == "idle" and core.controller.trajectory.velocity == Velocity(0, 0)
    state = replay.step(ros=1.46)
    assert state.mode == expected_mode and state.fault_revision == expected_fault


@pytest.mark.parametrize("event", [MotionEventKind.EXECUTION_FAULT, MotionEventKind.SOFTWARE_ESTOP])
def test_startup_classification_does_not_clear_preexisting_fault_or_estop(replay, event):
    replay.core._event(event, replay.clock.steady)
    state = replay.step(ros=1.46)
    assert state.mode == ("estop" if event is MotionEventKind.SOFTWARE_ESTOP else "execution_fault")
    assert (
        state.estop_revision if event is MotionEventKind.SOFTWARE_ESTOP else state.fault_revision
    ) > 0
    assert replay.recover()[-1].mode == state.mode


@pytest.mark.parametrize(
    "velocity,acceleration",
    [(Velocity(0.05, 0), Velocity(0, 0)), (Velocity(0, 0), Velocity(0, 0.05))],
)
def test_nonzero_trajectory_is_not_startup_waiting(replay, velocity, acceleration):
    replay.core.controller.trajectory = Trajectory(velocity, acceleration)
    state = replay.step(ros=1.46)
    assert state.mode == "execution_fault" and state.fault_revision == 1
