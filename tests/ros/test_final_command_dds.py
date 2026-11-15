"""Opt-in actual DDS -> native guard -> profile-selected SafetyGate regressions.

No sensor or vehicle is supplied: accepted transport is observed at the real
FinalCommandFence, while the unchanged motion core must keep /cmd_vel stopped.
This proves transport identity/barriers, not physical motion or stop performance.
Run only in an explicitly isolated, same-build Linux ROS workspace.
"""

import os
import time
from types import SimpleNamespace

import pytest

from senior_control.agent.manual_control import ManualIngress
from senior_control.models import Velocity
from senior_control.motion.contracts import ManualIntent
from senior_control.protocol import EmptyPayload, SimpleEnvelope
from senior_control.safety.arbiter import PersistentEstop
from senior_control.settings.models import DrivingSettings

COMMAND_TTL = 3.0  # Test-only: distinguish identity rejection from expiry during DDS discovery.


def wait(executor, predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return
    raise AssertionError("DDS condition timed out")


@pytest.fixture
def dds(tmp_path, monkeypatch):
    if os.environ.get("SENIOR_FINAL_COMMAND_DDS") != "1":
        pytest.skip("opt-in actual DDS/native-guard composition; no services started by default")
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter
    from senior_motion_msgs.msg import FinalCommand
    from senior_robot.command_node import CommandNode
    from senior_robot.motion_lifecycle import MotionExecutor
    from senior_robot.safety_node import SafetyGate

    domain = os.environ.get("SENIOR_MOTION_DOMAIN", "")
    assert domain.isdecimal() and 0 <= int(domain) <= 232, "select an isolated ROS domain"
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    rclpy.init()
    path = tmp_path / "estop.json"
    PersistentEstop.initialize(path, "paired-car")
    parameters = [
        Parameter("robot_id", value="paired-car"),
        Parameter("safety_state_path", value=str(path)),
        Parameter("command_ttl_seconds", value=COMMAND_TTL),
    ]
    nodes = []
    executor = MotionExecutor()

    def own(node):
        nodes.append(node)
        executor.add_node(node)
        return node

    try:
        command = own(CommandNode(parameter_overrides=parameters))
        gate = own(
            SafetyGate(
                parameter_overrides=[
                    Parameter("robot_id", value="paired-car"),
                    Parameter("use_sim_time", value=True),
                    Parameter("motion_core_profile", value="fortress-candidate-unvalidated-v1"),
                ]
            )
        )
        observer = own(rclpy.create_node("final_command_observer"))
        frames, outputs = [], []
        observer.create_subscription(FinalCommand, "/safety/final_command", frames.append, 10)
        observer.create_subscription(Twist, "/cmd_vel", outputs.append, 10)
        wait(executor, lambda: gate._command_topology_ready() and bool(frames))
        delivered, revoked = [], []
        consume, revoke = gate._motion_core.command, gate._motion_core.revoke

        def observe_command(message):
            delivered.append((message.generation, message.sequence, message.vx))
            return consume(message)

        def observe_revoke(reason, **kwargs):
            revoked.append(reason)
            return revoke(reason, **kwargs)

        gate._motion_core.command = observe_command
        gate._motion_core.revoke = observe_revoke
        yield SimpleNamespace(
            rclpy=rclpy,
            executor=executor,
            command=command,
            gate=gate,
            own=own,
            nodes=nodes,
            outputs=outputs,
            delivered=delivered,
            revoked=revoked,
        )
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def manual_frame(command, sequence=2):
    """Use the current authority owner, not a removed velocity/clock protocol."""
    from senior_motion_msgs.msg import FinalCommand

    now = time.monotonic()
    ingress = ManualIngress("paired-car", command._owner_boot, ttl=COMMAND_TTL)
    ingress.bind(1, 1, "page", now)
    authority = ingress.acquire("acquire", now).authority_id
    # Prepared authority only activates on the explicit initial neutral sample.
    settings = DrivingSettings(
        low_linear_mps=0.5,
        high_linear_mps=1.0,
        low_angular_rps=0.5,
        high_angular_rps=1.0,
        reverse_linear_mps=0.3,
    )
    neutral = ManualIntent(authority, 1, 1, 0.0, 0.0, "low")
    assert ingress.accept(neutral, settings, now, now + COMMAND_TTL, now) is not None
    intent = ManualIntent(authority, 1, sequence, 0.2, 0.0, "low")
    target = ingress.accept(intent, settings, now, now + COMMAND_TTL, now)
    assert target is not None
    return FinalCommand(
        robot_id="paired-car",
        owner_boot_id=command._owner_boot,
        session_id="page",
        generation=command._generation,
        connection_epoch=1,
        sequence=sequence,
        steady_deadline=target.sample.deadline_steady,
        vx=target.velocity.vx,
        wz=target.velocity.wz,
        request_id="intent",
        gesture=1,
        source_kind="manual",
        source_id=authority,
        source_epoch=target.source_epoch,
        received_steady=target.sample.received_steady,
        close_reason="intent",
        manual_linear=intent.linear,
        manual_linear_limit=target.linear_limit_mps,
        degraded_linear_limit=target.degraded_linear_mps,
    )


@pytest.mark.parametrize(
    "fault", ["stop_barrier", "duplicate_expected", "duplicate_other", "sole_other", "guard_lost"]
)
def test_actual_dds_identity_and_generation_fence_reach_current_owner(dds, fault):
    from senior_motion_msgs.msg import FinalCommand

    command, gate, executor = dds.command, dds.gate, dds.executor
    original = manual_frame(command)
    command._final_publisher.publish(original)
    sample = (original.generation, original.sequence, original.vx)
    wait(executor, lambda: sample in dds.delivered)
    assert gate._final_fence.usable(time.monotonic())

    if fault == "stop_barrier":
        # Lose an intermediate generation. A newer actual stop barrier must win.
        command._generation += 1
        stop = SimpleEnvelope(
            v=1,
            robot_id="paired-car",
            session_id="page",
            connection_epoch=1,
            request_id="stop",
            seq=3,
            expires_at=0.0,
            type="drive.stop",
            payload=EmptyPayload(),
        )
        assert command.submit(stop).accepted
        wait(executor, lambda: gate._motion_core.output_generation == command._generation)
        assert gate._motion_core._barrier_complete
        publish = command._final_publisher.publish
    elif fault == "guard_lost":
        process = gate._guard_process
        assert process is not None and process.poll() is None
        process.kill()
        process.wait(timeout=3)
        wait(executor, lambda: "command_topology_unqualified" in dds.revoked)
        publish = command._final_publisher.publish
    else:
        name = "command_node" if fault == "duplicate_expected" else "unexpected_publisher"
        attacker = dds.own(dds.rclpy.create_node(name))
        forged = attacker.create_publisher(FinalCommand, "/safety/final_command", 10)
        if fault == "sole_other":
            executor.remove_node(command)
            dds.nodes.remove(command)
            command.destroy_node()
        wait(executor, lambda: not gate._command_topology_ready())
        publish = forged.publish

    frontier = len(dds.delivered)
    for _ in range(12):
        assert time.monotonic() < original.steady_deadline, "expiry would mask identity rejection"
        publish(original)  # Same payload; actual publisher/GID and barrier checks stay real.
        executor.spin_once(timeout_sec=0.01)
        assert sample not in dds.delivered[frontier:]
        assert gate._motion_core.target.source == "idle"
    if fault == "stop_barrier":
        assert gate._final_fence._velocity == Velocity(0, 0)
    else:
        assert not gate._command_topology_ready()
        assert "command_topology_unqualified" in dds.revoked
    assert dds.outputs and all(m.linear.x == 0 and m.angular.z == 0 for m in dds.outputs)
