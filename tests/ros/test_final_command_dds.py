"""Actual local DDS publisher identity and zero-barrier regression."""

import time

import pytest

from senior_control.models import Velocity
from senior_control.protocol import EmptyPayload, SimpleEnvelope, VelocityEnvelope, VelocityPayload
from senior_control.safety.arbiter import PersistentEstop


def wait(executor, predicate, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise AssertionError("DDS condition timed out")


def test_old_dds_frame_cannot_cross_stop_barrier(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from senior_motion_msgs.msg import FinalCommand
    from senior_robot.command_node import CommandNode
    from senior_robot.safety_node import SafetyGate

    rclpy.init()
    parameter = Parameter("robot_id", value="paired-car")
    path = tmp_path / "estop.json"
    PersistentEstop.initialize(path, "paired-car")
    command = CommandNode(
        parameter_overrides=[parameter, Parameter("safety_state_path", value=str(path))],
        require_server_heartbeat=True,
    )
    gate = SafetyGate(parameter_overrides=[parameter])
    observer = rclpy.create_node("final_command_observer")
    seen = []
    observer.create_subscription(FinalCommand, "/safety/final_command", seen.append, 10)
    executor = SingleThreadedExecutor()
    for node in (command, gate, observer):
        executor.add_node(node)
    try:
        wait(executor, lambda: bool(seen))
        wait(executor, lambda: gate._command_topology_ready() and gate._final_identity is not None)
        now = time.monotonic()
        assert command.server_heartbeat(1, now + 0.8)
        base = dict(
            v=1,
            robot_id="paired-car",
            session_id="page",
            connection_epoch=1,
            request_id="x",
            expires_at=0.0,
        )
        assert command.submit(
            SimpleEnvelope(**base, seq=1, type="control.acquire", payload=EmptyPayload())
        ).accepted
        assert command.submit(
            VelocityEnvelope(
                **base, seq=2, type="drive.velocity", payload=VelocityPayload(vx=0.1, wz=0.0)
            ),
            now,
        ).accepted
        wait(executor, lambda: gate._command == Velocity(0.1, 0.0))
        wait(executor, lambda: any(m.vx == 0.1 for m in seen))
        old = next(m for m in reversed(seen) if m.vx == 0.1)
        # Model one lost generation without publishing its barrier to DDS.
        command._generation += 1
        assert command.submit(
            SimpleEnvelope(**base, seq=3, type="drive.stop", payload=EmptyPayload())
        ).accepted
        wait(executor, lambda: gate._command == Velocity(0.0, 0.0))
        command._final_publisher.publish(old)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.02)
            assert gate._command == Velocity(0.0, 0.0)
        now = time.monotonic()
        assert command.server_heartbeat(1, now + 0.8)
        assert command.submit(
            VelocityEnvelope(
                **base, seq=4, type="drive.velocity", payload=VelocityPayload(vx=0.1, wz=0.0)
            ),
            now,
        ).accepted
        wait(executor, lambda: gate._command == Velocity(0.1, 0.0))
        attacker = rclpy.create_node("unexpected_publisher")
        forged = attacker.create_publisher(FinalCommand, "/safety/final_command", 10)
        try:
            wait(executor, lambda: not gate._command_topology_ready())
            forged.publish(old)
            for _ in range(10):
                executor.spin_once(timeout_sec=0.02)
                assert gate._command == Velocity(0.0, 0.0)
        finally:
            attacker.destroy_node()
        gate._guard_process.kill()
        gate._guard_process.wait(timeout=2.0)
        wait(executor, lambda: not gate._command_topology_ready())
        assert not gate._final_fence.usable(time.monotonic()) or not gate._command_topology_ready()
    finally:
        for node in (observer, gate, command):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_ros_command_owner_estop_survives_restart(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_robot.command_node import CommandNode

    rclpy.init()
    path = tmp_path / "estop.json"
    PersistentEstop.initialize(path, "paired-car")
    parameters = [
        Parameter("robot_id", value="paired-car"),
        Parameter("safety_state_path", value=str(path)),
    ]
    node = CommandNode(parameter_overrides=parameters, require_server_heartbeat=True)
    try:
        now = time.monotonic()
        assert node.server_heartbeat(1, now + 0.8)
        request = SimpleEnvelope(
            v=1,
            robot_id="paired-car",
            session_id="page",
            connection_epoch=1,
            request_id="stop",
            seq=1,
            expires_at=0.0,
            type="safety.estop",
            payload=EmptyPayload(),
        )
        assert node.submit(request).accepted
    finally:
        node.destroy_node()
    node = CommandNode(parameter_overrides=parameters, require_server_heartbeat=True)
    try:
        now = time.monotonic()
        assert node.server_heartbeat(2, now + 0.8)
        assert node._arbiter.mode(now) == "estop"
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize(
    "event", ["renewal", "ticket_expired", "velocity_changed", "zero", "lease_changed", "gap"]
)
def test_identical_command_refresh_keeps_original_geometry_deadline(monkeypatch, event):
    from types import SimpleNamespace

    rclpy = pytest.importorskip("rclpy")
    from senior_motion_msgs.msg import FinalCommand
    from senior_robot import safety_node as module

    from senior_control.safety.async_guard import SafetyTicket

    rclpy.init()
    gate = module.SafetyGate()
    try:
        now = 10.0
        monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: now))
        monkeypatch.setattr(gate, "_command_topology_ready", lambda: True)
        monkeypatch.setattr(gate, "_report", lambda _reason: None)
        emitted = []
        monkeypatch.setattr(gate, "_publisher", SimpleNamespace(publish=emitted.append))
        gid = b"fixture-publisher"
        gate._final_fence.set_publisher(gid)

        def accept(generation, sequence, deadline, vx, *, session="page", epoch=1):
            gate._on_command(
                FinalCommand(
                    robot_id="robot",
                    owner_boot_id="fixture",
                    generation=generation,
                    session_id=session,
                    connection_epoch=epoch,
                    sequence=sequence,
                    steady_deadline=deadline,
                    vx=vx,
                ),
                gid,
            )

        accept(1, 0, 0.0, 0.0)
        accept(1, 1, 10.2, 0.1)
        generation = gate._command_generation
        ros_now = gate.get_clock().now().nanoseconds / 1e9
        ticket = SafetyTicket(
            generation, 1, 1, "motion-epoch", Velocity(0.1, 0.0), now, 10.2, ros_now, ros_now + 2.0
        )
        now = 10.1
        renewal_deadline = 10.15 if event == "gap" else 10.4
        accept(1, 2, renewal_deadline, 0.1)
        assert gate._command_generation == generation
        assert gate._command_deadline == renewal_deadline

        def drain():
            if event in {"zero", "lease_changed"}:
                accept(2, 0, 0.0, 0.0)
                if event == "lease_changed":
                    accept(2, 3, 10.45, 0.1, session="replacement", epoch=2)
                else:
                    accept(2, 3, 10.45, 0.1)
            else:
                accept(1, 3, 10.45, 0.2 if event == "velocity_changed" else 0.1)

        # The final drain may see another renewal just as a geometry result is emitted.
        if event == "ticket_expired":
            now = ticket.steady_deadline
        elif event == "gap":
            now = 10.16
        monkeypatch.setattr(gate, "_poll_command_guard", drain)
        gate._publish(ticket.command, "hard_sweep_clear", ticket=ticket)
        assert gate._final_fence.usable(now)
        assert emitted[-1].linear.x == (0.1 if event == "renewal" else 0.0)
    finally:
        gate.destroy_node()
        rclpy.shutdown()


def test_drive_feedback_follows_actual_output(monkeypatch):
    from types import SimpleNamespace

    rclpy = pytest.importorskip("rclpy")
    from senior_motion_msgs.msg import FinalCommand
    from senior_robot import safety_node as module

    rclpy.init()
    gate = module.SafetyGate()
    try:
        events = []
        monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=lambda: 10.0))
        monkeypatch.setattr(gate, "_poll_command_guard", lambda: None)
        monkeypatch.setattr(gate, "_report", lambda _reason: None)
        monkeypatch.setattr(
            gate, "_publisher", SimpleNamespace(publish=lambda msg: events.append(("output", msg)))
        )
        monkeypatch.setattr(
            gate,
            "_drive_feedback_publisher",
            SimpleNamespace(publish=lambda msg: events.append(("feedback", msg))),
        )
        gid = b"fixture"
        gate._final_fence.set_publisher(gid)
        gate._on_command(FinalCommand(robot_id="robot", owner_boot_id="boot", generation=1), gid)
        gate._on_command(
            FinalCommand(
                robot_id="robot",
                owner_boot_id="boot",
                session_id="page",
                generation=1,
                connection_epoch=3,
                sequence=2,
                steady_deadline=10.4,
                request_id="req-2",
                gesture=5,
                vx=0.2,
            ),
            gid,
        )
        assert events == []
        gate._publish(Velocity(0.0, 0.0), "scan_topology_unqualified")
        assert [kind for kind, _ in events] == ["output", "feedback"]
        assert events[0][1].linear.x == 0.0
        feedback = events[1][1]
        assert feedback.status == "blocked"
        assert feedback.reason == "scan_topology_unqualified"
        assert (feedback.request_id, feedback.gesture, feedback.connection_epoch) == ("req-2", 5, 3)
    finally:
        gate.destroy_node()
        rclpy.shutdown()
