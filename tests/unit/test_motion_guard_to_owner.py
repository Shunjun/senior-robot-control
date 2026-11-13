"""Native C++ sender -> actual gate decoder/fence -> actual local braking owner.

ROS I/O is replaced, not the command handler. Linux RMW/guard build is separate.
"""

import ast
import importlib.util
import socket
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace as N

from test_motion_core_handoff import Publisher, core  # noqa: F401

from senior_control.models import Velocity
from senior_control.protocol import DriveFeedbackKey
from senior_control.safety.arbiter import FinalCommandFence
from senior_control.safety.drive_feedback import OutputFeedback

ROOT = Path(__file__).resolve().parents[2]


def test_real_native_frame_crosses_gate_fence_before_retirement_ack(core, monkeypatch):  # noqa: F811
    spec = importlib.util.spec_from_file_location(
        "native_test", ROOT / "ros2/src/senior_motion_msgs/test/test_final_command_frame.py"
    )
    native = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(native)
    native.NativeRoundtrip.setUpClass()
    tree = ast.parse((ROOT / "ros2/src/senior_robot/senior_robot/safety_node.py").read_text())
    source = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SafetyGate")
    methods = [
        n
        for n in source.body
        if isinstance(n, ast.FunctionDef)
        and n.name in {"_poll_command_guard", "_on_command", "_replace_drive_feedback"}
    ]
    module = ast.Module(
        body=[ast.ClassDef(name="Gate", bases=[], keywords=[], body=methods, decorator_list=[])],
        type_ignores=[],
    )
    scope = {
        "time": time,
        "Velocity": Velocity,
        "DriveFeedbackMessage": lambda: N(),
        "DriveFeedbackKey": DriveFeedbackKey,
    }
    exec(compile(ast.fix_missing_locations(module), "<actual safety gate handlers>", "exec"), scope)
    g = scope["Gate"]()
    g._final_fence = FinalCommandFence()
    g._motion_core = core
    g.get_parameter = core.node.get_parameter
    g._drive_feedback_publisher = Publisher()
    g._guard_process = None
    g._feedback = OutputFeedback()
    g._publish_drive_feedback = lambda _: None
    parent, child = socket.socketpair(
        socket.AF_UNIX, socket.SOCK_SEQPACKET if sys.platform == "linux" else socket.SOCK_DGRAM
    )
    parent.setblocking(False)
    g._guard_socket = parent
    monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: 10.3)
    core.controller.trajectory.velocity = Velocity(0.4, 0)
    try:
        event_counts = []
        for _ in range(2):
            subprocess.run(
                [str(native.NativeRoundtrip.binary), str(child.fileno()), "navrelease"],
                pass_fds=(child.fileno(),),
                timeout=3,
                check=True,
            )
            g._poll_command_guard()
            event_counts.append(core.event_sequence)
        assert len(g._drive_feedback_publisher.messages) == 2  # idempotent re-ACK
        ack = g._drive_feedback_publisher.messages[-1]
        assert (ack.session_id, ack.request_id, ack.sequence, ack.status) == (
            "execution",
            "token",
            5,
            "retired",
        )
        assert core.controller.trajectory.velocity.vx == 0.4 and core.node.emitted == []
        assert core.target.source == "idle"
        assert core.target.source_id == core.boot
        assert event_counts[0] == event_counts[1]  # source close applied only once
        # The same production handler also consumes a real encoded intent. This
        # follows a new generation's neutral manual barrier, not navigation
        # resuming by itself.
        from senior_control.motion.state_machine import RecoveryObservation

        for i in range(5):
            core.machine.observe(
                RecoveryObservation("body", i, 9.5 + i * 0.125, 10.6, True, True), 10.3
            )
        for mode in ("manual_release", "manual_intent"):
            subprocess.run(
                [str(native.NativeRoundtrip.binary), str(child.fileno()), mode],
                pass_fds=(child.fileno(),),
                timeout=3,
                check=True,
            )
            g._poll_command_guard()
        assert core.target.source == "manual"
        assert core.target.source_id == "authority:42"
        assert core.target.deadline_steady == 10.55
        assert core.target.velocity == Velocity(0.4, 0.2)
        assert g._feedback.current.request_id == "token"
        revisions = []
        for mode in ("press1", "press1", "press2", "echo", "echo"):
            subprocess.run(
                [str(native.NativeRoundtrip.binary), str(child.fileno()), mode],
                pass_fds=(child.fileno(),),
                timeout=3,
                check=True,
            )
            g._poll_command_guard()
            revisions.append(core.machine.state.estop_revision)
        assert revisions == [1, 1, 2, 2, 2]
        assert not core._hardware  # software request never invents hardware evidence

    finally:
        parent.close()
        child.close()
        native.NativeRoundtrip.tearDownClass()
