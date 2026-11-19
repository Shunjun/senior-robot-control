"""Real C++ identity frame -> private socket -> real gate decoder and real core."""

import ast
import importlib.util
import socket
import time
from pathlib import Path
from types import SimpleNamespace as N

from test_task_terminal_release import rig  # noqa: F401

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionMode
from senior_control.safety.arbiter import FinalCommandFence


def test_native_identity_channel_preserves_newer_manual_generation(rig):  # noqa: F811
    path = Path("ros2/src/senior_motion_control/test/test_final_command_frame.py")
    spec = importlib.util.spec_from_file_location("terminal_frame_test", path)
    native = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(native)
    native.NativeRoundtrip.setUpClass()
    try:
        packet = native.NativeRoundtrip().send("terminal")
        collection_packet = native.NativeRoundtrip().send("collected")
    finally:
        native.NativeRoundtrip.tearDownClass()
    kind, gid, frame = native.codec.decode_frame(packet)
    assert kind == 3
    core = rig.core
    core.tick()

    def intent(kind, source, epoch, generation):
        return N(
            close_reason="intent",
            manual_linear=1.0 if kind == "manual" else 0.0,
            manual_linear_limit=0.01 if kind == "manual" else 0.0,
            degraded_linear_limit=0.3 if kind == "manual" else 0.0,
            source_kind=kind,
            source_id=source,
            source_epoch=epoch,
            owner_boot_id="boot",
            generation=generation,
            sequence=1,
            received_steady=rig.clock.steady,
            steady_deadline=rig.clock.steady + 0.2,
            vx=0.01,
            wz=0.0,
        )

    assert core.command(intent("navigation", "execution", 42, 4))
    output_close = N(**vars(frame))
    output_close.close_reason = "task_revoke"
    assert core.command(output_close)
    assert core.retirement_epoch(output_close) == 42
    assert core.command(intent("manual", "manual", 7, 6))
    target = core.target
    fence = FinalCommandFence()
    fence.set_publisher(gid)
    assert fence.accept(gid, "boot", 6, 0, 0.0, Velocity(0, 0), rig.clock.steady)
    assert fence.accept(
        gid,
        "boot",
        6,
        1,
        target.deadline_steady,
        target.velocity,
        rig.clock.steady,
        session="manual",
        epoch=7,
    )
    before = (fence._generation, fence._sequence, fence._authority, core.output_generation)
    tree = ast.parse(Path("ros2/src/senior_robot/senior_robot/safety_node.py").read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SafetyGate")
    methods = [
        n
        for n in cls.body
        if isinstance(n, ast.FunctionDef) and n.name in {"_on_task_terminal", "_poll_command_guard"}
    ]
    scope = {"time": time, "Velocity": Velocity, "DriveFeedbackMessage": N}
    module = ast.Module(
        body=[ast.ClassDef(name="Gate", bases=[], keywords=[], body=methods, decorator_list=[])],
        type_ignores=[],
    )
    exec(compile(ast.fix_missing_locations(module), "<actual identity channel>", "exec"), scope)
    gate = scope["Gate"]()
    gate._motion_core = core
    gate._final_fence = fence
    gate._guard_ready = True
    gate.get_parameter = lambda _: N(value="route-sim")
    receipts = []
    gate._drive_feedback_publisher = N(publish=receipts.append)
    gate._on_command = lambda *_: (_ for _ in ()).throw(
        AssertionError("identity entered motion frontier")
    )
    parent, child = socket.socketpair(
        socket.AF_UNIX,
        socket.SOCK_SEQPACKET
        if hasattr(socket, "SOCK_SEQPACKET") and __import__("sys").platform == "linux"
        else socket.SOCK_DGRAM,
    )
    parent.setblocking(False)
    gate._guard_socket = parent
    try:
        child.send(packet)
        gate._poll_command_guard()
        child.send(packet)
        gate._poll_command_guard()
        child.send(collection_packet)
        gate._poll_command_guard()
        child.send(collection_packet)
        gate._poll_command_guard()
        child.send(packet)  # Collected identity cannot be recreated from the old frame.
        gate._poll_command_guard()
    finally:
        parent.close()
        child.close()
    assert len(receipts) == 4
    assert all(
        (r.status, r.session_id, r.request_id, r.sequence, r.connection_epoch)
        == ("task_retired", "execution", "token", 5, 42)
        for r in receipts[:2]
    )
    assert (fence._generation, fence._sequence, fence._authority, core.output_generation) == before
    assert core.target is target and core.machine.state.mode is MotionMode.MANUAL
    assert core.machine.state.task_id is None

    assert [r.status for r in receipts[2:]] == ["task_collected", "task_collected"]
    assert not core._task_retirements and not core._terminal_released
