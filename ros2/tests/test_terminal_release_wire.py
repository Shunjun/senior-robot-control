"""Actual CommandNode receipt logic and IPC identity correlation; no ROS acceptance claim."""

import sys
from types import SimpleNamespace as N

import pytest
from motion_reference_test_support import final_command_message
from test_navigation_command_clock import navigation as command_fixture


@pytest.fixture
def wire(monkeypatch):
    r = command_fixture.__wrapped__(monkeypatch)
    node = r.node
    node._owner_boot, node._generation = "command-owner", 3
    node._retirement_core = (b"runtime", "core-one")
    node._navigation_source.task_epoch = 1
    node._awaiting_feedback = {}
    node.get_parameter = lambda _: N(value="robot")
    node.get_publishers_info_by_topic = lambda topic: [
        N(
            node_name="motion_reporter" if topic == "/motion/runtime" else "safety_gate",
            node_namespace="/",
            endpoint_gid=b"runtime" if topic == "/motion/runtime" else b"gate",
        )
    ]
    frames = []
    node._final_publisher = N(publish=frames.append)
    monkeypatch.setattr(
        sys.modules["command_clock_under_test"], "FinalCommand", final_command_message
    )
    del (
        node._publish_barrier
    )  # Exercise the real frame builder, including immutable original source.
    return N(node=node, frames=frames, rig=r)


def ack(wire, status="retired", **changes):
    proof = wire.node._retirements["private-execution"]
    packet = N(
        robot_id="robot",
        owner_boot_id="command-owner",
        session_id="private-execution",
        request_id=proof.token,
        sequence=proof.generation,
        connection_epoch=1,
        status=status,
        reason="core-one",
    )
    for k, v in changes.items():
        setattr(packet, k, v)
    wire.node._on_drive_feedback(packet, {"publisher_gid": b"gate"})
    return packet


def test_no_identity_event_before_output_retirement_and_exact_ack_is_required(wire):
    node = wire.node
    assert node.request_terminal_release("private-execution", 1, "cancel", "canceled") is None
    assert all(f.close_reason != "task_terminal" for f in wire.frames)
    ack(wire, request_id="wrong")
    assert node.terminal_release_status("cancel") is None
    ack(wire)
    assert node.terminal_release_status("cancel") is None
    identity = wire.frames[-1]
    assert identity.close_reason == "task_terminal" and identity.source_epoch == 1
    for changes in [
        dict(owner_boot_id="old"),
        dict(sequence=99),
        dict(connection_epoch=2),
        dict(session_id="other"),
    ]:
        ack(wire, "task_retired", **changes)
        assert not node._terminal_requests["cancel"].acknowledged
    ack(wire, "task_retired")
    count = len(wire.frames)
    assert node.terminal_release_status("cancel") is True
    assert node.request_terminal_release("private-execution", 1, "cancel", "canceled") is True
    assert len(wire.frames) == count
    assert node.request_terminal_release("private-execution", 2, "cancel", "canceled") is False


def test_known_identity_release_preserves_new_manual_and_command_generation(wire):
    node = wire.node
    node.clear_navigation_source("private-execution")
    ack(wire)
    marker = object()
    node._manual_target = marker
    node._generation = 99
    node._barrier_frame = marker
    before = (node._active, node._generation, node._barrier_frame, node._manual_target)
    assert node.request_terminal_release("private-execution", 1, "cancel", "canceled") is None
    assert wire.frames[-1].close_reason == "task_terminal"
    assert wire.frames[-1].generation < node._generation
    assert (node._active, node._generation, node._barrier_frame, node._manual_target) == before


@pytest.mark.parametrize("foreign", ["manual", "different_task", "prepared_manual"])
def test_missing_receipt_cannot_send_a_new_global_barrier_over_later_authority(wire, foreign):
    node = wire.node
    node._navigation_source = None
    node._last_close_source = ("idle", "", 0, 0.0, "", 0)
    if foreign == "prepared_manual":
        from senior_control.motion.manual_authority import ManualAuthorityState

        node._manual_ingress = N(view=lambda _: N(state=ManualAuthorityState.PREPARED))
    elif foreign == "manual":
        node._manual_target = object()
    else:
        node._navigation_source = N(execution_id="other", task_epoch=1)
    assert node.request_terminal_release("private-execution", 1, "cancel", "canceled") is None
    assert not wire.frames and not node._retirements


def test_core_restart_rotates_unknown_proof_before_original_handshake(wire):
    node = wire.node
    node.request_terminal_release("private-execution", 1, "cancel", "canceled")
    old_ack = ack(wire)
    node.terminal_release_status("cancel")
    assert node._terminal_requests["cancel"].proof is not None
    node.get_publishers_info_by_topic = lambda topic: [
        N(
            node_name="motion_reporter" if topic == "/motion/runtime" else "safety_gate",
            node_namespace="/",
            endpoint_gid=b"new-runtime" if topic == "/motion/runtime" else b"gate",
        )
    ]
    node._observe_retirement_core(b"new-runtime", "core-two")
    assert node._terminal_requests["cancel"].proof is None
    assert not node._retirements["private-execution"].acknowledged
    node._on_drive_feedback(old_ack, {"publisher_gid": b"gate"})
    assert not node._retirements["private-execution"].acknowledged
    count = len(wire.frames)
    node.terminal_release_status("cancel")
    assert all(f.close_reason == "task_revoke" for f in wire.frames[count:])
    assert node._retirements["private-execution"].token != old_ack.request_id
    assert node._retirements["private-execution"].generation > old_ack.sequence
    ack(wire, reason="core-two")
    node.terminal_release_status("cancel")
    assert wire.frames[-1].close_reason == "task_terminal"


def test_terminal_request_storage_is_bounded_and_never_evicts_for_a_new_effect(wire):
    node = wire.node
    for i in range(256):
        node._terminal_requests[str(i)] = N(execution_id="old", through_epoch=1, outcome="canceled")
    assert node.request_terminal_release("private-execution", 1, "overflow", "canceled") is None
    assert len(node._terminal_requests) == 256 and not wire.frames


@pytest.mark.asyncio
async def test_ipc_terminal_effect_is_explicit_exact_and_query_does_not_invent_unknown_effect():
    from senior_robot.agent_adapter import AdapterCore

    calls = []

    class Routes:
        async def release_terminal(self, *args, **kwargs):
            calls.append(args)
            return True

    core = AdapterCore(
        robot_id="robot",
        submit=lambda *_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        routes=Routes(),
    )
    common = dict(
        execution_id="execution",
        task_epoch=2,
        effect_id="terminal",
        terminal_outcome="canceled",
        effect_sequence=1,
    )
    unknown = await core.handle(
        dict(type="navigation.task_status", effect_kind="terminal_release", **common)
    )
    assert unknown["identity_released"] is False and not calls
    result = await core.handle(dict(type="navigation.task_terminal", **common))
    assert result["identity_released"] and calls == [("execution", 2, "terminal", "canceled")]
    for field, value in [
        ("task_epoch", 3),
        ("execution_id", "other"),
        ("terminal_outcome", "failed"),
    ]:
        changed = dict(common)
        changed[field] = value
        reply = await core.handle(dict(type="navigation.task_terminal", **changed))
        assert reply["error"] == "conflict"
    assert len(calls) == 1
    reply = await core.handle(
        dict(type="navigation.task_terminal", **{**common, "terminal_outcome": {}})
    )
    assert reply["error"] == "bad_request"


def test_ipc_port_never_accepts_mismatched_terminal_receipt():
    from senior_control.agent.ports import IpcNavigation
    from senior_control.tasks.lifecycle import Effect

    effect = Effect("terminal_release", "effect", "execution", 2, terminal_outcome="canceled")
    reply = dict(
        lifecycle_version=2,
        effect_id="effect",
        execution_id="execution",
        task_epoch=2,
        terminal_outcome="canceled",
        identity_released=True,
        accepted=True,
    )
    assert IpcNavigation._effect_result(reply, effect) is True
    for key, value in [
        ("effect_id", "other"),
        ("execution_id", "other"),
        ("task_epoch", 1),
        ("terminal_outcome", "succeeded"),
        ("identity_released", False),
        ("accepted", False),
    ]:
        assert IpcNavigation._effect_result({**reply, key: value}, effect) is None


def test_missing_local_receipt_handshake_learns_original_core_epoch(wire, monkeypatch):
    import ast
    from pathlib import Path

    from test_motion_core_exit_deadline import rig as core_fixture
    from test_motion_core_exit_deadline import start

    from senior_control.models import Velocity
    from senior_control.safety.arbiter import FinalCommandFence

    fixture = core_fixture.__wrapped__(monkeypatch)
    core_rig = next(fixture)
    try:
        start(core_rig, 0.5)
        node = wire.node
        node._navigation_source = None
        node._last_close_source = ("idle", "", 0, 0.0, "", 0)
        node._motion_runtime.value = N(mode="braking", task_id="route")
        node._retirement_core = (b"runtime", core_rig.core.boot)
        node._monotonic_clock = N(monotonic=lambda: core_rig.clock.steady)
        node.get_parameter = lambda _: N(value="route-sim")
        tree = ast.parse(
            Path("ros2/src/senior_motion_control/senior_motion_control/safety_node.py").read_text()
        )
        cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == "SafetyGate")
        methods = [
            x
            for x in cls.body
            if isinstance(x, ast.FunctionDef) and x.name in {"_on_command", "_on_task_terminal"}
        ]
        scope = {
            "time": N(monotonic=lambda: core_rig.clock.steady),
            "Velocity": Velocity,
            "DriveFeedbackMessage": N,
        }
        module = ast.Module(
            body=[
                ast.ClassDef(name="Gate", bases=[], keywords=[], body=methods, decorator_list=[])
            ],
            type_ignores=[],
        )
        exec(
            compile(ast.fix_missing_locations(module), "<actual retirement handshakes>", "exec"),
            scope,
        )
        gate = scope["Gate"]()
        gate._motion_core = core_rig.core
        gate._final_fence = FinalCommandFence()
        gate._final_fence.set_publisher(b"command")
        gate.get_parameter = lambda _: N(value="route-sim")
        gate._replace_drive_feedback = lambda *_: None
        gate._drive_feedback_publisher = N(
            publish=lambda msg: node._on_drive_feedback(msg, {"publisher_gid": b"gate"})
        )
        # Aggregate restart fences through epoch2; the actually retired source was epoch1.
        assert node.request_terminal_release("route", 2, "cancel", "canceled") is None
        close = wire.frames[-1]
        assert close.source_epoch == 2 and close.close_reason == "task_revoke"
        gate._on_command(close, b"command")
        receipt = node._retirements["route"]
        assert receipt.acknowledged and receipt.task_epoch == 1
        node.terminal_release_status("cancel")
        identity = wire.frames[-1]
        assert identity.source_epoch == 1 and identity.generation == receipt.generation
        gate._on_task_terminal(identity)
        assert node.terminal_release_status("cancel") is True
        assert core_rig.core.machine.state.task_id is None
    finally:
        try:
            next(fixture)
        except StopIteration:
            pass


def test_full_retirement_book_still_revokes_output_immediately_without_minting_ack(wire):
    node = wire.node
    record_type = sys.modules["command_clock_under_test"]._NavigationRetirement
    node._retirements = {
        str(i): record_type("token", 3, ("navigation", str(i), 1, 0.0, str(i), 1), "task_revoke", 1)
        for i in range(256)
    }
    node._publish_manual_target = lambda: False
    node._arbiter = N(end_navigation=lambda _: None, final_command=lambda _: None)
    node._publish = lambda _: None
    node.publish_command = type(node).publish_command.__get__(node)
    node.clear_navigation_source("private-execution")
    assert node._navigation_source is None and len(node._retirements) == 256
    assert "private-execution" not in node._retirements
    assert wire.frames, "metadata capacity must not suppress immediate source revocation"
    closed = wire.frames[-1]
    assert closed.generation > 3 and closed.sequence == 0
    assert closed.vx == closed.wz == closed.steady_deadline == 0 and not closed.request_id


@pytest.mark.parametrize("ack_first", [True, False])
@pytest.mark.parametrize("pre_cached", [True, False])
def test_missing_gid_old_ack_cannot_cross_feedback_incarnation(wire, ack_first, pre_cached):
    node = wire.node
    node.request_terminal_release("private-execution", 1, "cancel", "canceled")
    ack(wire)
    node.terminal_release_status("cancel")
    proof = node._retirements["private-execution"]
    old = N(
        robot_id="robot",
        owner_boot_id="command-owner",
        session_id="private-execution",
        request_id=proof.token,
        sequence=proof.generation,
        connection_epoch=1,
        status="task_retired",
        reason="core-one",
    )
    if pre_cached:
        node._on_drive_feedback(old, {})
        assert node.terminal_release_status("cancel") is True
    marker = object()
    node._manual_target = marker
    node._generation, node._barrier_frame = 99, marker
    before = node._generation, node._barrier_frame, len(wire.frames)
    node.get_publishers_info_by_topic = lambda topic: [
        N(
            node_name="motion_reporter" if topic == "/motion/runtime" else "safety_gate",
            node_namespace="/",
            endpoint_gid=b"new-gate" if topic.endswith("drive_feedback") else b"new-runtime",
        )
    ]
    if ack_first:
        node._on_drive_feedback(old, {})
        assert not node._terminal_requests["cancel"].acknowledged
        node._observe_retirement_core(b"new-runtime", "core-two")
    else:
        node._observe_retirement_core(b"new-runtime", "core-two")
        node._on_drive_feedback(old, {})
    assert node.terminal_release_status("cancel") is None
    assert not node._terminal_requests["cancel"].acknowledged
    assert node._retirements["private-execution"].token != old.request_id
    assert (node._generation, node._barrier_frame, len(wire.frames)) == before
    assert node._manual_target is marker


def test_cached_output_retirement_is_not_reused_across_unobserved_core_change(wire):
    node = wire.node
    node.clear_navigation_source("private-execution")
    ack(wire)
    assert node.navigation_source_retired("private-execution")
    node.get_publishers_info_by_topic = lambda topic: [
        N(
            node_name="motion_reporter" if topic == "/motion/runtime" else "safety_gate",
            node_namespace="/",
            endpoint_gid=b"new-runtime" if topic == "/motion/runtime" else b"new-gate",
        )
    ]
    assert not node.navigation_source_retired("private-execution")
    assert not node._retirements["private-execution"].acknowledged
