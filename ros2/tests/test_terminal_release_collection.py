"""Durable collection is bounded, retryable metadata work, never a motion event."""

import ast
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as N

import pytest
from senior_motion_control.arbiter import FinalCommandFence
from test_motion_core_exit_deadline import start
from test_task_terminal_release import intent
from test_task_terminal_release import rig as core_fixture
from test_terminal_release_wire import wire as command_fixture

from vehicle_core.values import Velocity


@pytest.fixture
def rig(monkeypatch):
    yield from core_fixture.__wrapped__(monkeypatch)


@pytest.fixture
def wire(monkeypatch):
    return command_fixture.__wrapped__(monkeypatch)


def gate_for(rig, wire):
    tree = ast.parse(
        Path("ros2/src/senior_motion_control/senior_motion_control/safety_node.py").read_text()
    )
    cls = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name == "SafetyGate")
    methods = [
        x
        for x in cls.body
        if isinstance(x, ast.FunctionDef) and x.name in {"_on_command", "_on_task_terminal"}
    ]
    module = ast.Module(
        body=[ast.ClassDef(name="Gate", bases=[], keywords=[], body=methods, decorator_list=[])],
        type_ignores=[],
    )
    scope = dict(
        time=N(monotonic=lambda: rig.clock.steady), Velocity=Velocity, DriveFeedbackMessage=N
    )
    exec(compile(ast.fix_missing_locations(module), "<actual retirement gate>", "exec"), scope)
    gate = scope["Gate"]()
    gate._motion_core = rig.core
    gate._final_fence = FinalCommandFence()
    gate._final_fence.set_publisher(b"command")
    gate.get_parameter = lambda _: N(value="route-sim")
    gate._replace_drive_feedback = lambda *_: None
    node = wire.node
    node._retirement_core = (b"runtime", rig.core.boot)
    node._monotonic_clock = N(monotonic=lambda: rig.clock.steady)
    node.get_parameter = lambda _: N(value="route-sim")
    gate._drive_feedback_publisher = N(publish=lambda packet: node._on_drive_feedback(packet, {}))

    def publish(frame):
        wire.frames.append(frame)
        if frame.close_reason in {"task_terminal", "task_settled"}:
            gate._on_task_terminal(frame)
        else:
            gate._on_command(frame, b"command")

    node._final_publisher = N(publish=publish)
    return gate


@pytest.mark.parametrize("reason", ["navigation_source_inactive", "qualification_lost"])
def test_original_expiry_fault_retirement_then_cancel_uses_unchanged_payload(rig, wire, reason):
    start(rig, 0.5)
    gate_for(rig, wire)
    node = wire.node
    node._navigation_source.execution_id = "route"
    node.clear_navigation_source("route", reason=reason)
    original = node._retirements["route"]
    assert original.acknowledged and original.task_epoch == 1
    revision = rig.core.machine.state.fault_revision
    node.request_terminal_release("route", 1, "cancel", "canceled", dispatch_sequence=2)
    assert node.terminal_release_status("cancel") is True
    assert node._retirements["route"] == original
    assert rig.core.machine.state.task_id is None
    assert rig.core.machine.state.fault_revision == revision


def test_real_sender_gate_core_269_settlements_and_old_scope_cannot_rehandshake(rig, wire):
    start(rig, 0.5)
    gate_for(rig, wire)
    node = wire.node
    template = node._navigation_source
    old = None
    for i in range(1, 270):
        execution = "route" if i == 1 else f"route-{i}"
        if i > 1:
            assert rig.core.command(intent(rig, execution, generation=node._generation + 1))
            node._generation += 1
        node._navigation_source = replace(
            template, execution_id=execution, task_epoch=1, received_steady=rig.clock.steady
        )
        node.clear_navigation_source(execution, reason="completed")
        assert node._retirements[execution].acknowledged
        node.request_terminal_release(
            execution, 1, f"terminal-{i}", "succeeded", dispatch_sequence=i * 3 + 1
        )
        assert node.terminal_release_status(f"terminal-{i}") is True
        if i == 1:
            old = wire.frames[-1]
        assert node.collect_terminal(execution, 1, f"collect-{i}", i * 3 + 2, i * 3 + 1) is None
        assert node.collect_terminal(execution, 1, f"collect-{i}", i * 3 + 2, i * 3 + 1) is True
        assert not node._terminal_requests and len(node._terminal_collections) == 1
        assert not node._retirements and not rig.core._task_retirements
    marker = object()
    node._manual_target = marker
    node._generation += 1
    before = node._generation, node._barrier_frame, len(wire.frames)
    assert (
        node.request_terminal_release("route", 1, "terminal-1", "succeeded", dispatch_sequence=4)
        is False
    )
    assert not rig.core.release_terminal(old)
    node._on_drive_feedback(
        N(
            robot_id="route-sim",
            owner_boot_id="command-owner",
            session_id="route",
            request_id=old.request_id,
            sequence=old.generation,
            connection_epoch=1,
            status="task_retired",
            reason=rig.core.boot,
        ),
        {},
    )
    assert (node._generation, node._barrier_frame, len(wire.frames)) == before
    assert node._manual_target is marker


@pytest.mark.asyncio
async def test_adapter_269_durable_collections_compact_and_reject_old_dispatch():
    from senior_robot.agent_adapter import AdapterCore

    calls = []

    class Routes:
        async def release_terminal(self, *args, **kwargs):
            calls.append(args)
            return True

        def collect_terminal(self, *args):
            return True

    adapter = AdapterCore(
        robot_id="robot",
        submit=lambda *_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        routes=Routes(),
    )
    for i in range(1, 270):
        fields = dict(
            execution_id=f"route-{i}",
            task_epoch=1,
            terminal_outcome="succeeded",
            effect_id=f"release-{i}",
            effect_sequence=i * 2,
        )
        reply = await adapter.handle(dict(type="navigation.task_terminal", **fields))
        assert reply["identity_released"]
        reply = await adapter.handle(
            dict(
                type="navigation.task_settled",
                **{
                    **fields,
                    "effect_id": f"collect-{i}",
                    "effect_sequence": i * 2 + 1,
                    "settled_through": i * 2,
                },
            )
        )
        assert reply["collected"] and len(adapter._task_effects) == 1
        assert len(adapter._task_effect_sequences) == 1
    count = len(calls)
    reply = await adapter.handle(
        dict(
            type="navigation.task_terminal",
            execution_id="route-1",
            effect_id="release-1",
            task_epoch=1,
            terminal_outcome="succeeded",
            effect_sequence=2,
        )
    )
    assert reply["error"] == "retired_scope" and len(calls) == count


@pytest.mark.asyncio
async def test_full_inflight_adapter_book_refuses_plan_before_reserving_route():
    from senior_robot.agent_adapter import AdapterCore

    started = []

    class Routes:
        async def start(self, *args, **kwargs):
            started.append(args)
            return True

    adapter = AdapterCore(
        robot_id="robot",
        submit=lambda *_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        routes=Routes(),
    )
    adapter._task_effects = {str(i): ("revoke", f"other-{i}", 1) for i in range(256)}
    reply = await adapter.handle(
        dict(
            type="navigation.task_start",
            execution_id="new",
            effect_sequence=300,
            plan=dict(
                execution_id="new",
                map_id="map",
                reference_revision=1,
                map_generation="map",
                poses=[[0.0, 0.0, 0.0]],
                task_epoch=1,
                action_ids=["a"],
                effect_id="plan-new",
            ),
        )
    )
    assert reply.get("error") == "unavailable" and not started
    assert len(adapter._task_effects) == 256


def test_settled_scope_behind_unresolved_frontier_never_reissues_identity(rig, wire):
    start(rig, 0.5)
    gate_for(rig, wire)
    node = wire.node
    node._navigation_source.execution_id = "route"
    node.clear_navigation_source("route", reason="completed")
    node.request_terminal_release("route", 1, "release", "succeeded", dispatch_sequence=10)
    assert node.terminal_release_status("release") is True
    node.collect_terminal("route", 1, "collect", 11, 3)
    assert node.collect_terminal("route", 1, "collect", 11, 3) is True
    marker = object()
    node._manual_target = marker
    count = len(wire.frames)
    assert (
        node.request_terminal_release("route", 1, "release", "succeeded", dispatch_sequence=10)
        is False
    )
    node.retire_navigation_source("route", 1)
    assert len(wire.frames) == count and node._manual_target is marker
    assert len(node._terminal_collections) == 1  # Bounded sparse settlement, not discarded gap.


@pytest.mark.asyncio
async def test_adapter_sparse_settlement_preserves_unresolved_gap_until_frontier_advances():
    from senior_robot.agent_adapter import AdapterCore

    releases = []

    class Routes:
        async def release_terminal(self, *args, **kwargs):
            releases.append(args)
            return True

        def collect_terminal(self, *args):
            return True

    adapter = AdapterCore(
        robot_id="robot",
        submit=lambda *_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        routes=Routes(),
    )
    adapter._task_effects["gap"] = ("revoke", "pending", 1)
    adapter._task_effect_sequences["gap"] = 1
    fields = dict(
        execution_id="route",
        task_epoch=1,
        terminal_outcome="succeeded",
        effect_id="release",
        effect_sequence=10,
    )
    assert (await adapter.handle(dict(type="navigation.task_terminal", **fields)))["accepted"]
    assert (
        await adapter.handle(
            dict(
                type="navigation.task_settled",
                **{**fields, "effect_id": "collect", "effect_sequence": 11, "settled_through": 0},
            )
        )
    )["collected"]
    assert adapter._task_effects["gap"] == ("revoke", "pending", 1)
    assert len(adapter._task_effects) == 2 and adapter._settled_sequence == 0
    count = len(releases)
    assert (await adapter.handle(dict(type="navigation.task_terminal", **fields)))[
        "error"
    ] == "retired_scope"
    assert len(releases) == count
    # A later live effect carries the committed collection frontier. This must
    # compact before admission, without skipping an outstanding cleanup itself.
    reply = await adapter.handle(
        dict(
            type="navigation.task_terminal",
            execution_id="next",
            task_epoch=1,
            effect_id="next-release",
            terminal_outcome="succeeded",
            effect_sequence=20,
            settled_through=19,
        )
    )
    assert reply["accepted"]
    assert list(adapter._task_effects) == ["next-release"]


@pytest.mark.asyncio
async def test_old_collection_ack_cannot_advance_newer_pending_frontier():
    from senior_robot.agent_adapter import AdapterCore

    adapter = AdapterCore(
        robot_id="robot",
        submit=lambda *_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        routes=N(),
    )
    adapter._settled_sequence = 20
    adapter._task_effects["live"] = ("revoke", "live", 2)
    adapter._task_effect_sequences["live"] = 21
    reply = await adapter.handle(
        dict(
            type="navigation.task_settled",
            execution_id="live",
            effect_id="old",
            task_epoch=100,
            effect_sequence=10,
            settled_through=100,
        )
    )
    assert reply["collected"]
    assert adapter._settled_sequence == 20
    assert adapter._task_effect_sequences == {"live": 21}
    assert adapter._task_effects == {"live": ("revoke", "live", 2)}


def test_new_durable_frontier_frees_acknowledged_collection_slots_before_capacity_check(wire):
    import sys

    node = wire.node
    record = sys.modules["command_clock_under_test"]._TerminalCollection
    node._terminal_collections = {str(i): record(str(i), 1, i, 0, True) for i in range(1, 257)}
    assert node.collect_terminal("settled", 1, "collect-new", 300, 299) is None
    assert len(node._terminal_collections) == 1
    assert wire.frames[-1].close_reason == "task_settled"


@pytest.mark.asyncio
async def test_new_plan_frontier_compacts_settled_markers_before_admission_at_capacity():
    from senior_robot.agent_adapter import AdapterCore

    started = []

    class Routes:
        async def start(self, *args, **kwargs):
            started.append(args)
            return True

    adapter = AdapterCore(
        robot_id="robot",
        submit=lambda *_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        routes=Routes(),
    )
    adapter._task_effects = {str(i): ("terminal_settled", str(i), 1) for i in range(1, 257)}
    adapter._task_effect_sequences = {str(i): i for i in range(1, 257)}
    reply = await adapter.handle(
        dict(
            type="navigation.task_start",
            execution_id="new",
            effect_sequence=300,
            settled_through=299,
            plan=dict(
                execution_id="new",
                map_id="map",
                reference_revision=1,
                map_generation="map",
                poses=[[0.0, 0.0, 0.0]],
                task_epoch=1,
                action_ids=["a"],
                effect_id="plan-new",
            ),
        )
    )
    assert reply.get("accepted") is True and len(started) == 1
    assert len(adapter._task_effects) == 1


@pytest.mark.asyncio
async def test_read_only_route_status_remains_available_after_collection_frontier():
    from senior_robot.agent_adapter import AdapterCore

    adapter = AdapterCore(
        robot_id="robot",
        submit=lambda *_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        routes=N(
            status=lambda execution: N(
                model_dump=lambda: {"execution_id": execution, "status": "running"}
            )
        ),
    )
    adapter._settled_sequence = 20
    reply = await adapter.handle(dict(type="navigation.task_status", execution_id="live"))
    assert reply["ok"] and reply["progress"]["status"] == "running"
