"""Actual Adapter/RouteService/CommandNode/Gate/core at the earliest-GC capacity gap."""

from dataclasses import replace
from types import SimpleNamespace as N

import pytest
from senior_navigation.navigation_adapter import RouteService
from senior_robot.agent_adapter import AdapterCore
from test_motion_core_exit_deadline import start
from test_task_terminal_release import intent
from test_terminal_release_collection import gate_for
from test_terminal_release_collection import rig as core_fixture
from test_terminal_release_collection import wire as command_fixture


@pytest.fixture
def rig(monkeypatch):
    yield from core_fixture.__wrapped__(monkeypatch)


@pytest.fixture
def wire(monkeypatch):
    return command_fixture.__wrapped__(monkeypatch)


def chain(rig, wire):
    start(rig, 0.5)
    gate = gate_for(rig, wire)
    node = wire.node
    queued = []

    def deliver(packet):
        if packet.status == "task_collected" and packet.request_id == "gc-1":
            queued.append(packet)  # Real core ACK exists; only transport delivery is delayed.
        else:
            node._on_drive_feedback(packet, {})

    gate._drive_feedback_publisher = N(publish=deliver)
    routes = RouteService(
        N(),
        qualified=lambda *_: True,
        source_retired=node.navigation_source_retired,
        retire_source=node.retire_navigation_source,
        release_identity=node.request_terminal_release,
        collect_identity=node.collect_terminal,
        query_collection=node.query_terminal_collection,
    )
    adapter = AdapterCore(
        robot_id="route-sim",
        submit=lambda *_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        routes=routes,
    )
    return N(
        node=node, routes=routes, adapter=adapter, queued=queued, template=node._navigation_source
    )


async def released(rig, state, number):
    node = state.node
    execution = "route" if number == 1 else f"route-{number}"
    if number > 1:
        generation = node._generation + 1
        assert rig.core.command(intent(rig, execution, generation=generation))
        node._generation = generation
    node._navigation_source = replace(
        state.template, execution_id=execution, task_epoch=1, received_steady=rig.clock.steady
    )
    node.clear_navigation_source(execution, reason="completed")
    request = dict(
        type="navigation.task_terminal",
        execution_id=execution,
        task_epoch=1,
        effect_id=f"release-{number}",
        terminal_outcome="succeeded",
        effect_sequence=number * 3,
        settled_through=0 if number == 1 else 3,
    )
    for _ in range(2):
        reply = await state.adapter.handle(request)
        if reply.get("identity_released"):
            break
    assert reply.get("identity_released") and rig.core.machine.state.task_id is None
    return dict(
        type="navigation.task_settled",
        execution_id=execution,
        task_epoch=1,
        effect_id=f"gc-{number}",
        effect_sequence=number * 3 + 1,
        settled_through=3,
    )


@pytest.mark.asyncio
async def test_metadata_scope_is_reserved_before_first_unknown_result(rig, wire):
    state = chain(rig, wire)
    gc = await released(rig, state, 1)
    reply = await state.adapter.handle(gc)
    assert reply.get("accepted") is None and len(state.queued) == 1
    assert state.adapter._task_effects["gc-1"] == ("terminal_settled", "route", 1)
    assert state.adapter._task_effect_sequences["gc-1"] == 4
    assert state.routes._collection_scopes[4] == ("route", 1, "gc-1")
    assert not state.node._terminal_collections["gc-1"].acknowledged
    # An immutable known reservation cannot be repurposed on retry.
    wrong = dict(gc, execution_id="different")
    before = len(wire.frames)
    assert (await state.adapter.handle(wrong)).get("error") == "conflict"
    assert len(wire.frames) == before


@pytest.mark.asyncio
async def test_full_r2_gap_queries_real_old_ack_then_compresses_without_new_slot(rig, wire):
    state = chain(rig, wire)
    first_gc = await released(rig, state, 1)
    assert (await state.adapter.handle(first_gc)).get("accepted") is None
    old_identity = next(f for f in wire.frames if f.close_reason == "task_terminal")
    # Reconstruct the reviewed R2 legacy state: upper owners did not reserve E1's
    # pending GC, while CommandNode retains its actual issued frame and delayed ACK.
    state.adapter._task_effects.pop("gc-1", None)
    state.adapter._task_effect_sequences.pop("gc-1", None)
    state.routes._collection_scopes.pop(4, None)
    for number in range(2, 257):
        gc = await released(rig, state, number)
        assert not (await state.adapter.handle(gc)).get("collected")
        assert (await state.adapter.handle(gc)).get("collected")
        assert len(state.adapter._task_effects) <= 256
        assert len(state.routes._collection_scopes) <= 256
        assert len(state.node._terminal_collections) <= 256
    last_gc = await released(rig, state, 257)
    assert last_gc["effect_sequence"] == 772
    assert len(state.adapter._task_effects) == 256
    assert sum(v[0] == "terminal_settled" for v in state.adapter._task_effects.values()) == 255
    assert state.adapter._task_effects["release-257"][0] == "terminal_release"
    assert len(state.node._terminal_collections) == 256
    assert not (await state.adapter.handle(last_gc)).get("collected")
    assert "gc-257" not in state.node._terminal_collections  # Receiver truly has no new slot.
    # Unrelated fresh metadata cannot consume a slot or fabricate an ACK at capacity.
    before = len(wire.frames)
    unknown = dict(last_gc, execution_id="unknown", effect_id="unknown", effect_sequence=775)
    assert (await state.adapter.handle(unknown)).get("error") == "unavailable"
    assert len(wire.frames) == before and len(state.node._terminal_collections) == 256
    # A newer real core manual target remains active while the old GC ACK is recovered.
    generation = state.node._generation + 1
    assert rig.core.command(intent(rig, "manual", generation=generation, kind="manual"))
    state.node._generation = generation
    marker = object()
    state.node._manual_target = marker
    target = rig.core.target
    unchanged = (state.node._generation, rig.core.output_generation, rig.core.event_sequence)
    state.node._on_drive_feedback(state.queued[0], {})
    # RouteService itself must also query an existing receiver obligation at
    # capacity if its R2-era local reservation is absent.
    assert state.routes.collect_terminal("route", 1, "gc-1", 4, 3) is True
    assert (await state.adapter.handle(first_gc)).get("collected")
    assert (await state.adapter.handle(first_gc)).get("collected")  # Lost upper response retries.
    assert state.adapter._settled_sequence == 3  # ACK is not its durable persistence.
    assert len(state.adapter._task_effects) == 256
    assert len(state.routes._collection_scopes) == 256
    assert len(state.node._terminal_collections) == 256
    # Model the next committed outbox frontier after GC4 is durably acknowledged:
    # all intermediate GC rows are done and GC772 is the only outstanding row.
    advanced = dict(last_gc, settled_through=771)
    assert not (await state.adapter.handle(advanced)).get("collected")
    assert (await state.adapter.handle(advanced)).get("collected")
    assert len(state.adapter._task_effects) == 1
    assert len(state.routes._collection_scopes) == 1
    assert len(state.node._terminal_collections) == 1
    assert not rig.core._task_retirements
    assert not rig.core.release_terminal(old_identity)
    old_live = dict(
        type="navigation.task_terminal",
        execution_id="route",
        task_epoch=1,
        effect_id="release-1",
        terminal_outcome="succeeded",
        effect_sequence=3,
    )
    assert (await state.adapter.handle(old_live)).get("error") == "retired_scope"
    assert (await state.adapter.handle(dict(first_gc, settled_through=999))).get("collected")
    assert state.adapter._settled_sequence == 771
    assert (
        state.node._generation,
        rig.core.output_generation,
        rig.core.event_sequence,
    ) == unchanged
    assert state.node._manual_target is marker and rig.core.target is target


def test_existing_collection_query_refuses_missing_sequence_without_allocating(wire):
    assert wire.node.query_terminal_collection("unknown", 1, "unknown", 0, 0) == (False, None)
    assert not wire.node._terminal_collections and not wire.frames
