"""A completed route retires its motion task; pause must retain its identity."""

import asyncio
from types import SimpleNamespace as N

import pytest
from test_motion_core_exit_deadline import rig as exit_rig
from test_motion_core_exit_deadline import start

from senior_control.models import OperationResult, OperationStatus
from senior_control.motion.contracts import MotionMode
from senior_control.navigation import RoutePlan


@pytest.fixture
def rig(monkeypatch):
    yield from exit_rig.__wrapped__(monkeypatch)


def barrier(rig, reason, source="route", kind="navigation"):
    return N(
        robot_id="route-sim",
        owner_boot_id="command-owner",
        session_id=source,
        generation=3,
        sequence=0,
        connection_epoch=1,
        steady_deadline=0.0,
        vx=0.0,
        wz=0.0,
        source_kind=kind,
        source_id=source,
        source_epoch=1,
        received_steady=rig.clock.steady,
        close_reason=reason,
        request_id="retire",
        gesture=0,
    )


def test_completed_motion_task_allows_next_independent_execution(rig):
    start(rig, 0.5)
    assert rig.core.machine.state.task_id == "route"
    done = barrier(rig, "task_done")
    assert rig.core.command(done) and rig.core.command(done)
    assert rig.core.machine.state.task_id is None
    assert rig.core.target.source == "idle"
    assert rig.core.command(
        N(
            close_reason="intent",
            source_kind="navigation",
            source_id="second",
            source_epoch=1,
            owner_boot_id="command-owner",
            generation=4,
            sequence=1,
            received_steady=rig.clock.steady,
            steady_deadline=rig.clock.steady + 0.2,
            vx=0.01,
            wz=0.0,
        )
    )
    assert rig.core.machine.state.mode is MotionMode.TASK
    assert rig.core.machine.state.task_id == "second"


def test_pause_keeps_motion_task_and_does_not_accept_other_execution(rig):
    start(rig, 0.5)
    assert rig.core.command(barrier(rig, "task_revoke"))
    assert rig.core.machine.state.task_id == "route" and rig.core.machine.task_paused
    assert not rig.core.command(
        N(
            close_reason="intent",
            source_kind="navigation",
            source_id="second",
            source_epoch=1,
            owner_boot_id="command-owner",
            generation=4,
            sequence=1,
            received_steady=rig.clock.steady,
            steady_deadline=rig.clock.steady + 0.2,
            vx=0.01,
            wz=0.0,
        )
    )


@pytest.mark.parametrize("source,kind", [("different", "navigation"), ("route", "manual")])
def test_completion_cannot_retire_unrelated_or_non_navigation_identity(rig, source, kind):
    start(rig, 0.5)
    target = rig.core.target
    assert not rig.core.command(barrier(rig, "task_done", source, kind))
    assert rig.core.machine.state.task_id == "route" and rig.core.target is target


@pytest.mark.parametrize("outcome", ["succeeded", "failed", "canceled"])
async def test_only_real_route_success_uses_completion_retirement(outcome):
    from senior_robot.navigation_adapter import NavigationAdapter, RouteService

    result = asyncio.get_running_loop().create_future()
    closed = []

    class Executor:
        async def start(self, goal):
            return "goal"

        async def result(self, execution):
            return await result

        async def cancel(self, execution):
            return OperationResult(execution, OperationStatus.SUCCEEDED)

    service = RouteService(
        NavigationAdapter(Executor()),
        qualified=lambda *_: True,
        clear_source=lambda x: closed.append(("revoke", x)),
        complete_source=lambda x: closed.append(("complete", x)),
        source_retired=lambda _: bool(closed),
    )
    plan = RoutePlan(
        execution_id="task",
        task_id="definition",
        task_epoch=1,
        effect_id="plan",
        action_ids=["0:point"],
        map_id="map",
        reference_revision=1,
        map_generation="map",
        poses=[[1.0, 0.0, 0.0]],
    )
    assert await service.start(plan)
    if outcome == "canceled":
        assert await service.cancel("task", task_epoch=1)
    else:
        result.set_result(outcome == "succeeded")
    for _ in range(30):
        if service.current is None:
            break
        await asyncio.sleep(0.01)
    final = service.status("task")
    assert final.status == outcome and final.controller_output_exited
    if outcome == "succeeded":
        assert closed == [("complete", "task")]
        assert final.completed_action_ids == ["0:point"]
    else:
        assert closed and all(kind == "revoke" for kind, _ in closed)
        assert final.completed_action_ids == []


def test_completion_uses_existing_correlated_retirement_barrier(monkeypatch):
    from test_navigation_small_intent import rig as command_fixture

    node = command_fixture.__wrapped__(monkeypatch).r.node
    node.complete_navigation_source("private-execution")
    assert node._navigation_source is None
    assert node._close_reason == "task_done"
    assert not node.navigation_source_retired("private-execution")
    assert node._retirements["private-execution"][2] is False


def test_completion_never_recovers_latched_physical_fault(rig):
    from senior_control.motion.contracts import MotionEventKind as K

    start(rig, 0.5)
    rig.core._event(K.EXECUTION_FAULT, rig.clock.steady)
    revision = rig.core.machine.state.fault_revision
    assert rig.core.command(barrier(rig, "task_done"))
    assert rig.core.machine.state.mode is MotionMode.EXECUTION_FAULT
    assert rig.core.machine.state.fault_revision == revision
    assert rig.core.machine.state.task_id is None
