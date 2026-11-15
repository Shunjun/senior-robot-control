"""Terminal identity release is separate from output retirement and live authority."""

from types import SimpleNamespace as N

import pytest
from test_motion_core_exit_deadline import rig as core_fixture
from test_motion_core_exit_deadline import start
from test_navigation_completion_handoff import barrier

from senior_control.motion.contracts import MotionMode
from senior_control.tasks import lifecycle as task

CTX = task.Context(True, True, "pose", "map")


@pytest.fixture
def rig(monkeypatch):
    yield from core_fixture.__wrapped__(monkeypatch)


def terminal(rig, *, epoch=1, source="route", generation=3, token="retire"):
    packet = barrier(rig, "task_terminal", source)
    packet.source_epoch, packet.generation, packet.request_id = epoch, generation, token
    return packet


@pytest.mark.parametrize("phase", ["running", "paused"])
def test_cancel_requires_independent_terminal_ack_before_settling(phase):
    state = task.Lifecycle(task_id="route", epoch=1, phase=phase, actions=("a",))
    stopped = task.command(state, task.Command("cancel", "cancel", "route"), CTX)
    if phase == "running":
        assert stopped.effects[0].kind == "revoke"
        stopped = task.acknowledge(
            stopped.state, stopped.effects[0].effect_id, 1, ok=True, context=CTX
        )
    assert stopped.state.phase == "revoking"
    effect = stopped.effects[0]
    assert effect.kind == "terminal_release" and effect.terminal_outcome == "canceled"
    unknown = task.acknowledge(stopped.state, effect.effect_id, 1, ok=False, context=CTX)
    assert unknown.state == stopped.state
    settled = task.acknowledge(stopped.state, effect.effect_id, 1, ok=True, context=CTX)
    assert settled.state.phase == "canceled" and settled.state.pending is None


def test_core_output_retirement_does_not_itself_release_task_identity(rig):
    start(rig, 0.5)
    assert rig.core.command(barrier(rig, "task_done"))
    assert rig.core.machine.state.task_id == "route"
    assert rig.core.release_terminal(terminal(rig))
    assert rig.core.machine.state.task_id is None


def test_core_terminal_without_exact_retirement_proof_is_rejected(rig):
    start(rig, 0.5)
    assert not rig.core.release_terminal(terminal(rig))
    assert rig.core.machine.state.task_id == "route"


@pytest.mark.parametrize(
    "field,value",
    [
        ("source_epoch", 2),
        ("source_id", "other"),
        ("owner_boot_id", "other-owner"),
        ("generation", 4),
        ("request_id", "wrong"),
        ("connection_epoch", 2),
        ("received_steady", -1.0),
        ("source_kind", "manual"),
    ],
)
def test_identity_requires_exact_original_retirement_scope(rig, field, value):
    start(rig, 0.5)
    assert rig.core.command(barrier(rig, "task_revoke"))
    msg = terminal(rig)
    setattr(msg, field, value)
    assert not rig.core.release_terminal(msg)
    assert rig.core.machine.state.task_id == "route"
    assert rig.core.machine.state.fault_revision == 0


def intent(rig, source="next", epoch=1, generation=5, kind="navigation"):
    return N(
        close_reason="intent",
        source_kind=kind,
        source_id=source,
        source_epoch=epoch,
        owner_boot_id="command-owner",
        generation=generation,
        sequence=1,
        received_steady=rig.clock.steady,
        steady_deadline=rig.clock.steady + 0.2,
        manual_linear=1.0 if kind == "manual" else 0.0,
        manual_linear_limit=0.01 if kind == "manual" else 0.0,
        degraded_linear_limit=0.3 if kind == "manual" else 0.0,
        vx=0.01,
        wz=0.0,
    )


@pytest.mark.parametrize("successor,epoch", [("route", 2), ("next", 1)])
def test_unconsumed_old_epoch_rejected_and_duplicate_consumed_ack_cannot_clear_successor(
    rig, successor, epoch
):
    start(rig, 0.5)
    first = barrier(rig, "task_revoke")
    assert rig.core.command(first)
    second = barrier(rig, "task_revoke")
    second.generation, second.request_id = 4, "second-proof"
    assert rig.core.command(second)
    second.close_reason = "task_terminal"
    assert rig.core.release_terminal(second)
    assert rig.core.command(intent(rig, successor, epoch=epoch))
    old = terminal(rig)
    assert not rig.core.release_terminal(old)
    assert rig.core.release_terminal(second)  # ACK only for the previously consumed identity.
    assert rig.core._task_identity == (successor, epoch)
    assert rig.core.machine.state.task_id == successor and rig.core.target.source == "navigation"


def test_terminal_identity_does_not_revoke_current_manual_or_touch_output_frontiers(rig):
    start(rig, 0.5)
    assert rig.core.command(barrier(rig, "task_revoke"))
    assert rig.core.command(intent(rig, "manual-authority", epoch=7, generation=4, kind="manual"))
    before = (
        rig.core.target,
        rig.core.output_generation,
        rig.core._input_floor,
        rig.core.controller.trajectory,
        rig.core.scheduler._flight,
        len(rig.node.output),
    )
    assert rig.core.release_terminal(terminal(rig))
    after = (
        rig.core.target,
        rig.core.output_generation,
        rig.core._input_floor,
        rig.core.controller.trajectory,
        rig.core.scheduler._flight,
        len(rig.node.output),
    )
    assert after == before and rig.core.target is before[0]
    assert rig.core.machine.state.mode is MotionMode.MANUAL
    assert rig.core.machine.state.task_id is None


@pytest.mark.parametrize(
    "event,mode",
    [("execution_fault", MotionMode.EXECUTION_FAULT), ("software_estop", MotionMode.ESTOP)],
)
def test_terminal_identity_preserves_physical_latches(rig, event, mode):
    from senior_control.motion.contracts import MotionEventKind

    start(rig, 0.5)
    assert rig.core.command(barrier(rig, "task_revoke"))
    rig.core._event(MotionEventKind(event), rig.clock.steady)
    revisions = (rig.core.machine.state.fault_revision, rig.core.machine.state.estop_revision)
    assert rig.core.release_terminal(terminal(rig))
    assert rig.core.machine.state.mode is mode
    assert (
        rig.core.machine.state.fault_revision,
        rig.core.machine.state.estop_revision,
    ) == revisions


def test_real_monotonic_rollback_still_refuses_first_identity_release(rig):
    start(rig, 0.5)
    assert rig.core.command(barrier(rig, "task_revoke"))
    packet = terminal(rig)
    rig.clock.steady -= 0.001
    assert not rig.core.release_terminal(packet)
    assert rig.core.machine.state.task_id == "route"
    assert rig.core.machine.state.fault_revision == 1
    assert not rig.core._terminal_released


def test_success_is_also_a_durable_identity_obligation():
    state = task.Lifecycle(
        task_id="route", epoch=1, phase="running", actions=("a",), completed=("a",)
    )
    result = task.event(state, task.Event("completed", 1), CTX)
    assert result.state.phase == "revoking" and result.effects[0].kind == "terminal_release"
    assert result.effects[0].terminal_outcome == "succeeded"


@pytest.mark.asyncio
@pytest.mark.parametrize("previous_phase", ["running", "paused"])
async def test_real_cancel_route_reducer_core_allows_distinct_next_execution(rig, previous_phase):
    import asyncio

    from senior_robot.navigation_adapter import NavigationAdapter, RouteService

    from senior_control.models import OperationResult, OperationStatus
    from senior_control.navigation import RoutePlan

    start(rig, 0.5)
    retired = []

    class Executor:
        async def start(self, _):
            return "goal"

        async def result(self, _):
            return await asyncio.get_running_loop().create_future()

        async def cancel(self, execution):
            return OperationResult(execution, OperationStatus.SUCCEEDED)

    def close(execution):
        assert execution == "route"
        retired.append(rig.core.command(barrier(rig, "task_revoke")))

    # Controlled transport reports only the real core's accepted output retirement.
    service = RouteService(
        NavigationAdapter(Executor()),
        qualified=lambda *_: True,
        clear_source=close,
        source_retired=lambda _: bool(retired) and all(retired),
        release_identity=lambda *_: rig.core.release_terminal(terminal(rig)),
    )
    assert await service.start(
        RoutePlan(
            execution_id="route",
            task_id="definition",
            task_epoch=1,
            effect_id="plan",
            action_ids=["a"],
            map_id="map",
            reference_revision=1,
            map_generation="map",
            poses=[[1.0, 0.0, 0.0]],
        )
    )
    state = task.Lifecycle(task_id="route", epoch=1, phase="running", actions=("a",))
    if previous_phase == "paused":
        paused = task.command(state, task.Command("pause", "pause", "route"), CTX)
        assert await service.cancel("route", task_epoch=1)
        state = task.acknowledge(
            paused.state, paused.effects[0].effect_id, 1, ok=True, context=CTX
        ).state
        assert state.phase == "paused"
    result = task.command(state, task.Command("cancel", "cancel", "route"), CTX)
    if result.effects[0].kind == "revoke":
        assert await service.cancel("route", task_epoch=1)
        assert service.output_retired("route", 1)
        result = task.acknowledge(
            result.state, result.effects[0].effect_id, 1, ok=True, context=CTX
        )
    assert result.state.phase == "revoking" and rig.core.machine.state.task_id == "route"
    effect = result.effects[0]
    assert await service.release_terminal("route", 1, effect.effect_id, effect.terminal_outcome)
    state = task.acknowledge(result.state, effect.effect_id, 1, ok=True, context=CTX).state
    assert state.phase == "canceled" and rig.core.machine.state.task_id is None
    assert rig.core.command(intent(rig))
    assert rig.core.machine.state.task_id == "next"


def test_unknown_retained_task_epoch_cannot_mint_a_release_receipt(rig):
    start(rig, 0.5)
    rig.core._task_identity = None  # Controlled unknown retained-identity checkpoint.
    close = barrier(rig, "task_revoke")
    assert rig.core.command(close)
    assert rig.core.retirement_epoch(close) is None
    assert not rig.core.release_terminal(terminal(rig))
    assert rig.core.machine.state.task_id == "route"


def test_retirement_of_live_reopened_source_is_refused_even_at_same_epoch(rig):
    from senior_control.motion.contracts import MotionTarget

    start(rig, 0.5)
    old_target = rig.core.target
    assert rig.core.command(barrier(rig, "task_revoke"))
    # Adversarial checkpoint: a producer reopened a live source despite its retired epoch.
    rig.core.target = MotionTarget(
        old_target.velocity, old_target.source, old_target.source_id, old_target.deadline_steady
    )
    assert not rig.core.release_terminal(terminal(rig))
    assert rig.core.machine.state.task_id == "route"


def test_clock_rollback_does_not_ack_an_already_absent_identity_as_fresh_release(rig):
    start(rig, 0.5)
    assert rig.core.command(barrier(rig, "task_revoke"))
    assert rig.core.release_terminal(terminal(rig))
    next_close = barrier(rig, "task_revoke")
    next_close.generation, next_close.request_id = 4, "another-boundary"
    assert rig.core.command(next_close)
    next_close.close_reason = "task_terminal"
    rig.clock.steady -= 0.001
    assert not rig.core.release_terminal(next_close)
    assert rig.core.machine.state.fault_revision == 1


def test_full_core_proof_book_still_installs_stop_but_never_issues_unrecorded_ack(rig):
    start(rig, 0.5)
    rig.core._task_retirements = {
        ("command-owner", str(i), i + 1, "old"): (1, rig.clock.steady, 1) for i in range(256)
    }
    close = barrier(rig, "task_revoke")
    assert rig.core.command(close)
    assert rig.core.target.source == "idle"
    assert rig.core.retirement_epoch(close) is None
    assert len(rig.core._task_retirements) == 256
    assert not rig.core.release_terminal(terminal(rig))


@pytest.mark.parametrize("reason", ["expiry", "fault"])
def test_original_expiry_or_fault_can_prove_output_retirement_before_cancel(rig, reason):
    start(rig, 0.5)
    close = barrier(rig, reason)
    assert rig.core.command(close)
    revision = rig.core.machine.state.fault_revision
    assert rig.core.retirement_epoch(close) == 1
    assert rig.core.release_terminal(terminal(rig))
    assert rig.core.machine.state.task_id is None
    assert rig.core.machine.state.fault_revision == revision
    if reason == "fault":
        assert rig.core.machine.state.mode is MotionMode.EXECUTION_FAULT


@pytest.mark.parametrize("incoming,phase", [("cancel", "canceled"), ("failed", "failed")])
def test_outcome_is_immutable_request_snapshot_during_terminal_ack_window(incoming, phase):
    state = task.Lifecycle(
        task_id="route", epoch=1, phase="running", actions=("a",), completed=("a",)
    )
    pending = task.event(state, task.Event("completed", 1), CTX)
    effect = pending.state.pending
    assert effect.terminal_outcome == "succeeded"
    if incoming == "cancel":
        waiting = task.command(pending.state, task.Command("cancel", "cancel", "route"), CTX)
    else:
        waiting = task.event(pending.state, task.Event("failed", 1), CTX)
    assert waiting.state.phase == "revoking" and waiting.state.pending == effect
    assert not waiting.effects
    result = task.acknowledge(waiting.state, effect.effect_id, 1, ok=True, context=CTX)
    assert result.state.phase == phase


def test_settled_identity_collection_allows_more_than_256_completions(rig):
    # Real command/task_done/identity boundaries; this is a controlled owner
    # protocol sequence, not 269 simulated navigation arrivals.
    start(rig, 0.5)
    for i in range(1, 270):
        source = "route" if i == 1 else f"route-{i}"
        if i > 1:
            assert rig.core.command(intent(rig, source, epoch=1, generation=i * 2 + 2))
        close = barrier(rig, "task_done", source)
        close.generation, close.request_id = i * 2 + 3, f"nonce-{i}"
        assert rig.core.command(close)
        identity = N(**vars(close))
        identity.close_reason = "task_terminal"
        assert rig.core.retirement_epoch(close) == 1
        assert rig.core.release_terminal(identity)
        collection = N(**vars(identity))
        collection.close_reason = "task_settled"
        collection.generation = 0
        assert rig.core.collect_terminal(collection)
        assert rig.core.collect_terminal(collection)  # Lost cleanup ACK is retryable.
        assert not rig.core._task_retirements and not rig.core._terminal_released
        assert not rig.core.release_terminal(identity)  # Forgotten proof never recreates authority.


def test_collection_refuses_live_or_unreleased_identity(rig):
    start(rig, 0.5)
    msg = terminal(rig)
    msg.close_reason = "task_settled"
    msg.generation = 0
    assert not rig.core.collect_terminal(msg)
    close = barrier(rig, "task_revoke")
    assert rig.core.command(close)
    assert not rig.core.collect_terminal(msg)
    assert rig.core.retirement_epoch(close) == 1


def test_higher_epoch_pause_resume_compacts_only_superseded_proofs_then_cancel(rig):
    from senior_control.motion.state_machine import RecoveryObservation

    start(rig, 0.5)
    old = None
    for epoch in range(1, 270):
        close = barrier(rig, "task_revoke")
        close.source_epoch, close.connection_epoch = epoch, epoch
        close.generation, close.request_id = epoch * 2 + 1, f"pause-{epoch}"
        assert rig.core.command(close)
        assert rig.core.retirement_epoch(close) == epoch
        if epoch == 1:
            old = N(**vars(close))
            old.close_reason = "task_terminal"
        if epoch == 269:
            identity = N(**vars(close))
            identity.close_reason = "task_terminal"
            assert rig.core.release_terminal(identity)
            collected = N(**vars(close))
            collected.close_reason = "task_settled"
            collected.generation = 0
            assert rig.core.collect_terminal(collected)
            break
        # Explicit independent test observations exercise the original five / .5s
        # resume gate. No output command or synthetic zero is used as body evidence.
        for n in range(5):
            rig.clock.steady += 0.125
            rig.core.machine.observe(
                RecoveryObservation(
                    "independent-test-body",
                    epoch * 5 + n,
                    rig.clock.steady,
                    rig.clock.steady + 0.2,
                    True,
                    True,
                ),
                rig.clock.steady,
            )
        assert rig.core.command(intent(rig, "route", epoch=epoch + 1, generation=epoch * 2 + 2))
        assert not rig.core._task_retirements
        assert not rig.core.release_terminal(old)
    assert not rig.core._task_retirements and rig.core.machine.state.task_id is None
