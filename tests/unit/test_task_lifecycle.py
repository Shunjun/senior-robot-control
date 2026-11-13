"""Vehicle-owned task transitions, with no runner or ROS side effects."""

from dataclasses import FrozenInstanceError, replace

import pytest

from senior_control.tasks.lifecycle import (
    Action,
    Command,
    Context,
    Effect,
    Event,
    EventKind,
    Lifecycle,
    Receipt,
    Transition,
    acknowledge,
    command,
    event,
    query,
)

CONTEXT = Context(connected=True, motion_allowed=True, pose_ref="pose-now", map_ref="map-now")
SUBMIT = Command("request", "submit", "task", ("a", "b"))


def accepted() -> Transition:
    return command(Lifecycle(), SUBMIT, CONTEXT)


def ack(
    state: Lifecycle, effect: Effect, *, ok: bool = True, context: Context = CONTEXT
) -> Transition:
    return acknowledge(state, effect.effect_id, effect.epoch, ok=ok, context=context)


def running() -> Lifecycle:
    result = accepted()
    return ack(result.state, result.effects[0]).state


def control(state: Lifecycle, action: Action, request: str = "control") -> Transition:
    return command(state, Command(request, action, "task"), CONTEXT)


def test_acceptance_is_immutable_and_start_requires_ack() -> None:
    result = accepted()
    assert result.state.phase == "starting"
    assert result.receipt == Receipt("request", "task", True, "accepted", 1)
    assert result.effects[0].kind == "plan"
    assert result.effects[0].remaining == ("a", "b")
    assert ack(result.state, result.effects[0]).state.phase == "running"
    with pytest.raises(FrozenInstanceError):
        result.state.phase = "running"  # type: ignore[misc]


def test_offline_completion_reconnect_queries_same_identity_without_reexecution() -> None:
    state = running()
    offline = event(state, Event("link_changed"), replace(CONTEXT, connected=False))
    assert offline.state == state and offline.effects == ()
    state = event(state, Event("action_completed", state.epoch, "a"), CONTEXT).state
    state = event(state, Event("action_completed", state.epoch, "b"), CONTEXT).state
    state = event(state, Event("completed", state.epoch), CONTEXT).state
    assert state.phase == "succeeded" and state.completed == ("a", "b")
    replay = command(state, SUBMIT, replace(CONTEXT, connected=False))
    assert replay.state == state and replay.effects == ()
    assert replay.receipt == query(state, "request") == accepted().receipt
    assert query(state, "unknown") is None
    assert event(state, Event("link_changed"), CONTEXT).state == state


def test_same_request_different_payload_conflicts() -> None:
    state = accepted().state
    result = command(state, replace(SUBMIT, actions=("other",)), CONTEXT)
    assert result.receipt is not None and result.receipt.reason == "request_conflict"
    assert result.state == state and not result.effects


@pytest.mark.parametrize("scheduled", [False, True])
def test_offline_never_starts_new_remote_or_scheduled_task(scheduled: bool) -> None:
    result = command(
        Lifecycle(), replace(SUBMIT, scheduled=scheduled), replace(CONTEXT, connected=False)
    )
    assert result.state.phase == "empty" and not result.effects
    assert result.receipt is not None and result.receipt.reason == "offline"


def test_takeover_requires_revoke_boundary_then_explicit_resume_from_current_reference() -> None:
    state = running()
    state = event(state, Event("action_completed", state.epoch, "a"), CONTEXT).state
    takeover = control(state, "takeover")
    assert takeover.state.phase == "revoking"
    assert [effect.kind for effect in takeover.effects] == ["revoke"]
    assert control(takeover.state, "resume", "too-soon").effects == ()
    revoked = ack(takeover.state, takeover.effects[0])
    assert revoked.state.phase == "paused"
    assert [effect.kind for effect in revoked.effects] == ["manual_ready"]
    assert revoked.state.completed == ("a",)
    kinds: tuple[EventKind, ...] = ("manual_released", "link_changed", "feedback_recovered")
    for kind in kinds:
        assert event(revoked.state, Event(kind), CONTEXT).state == revoked.state
    resumed = control(revoked.state, "resume", "resume")
    assert resumed.state.epoch > state.epoch
    assert resumed.effects[0].remaining == ("b",)
    assert resumed.effects[0].pose_ref == "pose-now"
    assert resumed.effects[0].map_ref == "map-now"
    assert resumed.state.task_id == "task"


@pytest.mark.parametrize("action", ["pause", "cancel", "takeover"])
def test_duplicate_command_and_ack_do_not_repeat_effects(action: Action) -> None:
    stopped = control(running(), action)
    replay = control(stopped.state, action)
    assert replay.effects == () and replay.receipt == stopped.receipt
    settled = ack(stopped.state, stopped.effects[0])
    duplicate = ack(settled.state, stopped.effects[0])
    assert duplicate.state == settled.state and duplicate.effects == ()


@pytest.mark.parametrize("action", ["pause", "cancel"])
def test_explicit_decision_supersedes_pending_takeover_and_feedback_recovery(
    action: Action,
) -> None:
    pending = control(running(), "takeover")
    decided = control(pending.state, action, "decision")
    assert decided.effects == ()
    settled = ack(decided.state, pending.effects[0])
    assert settled.effects == ()
    assert settled.state.phase == ("canceled" if action == "cancel" else "paused")
    recovered = event(settled.state, Event("feedback_recovered"), CONTEXT)
    assert recovered.effects == () and recovered.state == settled.state


@pytest.mark.parametrize("kind", ["fault", "estop"])
def test_execution_fault_and_estop_recovery_leave_task_paused(kind: EventKind) -> None:
    pending = event(running(), Event(kind), CONTEXT)
    assert pending.effects[0].kind == "revoke"
    paused = ack(pending.state, pending.effects[0]).state
    assert event(paused, Event("feedback_recovered"), CONTEXT).state == paused
    assert control(paused, "resume").effects[0].kind == "plan"


def test_feedback_recovery_replans_offline_but_not_before_boundary_ack() -> None:
    state = running()
    pending = event(state, Event("feedback_lost"), CONTEXT)
    paused = ack(pending.state, pending.effects[0]).state
    result = event(paused, Event("feedback_recovered"), replace(CONTEXT, connected=False))
    assert result.state.task_id == state.task_id and result.state.epoch > state.epoch
    assert result.effects[0].kind == "plan"


@pytest.mark.parametrize("action", ["pause", "cancel", "takeover"])
def test_explicit_stop_prevents_feedback_automatic_resume(action: Action) -> None:
    pending = event(running(), Event("feedback_lost"), CONTEXT)
    stopped = control(pending.state, action)
    paused = ack(stopped.state, pending.effects[0]).state
    assert event(paused, Event("feedback_recovered"), CONTEXT).effects == ()


def test_revoke_failed_wrong_or_late_ack_never_grants_manual() -> None:
    pending = control(running(), "takeover")
    effect = pending.effects[0]
    for effect_id, epoch, ok in [
        (effect.effect_id, effect.epoch, False),
        ("wrong", effect.epoch, True),
        (effect.effect_id, effect.epoch + 1, True),
    ]:
        result = acknowledge(pending.state, effect_id, epoch, ok=ok, context=CONTEXT)
        assert result.state == pending.state and not result.effects
    paused = ack(pending.state, effect).state
    resumed = control(paused, "resume", "resume")
    assert ack(resumed.state, effect).state == resumed.state


def test_plan_ack_after_takeover_does_not_reopen_navigation() -> None:
    started = accepted()
    takeover = control(started.state, "takeover")
    assert ack(takeover.state, started.effects[0]).state == takeover.state
    assert ack(takeover.state, takeover.effects[0]).state.phase == "paused"


def test_unknown_start_failure_requires_confirmed_revoke() -> None:
    started = accepted()
    failed = ack(started.state, started.effects[0], ok=False)
    assert failed.state.phase == "revoking"
    assert failed.effects[0].kind == "revoke"
    assert ack(failed.state, failed.effects[0]).state.phase == "paused"


@pytest.mark.parametrize("during_revoke", [False, True])
def test_cancel_and_completion_race_does_not_resurrect(during_revoke: bool) -> None:
    state = running()
    state = event(state, Event("action_completed", state.epoch, "a"), CONTEXT).state
    state = event(state, Event("action_completed", state.epoch, "b"), CONTEXT).state
    if during_revoke:
        cancel = control(state, "cancel")
        state = event(cancel.state, Event("completed", state.epoch), CONTEXT).state
        state = ack(state, cancel.effects[0]).state
        assert state.phase == "canceled"
    else:
        state = event(state, Event("completed", state.epoch), CONTEXT).state
        assert control(state, "cancel").state.phase == "succeeded"
    actions: tuple[Action, ...] = ("resume", "takeover", "pause", "cancel")
    for action in actions:
        result = control(state, action, action)
        assert result.state.phase == state.phase and result.effects == ()
        assert result.receipt is not None and not result.receipt.accepted


def test_progress_is_ordered_deduplicated_and_old_epoch_cannot_complete_new_run() -> None:
    state = running()
    assert event(state, Event("action_completed", state.epoch, "b"), CONTEXT).state == state
    assert event(state, Event("completed", state.epoch), CONTEXT).state == state
    first = event(state, Event("action_completed", state.epoch, "a"), CONTEXT).state
    assert event(first, Event("action_completed", state.epoch, "a"), CONTEXT).state == first
    pending = control(first, "takeover")
    paused = ack(pending.state, pending.effects[0]).state
    resumed = control(paused, "resume", "resume")
    new = ack(resumed.state, resumed.effects[0]).state
    kinds: tuple[EventKind, ...] = ("action_completed", "completed")
    for kind in kinds:
        assert event(new, Event(kind, first.epoch, "b"), CONTEXT).state == new
    assert new.completed == ("a",) and new.remaining == ("b",)


@pytest.mark.parametrize("initial", ["starting", "running", "revoking", "paused"])
def test_restart_never_resumes_and_fences_old_callbacks(initial: str) -> None:
    started = accepted()
    state = started.state
    if initial != "starting":
        state = ack(state, started.effects[0]).state
    if initial in {"revoking", "paused"}:
        stop = control(state, "takeover")
        state = stop.state if initial == "revoking" else ack(stop.state, stop.effects[0]).state
    restarted = event(state, Event("restart"), CONTEXT)
    assert all(effect.kind != "plan" for effect in restarted.effects)
    assert restarted.state.epoch > state.epoch
    assert (
        event(restarted.state, Event("action_completed", state.epoch, "a"), CONTEXT).state
        == restarted.state
    )
    if restarted.effects:
        restarted = ack(restarted.state, restarted.effects[0])
    assert restarted.state.phase == "paused"
    assert event(restarted.state, Event("feedback_recovered"), CONTEXT).effects == ()


def test_restart_retains_cancel_obligation_without_granting_manual() -> None:
    cancel = control(running(), "cancel")
    restarted = event(cancel.state, Event("restart"), CONTEXT)
    settled = ack(restarted.state, restarted.effects[0])
    assert settled.state.phase == "canceled" and settled.effects == ()


def test_unhealthy_or_missing_current_reference_never_replans() -> None:
    pending = control(running(), "pause")
    paused = ack(pending.state, pending.effects[0]).state
    for context in (
        replace(CONTEXT, motion_allowed=False),
        replace(CONTEXT, pose_ref=None),
        replace(CONTEXT, map_ref=None),
    ):
        result = command(paused, Command("resume", "resume", "task"), context)
        assert result.effects == () and result.state.phase == "paused"


def test_ordinary_obstacle_does_not_cancel_or_change_task_epoch() -> None:
    state = running()
    assert event(state, Event("obstacle"), CONTEXT) == Transition(state)


def test_second_identity_cannot_replace_owned_task() -> None:
    state = running()
    result = command(state, replace(SUBMIT, request_id="other", task_id="other"), CONTEXT)
    assert result.receipt is not None and result.receipt.reason == "already_owned"
    assert result.state.task_id == state.task_id and result.effects == ()


def test_committed_action_during_revoke_is_kept_and_never_replanned() -> None:
    pending = control(running(), "takeover")
    progressed = event(pending.state, Event("action_completed", pending.state.epoch, "a"), CONTEXT)
    assert progressed.state.phase == "revoking" and progressed.state.completed == ("a",)
    paused = ack(progressed.state, pending.effects[0]).state
    resumed = control(paused, "resume", "resume")
    assert resumed.effects[0].remaining == ("b",)


def test_last_action_during_takeover_settles_success_only_after_revocation() -> None:
    pending = control(running(), "takeover")
    state = pending.state
    for action in ("a", "b"):
        state = event(state, Event("action_completed", state.epoch, action), CONTEXT).state
    assert state.phase == "revoking"
    settled = ack(state, pending.effects[0])
    assert settled.state.phase == "succeeded"
    assert settled.effects[0].kind == "manual_ready"
    assert control(settled.state, "resume", "resume").effects == ()


def test_new_takeover_request_while_already_paused_does_not_grant_twice() -> None:
    pending = control(running(), "takeover")
    paused = ack(pending.state, pending.effects[0]).state
    assert control(paused, "takeover", "new-request").effects == ()


def test_old_revoke_ack_after_restart_cannot_cross_new_boundary() -> None:
    pending = control(running(), "takeover")
    restarted = event(pending.state, Event("restart"), CONTEXT)
    assert ack(restarted.state, pending.effects[0]).state == restarted.state
    settled = ack(restarted.state, restarted.effects[0])
    assert settled.state.phase == "paused" and not settled.effects


def test_unrecoverable_task_failure_requires_revoke_and_is_terminal() -> None:
    state = running()
    failed = event(state, Event("failed", state.epoch), CONTEXT)
    assert failed.state.phase == "revoking" and failed.effects[0].kind == "revoke"
    settled = ack(failed.state, failed.effects[0]).state
    assert settled.phase == "failed"
    assert control(settled, "resume").effects == ()
    assert event(settled, Event("feedback_recovered"), CONTEXT).state == settled
    assert event(settled, Event("completed", state.epoch), CONTEXT).state == settled


def test_duplicate_resume_keeps_original_receipt_and_plan_epoch() -> None:
    stopped = control(running(), "pause")
    paused = ack(stopped.state, stopped.effects[0]).state
    cmd = Command("resume", "resume", "task")
    resumed = command(paused, cmd, CONTEXT)
    replay = command(resumed.state, cmd, replace(CONTEXT, pose_ref="newer-pose"))
    assert replay.state == resumed.state and replay.receipt == resumed.receipt
    assert not replay.effects


def test_rejected_request_replay_returns_original_result_after_connection_recovers() -> None:
    rejected = command(Lifecycle(), SUBMIT, replace(CONTEXT, connected=False))
    replay = command(rejected.state, SUBMIT, CONTEXT)
    assert replay.receipt == rejected.receipt
    assert replay.state == rejected.state and replay.effects == ()


def test_cancel_cannot_be_weakened_by_pause_takeover_or_feedback() -> None:
    canceled = control(running(), "cancel")
    state = canceled.state
    for action in ("pause", "takeover"):
        state = control(state, action, action).state
        assert state.stopped_for == "cancel"
    state = event(state, Event("feedback_lost"), CONTEXT).state
    state = event(state, Event("feedback_recovered"), CONTEXT).state
    settled = ack(state, canceled.effects[0])
    assert settled.state.phase == "canceled" and settled.effects == ()


def test_confirmed_completion_before_plan_ack_does_not_leave_or_reopen_pending_start() -> None:
    started = accepted()
    state = started.state
    for action in ("a", "b"):
        state = event(state, Event("action_completed", state.epoch, action), CONTEXT).state
    state = event(state, Event("completed", state.epoch), CONTEXT).state
    assert state.phase == "succeeded" and state.pending is None
    assert ack(state, started.effects[0]).state == state


@pytest.mark.parametrize("offline", [False, True])
def test_recovery_edge_before_revoke_ack_replans_once_with_latest_reference(offline: bool) -> None:
    lost = event(running(), Event("feedback_lost"), CONTEXT)
    recovered = event(lost.state, Event("feedback_recovered"), CONTEXT)
    assert recovered.state.phase == "revoking" and recovered.effects == ()
    latest = replace(CONTEXT, connected=not offline, pose_ref="ack-pose", map_ref="ack-map")
    settled = acknowledge(
        recovered.state, lost.effects[0].effect_id, lost.state.epoch, ok=True, context=latest
    )
    assert settled.state.phase == "starting"
    assert settled.effects[0].kind == "plan"
    assert settled.effects[0].pose_ref == "ack-pose"
    assert settled.effects[0].map_ref == "ack-map"
    assert (
        acknowledge(
            settled.state, lost.effects[0].effect_id, lost.state.epoch, ok=True, context=latest
        ).effects
        == ()
    )


@pytest.mark.parametrize("action", ["pause", "takeover", "cancel"])
def test_failure_settlement_is_not_downgraded_by_control(action: Action) -> None:
    state = running()
    failed = event(state, Event("failed", state.epoch), CONTEXT)
    controlled = control(failed.state, action)
    settled = ack(controlled.state, failed.effects[0])
    assert settled.state.phase == ("canceled" if action == "cancel" else "failed")
    assert settled.effects == ()
    assert control(settled.state, "resume", "resume").effects == ()


@pytest.mark.parametrize(
    "kind", ["fault", "estop", "feedback_lost", "feedback_recovered", "restart"]
)
def test_failure_survives_later_health_and_restart_events(kind: EventKind) -> None:
    state = running()
    failed = event(state, Event("failed", state.epoch), CONTEXT)
    later = event(failed.state, Event(kind), CONTEXT)
    assert later.state.phase == "revoking" and later.state.stopped_for == "failure"
    assert later.state.pending is not None
    settled = ack(later.state, later.state.pending)
    assert settled.state.phase == "failed" and settled.effects == ()


@pytest.mark.parametrize("action", ["pause", "takeover", "cancel"])
def test_failure_arriving_after_control_preserves_terminal_priority(action: Action) -> None:
    stopped = control(running(), action)
    failed = event(stopped.state, Event("failed", stopped.state.epoch), CONTEXT)
    settled = ack(failed.state, stopped.effects[0])
    assert settled.state.phase == ("canceled" if action == "cancel" else "failed")
    assert settled.effects == ()


def test_recovery_then_feedback_loss_before_ack_cannot_restart() -> None:
    lost = event(running(), Event("feedback_lost"), CONTEXT)
    recovered = event(lost.state, Event("feedback_recovered"), CONTEXT)
    lost_again = event(recovered.state, Event("feedback_lost"), CONTEXT)
    settled = ack(lost_again.state, lost.effects[0])
    assert settled.state.phase == "paused" and settled.effects == ()


@pytest.mark.parametrize("action", ["pause", "takeover", "cancel"])
def test_explicit_control_clears_early_recovery_intent(action: Action) -> None:
    lost = event(running(), Event("feedback_lost"), CONTEXT)
    recovered = event(lost.state, Event("feedback_recovered"), CONTEXT)
    decided = control(recovered.state, action)
    settled = ack(decided.state, lost.effects[0])
    assert all(effect.kind != "plan" for effect in settled.effects)
    assert not settled.state.recovery_requested
    assert event(settled.state, Event("admission_changed"), CONTEXT).effects == ()
    assert event(settled.state, Event("feedback_recovered"), CONTEXT).effects == ()


@pytest.mark.parametrize("kind", ["fault", "estop", "restart"])
def test_health_stop_and_restart_clear_early_recovery_intent(kind: EventKind) -> None:
    lost = event(running(), Event("feedback_lost"), CONTEXT)
    recovered = event(lost.state, Event("feedback_recovered"), CONTEXT)
    stopped = event(recovered.state, Event(kind), CONTEXT)
    assert stopped.state.pending is not None
    settled = ack(stopped.state, stopped.state.pending)
    assert settled.state.phase == "paused" and settled.effects == ()
    assert not settled.state.recovery_requested
    assert event(settled.state, Event("admission_changed"), CONTEXT).effects == ()
    assert event(settled.state, Event("feedback_recovered"), CONTEXT).effects == ()


def test_failure_keeps_progress_during_revocation_without_claiming_completion_early() -> None:
    state = running()
    failed = event(state, Event("failed", state.epoch), CONTEXT)
    progressed = event(failed.state, Event("action_completed", state.epoch, "a"), CONTEXT)
    stopped = control(progressed.state, "takeover")
    assert stopped.state.phase == "revoking" and stopped.effects == ()
    settled = ack(stopped.state, failed.effects[0])
    assert settled.state.phase == "failed" and settled.state.completed == ("a",)
    assert settled.effects == ()


@pytest.mark.parametrize("missing", ["motion", "pose", "map"])
def test_early_recovery_waits_for_latest_admission_without_repeating_recovery_edge(
    missing: str,
) -> None:
    lost = event(running(), Event("feedback_lost"), CONTEXT)
    recovered = event(lost.state, Event("feedback_recovered"), CONTEXT)
    latest = {
        "motion": replace(CONTEXT, motion_allowed=False),
        "pose": replace(CONTEXT, pose_ref=None),
        "map": replace(CONTEXT, map_ref=None),
    }[missing]
    settled = ack(recovered.state, lost.effects[0], context=latest)
    assert settled.state.phase == "paused" and settled.effects == ()
    fresh = replace(CONTEXT, pose_ref="later-pose", map_ref="later-map", connected=False)
    admitted = event(settled.state, Event("admission_changed"), fresh)
    assert admitted.state.phase == "starting" and admitted.effects[0].kind == "plan"
    assert admitted.effects[0].pose_ref == "later-pose"
    assert admitted.effects[0].map_ref == "later-map"


def test_feedback_loss_clears_recovery_waiting_for_admission() -> None:
    lost = event(running(), Event("feedback_lost"), CONTEXT)
    recovered = event(lost.state, Event("feedback_recovered"), CONTEXT)
    settled = ack(recovered.state, lost.effects[0], context=replace(CONTEXT, motion_allowed=False))
    lost_again = event(settled.state, Event("feedback_lost"), CONTEXT)
    assert event(lost_again.state, Event("admission_changed"), CONTEXT).effects == ()


def test_completed_progress_consumes_early_recovery_without_empty_replan() -> None:
    lost = event(running(), Event("feedback_lost"), CONTEXT)
    recovered = event(lost.state, Event("feedback_recovered"), CONTEXT)
    state = recovered.state
    for action in ("a", "b"):
        state = event(state, Event("action_completed", state.epoch, action), CONTEXT).state
    settled = ack(state, lost.effects[0])
    assert settled.state.phase == "succeeded" and settled.effects == ()
    assert not settled.state.recovery_requested


@pytest.mark.parametrize("kind", ["fault", "estop", "feedback_lost"])
def test_failure_after_health_stop_upgrades_settlement(kind: EventKind) -> None:
    stopped = event(running(), Event(kind), CONTEXT)
    failed = event(stopped.state, Event("failed", stopped.state.epoch), CONTEXT)
    assert failed.state.phase == "revoking" and failed.state.stopped_for == "failure"
    assert failed.effects == ()
    assert ack(failed.state, stopped.effects[0]).state.phase == "failed"


def test_failed_revoke_ack_keeps_early_recovery_until_successful_boundary() -> None:
    lost = event(running(), Event("feedback_lost"), CONTEXT)
    recovered = event(lost.state, Event("feedback_recovered"), CONTEXT)
    unconfirmed = ack(recovered.state, lost.effects[0], ok=False)
    assert unconfirmed.state == recovered.state and unconfirmed.effects == ()
    settled = ack(unconfirmed.state, lost.effects[0])
    assert settled.state.phase == "starting" and settled.effects[0].kind == "plan"
    assert not settled.state.recovery_requested
