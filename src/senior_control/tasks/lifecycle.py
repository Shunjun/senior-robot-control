"""Pure task ownership reducer; one aggregate per execution, no clock or I/O.

The caller serializes events and atomically persists the returned state, receipt,
and effects BEFORE dispatching effects. ``pending`` retains an unresolved navigation
obligation; a timeout is not an acknowledgement. Query it, never blindly resend a
plan. T5 owns motion admission/health/ESTOP; this module owns identity, progress,
and execution epochs. Context references are captured by the local adapter, never
trusted directly from a browser. No connection epoch participates in execution.
"""

from dataclasses import dataclass, replace
from typing import Literal

Action = Literal["submit", "pause", "cancel", "takeover", "resume"]
Phase = Literal[
    "empty", "starting", "running", "revoking", "paused", "succeeded", "canceled", "failed"
]
StopReason = Literal[
    "pause", "cancel", "takeover", "feedback", "fault", "estop", "restart", "failure"
]
EventKind = Literal[
    "link_changed",
    "manual_released",
    "obstacle",
    "feedback_lost",
    "feedback_recovered",
    "admission_changed",
    "fault",
    "estop",
    "restart",
    "action_completed",
    "completed",
    "failed",
]
TERMINAL = frozenset({"succeeded", "canceled", "failed"})
# Settlement precedence, not T5 motion/health precedence. Equal ranks use the
# latest explicit decision; feedback must never erase a stronger stop reason.
STOP_PRIORITY: dict[StopReason, int] = {
    "feedback": 0,
    "takeover": 1,
    "pause": 1,
    "fault": 2,
    "estop": 2,
    "restart": 3,
    "failure": 4,
    "cancel": 5,
}


@dataclass(frozen=True)
class Context:
    """Current T5 admission plus qualified current pose/map references."""

    connected: bool
    motion_allowed: bool
    pose_ref: str | None = None
    map_ref: str | None = None


@dataclass(frozen=True)
class Command:
    request_id: str
    action: Action
    task_id: str
    actions: tuple[str, ...] = ()
    scheduled: bool = False

    def __post_init__(self) -> None:
        if not self.request_id.strip() or not self.task_id.strip():
            raise ValueError("request and task identity required")
        if self.action == "submit":
            if not self.actions or any(not item.strip() for item in self.actions):
                raise ValueError("submission needs action identities")
            if len(set(self.actions)) != len(self.actions):
                raise ValueError("action identities must be unique")
        elif self.actions or self.scheduled:
            raise ValueError("only submission carries a task definition")


@dataclass(frozen=True)
class Receipt:
    """Stable command result, distinct from the task's current execution status."""

    request_id: str
    task_id: str
    accepted: bool
    reason: str
    version: int


@dataclass(frozen=True)
class Effect:
    kind: Literal["plan", "revoke", "manual_ready"]
    effect_id: str
    task_id: str
    epoch: int
    remaining: tuple[str, ...] = ()
    pose_ref: str | None = None
    map_ref: str | None = None


@dataclass(frozen=True)
class Lifecycle:
    """Persist the complete snapshot; version is a store CAS, not a wire sequence.

    Request records are not evicted here: the adapter must retain durable dedup
    tombstones or reject admission at capacity, never silently forget identities.
    """

    version: int = 0
    task_id: str | None = None
    phase: Phase = "empty"
    actions: tuple[str, ...] = ()
    completed: tuple[str, ...] = ()
    epoch: int = 0
    pending: Effect | None = None
    stopped_for: StopReason | None = None
    recovery_requested: bool = False
    requests: tuple[tuple[Command, Receipt], ...] = ()

    @property
    def remaining(self) -> tuple[str, ...]:
        return self.actions[len(self.completed) :]


@dataclass(frozen=True)
class Event:
    kind: EventKind
    epoch: int = 0
    action_id: str | None = None


@dataclass(frozen=True)
class Transition:
    state: Lifecycle
    effects: tuple[Effect, ...] = ()
    receipt: Receipt | None = None


def query(state: Lifecycle, request_id: str) -> Receipt | None:
    """None means unknown, never proof that submission failed or should be retried."""
    return next((receipt for cmd, receipt in state.requests if cmd.request_id == request_id), None)


def command(state: Lifecycle, cmd: Command, context: Context) -> Transition:
    for original, receipt in state.requests:
        if original.request_id == cmd.request_id:
            if original == cmd:
                return Transition(state, receipt=receipt)
            conflict = Receipt(
                cmd.request_id, cmd.task_id, False, "request_conflict", state.version
            )
            return Transition(state, receipt=conflict)
    reason = _reject(state, cmd, context)
    if reason is not None:
        result = Transition(state)
    elif cmd.action == "submit":
        owned = replace(state, task_id=cmd.task_id, actions=cmd.actions)
        result = _plan(owned, context)
    elif cmd.action == "resume":
        result = _plan(state, context)
    else:
        result = _stop(state, cmd.action)
    # One version per serialized durable transition, including rejected receipts.
    version = state.version + 1
    receipt = Receipt(cmd.request_id, cmd.task_id, reason is None, reason or "accepted", version)
    stored = replace(result.state, version=version, requests=state.requests + ((cmd, receipt),))
    return Transition(stored, result.effects, receipt)


def _reject(state: Lifecycle, cmd: Command, context: Context) -> str | None:
    if cmd.action == "submit":
        if not context.connected:
            return "offline"
        if state.task_id is not None:
            return "already_owned"
        return None if _can_plan(context) else "motion_unavailable"
    if state.task_id != cmd.task_id:
        return "unknown_task"
    if state.phase in TERMINAL:
        return "terminal"
    if cmd.action == "resume":
        if not context.connected:
            return "offline"
        if state.phase != "paused" or state.pending is not None:
            return "not_paused"
        return None if _can_plan(context) else "motion_unavailable"
    return None


def _can_plan(context: Context) -> bool:
    return context.motion_allowed and bool(context.pose_ref) and bool(context.map_ref)


def _effect(state: Lifecycle, kind: Literal["plan", "revoke", "manual_ready"]) -> Effect:
    assert state.task_id is not None
    return Effect(
        kind,
        f"{state.task_id}:{state.epoch}:{state.version + 1}:{kind}",
        state.task_id,
        state.epoch,
    )


def _plan(state: Lifecycle, context: Context) -> Transition:
    state = replace(
        state, epoch=state.epoch + 1, phase="starting", stopped_for=None, recovery_requested=False
    )
    effect = replace(
        _effect(state, "plan"),
        remaining=state.remaining,
        pose_ref=context.pose_ref,
        map_ref=context.map_ref,
    )
    return Transition(replace(state, pending=effect), (effect,))


def _settlement_reason(current: StopReason | None, incoming: StopReason) -> StopReason:
    if current is not None and STOP_PRIORITY[current] > STOP_PRIORITY[incoming]:
        return current
    return incoming


def _recover(state: Lifecycle, context: Context) -> Transition:
    if (
        state.phase == "paused"
        and state.stopped_for == "feedback"
        and state.recovery_requested
        and _can_plan(context)
    ):
        return _plan(state, context)
    return Transition(state)


def _stop(state: Lifecycle, reason: StopReason) -> Transition:
    reason = _settlement_reason(state.stopped_for, reason)
    state = replace(state, recovery_requested=False)
    if state.phase == "revoking":
        # Reuse the existing boundary obligation; duplicate stops never send again.
        return Transition(replace(state, stopped_for=reason))
    if state.phase == "paused":
        phase: Phase = "canceled" if reason == "cancel" else "paused"
        changed = replace(state, phase=phase, stopped_for=reason)
        effects = (
            (_effect(changed, "manual_ready"),)
            if reason == "takeover" and state.stopped_for != "takeover"
            else ()
        )
        return Transition(changed, effects)
    effect = _effect(state, "revoke")
    return Transition(
        replace(state, phase="revoking", stopped_for=reason, pending=effect), (effect,)
    )


def acknowledge(
    state: Lifecycle, effect_id: str, epoch: int, *, ok: bool, context: Context
) -> Transition:
    """Ack must prove source-fence AND old-output revocation for a revoke effect.

    Revoke covers this task's sources through effect.epoch (including previous
    epochs after restart). Deliver committed action acknowledgements before the
    boundary ack, and deduplicate physical actions by stable task/action identity.

    A mere ROS cancel request/response is insufficient. Failed/unknown revocation
    retains the obligation. A plan failure also needs revocation: acceptance may
    have occurred before its response was lost. Duplicate/stale acks do nothing.
    Context is sampled at acknowledgement time. Recovery remembers only intent,
    never the earlier health/pose/map snapshot, and cannot bypass this admission.
    """
    effect = state.pending
    if effect is None or effect.effect_id != effect_id or effect.epoch != epoch:
        return Transition(state)
    if effect.kind == "revoke":
        if not ok:
            return Transition(state)
        phase: Phase = (
            "canceled"
            if state.stopped_for == "cancel"
            else "failed"
            if state.stopped_for == "failure"
            else "succeeded"
            if not state.remaining
            else "paused"
        )
        changed = replace(
            state,
            version=state.version + 1,
            phase=phase,
            pending=None,
            recovery_requested=state.recovery_requested if phase == "paused" else False,
        )
        effects = (_effect(changed, "manual_ready"),) if state.stopped_for == "takeover" else ()
        return Transition(changed, effects) if effects else _recover(changed, context)
    if not ok:
        result = _stop(state, "fault")
        return replace(result, state=replace(result.state, version=state.version + 1))
    return Transition(replace(state, version=state.version + 1, phase="running", pending=None))


def event(state: Lifecycle, incoming: Event, context: Context) -> Transition:
    """Trusted local events; links/releases/ordinary obstacles never cancel a task."""
    if state.task_id is None or state.phase in TERMINAL:
        return Transition(state)
    result = _event(state, incoming, context)
    if result.state == state:
        return result
    return replace(result, state=replace(result.state, version=state.version + 1))


def _event(state: Lifecycle, incoming: Event, context: Context) -> Transition:
    if incoming.kind in {"link_changed", "manual_released", "obstacle"}:
        return Transition(state)
    if incoming.kind == "restart":
        # Invalidate all prior callbacks, even if their external goal survived.
        if state.phase == "paused":
            return Transition(
                replace(
                    state, epoch=state.epoch + 1, stopped_for="restart", recovery_requested=False
                )
            )
        fenced = replace(
            state, epoch=state.epoch + 1, phase="starting", pending=None, stopped_for=None
        )
        result = _stop(fenced, _settlement_reason(state.stopped_for, "restart"))
        return result
    if incoming.kind in {"fault", "estop", "feedback_lost"}:
        reason: StopReason = "feedback" if incoming.kind == "feedback_lost" else incoming.kind
        return _stop(state, reason)
    if incoming.kind == "feedback_recovered":
        if state.stopped_for == "feedback" and state.phase in {"paused", "revoking"}:
            return _recover(replace(state, recovery_requested=True), context)
        return Transition(state)
    if incoming.kind == "admission_changed":
        return _recover(state, context)
    if state.phase not in {"starting", "running", "revoking"} or incoming.epoch != state.epoch:
        return Transition(state)
    if incoming.kind == "failed":
        return _stop(state, "failure")
    # Committed actions arriving during revocation remain durable progress.
    # They do not reopen output; callbacks after a new epoch are fenced above.
    if incoming.kind == "action_completed":
        if state.remaining and incoming.action_id == state.remaining[0]:
            return Transition(replace(state, completed=state.completed + (incoming.action_id,)))
    elif (
        incoming.kind == "completed"
        and not state.remaining
        and state.phase in {"starting", "running"}
    ):
        return Transition(replace(state, phase="succeeded", pending=None))
    return Transition(state)
