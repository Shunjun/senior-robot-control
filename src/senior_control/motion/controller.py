"""Pure closed-loop candidate generator; no actuator or authority side effects."""

import math
from dataclasses import dataclass, field, replace

from senior_control.models import Velocity
from senior_control.motion.constraints import (
    BrakingProof,
    ConstraintEngine,
    ConstraintResult,
    ConstraintSnapshot,
    TickEvaluation,
)
from senior_control.motion.contracts import BrakeKind
from senior_control.motion.source_modes import SourceMode
from senior_control.motion.stillness import stationary
from senior_control.motion.trajectory import Trajectory, TrajectoryStep, scale_wheels


@dataclass(frozen=True)
class Feedback:
    velocity: Velocity
    stamp_steady: float
    deadline_steady: float
    epoch: str
    sequence: int
    current_supported: bool
    basis: str
    # //Compatible: legacy pure callers provide complete, non-unified feedback.
    observed_axes: tuple[bool, bool] = (True, True)
    unified: bool = False

    def fresh(self, now: float) -> bool:
        return (
            self.current_supported
            and bool(self.basis)
            and bool(self.epoch)
            and type(self.sequence) is int
            and self.sequence >= 0
            and all(
                math.isfinite(x)
                for x in (
                    self.velocity.vx,
                    self.velocity.wz,
                    self.stamp_steady,
                    self.deadline_steady,
                )
            )
            and self.stamp_steady <= now < self.deadline_steady
        )


@dataclass(frozen=True)
class ControllerConfig:
    kp: Velocity
    ki: Velocity
    integral_limit: Velocity
    track_width_m: float
    wheel_speed_mps: float
    still_speed_mps: float
    still_yaw_rps: float
    disagreement_mps: float
    basis: str
    synthetic: bool

    def __post_init__(self) -> None:
        numbers = (
            self.kp.vx,
            self.kp.wz,
            self.ki.vx,
            self.ki.wz,
            self.integral_limit.vx,
            self.integral_limit.wz,
            self.track_width_m,
            self.wheel_speed_mps,
            self.still_speed_mps,
            self.still_yaw_rps,
            self.disagreement_mps,
        )
        if (
            not all(math.isfinite(x) and x >= 0 for x in numbers)
            or min(self.track_width_m, self.wheel_speed_mps) <= 0
            or not self.basis
        ):
            raise ValueError("explicit controller capability required")


@dataclass(frozen=True)
class ControlDecision:
    command: Velocity | None
    reason: str
    verified: bool
    emergency: bool
    evidence_identity: tuple[str, ...]
    evidence_deadline: float


@dataclass(frozen=True, eq=False)
class PreparedControl:
    """One controller-owned, single-use candidate; not an output receipt."""

    owner: object = field(repr=False)
    decision: ControlDecision
    context: ConstraintSnapshot
    sampled_now: float
    dt: float
    body: Feedback | None
    wheel: Feedback | None
    step: TrajectoryStep | None
    integral: Velocity
    braking: bool
    brake_deadline: float | None
    fallback: bool = False


def _bounded(v: Velocity, cap: Velocity) -> Velocity:
    def component(x: float, bound: float) -> float:
        return max(-bound, min(bound, x))

    return Velocity(component(v.vx, cap.vx), component(v.wz, cap.wz))


class MotionController:
    """Owns command trajectory and PI state, not source qualification or ESTOP latch.

    Body Feedback must be an explicitly supported CURRENT reference. An old
    BodyMotion interval mean cannot just be relabelled here. Wheel Feedback is
    tracking/slip evidence. Stillness consumes only separate raw wheel/IMU samples.
    """

    def __init__(
        self,
        engine: ConstraintEngine,
        config: ControllerConfig,
        initial_velocity: Velocity,
        initial_acceleration: Velocity,
    ):
        self.engine, self.config = engine, config
        self.trajectory = Trajectory(initial_velocity, initial_acceleration)
        self.integral = Velocity(0, 0)
        self._prepared: PreparedControl | None = None
        self._last_now: float | None = None
        self._context_key: tuple[str, ...] | None = None
        self._last_feedback: dict[str, tuple[str, int, float]] = {}
        self._brake_deadline: float | None = None
        self.measured_still = False
        self._direction = Velocity(
            math.copysign(1, initial_velocity.vx) if initial_velocity.vx else 0,
            math.copysign(1, initial_velocity.wz) if initial_velocity.wz else 0,
        )

    def _admit_feedback(self, role: str, value: Feedback | None, now: float) -> Feedback | None:
        if value is None or not value.fresh(now):
            return None
        old = self._last_feedback.get(role)
        # The unified owner already fences raw provenance. Its surviving source
        # can have an older receipt than an expired contributor. Compare control
        # sampling order here; freshness still uses the original root deadline.
        key = (value.epoch, value.sequence, now if value.unified else value.stamp_steady)
        if old is not None:
            if key[2] < old[2] or (key[0] == old[0] and key[1] < old[1]):
                return None
            if key[0] != old[0]:
                self.integral = Velocity(0, 0)
        self._last_feedback[role] = key
        return value

    def _still(self, wheel, imu, now):
        self.measured_still = stationary(
            wheel,
            imu,
            now,
            linear_limit=self.config.still_speed_mps,
            yaw_limit=self.config.still_yaw_rps,
        )

    def _decision(
        self,
        command: Velocity | None,
        reason: str,
        verified: bool,
        emergency: bool,
        ctx: ConstraintSnapshot,
    ) -> ControlDecision:
        deadline = min(
            ctx.motion.valid_until_steady,
            ctx.perception.deadline_steady,
        )
        return ControlDecision(
            command, reason, verified, emergency, self.engine.identity(ctx), deadline
        )

    def reset_at_stillness(self, wheel, imu, now: float) -> bool:
        """Reset only from the same raw wheel/IMU evidence as the state owner."""
        self._prepared = None
        self._still(wheel, imu, now)
        if not self.measured_still:
            return False
        self.trajectory = Trajectory(Velocity(0, 0), Velocity(0, 0))
        self.integral = Velocity(0, 0)
        self._direction = Velocity(0, 0)
        self._last_now = now
        self._brake_deadline = None
        return True

    def _check_brake(self, proof: BrakingProof, now: float) -> tuple[str | None, float | None]:
        deadline = self._brake_deadline
        if deadline is None:
            if not proof.brake.stop_verified or proof.deadline_steady is None:
                return "unverified_brake", deadline
            deadline = proof.deadline_steady
        if now > deadline and not self.measured_still:
            return "stop_response_failed", deadline
        return None, deadline

    def _raw_pi(self, goal: Velocity, error: Velocity, trial: Velocity) -> Velocity:
        return Velocity(
            goal.vx + self.config.kp.vx * error.vx + trial.vx,
            goal.wz + self.config.kp.wz * error.wz + trial.wz,
        )

    def _pi_target(
        self,
        goal: Velocity,
        reference: Feedback | None,
        cap: Velocity,
        dt: float,
        *,
        integrate: bool,
        slip: bool,
    ) -> tuple[Velocity, Velocity, bool]:
        if goal == Velocity(0, 0):
            return goal, Velocity(0, 0), False
        error = (
            Velocity(goal.vx - reference.velocity.vx, goal.wz - reference.velocity.wz)
            if reference
            else Velocity(0, 0)
        )
        if reference is not None:
            error = Velocity(
                error.vx if reference.observed_axes[0] else 0.0,
                error.wz if reference.observed_axes[1] else 0.0,
            )
        trial = (
            _bounded(
                Velocity(
                    self.integral.vx + self.config.ki.vx * error.vx * dt,
                    self.integral.wz + self.config.ki.wz * error.wz * dt,
                ),
                self.config.integral_limit,
            )
            if integrate and not slip
            else Velocity(0, 0)
        )
        if reference is not None:
            trial = Velocity(
                trial.vx if reference.observed_axes[0] else 0.0,
                trial.wz if reference.observed_axes[1] else 0.0,
            )
        requested = self._raw_pi(goal, error, trial)
        raw = Velocity(
            requested.vx if requested.vx * goal.vx > 0 else 0,
            requested.wz if requested.wz * goal.wz > 0 else 0,
        )
        if slip:
            raw = _bounded(raw, Velocity(abs(goal.vx), abs(goal.wz)))
        raw = scale_wheels(
            _bounded(raw, cap), self.config.track_width_m, self.config.wheel_speed_mps
        )
        return raw, trial, integrate and not slip and raw == requested

    def _goal(
        self,
        target: Velocity,
        ctx: ConstraintSnapshot,
        fresh: ConstraintResult,
        reference: Feedback | None,
    ) -> tuple[Velocity, Velocity, bool, str]:
        reason = "tracking"
        braking = target == Velocity(0, 0) or fresh.permit is None or reference is None
        if reference is None:
            reason = "feedback_braking"
        elif fresh.permit is None:
            reason = fresh.brake.reason if fresh.brake else "constraint_braking"
        elif target == Velocity(0, 0):
            reason = "target_braking"
        reversing = target.vx * self._direction.vx < 0 or target.wz * self._direction.wz < 0
        if reversing and not (
            self.measured_still
            and self.trajectory.velocity == Velocity(0, 0)
            and self.trajectory.acceleration == Velocity(0, 0)
        ):
            braking = True
            reason = "reverse_waits_for_stillness"
        goal = Velocity(0, 0) if braking else target
        cap = (
            Velocity(ctx.envelope.linear_cap_mps, ctx.envelope.yaw_cap_rps)
            if ctx.envelope
            else Velocity(0, 0)
        )
        if fresh.permit is not None and not braking:
            compatible = [
                v for v in fresh.permit.velocities if v.vx * goal.vx >= 0 and v.wz * goal.wz >= 0
            ]
            if not compatible:
                goal = Velocity(0, 0)
                braking = True
                reason = "direction_braking"
            else:
                cap = Velocity(
                    min(cap.vx, max(abs(v.vx) for v in compatible)),
                    min(cap.wz, max(abs(v.wz) for v in compatible)),
                )
        if not braking and (
            abs(self.trajectory.velocity.vx) > cap.vx + 1e-12
            or abs(self.trajectory.velocity.wz) > cap.wz + 1e-12
        ):
            goal = Velocity(0, 0)
            braking = True
            reason = "cap_transition_braking"
        return goal, cap, braking, reason

    def propose_tick(
        self,
        target: Velocity,
        ctx: ConstraintSnapshot,
        permission: ConstraintResult | None,
        body: Feedback | None,
        wheel: Feedback | None,
        now: float,
        *,
        estop: bool = False,
        # //Compatible: old callers without an IMU never gain conditional rearm.
        imu: Feedback | None = None,
        still_wheel: Feedback | None = None,
        evaluation: TickEvaluation | None = None,
    ) -> PreparedControl:
        """Admit observations once; stage command state until actual selection."""
        self._prepared = None
        # IMU uses only velocity.wz: its unused vx is NOT body linear evidence.
        if estop:
            self._still(still_wheel, imu, now)
            if math.isfinite(now) and (self._last_now is None or now >= self._last_now):
                self._last_now = now
            return self._failure(ctx, now, "user_estop_override", command=Velocity(0, 0))
        if self._last_now is None:
            self._last_now = now
            return self._failure(ctx, now, "clock_initialized", keep_integral=True)
        dt = now - self._last_now
        self._last_now = now
        if not math.isfinite(now) or not math.isfinite(dt) or dt <= 0:
            return self._failure(ctx, now, "invalid_tick", emergency=True)
        body = self._admit_feedback("body", body, now)
        wheel = self._admit_feedback("wheel", wheel, now)
        imu = self._admit_feedback("imu", imu, now)
        self._still(still_wheel, imu, now)
        key = (*self.engine.identity(ctx), ctx.mode.value)
        if key != self._context_key:
            self.integral = Velocity(0, 0)
        self._context_key = key
        reference = body
        if reference is None and ctx.mode is SourceMode.WHEEL_IMU_LIMITED:
            reference = wheel
        slip = (
            body is not None
            and not body.unified
            and wheel is not None
            and abs(body.velocity.vx - wheel.velocity.vx) > self.config.disagreement_mps
        )
        if evaluation is not None and not evaluation.matches(self.engine, ctx, now):
            evaluation = None
        stopping = replace(ctx, target=replace(ctx.target, velocity=Velocity(0, 0)))
        stop = evaluation.zero_stop() if evaluation else self.engine.refine(stopping, now)
        if stop.brake is not None and stop.brake.kind is BrakeKind.EMERGENCY:
            return self._failure(ctx, now, stop.brake.reason, emergency=True)
        fresh = (
            evaluation.tighten(permission)
            if evaluation
            else self.engine.tighten(permission, ctx, now)
        )
        goal, cap, braking, reason = self._goal(target, ctx, fresh, reference)
        return self._candidate(
            goal,
            cap,
            braking,
            reason,
            ctx,
            fresh,
            body,
            wheel,
            now,
            dt,
            integrate_body=body is not None,
            slip=slip,
            evaluation=evaluation,
        )

    def _candidate(
        self,
        goal: Velocity,
        cap: Velocity,
        braking: bool,
        reason: str,
        ctx: ConstraintSnapshot,
        fresh: ConstraintResult | None,
        body: Feedback | None,
        wheel: Feedback | None,
        now: float,
        dt: float,
        *,
        integrate_body: bool,
        slip: bool,
        evaluation: TickEvaluation | None = None,
        sampled_now: float | None = None,
        fallback: bool = False,
    ) -> PreparedControl:
        raw, trial, integrate = self._pi_target(
            goal,
            body
            if body is not None
            else wheel
            if ctx.mode is SourceMode.WHEEL_IMU_LIMITED
            else None,
            cap,
            dt,
            integrate=integrate_body,
            slip=slip,
        )
        if slip:
            reason = "inconsistent_feedback_limited"
        try:
            step = self.trajectory.propose(raw, ctx.response, dt)
        except ValueError as exc:
            return self._failure(ctx, now, str(exc), emergency=True)
        candidate = scale_wheels(
            step.velocity, self.config.track_width_m, self.config.wheel_speed_mps
        )
        roundoff = 4 * max(
            math.ulp(step.velocity.vx),
            math.ulp(step.velocity.wz),
            math.ulp(self.config.wheel_speed_mps),
        )
        if (
            max(abs(candidate.vx - step.velocity.vx), abs(candidate.wz - step.velocity.wz))
            > roundoff
        ):
            return self._failure(ctx, now, "wheel_limit_infeasible_step", emergency=True)
        # Normalize at most four ULP of wheel-combination arithmetic, not a
        # physical clamp. Larger changes cannot inherit acceleration/jerk.
        step = replace(step, velocity=candidate)
        permitted = False
        if not braking:
            permitted = (
                evaluation.validate_output(fresh, candidate)
                if evaluation
                else self.engine.validate_output(fresh, ctx, candidate, now)
            )
        # Braking consumes the separate T4 zero-goal STOP proof. validate_output
        # deliberately rejects motion without a permit; it is not a stop proof.
        brake_proof = (
            self.engine.braking_proof(
                ctx, self.trajectory.velocity, self.trajectory.acceleration, candidate, now
            )
            if braking
            else None
        )
        brake_deadline = None
        stop_verified = brake_proof is not None and brake_proof.brake.stop_verified
        if braking:
            if not stop_verified:
                return self._failure(
                    ctx,
                    now,
                    brake_proof.brake.reason if brake_proof else "unverified_brake",
                    emergency=True,
                )
            assert brake_proof is not None
            failure, brake_deadline = self._check_brake(brake_proof, now)
            if failure:
                return self._failure(ctx, now, failure, emergency=True)
        elif not permitted:
            return self._failure(ctx, now, "final_candidate_rejected", emergency=True)
        integral = (
            self.integral
            if goal != Velocity(0, 0) and integrate_body and not slip
            else Velocity(0, 0)
        )
        if integrate:
            integral = Velocity(
                trial.vx
                if candidate.vx == raw.vx or abs(trial.vx) < abs(integral.vx)
                else integral.vx,
                trial.wz
                if candidate.wz == raw.wz or abs(trial.wz) < abs(integral.wz)
                else integral.wz,
            )
        decision = self._decision(candidate, reason, permitted or stop_verified, False, ctx)
        if not braking and fresh is not None and fresh.permit is not None:
            reference = body if body is not None else wheel
            decision = replace(
                decision,
                evidence_deadline=min(
                    decision.evidence_deadline,
                    fresh.permit.deadline_steady,
                    *(
                        (ctx.target.deadline_steady,)
                        if ctx.target.deadline_steady is not None
                        else ()
                    ),
                    reference.deadline_steady if reference else decision.evidence_deadline,
                ),
            )
        prepared = PreparedControl(
            self,
            decision,
            ctx,
            now if sampled_now is None else sampled_now,
            dt,
            body,
            wheel,
            step,
            integral,
            braking,
            brake_deadline,
            fallback,
        )
        self._prepared = prepared
        return prepared

    def _failure(
        self,
        ctx: ConstraintSnapshot,
        now: float,
        reason: str,
        *,
        emergency: bool = False,
        command: Velocity | None = None,
        keep_integral: bool = False,
    ) -> PreparedControl:
        decision = self._decision(command, reason, False, emergency, ctx)
        prepared = PreparedControl(
            self,
            decision,
            ctx,
            now,
            0,
            None,
            None,
            None,
            self.integral if keep_integral else Velocity(0, 0),
            False,
            None,
        )
        self._prepared = prepared
        return prepared

    def _consume(self, prepared: PreparedControl) -> None:
        if prepared.owner is not self or self._prepared is not prepared:
            raise ValueError("stale or foreign prepared control")
        self._prepared = None

    def discard(self, prepared: PreparedControl) -> None:
        if prepared.owner is self and self._prepared is prepared:
            self._prepared = None

    @staticmethod
    def _same_feedback_evidence(current: Feedback | None, original: Feedback | None) -> bool:
        if current is None or original is None:
            return current is original
        if not (current.unified and original.unified):
            return current == original
        # Current-time projection may change velocity, never the original roots
        # represented by this clock/source generation and evidence sequence.
        return (
            current.epoch == original.epoch
            and current.sequence == original.sequence
            and current.observed_axes == original.observed_axes
            and current.current_supported == original.current_supported
            and current.stamp_steady == original.stamp_steady
            and current.deadline_steady <= original.deadline_steady + 1e-9
        )

    def prepare_stop(
        self,
        prepared: PreparedControl,
        ctx: ConstraintSnapshot,
        body: Feedback | None,
        wheel: Feedback | None,
        now: float,
    ) -> PreparedControl:
        """Use the uncommitted trajectory; caller requalifies the original packet.

        The fresh context must include current local ROS prediction bounds.
        No t0 evaluation, feedback admission or second controller tick occurs.
        """
        self._consume(prepared)
        if (
            prepared.braking
            or prepared.fallback
            or not prepared.decision.verified
            or prepared.decision.emergency
            or prepared.decision.command in (None, Velocity(0, 0))
            or not math.isfinite(now)
            or now < prepared.sampled_now
            or ctx.target != prepared.context.target
            or ctx.mode is not prepared.context.mode
            or self.engine.identity(ctx) != self.engine.identity(prepared.context)
            or not self._same_feedback_evidence(body, prepared.body)
            or not self._same_feedback_evidence(wheel, prepared.wheel)
            or (body is not None and not body.fresh(now))
            or wheel is None
            or not wheel.fresh(now)
        ):
            raise ValueError("invalid stop fallback")
        assert ctx.envelope is not None
        slip = (
            body is not None
            and not body.unified
            and abs(body.velocity.vx - wheel.velocity.vx) > self.config.disagreement_mps
        )
        return self._candidate(
            Velocity(0, 0),
            Velocity(ctx.envelope.linear_cap_mps, ctx.envelope.yaw_cap_rps),
            True,
            "waiting_refinement",
            ctx,
            None,
            body,
            wheel,
            now,
            prepared.dt,
            integrate_body=body is not None,
            slip=slip,
            sampled_now=prepared.sampled_now,
            fallback=True,
        )

    def commit(self, prepared: PreparedControl) -> ControlDecision:
        """Consume once, after the owner successfully delivers the selected output."""
        self._consume(prepared)
        decision = prepared.decision
        self.integral = prepared.integral
        if prepared.step is not None:
            self.trajectory.commit(prepared.step)
            if prepared.braking:
                self._brake_deadline = prepared.brake_deadline
            else:
                self._brake_deadline = None
            for component in ("vx", "wz"):
                value = getattr(prepared.step.velocity, component)
                if value:
                    self._direction = replace(
                        self._direction, **{component: math.copysign(1, value)}
                    )
        return decision

    def tick(
        self,
        target: Velocity,
        ctx: ConstraintSnapshot,
        permission: ConstraintResult | None,
        body: Feedback | None,
        wheel: Feedback | None,
        now: float,
        *,
        estop: bool = False,
        imu: Feedback | None = None,
        still_wheel: Feedback | None = None,
        evaluation: TickEvaluation | None = None,
    ) -> ControlDecision:
        # //Compatible: existing pure callers own delivery and commit immediately.
        return self.commit(
            self.propose_tick(
                target,
                ctx,
                permission,
                body,
                wheel,
                now,
                estop=estop,
                imu=imu,
                still_wheel=still_wheel,
                evaluation=evaluation,
            )
        )
