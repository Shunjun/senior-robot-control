"""Pure closed-loop candidate generator; no actuator or authority side effects."""

import math
from dataclasses import dataclass, replace

from senior_control.models import Velocity
from senior_control.motion.constraints import (
    BrakingProof,
    ConstraintEngine,
    ConstraintResult,
    ConstraintSnapshot,
)
from senior_control.motion.contracts import BrakeKind
from senior_control.motion.source_modes import SourceMode
from senior_control.motion.trajectory import Trajectory, scale_wheels


@dataclass(frozen=True)
class Feedback:
    velocity: Velocity
    stamp_steady: float
    deadline_steady: float
    epoch: str
    sequence: int
    current_supported: bool
    basis: str

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
    max_tick_s: float
    still_speed_mps: float
    still_yaw_rps: float
    still_samples: int
    still_span_s: float
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
            self.max_tick_s,
            self.still_speed_mps,
            self.still_yaw_rps,
            self.still_span_s,
            self.disagreement_mps,
        )
        if (
            not all(math.isfinite(x) and x >= 0 for x in numbers)
            or min(self.track_width_m, self.wheel_speed_mps, self.max_tick_s) <= 0
            or self.still_samples < 1
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


def _bounded(v: Velocity, cap: Velocity) -> Velocity:
    def component(x: float, bound: float) -> float:
        return max(-bound, min(bound, x))

    return Velocity(component(v.vx, cap.vx), component(v.wz, cap.wz))


class MotionController:
    """Owns command trajectory and PI state, not source qualification or ESTOP latch.

    Body Feedback must be an explicitly supported CURRENT reference. An old
    BodyMotion interval mean cannot just be relabelled here. Wheel Feedback is
    tracking/slip evidence and never establishes independent body stillness.
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
        self._last_now: float | None = None
        self._context_key: tuple[str, ...] | None = None
        self._last_feedback: dict[str, tuple[str, int, float]] = {}
        self._still_first: float | None = None
        self._still_count = 0
        self._brake_deadline: float | None = None
        self._still_key: tuple[str, int, float] | None = None
        self.measured_still = False
        self._limited_epoch: tuple[object, ...] | None = None
        self._limited_key: tuple[int, int, float] | None = None
        self._limited_first: float | None = None
        self._limited_count = 0
        self._limited_until = -math.inf
        self._limited_complete = False
        self._direction = Velocity(
            math.copysign(1, initial_velocity.vx) if initial_velocity.vx else 0,
            math.copysign(1, initial_velocity.wz) if initial_velocity.wz else 0,
        )

    def _admit_feedback(self, role: str, value: Feedback | None, now: float) -> Feedback | None:
        if value is None or not value.fresh(now):
            return None
        old = self._last_feedback.get(role)
        key = (value.epoch, value.sequence, value.stamp_steady)
        if old is not None:
            if key[2] < old[2] or (key[0] == old[0] and key[1] < old[1]):
                return None
            if key[0] != old[0]:
                self.integral = Velocity(0, 0)
                self._still_count = 0
                self._still_first = None
                self._still_key = None
        self._last_feedback[role] = key
        return value

    def _still(self, body: Feedback | None, wheel: Feedback | None) -> None:
        if (
            body is None
            or wheel is None
            or max(abs(body.velocity.vx), abs(wheel.velocity.vx)) > self.config.still_speed_mps
            or max(abs(body.velocity.wz), abs(wheel.velocity.wz)) > self.config.still_yaw_rps
        ):
            self._still_first = None
            self._still_count = 0
            self.measured_still = False
            self._still_key = None
            return
        key = (body.epoch, body.sequence, body.stamp_steady)
        if key != self._still_key:
            self._still_key = key
            self._still_count += 1
            if self._still_first is None:
                self._still_first = body.stamp_steady
        self.measured_still = (
            self._still_count >= self.config.still_samples
            and body.stamp_steady >= (self._still_first or 0) + self.config.still_span_s
        )

    def _decision(
        self,
        command: Velocity | None,
        reason: str,
        verified: bool,
        emergency: bool,
        ctx: ConstraintSnapshot,
    ) -> ControlDecision:
        if command is None or emergency or not verified:
            self.invalidate_limited_rearm()
        deadline = min(
            ctx.motion.valid_until_steady,
            ctx.perception.deadline_steady,
            ctx.response.valid_until_steady,
            ctx.envelope.valid_until_steady if ctx.envelope else -math.inf,
        )
        return ControlDecision(
            command, reason, verified, emergency, self.engine.identity(ctx), deadline
        )

    def invalidate_limited_rearm(self) -> None:
        """A broken conditional stop cannot inherit its old completion deadline."""
        if self._limited_epoch is not None:
            self._brake_deadline = None
        self._limited_epoch = None
        self._limited_key = None
        self._limited_first = None
        self._limited_count = 0
        self._limited_until = -math.inf
        self._limited_complete = False

    def limited_rearm_ready(
        self,
        now: float,
        *,
        epochs: tuple[str, str] | None = None,
    ) -> bool:
        """Simulation-model permission, never independent body stillness."""
        return bool(
            self._limited_complete
            and self._last_now is not None
            and self._last_now <= now < self._limited_until
            and (epochs is None or (self._limited_epoch and self._limited_epoch[:2] == epochs))
        )

    def _limited_observation(
        self,
        ctx: ConstraintSnapshot,
        body: Feedback | None,
        wheel: Feedback | None,
        imu: Feedback | None,
        now: float,
    ) -> None:
        e = ctx.envelope
        if not (
            body is None
            and wheel is not None
            and imu is not None
            and ctx.mode is SourceMode.WHEEL_IMU_LIMITED
            and self.config.synthetic
            and e is not None
            and e.synthetic
            and e.brake_drive_overshoot is not None
            and e.brake_tracking_basis
        ):
            self.invalidate_limited_rearm()
            return
        epoch = (
            wheel.epoch,
            imu.epoch,
            ctx.motion.source_epoch,
            ctx.perception.source_epoch,
            ctx.response,
            e.brake_tracking_basis,
            e.brake_drive_overshoot,
            e.linear_interval,
            ctx.scope_id,
            ctx.config_version,
        )
        if epoch != self._limited_epoch or now >= self._limited_until:
            self.invalidate_limited_rearm()
            self._brake_deadline = None
            self._limited_epoch = epoch
        self._limited_until = min(
            wheel.deadline_steady,
            imu.deadline_steady,
            ctx.motion.valid_until_steady,
            ctx.perception.deadline_steady,
            ctx.response.valid_until_steady,
            e.valid_until_steady,
        )
        quiet = (
            abs(wheel.velocity.vx) <= self.config.still_speed_mps
            and abs(wheel.velocity.wz) <= self.config.still_yaw_rps
            and abs(imu.velocity.wz) <= self.config.still_yaw_rps
        )
        if not quiet:
            self._limited_count = 0
            self._limited_first = None
            self._limited_key = None
            self._limited_complete = False
            return
        stamp = min(wheel.stamp_steady, imu.stamp_steady)
        key = (wheel.sequence, imu.sequence, stamp)
        if self._limited_key is None or all(
            a > b for a, b in zip(key[:2], self._limited_key[:2], strict=True)
        ):
            self._limited_key = key
            self._limited_count += 1
            if self._limited_first is None:
                self._limited_first = stamp

    def _limited_stop_complete(self, now: float) -> bool:
        return bool(
            self._limited_epoch is not None
            and now < self._limited_until
            and self._brake_deadline is not None
            and now >= self._brake_deadline
            and self._limited_count >= self.config.still_samples
            and self._limited_first is not None
            and self._limited_key is not None
            and self._limited_key[2] >= self._limited_first + self.config.still_span_s
            and self.trajectory.velocity == Velocity(0, 0)
            and self.trajectory.acceleration == Velocity(0, 0)
        )

    def reset_at_stillness(self, body: Feedback, wheel: Feedback, now: float) -> bool:
        """Composition calls only after independent state-machine reset acceptance."""
        b = self._admit_feedback("body", body, now)
        w = self._admit_feedback("wheel", wheel, now)
        self._still(b, w)
        if not self.measured_still:
            return False
        self.trajectory = Trajectory(Velocity(0, 0), Velocity(0, 0))
        self.integral = Velocity(0, 0)
        self._direction = Velocity(0, 0)
        self._last_now = now
        self._brake_deadline = None
        return True

    def _brake_supervision(
        self, proof: BrakingProof, body: Feedback | None, wheel: Feedback | None, now: float
    ) -> str | None:
        if self._brake_deadline is None:
            if not proof.brake.stop_verified or proof.deadline_steady is None:
                return "unverified_brake"
            self._brake_deadline = proof.deadline_steady
        if now > self._brake_deadline:
            if body is None or wheel is None:
                if (
                    self._limited_epoch is not None
                    and wheel is not None
                    and (
                        abs(wheel.velocity.vx) > self.config.still_speed_mps
                        or abs(wheel.velocity.wz) > self.config.still_yaw_rps
                    )
                ):
                    return "stop_response_failed"
                if self._limited_stop_complete(now):
                    return None
                return "stop_boundary_without_feedback"
            if (
                max(abs(body.velocity.vx), abs(wheel.velocity.vx)) > self.config.still_speed_mps
                or max(abs(body.velocity.wz), abs(wheel.velocity.wz)) > self.config.still_yaw_rps
            ):
                return "stop_response_failed"
        return None

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
            self.integral = Velocity(0, 0)
            return goal, self.integral, False
        error = (
            Velocity(goal.vx - reference.velocity.vx, goal.wz - reference.velocity.wz)
            if reference
            else Velocity(0, 0)
        )
        if not integrate or slip:
            self.integral = Velocity(0, 0)
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
            (self.measured_still or self.limited_rearm_ready(self._last_now or 0))
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
        # //Compatible: old callers without an IMU never gain conditional rearm.
        imu: Feedback | None = None,
        require_limited_rearm: bool = False,
    ) -> ControlDecision:
        # IMU uses only velocity.wz: its unused vx is NOT body linear evidence.
        if estop:
            self.integral = Velocity(0, 0)
            self._still(
                self._admit_feedback("body", body, now), self._admit_feedback("wheel", wheel, now)
            )
            if math.isfinite(now) and (self._last_now is None or now >= self._last_now):
                self._last_now = now
            return self._decision(Velocity(0, 0), "user_estop_override", False, False, ctx)
        if self._last_now is None:
            self._last_now = now
            return self._decision(None, "clock_initialized", False, False, ctx)
        dt = now - self._last_now
        self._last_now = now
        if not math.isfinite(now) or not 0 < dt <= self.config.max_tick_s:
            self.integral = Velocity(0, 0)
            return self._decision(None, "invalid_tick", False, True, ctx)
        body = self._admit_feedback("body", body, now)
        wheel = self._admit_feedback("wheel", wheel, now)
        imu = self._admit_feedback("imu", imu, now)
        self._still(body, wheel)
        self._limited_observation(ctx, body, wheel, imu, now)
        key = (*self.engine.identity(ctx), ctx.mode.value)
        if key != self._context_key:
            self.integral = Velocity(0, 0)
        self._context_key = key
        reference = body
        if reference is None and ctx.mode is SourceMode.WHEEL_IMU_LIMITED:
            reference = wheel
        slip = (
            body is not None
            and wheel is not None
            and abs(body.velocity.vx - wheel.velocity.vx) > self.config.disagreement_mps
        )
        stopping = replace(ctx, target=replace(ctx.target, velocity=Velocity(0, 0)))
        stop = self.engine.refine(stopping, now)
        if stop.brake is not None and stop.brake.kind is BrakeKind.EMERGENCY:
            self.integral = Velocity(0, 0)
            return self._decision(None, stop.brake.reason, False, True, ctx)
        fresh = self.engine.tighten(permission, ctx, now)
        goal, cap, braking, reason = self._goal(target, ctx, fresh, reference)
        if require_limited_rearm and not self.limited_rearm_ready(now):
            goal, braking, reason = Velocity(0, 0), True, "limited_rearm_not_ready"
        raw, trial, integrate = self._pi_target(
            goal, reference, cap, dt, integrate=body is not None, slip=slip
        )
        if slip:
            reason = "inconsistent_feedback_limited"
        try:
            step = self.trajectory.propose(raw, ctx.response, dt)
        except ValueError as exc:
            self.integral = Velocity(0, 0)
            return self._decision(None, str(exc), False, True, ctx)
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
            return self._decision(None, "wheel_limit_infeasible_step", False, True, ctx)
        # Normalize at most four ULP of wheel-combination arithmetic, not a
        # physical clamp. Larger changes cannot inherit acceleration/jerk.
        step = replace(step, velocity=candidate)
        permitted = self.engine.validate_output(fresh, ctx, candidate, now)
        # Braking consumes the separate T4 zero-goal STOP proof. validate_output
        # deliberately rejects motion without a permit; it is not a stop proof.
        brake_proof = (
            self.engine.braking_proof(
                ctx, self.trajectory.velocity, self.trajectory.acceleration, candidate, now
            )
            if braking
            else None
        )
        stop_verified = brake_proof is not None and brake_proof.brake.stop_verified
        if braking:
            if not stop_verified:
                self.integral = Velocity(0, 0)
                return self._decision(
                    None,
                    brake_proof.brake.reason if brake_proof else "unverified_brake",
                    False,
                    True,
                    ctx,
                )
            assert brake_proof is not None
            failure = self._brake_supervision(brake_proof, body, wheel, now)
            if failure:
                self.integral = Velocity(0, 0)
                return self._decision(None, failure, False, True, ctx)
        elif not permitted:
            self.integral = Velocity(0, 0)
            return self._decision(None, "final_candidate_rejected", False, True, ctx)
        self.trajectory.commit(step)
        if braking:
            self._limited_complete = self._limited_stop_complete(now)
            if self._limited_complete:
                reason = "limited_drive_settled"
        else:
            self.invalidate_limited_rearm()
        if not braking:
            self._brake_deadline = None
        if integrate:
            # Conditional integration includes rate/jerk saturation, not only
            # amplitude clamps. Allow an existing integral to unwind.
            self.integral = Velocity(
                trial.vx
                if candidate.vx == raw.vx or abs(trial.vx) < abs(self.integral.vx)
                else self.integral.vx,
                trial.wz
                if candidate.wz == raw.wz or abs(trial.wz) < abs(self.integral.wz)
                else self.integral.wz,
            )
        for component in ("vx", "wz"):
            value = getattr(candidate, component)
            if value:
                self._direction = replace(self._direction, **{component: math.copysign(1, value)})
        decision = self._decision(candidate, reason, permitted or stop_verified, False, ctx)
        deadline = decision.evidence_deadline
        if not braking and fresh.permit is not None:
            deadline = min(
                deadline,
                fresh.permit.deadline_steady,
                ctx.target.deadline_steady,
                reference.deadline_steady if reference else deadline,
            )
        return replace(decision, evidence_deadline=deadline)
