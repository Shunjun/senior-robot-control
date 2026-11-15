"""Software stopping-risk policy over the single current-motion estimate.

These intervals are assumptions for prediction, never measured physical guarantees.
Independent stillness/reset evidence is a separate concern.
"""

from dataclasses import dataclass, field

from senior_control.models import Velocity
from senior_control.motion.constraints import AxisStop, stop_axis
from senior_control.motion.contracts import ResponseBounds
from senior_control.motion.estimator import AxisEstimate, MotionEstimate


def _axis_domain(
    axis: AxisEstimate, trajectory: float, measurement_error: float
) -> tuple[float, float]:
    # Total known error already contains alignment/age and source disagreement.
    # Unknown measurement error uses the deployment assumption plus that same
    # projection ledger once. A bounded model-only hold retains its original TTL.
    error = max(measurement_error + axis.prediction_error, axis.error or 0.0)
    assert axis.value is not None
    low, high = axis.value - error, axis.value + error
    # A zero command is a goal, not evidence of stillness. Including it in a
    # strictly forward measured domain would erase reverse_requires_stop.
    return (min(low, trajectory), max(high, trajectory)) if trajectory else (low, high)


@dataclass(frozen=True)
class _BrakeProjection:
    speed: float
    acceleration: float
    stop: AxisStop

    def remaining(self, elapsed: float) -> float:
        if elapsed >= self.stop.duration:
            return 0.0
        velocity, acceleration = self.speed, self.acceleration
        for phase in self.stop.phases:
            if elapsed < phase.duration:
                return max(0.0, velocity + acceleration * elapsed + phase.jerk * elapsed**2 / 2)
            elapsed -= phase.duration
            velocity, acceleration = phase.end_velocity, phase.end_acceleration
        return 0.0


@dataclass
class _AxisResidual:
    interval: tuple[float, float] | None = None
    at: float = 0.0
    support_deadline: float = float("-inf")
    brake: tuple[_BrakeProjection, _BrakeProjection] | None = None
    reason: str = "no_axis_history_assumption"

    def _advance(self, now: float, acceleration: float) -> tuple[float, float]:
        assert self.interval is not None
        elapsed = max(0.0, now - self.at)
        if self.brake is not None:
            return -self.brake[0].remaining(elapsed), self.brake[1].remaining(elapsed)
        growth = acceleration * elapsed
        return self.interval[0] - growth, self.interval[1] + growth

    def domain(
        self, axis: AxisEstimate, trajectory: float, error: float, acceleration: float, now: float
    ) -> tuple[float, float]:
        if axis.value is not None and now < axis.deadline_steady:
            self.interval = _axis_domain(axis, trajectory, error)
            self.at, self.support_deadline = now, axis.deadline_steady
            self.brake = None
            self.reason = axis.reason
        elif self.interval is None:
            # Startup prior only, not a claim that a lost previously observed
            # axis has stopped. No last motion exists in this process to retain.
            self.interval = min(0.0, trajectory - error), max(0.0, trajectory + error)
            self.at = now
        else:
            interval = self._advance(now, acceleration)
            if self.brake is None:
                if trajectory:
                    interval = min(interval[0], trajectory), max(interval[1], trajectory)
                self.interval, self.at = interval, now
                self.reason = "expired_axis_residual"
            else:
                # Successful zero output superseded the old trajectory command.
                # The response model is not a new observation or reset evidence.
                self.reason = (
                    "assumed_zero_response" if interval != (0.0, 0.0) else "assumed_settled"
                )
            return interval
        return self.interval

    def commanded(
        self,
        value: float,
        now: float,
        acceleration: float,
        decel: float,
        jerk: float,
        lag: float,
        overshoot: float,
    ) -> None:
        if self.interval is None or now < self.support_deadline:
            return
        if value == 0 and self.brake is not None:
            return  # Repeated zero commands do not restart the settling clock.
        interval = self._advance(now, acceleration)
        self.interval, self.at = interval, now
        if value != 0:
            self.interval = min(interval[0], value - overshoot), max(interval[1], value + overshoot)
            self.brake = None
            return
        # Only an actual delivered zero-axis command starts this response. Use
        # the existing whole stop-family overshoot and outward acceleration cap.
        speeds = max(-interval[0], overshoot), max(interval[1], overshoot)
        negative, positive = (
            _BrakeProjection(speed, acceleration, stop_axis(speed, acceleration, decel, jerk, lag))
            for speed in speeds
        )
        self.brake = negative, positive


@dataclass
class StoppingRisk:
    """Two-axis actuator-response memory, separate from sensor fusion and TTLs."""

    linear: _AxisResidual = field(default_factory=_AxisResidual)
    yaw: _AxisResidual = field(default_factory=_AxisResidual)

    def domains(
        self,
        estimate: MotionEstimate,
        trajectory: Velocity,
        response: ResponseBounds,
        *,
        linear_error: float,
        yaw_error: float,
    ) -> tuple[tuple[float, float], tuple[float, float]]:
        return (
            self.linear.domain(
                estimate.vx, trajectory.vx, linear_error, response.accel, estimate.computed_steady
            ),
            self.yaw.domain(
                estimate.wz, trajectory.wz, yaw_error, response.yaw_accel, estimate.computed_steady
            ),
        )

    def commanded(
        self, command: Velocity, now: float, response: ResponseBounds, overshoot: Velocity
    ) -> None:
        self.linear.commanded(
            command.vx,
            now,
            response.accel,
            response.decel,
            response.jerk,
            response.actuator_lag,
            overshoot.vx,
        )
        self.yaw.commanded(
            command.wz,
            now,
            response.yaw_accel,
            response.yaw_decel,
            response.yaw_jerk,
            response.actuator_lag,
            overshoot.wz,
        )

    def reaction(self, trajectory: Velocity, response: ResponseBounds) -> Velocity:
        def upper(axis: _AxisResidual, current: float, limit: float) -> float:
            return (
                max(current, limit)
                if axis.reason in {"expired_axis_residual", "assumed_zero_response"}
                else current
            )

        return Velocity(
            upper(self.linear, trajectory.vx, response.accel),
            upper(self.yaw, trajectory.wz, response.yaw_accel),
        )
