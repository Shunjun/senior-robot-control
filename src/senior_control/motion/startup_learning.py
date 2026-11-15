"""One start-owned attempt. This owner proposes targets, never actuator commands."""

import math
import os
from pathlib import Path

from senior_control.sensors.policy import default_policy


class StartupLearning:
    active_states = {"initializing", "sampling", "moving", "braking", "validating"}

    def __init__(self, claim: Path | None, *, linear_error_mps=0.02, yaw_error_rps=0.02):
        self.state = "disabled"
        self.reason = "no_program_start_context"
        self.distance = 0.0
        self.motion_error = 0.0
        self.motion_radius = 0.0
        self.linear_error_mps = linear_error_mps
        self.yaw_error_rps = yaw_error_rps
        self.independent_distance = 0.0
        self.yaw = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.started = None
        self.previous = None
        self.key = None
        self.samples = 0
        self.validation_samples = 0
        self.motion_verified = True
        self.generation = None
        if claim is not None:
            try:
                fd = os.open(claim, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
                with os.fdopen(fd, "w") as file:
                    file.write("attempt_claimed_before_motion\n")
                    file.flush()
                    os.fsync(file.fileno())
                self.state, self.reason = "initializing", "start_owned_attempt"
            except FileExistsError:
                self.state, self.reason = "interrupted", "previous_attempt_no_automatic_retry"
            except OSError:
                self.state, self.reason = "failed", "claim_unavailable"

    @property
    def active(self):
        return self.state in self.active_states

    @property
    def provisional(self):
        return self.state in {"sampling", "moving", "braking", "validating"}

    @property
    def velocity(self):
        return min(0.05, max(0.0, self.distance_remaining)) if self.state == "moving" else 0.0

    @property
    def distance_remaining(self):
        return 0.15 - self.distance

    def interrupt(self, reason):
        if self.active:
            self.state, self.reason = "interrupted", reason

    def advance(self, estimate, wheel, *, now, stationary):
        if not self.active:
            return
        if self.started is None:
            self.started = now
        if now < self.started or now - self.started > 30:
            self.state, self.reason = "failed", "learning_time_budget"
            return
        if estimate is None or not estimate.healthy:
            if self.state != "initializing":
                self.interrupt("motion_feedback_unavailable")
            return
        if self.generation is not None and estimate.generation != self.generation:
            self.interrupt("motion_generation_changed")
            return
        self.generation = estimate.generation
        axis, gyro = estimate.vx, estimate.wz
        if (
            axis.value is None
            or gyro.value is None
            or now >= min(axis.deadline_steady, gyro.deadline_steady)
        ):
            if self.state != "initializing":
                self.interrupt("motion_axis_unavailable")
            return
        key = tuple((r.family, r.epoch, r.sequence) for a in (axis, gyro) for r in a.roots)
        if key == self.key:
            return
        self.key = key
        if self.state == "initializing":
            self.samples = self.samples + 1 if stationary else 0
            if self.samples >= 5:
                self.state, self.reason = "sampling", "physical_interior_prior"
            self.previous = now
            return
        dt = now - self.previous if self.previous is not None else 0
        self.previous = now
        if dt < 0 or dt >= default_policy().hard_age_s:
            self.interrupt("motion_integration_gap")
            return
        if self.state in {"moving", "braking"}:
            self.distance += max(0.0, axis.value) * dt
            self.yaw += gyro.value * dt
            independent = estimate.independent_axes
            lateral = getattr(estimate, "vy", None)
            valid = (
                independent is not None
                and all(a.observed for a in independent)
                and lateral is not None
                and lateral.observed
                and lateral.independent
                and now < min(lateral.deadline_steady, *(a.deadline_steady for a in independent))
            )
            # Wheel/IMU-only progress is an estimate, never independent proof.
            if valid:
                v = independent[0].value
                linear_error = math.hypot(
                    independent[0].error
                    if independent[0].error is not None
                    else self.linear_error_mps,
                    lateral.error if lateral.error is not None else self.linear_error_mps,
                )
                yaw_error = (
                    independent[1].error if independent[1].error is not None else self.yaw_error_rps
                )
                self.motion_error += (linear_error + self.motion_radius * yaw_error) * dt
                from senior_control.models import Pose2D
                from senior_control.motion.kinematics import compose, exponential

                pose = compose(
                    Pose2D(self.pose_x, self.pose_y, self.pose_yaw),
                    exponential(v, lateral.value, independent[1].value, dt),
                )
                self.pose_x, self.pose_y, self.pose_yaw = pose.x, pose.y, pose.yaw
                self.independent_distance += max(0.0, v) * dt
                if wheel is None or abs(v - wheel) > max(0.03, abs(v) * 0.35):
                    self.motion_verified = False
            else:
                self.motion_verified = False
            if abs(self.yaw) > 0.05 or axis.value < -0.01:
                self.interrupt("not_straight_forward")
            elif self.distance >= 0.148:
                self.state, self.reason = "braking", "target_distance_reached"
            if self.state == "braking" and stationary:
                self.state, self.reason = "validating", "post_motion_validation"

    def frame(self, model, observation):
        from senior_control.models import Pose2D

        if self.provisional:
            model.observe(
                observation,
                Pose2D(self.pose_x, self.pose_y, self.pose_yaw),
                force=self.state in {"sampling", "validating"},
            )
        if self.state == "sampling" and len(model.frames) >= 5:
            self.state, self.reason = "moving", "learning_forward"
        elif self.state == "validating":
            self.validation_samples += 1
            if self.validation_samples >= 5:
                count = model.validate(
                    independent_motion=(self.motion_verified and self.independent_distance >= 0.08),
                    motion_error_m=self.motion_error,
                )
                self.state = (
                    "completed"
                    if count
                    or not any(model.interior(p) for _, points in model.frames for p in points)
                    else "ambiguous"
                )
                self.reason = "verified_local_returns" if count else "classification_unverified"

    def summary(self, learned_count=0):
        return dict(
            state=self.state,
            reason=self.reason,
            estimated_distance_m=self.distance,
            independent_distance_m=self.independent_distance,
            independent_motion_verified=self.motion_verified and self.independent_distance >= 0.08,
            classification=(
                "provisional"
                if self.provisional
                else "learned"
                if learned_count
                else "ambiguous"
                if self.state == "ambiguous"
                else "prior"
                if self.state == "initializing"
                else "none"
            ),
            learned_local_returns=learned_count,
        )


def start_environment(environment: dict[str, str], root: Path) -> dict[str, str]:
    """Called once by the program-start owner; children inherit the same claim.

    A fresh explicit start is a fresh attempt. Reconnection and child restarts
    retain the run's consumed path; no component may manufacture a new run id.
    """
    from uuid import uuid4

    result = dict(environment)
    result["SENIOR_STARTUP_LEARNING_CLAIM"] = str(root / (".startup-learning-" + uuid4().hex))
    return result
