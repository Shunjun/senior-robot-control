"""One start-owned attempt. This owner proposes targets, never actuator commands."""

import math
import os
from pathlib import Path

from senior_control.models import Pose2D
from senior_control.motion.learning_pose_history import LearningPose, LearningPoseHistory
from senior_control.sensors.policy import default_policy


class StartupLearning:
    active_states = {"initializing", "sampling", "moving", "braking", "validating"}

    def __init__(self, claim: Path | None, *, linear_error_mps=0.02, yaw_error_rps=0.02):
        self.state = "disabled"
        self.reason = "no_program_start_context"
        self.distance = 0.0
        self.motion_error = 0.0
        self.position_error = 0.0
        self.yaw_error = 0.0
        self.pose_history = LearningPoseHistory()
        self.validation_after_ros = None
        self._frame_stamps = set()
        self.previous_ros = None
        self.motion_radius = 0.0
        self.linear_error_mps = linear_error_mps
        self.yaw_error_rps = yaw_error_rps
        self.independent_distance = 0.0
        self.yaw = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.started = None
        self.motion_withdrawn = False
        self.resample_required = False
        self.previous = None
        self.key = None
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
        return (
            min(0.05, max(0.0, self.distance_remaining))
            if self.state == "moving" and not self.motion_withdrawn
            else 0.0
        )

    @property
    def distance_remaining(self):
        return 0.15 - self.distance

    @property
    def motion_started(self):
        return self.started is not None

    def motion_emitted(self, now):
        if not self.motion_started:
            self.started = now

    def hold_motion(self):
        # Withdrawal is irreversible only after an actual motion attempt. A
        # pending attempt continues static learning and never creates a new claim.
        if self.motion_started:
            self.motion_withdrawn = True

    def interrupt(self, reason):
        if self.active:
            self.state, self.reason = "interrupted", reason

    def advance(self, estimate, wheel, *, now, stationary):
        if not self.active:
            return
        if (
            self.started is not None
            and not self.motion_withdrawn
            and (now < self.started or now - self.started > 30)
        ):
            self.state, self.reason = "failed", "learning_time_budget"
            return
        if estimate is None or not estimate.healthy:
            if self.motion_started and not self.motion_withdrawn:
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
            if self.motion_started and not self.motion_withdrawn:
                self.interrupt("motion_axis_unavailable")
            return
        key = tuple((r.family, r.epoch, r.sequence) for a in (axis, gyro) for r in a.roots)
        if key == self.key:
            return
        self.key = key
        if self.state == "initializing":
            if stationary:
                self.state, self.reason = "sampling", "physical_interior_prior"
            self.previous = now
            self.previous_ros = estimate.computed_ros
            self._record_pose(estimate, stationary)
            return
        steady_gap = now - self.previous if self.previous is not None else 0
        dt = estimate.computed_ros - self.previous_ros if self.previous_ros is not None else 0
        self.previous, self.previous_ros = now, estimate.computed_ros
        if dt < 0 or steady_gap < 0 or steady_gap >= default_policy().hard_age_s:
            if not self.motion_started:
                self.state = "sampling"
                self.resample_required = True
                self._frame_stamps.clear()
                self.pose_history.samples.clear()
                self._record_pose(estimate, stationary)
            else:
                self.interrupt("motion_integration_gap")
            return
        if self.state in {"moving", "braking", "validating"}:
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
                self.position_error += linear_error * dt
                self.yaw_error += yaw_error * dt
                self.motion_error = self.position_error + self.motion_radius * self.yaw_error
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
            elif self.state == "moving" and self.distance >= 0.148:
                self.state, self.reason = "braking", "target_distance_reached"
            if (
                self.state == "braking" or (self.state == "moving" and self.motion_withdrawn)
            ) and stationary:
                self.state, self.reason = "validating", "post_motion_validation"
                self.validation_after_ros = estimate.computed_ros
        self._record_pose(estimate, stationary)

    def _record_pose(self, estimate, stationary):
        self.pose_history.add(
            LearningPose(
                estimate.computed_ros,
                Pose2D(self.pose_x, self.pose_y, self.pose_yaw),
                self.position_error,
                self.yaw_error,
                stationary,
                self.motion_verified,
                estimate.generation,
            )
        )

    def frame(self, model, observation):
        if self.resample_required:
            model.clear()
            self.resample_required = False
        stamp = observation.scan.stamp
        key = round(stamp * 1e9)
        sample = self.pose_history.at(stamp)
        if sample is None or key in self._frame_stamps or not self.provisional:
            return
        if len(self._frame_stamps) >= 2048:
            self.interrupt("learning_frame_budget")
            return
        self._frame_stamps.add(key)
        retained = model.observe(
            observation,
            sample.pose,
            pose_error_m=sample.position_error_m,
            pose_yaw_error_rad=sample.yaw_error_rad,
            force=self.state in {"sampling", "validating"} or not self.motion_started,
        )
        if self.state == "moving" and not self.motion_started:
            model.retain_recent(5)
            self._frame_stamps = {round(t * 1e9) for t in model.frame_stamps}
        if self.state == "sampling" and len(model.frames) >= 5:
            self.state, self.reason = "moving", "learning_forward"
        elif (
            self.state == "validating"
            and retained
            and self.validation_after_ros is not None
            and stamp >= self.validation_after_ros
            and sample.stationary
        ):
            self.validation_samples += 1
            if self.validation_samples >= 5:
                count = model.validate(
                    independent_motion=(
                        not self.motion_withdrawn
                        and self.motion_verified
                        and self.independent_distance >= 0.08
                    ),
                )
                self.state = (
                    "completed"
                    if (
                        not self.motion_withdrawn
                        and (
                            count
                            or not any(
                                model.interior(p) for _, points in model.frames for p in points
                            )
                        )
                    )
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
