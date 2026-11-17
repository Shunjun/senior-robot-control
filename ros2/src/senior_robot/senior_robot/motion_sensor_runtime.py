"""ROS-independent composition of the one current-motion estimation owner.

No consumer selects raw sources. Tracking, independent-body diagnostics and
raw wheel/IMU stillness are separate views. This module never supplies a
stopping envelope or a truth bound.
"""

from dataclasses import asdict

from senior_control.models import Pose2D, Velocity
from senior_control.motion.controller import Feedback
from senior_control.motion.estimator import MotionEstimation, MotionObservation, Origin, origin_key
from senior_control.motion.kinematics import body_delta, logarithm
from senior_control.motion.observations import Frame, Source, SourceStatus
from senior_control.motion.pose_estimator import PoseEstimation, PoseObservation
from senior_control.motion.sample_types import DeltaObservation, MotionSample, PointObservation
from senior_control.motion.source_modes import SourceMode
from senior_control.motion.stillness import stationary
from senior_control.runtime.diagnostics import QueueSnapshot, RuntimeDiagnostics
from senior_control.sensors.sample_time import SampleTime, interval_ns


class MotionSensorRuntime:
    def __init__(self, profile, diagnostics: RuntimeDiagnostics):
        self.profile = profile
        self.diagnostics = diagnostics
        a = profile.acceleration_model().upper
        self.estimator = MotionEstimation(acceleration=(a.vx, a.vx, a.wz))
        from senior_control.localization.gps_course import GPSCourseObserver

        self.gps_course = GPSCourseObserver()
        self.gps_heading = None
        self.latest = None
        self.independent = None
        self.tracking = None
        self.raw_wheel = None
        self.raw_imu = None
        self._raw_evidence_sequence = 0
        self.sequence = 0
        self._evidence_key = None
        self._evidence_sequence = 0
        self._independent_key = None
        self._independent_sequence = 0
        self.poses = []
        self.pose_estimator = PoseEstimation(
            unknown_speed_mps=profile.assumed_body_cap,
            unknown_yaw_rps=profile.yaw_cap,
            linear_error_mps=profile.linear_error,
            angular_error_rps=profile.yaw_error,
            lateral_drift_mps=profile.lateral_drift_mps,
            position_error_m=profile.position_error,
            orientation_error_rad=profile.orientation_error_rad,
        )

    def invalidate(self, source: Source) -> None:
        self.estimator.invalidate(source.value)
        if source is Source.WHEEL:
            self.raw_wheel = None
        if source is Source.IMU:
            self.raw_imu = None

    def ingest(
        self,
        sample: MotionSample,
        now_ros: float,
        now_steady: float,
        received_ros: float | None = None,
        origins: tuple[Origin, ...] | None = None,
    ) -> bool:
        source = sample.source.value
        if sample.frame not in (Frame.BODY, Frame.SENSOR) or (
            not isinstance(sample, DeltaObservation) and sample.frame is not Frame.BODY
        ):
            raise ValueError("motion sample needs an actual body-frame transform")
        times = (
            (sample.time,)
            if isinstance(sample, PointObservation)
            else (sample.first_time, sample.last_time)
        )
        physical = None if len(times) == 1 else interval_ns(*times)
        coordinate = times[-1].stamp_ns
        roots = origins or tuple(
            Origin(source, sample.epoch, sample.sequence, t) for t in dict.fromkeys(times)
        )
        if isinstance(sample, DeltaObservation):
            values = logarithm(
                body_delta(sample.delta, sample.body_from_sensor),
                ((physical.end_ns - physical.start_ns) / 1e9),
            )
        elif isinstance(sample, PointObservation):
            if sample.source is Source.IMU:
                values = (None, None, sample.values[0])
            elif sample.source is Source.WHEEL:
                values = (sample.values[0], None, sample.values[1])
            else:
                values = (sample.values[0], sample.values[1], None)
        else:
            values = (
                (sample.values[0], None, sample.values[2])
                if sample.source is Source.WHEEL
                else sample.values
            )
        quality = sample.quality if sample.source not in (Source.WHEEL, Source.IMU) else None
        if sample.status is SourceStatus.UNAVAILABLE:
            quality = 0.0
        elif sample.status is SourceStatus.DRIFT:
            quality = (0.5 if quality is None else quality) * 0.25
        independent = (sample.source is not Source.WHEEL,) * 3
        o = MotionObservation(
            source,
            tuple(values),
            (sample.start_ros + sample.end_ros) / 2,
            roots,
            quality,
            independent=independent,
            measured_steady=sample.end_steady,
            interval_half_s=0
            if physical is None
            else ((physical.end_ns - physical.start_ns) / 1e9) / 2,
            stamp_ns=coordinate,
            measurement_interval=physical,
            measurement_delta=(
                body_delta(sample.delta, sample.body_from_sensor)
                if isinstance(sample, DeltaObservation)
                else getattr(sample, "measurement_delta", None)
            ),
            epoch=sample.epoch,
            sequence=sample.sequence,
        )
        accepted = self.ingest_observation(o, now_ros, now_steady)
        if accepted and sample.source in {Source.WHEEL, Source.IMU}:
            self._raw_evidence_sequence += 1
        if accepted and sample.source is Source.WHEEL:
            self.raw_wheel = o if quality != 0 else None
        if accepted and sample.source is Source.IMU:
            self.raw_imu = o if quality != 0 else None
        return accepted

    def ingest_observation(
        self, observation: MotionObservation, now_ros: float, now_steady: float
    ) -> bool:
        key = origin_key(observation.roots)
        try:
            age = self.estimator._age(observation.roots, now_ros, now_steady)
        except ValueError:
            self.diagnostics.sensor_attempt(
                observation.source, "raw", None, ("invalid_time",), key=key
            )
            raise
        # Record before admission: replay, poor quality, late and revoked samples
        # are observations too, never silently omitted from age diagnostics.
        accepted = self.estimator.ingest(observation, now_ros=now_ros, now_steady=now_steady)
        outcome = (
            "expired"
            if age.band == "invalid"
            else "unavailable"
            if observation.quality == 0
            else "accepted"
            if accepted
            else "rejected"
        )
        self.diagnostics.sensor_attempt(observation.source, "raw", age, (outcome,), key=key)
        return accepted

    def estimate(self, now_ros: float, now_steady: float, *, now_ros_ns: int | None = None):
        if self.gps_heading is not None and now_steady >= self.gps_heading.deadline_steady:
            self.gps_heading = None
        heading = self.gps_heading
        self.diagnostics.summary(
            "gps_course",
            dict(
                ready=heading is not None,
                yaw_rad=None if heading is None else heading.yaw_rad,
                sigma_rad=None if heading is None else heading.sigma_rad,
                stamp_ros=None if heading is None else heading.stamp_ros,
                deadline_steady=None if heading is None else heading.deadline_steady,
                basis="single antenna ENU statistical course; heading-only input",
            ),
        )
        self.sequence += 1
        generation = self.estimator.generation
        r = self.estimator.estimate(now_ros, now_steady, stamp_ns=now_ros_ns)
        if r.generation != generation:
            # The owner revoked its window; raw recovery evidence shares that window.
            # Only subsequent accepted raw inputs can rebuild these caches.
            self.raw_wheel = None
            self.raw_imu = None
        self.latest = r
        self.pose_estimator.advance(r)
        gyro = None if r.independent_axes is None else r.independent_axes[1]
        self.gps_course.record_motion(
            measurement_steady=r.measurement_host_steady
            if r.measurement_host_steady is not None
            else now_steady,
            stamp_ns=r.stamp_ns,
            interval=r.measurement_interval,
            wheel_vx=self.wheel_linear(now_steady),
            imu_wz=(
                gyro.value
                if gyro is not None
                and gyro.observed
                and "imu" in gyro.sources
                and now_steady < gyro.deadline_steady
                else None
            ),
            generation=r.generation,
        )
        if self.gps_course.anchor is None:
            self.gps_heading = None
        for name, retained, capacity in (
            ("motion.estimator.history", self.estimator.retained_samples, 192),
            ("motion.pose.history", self.pose_estimator.retained_samples, 1536),
        ):
            self.diagnostics.queue(
                QueueSnapshot(name, None, capacity, kind="history", retained=retained),
                now_steady,
                owner=self,
            )
        for attempt in self.estimator.evaluations:
            self.diagnostics.sensor_attempt(
                attempt.observation.source, "use", attempt.age, attempt.outcomes
            )
        observed = [a for a in (r.vx, r.wz) if a.observed]
        roots = tuple(dict.fromkeys(root for a in observed for root in a.roots))
        key = tuple((root.family, root.epoch, root.sequence) for root in roots)
        if key != self._evidence_key:
            self._evidence_key = key
            self._evidence_sequence += 1
        epoch = (
            "clock:"
            + str(r.generation)
            + "|"
            + (
                "|".join(sorted({root.family + ":" + root.epoch for root in roots}))
                or "unavailable"
            )
        )
        stamp = max((root.measurement_steady for root in roots), default=now_steady)
        self.tracking = (
            None
            if not r.healthy
            else Feedback(
                Velocity(r.vx.value or 0.0, r.wz.value or 0.0),
                stamp,
                r.deadline_steady,
                epoch,
                self._evidence_sequence,
                True,
                "unified control estimate; per-axis observed mask; no physical truth bound",
                observed_axes=(r.vx.observed, r.wz.observed),
                unified=True,
            )
        )
        independent = r.independent_axes
        self.independent = None
        if independent is not None and all(a.observed for a in independent):
            independent_roots = tuple(
                dict.fromkeys(root for axis in independent for root in axis.roots)
            )
            independent_key = tuple(
                (root.family, root.epoch, root.sequence) for root in independent_roots
            )
            if independent_key != self._independent_key:
                self._independent_sequence += 1
                self._independent_key = independent_key
            independent_epoch = (
                "clock:"
                + str(r.generation)
                + "|"
                + "|".join(sorted({root.family + ":" + root.epoch for root in independent_roots}))
            )
            self.independent = Feedback(
                Velocity(independent[0].value, independent[1].value),
                min(root.measurement_steady for root in independent_roots),
                min(a.deadline_steady for a in independent),
                independent_epoch,
                self._independent_sequence,
                True,
                "independent-sensor current prediction; not a guaranteed speed bound",
            )
        mode = (
            SourceMode.UNAVAILABLE
            if not r.healthy
            else SourceMode.FUSED_REFERENCE
            if independent is not None and independent[0].observed
            else SourceMode.WHEEL_IMU_LIMITED
        )
        health = [
            dict(source=source, ready=True, epoch=epoch, valid_until_steady=r.deadline_steady)
            for source in sorted({s for a in observed for s in a.sources})
        ]
        ref = dict(
            mode=mode.value,
            start_steady=min(
                (
                    a.support_start_steady
                    if a.support_start_steady is not None
                    else min(root.measurement_steady for root in a.roots)
                )
                for a in observed
            )
            if observed
            else now_steady,
            end_steady=max(
                (
                    a.support_end_steady
                    if a.support_end_steady is not None
                    else max(root.measurement_steady for root in a.roots)
                )
                for a in observed
            )
            if observed
            else now_steady,
            computed_ros=now_ros,
            estimate_kind="current_prediction",
            average=dict(vx=r.vx.value or 0.0, vy=r.vy.value, wz=r.wz.value or 0.0),
            source_health=health,
        )
        return ref, self.tracking, self.tracking

    def raw_feedback(self, source, now):
        observation = self.raw_wheel if source == "wheel" else self.raw_imu
        if observation is None or self.latest is None:
            return None
        try:
            age = self.estimator._age(observation.roots, self.latest.computed_ros, now)
        except ValueError:
            return None
        if age.band == "invalid":
            return None
        vx = observation.values[0] if source == "wheel" else 0.0
        wz = observation.values[2]
        if vx is None or wz is None:
            return None
        return Feedback(
            Velocity(vx, wz),
            observation.measured_steady,
            age.deadline_steady,
            observation.identity_epoch,
            observation.identity_sequence,
            True,
            "raw wheel odometry" if source == "wheel" else "raw IMU yaw rate",
            observed_axes=(source == "wheel", True),
        )

    def stationary(self, now):
        if self.raw_wheel is None or self.raw_imu is None:
            return False
        config = self.profile.controller()
        return stationary(
            self.raw_feedback("wheel", now),
            self.raw_feedback("imu", now),
            now,
            linear_limit=config.still_speed_mps,
            yaw_limit=config.still_yaw_rps,
        )

    def recovery(self, now):
        wheel, imu = self.raw_feedback("wheel", now), self.raw_feedback("imu", now)
        if wheel is None or imu is None:
            return None
        from senior_control.motion.state_machine import RecoveryObservation

        return RecoveryObservation(
            wheel.epoch + "|" + imu.epoch,
            self._raw_evidence_sequence,
            min(wheel.stamp_steady, imu.stamp_steady),
            min(wheel.deadline_steady, imu.deadline_steady),
            True,
            self.stationary(now),
            manual_rearm_ready=True,
        )

    def ingest_poses(self, rows):
        for row in rows:
            root = Origin("wheel", row["epoch"], row["sequence"], SampleTime(**row["time"]))
            self.pose_estimator.ingest(
                PoseObservation(
                    "wheel_pose",
                    Pose2D(row["x"], row["y"], row["yaw"]),
                    root.stamp_ros,
                    (root,),
                    measurement_steady=root.measurement_steady,
                    stamp_ns=root.time.stamp_ns,
                )
            )

    def wheel_linear(self, now_steady, *, with_deadline=False):
        # //Compatible: existing callers retain scalar raw-wheel velocity.
        o = self.raw_wheel
        if o is None or self.latest is None:
            return None
        try:
            age = self.estimator._age(o.roots, self.latest.computed_ros, now_steady)
        except ValueError:
            return None
        if age.band == "invalid":
            return None
        return (o.values[0], age.deadline_steady) if with_deadline else o.values[0]

    def report(self):
        return None if self.latest is None else asdict(self.latest)

    def ingest_packet(self, packet):
        """Consume current raw-source snapshots without reconstructing missing receipts."""
        for row in packet.get("sources", []):
            self._ingest_packet_sample(row, packet)
        self.poses = packet.get("poses", [])

    def _ingest_packet_sample(self, row, packet):
        sample = row["sample"]
        source = row["source"]
        if "time" in sample:
            times = (SampleTime(**sample["time"]),)
        else:
            times = (SampleTime(**sample["first_time"]), SampleTime(**sample["last_time"]))
        support = None if len(times) == 1 else interval_ns(*times)
        values = sample.get("values")
        if "delta" in sample:
            values = logarithm(Pose2D(**sample["delta"]), (support.end_ns - support.start_ns) / 1e9)
        elif source == "wheel":
            values = (values[0], None, values[-1])
        elif source == "imu":
            values = (None, None, values[0])
        roots = tuple(
            Origin(source, sample["epoch"], sample["sequence"], t) for t in dict.fromkeys(times)
        )
        observation = MotionObservation(
            source,
            tuple(values),
            times[-1].stamp_ns / 1e9,
            roots,
            sample.get("quality", 0.5) if sample.get("status", "valid") == "valid" else 0.0,
            independent=(source != "wheel",) * 3,
            measured_steady=times[-1].sample_steady_ns / 1e9,
            interval_half_s=0 if support is None else (support.end_ns - support.start_ns) / 2e9,
            stamp_ns=times[-1].stamp_ns,
            measurement_interval=support,
            measurement_delta=Pose2D(**sample["delta"]) if "delta" in sample else None,
        )
        self.estimator.ingest(observation)
        if source == "wheel":
            self.raw_wheel = observation
