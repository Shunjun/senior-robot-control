"""ROS-independent composition of the one current-motion estimation owner.

No consumer selects raw sources. Tracking and independent-body evidence are
separate views; this module never supplies a stopping envelope or a truth bound.
"""

from dataclasses import asdict

from senior_control.models import Pose2D, Velocity
from senior_control.motion.controller import Feedback
from senior_control.motion.estimator import MotionEstimation, MotionObservation, Origin
from senior_control.motion.kinematics import body_delta, logarithm
from senior_control.motion.observations import Frame, Source, SourceStatus
from senior_control.motion.pose_estimator import PoseEstimation, PoseObservation
from senior_control.motion.sample_types import DeltaObservation, MotionSample, PointObservation
from senior_control.motion.source_modes import SourceMode
from senior_control.runtime.diagnostics import QueueSnapshot, RuntimeDiagnostics


class MotionSensorRuntime:
    def __init__(self, profile, diagnostics: RuntimeDiagnostics):
        self.profile = profile
        self.diagnostics = diagnostics
        a = profile.acceleration_model().upper
        self.estimator = MotionEstimation(acceleration=(a.vx, a.vx, a.wz))
        self.latest = None
        self.independent = None
        self.tracking = None
        self.raw_wheel = None
        self.sequence = 0
        self._evidence_key = None
        self._evidence_sequence = 0
        self._independent_key = None
        self._independent_sequence = 0
        self._still_axes = [None, None]
        self.stationary_supported = False
        self._still_generation = None
        self.poses = []
        self.pose_estimator = PoseEstimation(unknown_speed_mps=profile.assumed_body_cap)

    def invalidate(self, source: Source) -> None:
        self.estimator.invalidate(source.value)
        if source is Source.WHEEL:
            self.raw_wheel = None

    def ingest(
        self,
        sample: MotionSample,
        now_ros: float,
        now_steady: float,
        received_ros: float | None = None,
        origins: tuple[Origin, ...] | None = None,
    ) -> bool:
        receipt_ros = now_ros if received_ros is None else received_ros
        source = sample.source.value
        if (
            sample.frame not in (Frame.BODY, Frame.SENSOR)
            or not isinstance(sample, DeltaObservation)
            and sample.frame is not Frame.BODY
        ):
            raise ValueError("motion sample needs an actual body-frame transform")
        roots = origins or tuple(
            Origin(source, sample.epoch, sample.sequence, t, receipt_ros, sample.received_steady)
            for t in dict.fromkeys((sample.start_ros, sample.end_ros))
        )
        if isinstance(sample, DeltaObservation):
            values = logarithm(
                body_delta(sample.delta, sample.body_from_sensor), sample.end_ros - sample.start_ros
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
            interval_half_s=(sample.end_ros - sample.start_ros) / 2,
            epoch=sample.epoch,
            sequence=sample.sequence,
        )
        accepted = self.estimator.ingest(o, now_ros=now_ros, now_steady=now_steady)
        age = self.estimator._age(roots, now_ros, now_steady) if quality != 0 else None
        if accepted and age is not None:
            self.diagnostics.age(source, age, key=(sample.epoch, sample.sequence))
        if accepted and sample.source is Source.WHEEL:
            self.raw_wheel = o if quality != 0 else None
        return accepted

    def ingest_observation(
        self, observation: MotionObservation, now_ros: float, now_steady: float
    ) -> bool:
        accepted = self.estimator.ingest(observation, now_ros=now_ros, now_steady=now_steady)
        age = (
            self.estimator._age(observation.roots, now_ros, now_steady)
            if observation.quality != 0
            else None
        )
        if accepted and age is not None:
            self.diagnostics.age(
                observation.source,
                age,
                key=tuple((r.family, r.epoch, r.sequence) for r in observation.roots),
            )
        return accepted

    def estimate(self, now_ros: float, now_steady: float):
        self.sequence += 1
        r = self.latest = self.estimator.estimate(now_ros, now_steady)
        self.pose_estimator.advance(r)
        for name, retained, capacity in (
            ("motion.estimator.history", self.estimator.retained_samples, 192),
            ("motion.pose.history", self.pose_estimator.retained_samples, 1280),
        ):
            self.diagnostics.queue(
                QueueSnapshot(name, None, capacity, kind="history", retained=retained),
                now_steady,
                owner=self,
            )
        for name, axis in (("vx", r.vx), ("vy", r.vy), ("wz", r.wz)):
            if axis.roots:
                age = self.estimator._age(axis.roots, now_ros, now_steady)
                self.diagnostics.age("motion." + name, age, usage=True)
        if self._still_generation != r.generation:
            self._still_axes = [None, None]
            self._still_generation = r.generation
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
        stamp = max((root.received_steady for root in roots), default=now_steady)
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
                min(root.received_steady for root in independent_roots),
                min(a.deadline_steady for a in independent),
                independent_epoch,
                self._independent_sequence,
                True,
                "independent-sensor current prediction; not a guaranteed speed bound",
            )
        self._observe_stillness(independent)
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
            start_ros=now_ros,
            end_ros=now_ros,
            average=dict(vx=r.vx.value or 0.0, vy=r.vy.value, wz=r.wz.value or 0.0),
            source_health=health,
        )
        return ref, self.tracking, self.tracking

    def _observe_stillness(self, axes):
        # Existing physical reset/stillness evidence, never a driving admission
        # gate. New wheel ticks or projections cannot count as new independent vx.
        config = self.profile.controller()
        for i, limit in enumerate((config.still_speed_mps, config.still_yaw_rps)):
            axis = None if axes is None else axes[i]
            if axis is None or not axis.observed or abs(axis.value) > limit:
                self._still_axes[i] = None
                continue
            key = tuple((r.family, r.epoch, r.sequence) for r in axis.roots)
            domain = tuple(sorted({(r.family, r.epoch) for r in axis.roots}))
            stamp = min(r.received_steady for r in axis.roots)
            prior = self._still_axes[i]
            if prior is None or prior[0] != domain:
                self._still_axes[i] = (domain, key, stamp, stamp, 1)
            elif prior[1] != key:
                self._still_axes[i] = (
                    domain,
                    key,
                    prior[2],
                    stamp,
                    min(config.still_samples, prior[4] + 1),
                )
        self.stationary_supported = all(
            row is not None
            and row[4] >= config.still_samples
            and row[3] >= row[2] + config.still_span_s
            for row in self._still_axes
        )

    def ingest_poses(self, rows):
        for row in rows:
            root = Origin(
                "wheel",
                row["epoch"],
                row["sequence"],
                row["stamp_ros"],
                row["received_ros"],
                row["received_steady"],
            )
            self.pose_estimator.ingest(
                PoseObservation(
                    "wheel_pose", Pose2D(row["x"], row["y"], row["yaw"]), row["stamp_ros"], (root,)
                )
            )

    def wheel_linear(self, now_steady):
        o = self.raw_wheel
        if o is None or self.latest is None:
            return None
        try:
            age = self.estimator._age(o.roots, self.latest.computed_ros, now_steady)
        except ValueError:
            return None
        return None if age.band == "invalid" else o.values[0]

    def report(self):
        return None if self.latest is None else asdict(self.latest)

    def ingest_packet(self, packet):
        """//Compatible: offline tools can supply old raw-source snapshots, not fused votes."""
        for row in packet.get("sources", []):
            s = row["sample"]
            source = row["source"]
            stamp = s.get("stamp_ros", s.get("end_ros"))
            if stamp is None or "values" not in s and "delta" not in s:
                # Condensed historical captures omitted raw values. Never invert
                # their fused interval into an invented independent observation.
                continue
            start = s.get("start_ros", stamp)
            # //Compatible: historical captures lack the receipt ROS clock. The
            # later snapshot clock conservatively charges delay; never assume RTF=1.
            receipt_ros = s.get("received_ros", packet["sampled_ros"])
            values = s.get("values")
            if "delta" in s:
                d = s["delta"]
                values = logarithm(Pose2D(d["x"], d["y"], d["yaw"]), stamp - start)
            elif source == "wheel":
                values = (values[0], None, values[-1])
            elif source == "imu":
                values = (None, None, values[0])
            roots = tuple(
                Origin(source, s["epoch"], s["sequence"], t, receipt_ros, s["received_steady"])
                for t in dict.fromkeys((start, stamp))
            )
            o = MotionObservation(
                source,
                tuple(values),
                (start + stamp) / 2,
                roots,
                s.get("quality", 0.5) if s.get("status", "valid") == "valid" else 0.0,
                independent=(source != "wheel",) * 3,
                interval_half_s=(stamp - start) / 2,
            )
            self.estimator.ingest(o)
            if source == "wheel":
                self.raw_wheel = o
        self.poses = packet.get("poses", [])
