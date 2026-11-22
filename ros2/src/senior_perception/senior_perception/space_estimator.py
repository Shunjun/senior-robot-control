"""One owner combines time-aligned occupied/free evidence; missing is unknown.

Different obstacles are never averaged. Unassociated geometry stays a union,
which also preserves conflicting same-track occupied extents.
"""

import math
from dataclasses import dataclass

from senior_motion_estimation.estimator import Origin, validate_origins
from senior_motion_estimation.kinematics import compose, inverse
from shapely.affinity import rotate, translate
from shapely.geometry import GeometryCollection
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from vehicle_core.sensor_policy import default_policy
from vehicle_core.values import Pose2D


@dataclass(frozen=True)
class SpaceObservation:
    source: str
    occupied: BaseGeometry
    free: BaseGeometry
    roots: tuple[Origin, ...]
    error_m: float | None = 0.0
    frame: str = "scan_reference"
    pose_in_frame: Pose2D | None = None
    frame_epoch: str = "default"
    # //Compatible: append fallback policy after existing positional geometry fields.
    software_error_m: float | None = None


@dataclass(frozen=True)
class SpaceEstimate:
    occupied: BaseGeometry
    free: BaseGeometry
    sources: tuple[str, ...]
    roots: tuple[Origin, ...]
    deadline_steady: float
    frame: str
    reference_pose: Pose2D | None = None
    frame_epoch: str = "default"
    alignment_error_m: float | None = 0.0
    software_alignment_error_m: float = 0.0
    unknown_pose_sources: tuple[str, ...] = ()


class SpaceEstimation:
    def __init__(
        self,
        *,
        frame: str = "scan_reference",
        capacity: int = 32,
        frame_epoch: str = "default",
        software_pose_error_m: float = 0.03,
    ):
        if not frame or not frame_epoch or not 1 <= capacity <= 128:
            raise ValueError("invalid space ownership policy")
        if not math.isfinite(software_pose_error_m) or software_pose_error_m < 0:
            raise ValueError("invalid space software pose error policy")
        self.software_pose_error = software_pose_error_m
        self.frame = frame
        self.capacity = capacity
        self.frame_epoch = frame_epoch
        self._evidence: dict[str, SpaceObservation] = {}
        self._frontiers: dict[str, Origin] = {}
        self._retired: dict[str, set[str]] = {}

    def invalidate(self, source: str) -> None:
        self._evidence.pop(source, None)

    def reset_frame(self, epoch: str) -> None:
        if not epoch:
            raise ValueError("space frame epoch required")
        if epoch != self.frame_epoch:
            self._evidence.clear()
            self.frame_epoch = epoch

    def ingest(self, o: SpaceObservation) -> bool:
        validate_origins(o.roots)
        if (
            not o.source
            or not o.roots
            or o.frame != self.frame
            or o.frame_epoch != self.frame_epoch
            or (
                o.pose_in_frame is not None
                and not all(
                    math.isfinite(v)
                    for v in (o.pose_in_frame.x, o.pose_in_frame.y, o.pose_in_frame.yaw)
                )
            )
            or not o.occupied.is_valid
            or not o.free.is_valid
            or o.error_m is not None
            and (not math.isfinite(o.error_m) or o.error_m < 0)
            or o.software_error_m is not None
            and (not math.isfinite(o.software_error_m) or o.software_error_m < 0)
        ):
            raise ValueError("invalid aligned space evidence")
        root = max(o.roots, key=lambda r: (r.measurement_steady, r.family))
        old = self._frontiers.get(o.source)
        retired = self._retired.setdefault(o.source, set())
        if root.epoch in retired:
            return False
        if old is not None:
            if root.epoch == old.epoch:
                if (
                    root.sequence <= old.sequence
                    or root.measurement_steady < old.measurement_steady
                ):
                    return False
            else:
                if root.received_steady < old.received_steady or len(retired) >= 32:
                    return False
                retired.add(old.epoch)
        elif len(self._frontiers) >= self.capacity:
            raise ValueError("space source capacity")
        self._frontiers[o.source] = root
        self._evidence[o.source] = o
        return True

    def at(
        self,
        now_ros: float,
        now_steady: float,
        *,
        reference_pose: Pose2D | None = None,
        alignment_error_m: float | None = 0.0,
        software_alignment_error_m: float | None = None,
    ) -> SpaceEstimate:
        if reference_pose is not None and not all(
            math.isfinite(v) for v in (reference_pose.x, reference_pose.y, reference_pose.yaw)
        ):
            raise ValueError("invalid space reference pose")
        if (
            alignment_error_m is not None
            and (not math.isfinite(alignment_error_m) or alignment_error_m < 0)
        ) or (
            software_alignment_error_m is not None
            and (not math.isfinite(software_alignment_error_m) or software_alignment_error_m < 0)
        ):
            raise ValueError("invalid relative alignment error")
        live = []
        deadlines: list[float] = []
        p = default_policy()
        for o in self._evidence.values():
            try:
                ages = [
                    p.age(
                        measured_steady=r.measurement_steady,
                        received_steady=r.received_steady,
                        now_steady=now_steady,
                    )
                    for r in o.roots
                ]
            except ValueError:
                continue
            if any(a.band == "invalid" for a in ages):
                continue
            live.append(o)
            deadlines.extend(a.deadline_steady for a in ages)

        def aligned(o: SpaceObservation, geometry: BaseGeometry) -> BaseGeometry:
            origin = o.pose_in_frame or Pose2D(0, 0, 0)
            if reference_pose is not None:
                origin = compose(inverse(reference_pose), origin)
            if origin == Pose2D(0, 0, 0):
                return geometry
            return translate(
                rotate(geometry, origin.yaw, origin=(0, 0), use_radians=True), origin.x, origin.y
            )

        def error(o: SpaceObservation) -> float:
            if reference_pose is not None and o.pose_in_frame == reference_pose:
                # This observation and its reference share the exact same SE(2)
                # transform. Its global pose error cancels, not its sensor error
                # (already charged upstream) or the query's relative prediction.
                return 0.0
            if o.error_m is not None:
                return o.error_m
            return self.software_pose_error if o.software_error_m is None else o.software_error_m

        occupied = (
            unary_union(
                [aligned(o, o.occupied.buffer(error(o)) if error(o) else o.occupied) for o in live]
            )
            if live
            else GeometryCollection()
        )
        free = (
            unary_union(
                [aligned(o, o.free.buffer(-error(o)) if error(o) else o.free) for o in live]
            )
            if live
            else GeometryCollection()
        )
        if software_alignment_error_m is None:
            software_alignment_error_m = (
                self.software_pose_error if alignment_error_m is None else alignment_error_m
            )
        query_error = (
            alignment_error_m if alignment_error_m is not None else software_alignment_error_m
        )
        if query_error:
            occupied = occupied.buffer(query_error)
            free = free.buffer(-query_error)
        return SpaceEstimate(
            occupied,
            free.difference(occupied),
            tuple(o.source for o in live),
            tuple(dict.fromkeys(r for o in live for r in o.roots)),
            min(deadlines, default=now_steady),
            self.frame,
            reference_pose,
            self.frame_epoch,
            alignment_error_m,
            software_alignment_error_m,
            tuple(o.source for o in live if o.error_m is None),
        )
