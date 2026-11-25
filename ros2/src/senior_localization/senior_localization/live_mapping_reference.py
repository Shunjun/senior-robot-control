"""Navigation reference installed only by the owned native SLAM verifier.

Raster cuts and anchor/odometry pairs come from that verifier's private services.
A fresh original scan/odometry receipt advances the body between graph keyframes;
a Home marker or operator seed never supplies a vehicle pose here.
"""

import time
from collections.abc import Callable
from typing import Any

from senior_mapping.references import rebase_pose
from senior_mapping.workspace import WorkspaceGrid

from vehicle_core.sample_time import SampleTime
from vehicle_core.sensor_policy import default_policy
from vehicle_core.values import MapRef, Pose2D


class LiveMappingReference:
    def __init__(self, node: Any, fallback: Any = None) -> None:
        self.node, self.fallback = node, fallback
        self.enabled = False
        self.boot = ""
        self.safety_reason = "motion runtime qualification required"
        self.safety_ages = ""
        self._grid: WorkspaceGrid | None = None
        self._generation: str | None = None
        self._anchor_id = ""
        self._map_basis: Pose2D | None = None
        self._odom_basis: Pose2D | None = None
        self._source: tuple[Any, ...] | None = None
        self._latest: tuple[SampleTime, Pose2D, int] | None = None
        self._available = False
        self._coverage = False
        self.coverage_reason = "current_scan_unavailable"
        self._map_owner: Callable[[], bool] | None = None

    def begin(self, boot: str) -> None:
        self.enabled, self.boot = True, boot
        self.withdraw()
        self._source = self._latest = None

    def withdraw(self) -> None:
        self._grid = self._map_basis = self._odom_basis = None
        self._generation = None
        self._available = False
        self.set_current_coverage(False, "reference_unavailable")

    def invalidate_pose(self) -> None:
        self._available = False
        self.set_current_coverage(False, "current_scan_unavailable")

    def set_current_coverage(self, valid: bool, reason: str | None = None) -> None:
        self._coverage = valid
        self.coverage_reason = reason

    def install(
        self,
        boot: str,
        grid: WorkspaceGrid,
        generation: str,
        anchor_id: str,
        map_basis: Pose2D,
        odom_basis: Pose2D,
    ) -> None:
        if not self.enabled or boot != self.boot or not generation or grid.display_only:
            raise ValueError("mapping reference is not an owned qualified cut")
        if (
            self._grid is None
            or self._grid.map_ref != grid.map_ref
            or self._generation != generation
        ):
            self.invalidate_pose()
        self._grid, self._generation = grid, generation
        self._anchor_id, self._map_basis, self._odom_basis = anchor_id, map_basis, odom_basis

    def observe(self, boot: str, header: Any, odometry: Pose2D) -> None:
        if boot != self.boot or not header.valid:
            self.invalidate_pose()
            return
        source = (
            header.bridge_boot_id,
            header.source_epoch,
            header.capture_generation,
            header.clock_generation,
            header.raw_gid,
            header.bridge_gid,
        )
        if self._source is not None and source != self._source:
            self.invalidate_pose()
            return
        sample = SampleTime(
            header.stamp.sec * 1_000_000_000 + header.stamp.nanosec,
            header.received_ros.sec * 1_000_000_000 + header.received_ros.nanosec,
            header.received_steady_ns,
            header.sample_steady_ns,
            header.clock_generation,
        )
        if self._latest is not None and header.sequence <= self._latest[2]:
            # A delayed receipt has no authority over a newer accepted sample.
            if (
                header.sequence == self._latest[2]
                and (sample, odometry, header.sequence) != self._latest
            ):
                self.invalidate_pose()
            return
        self._source = source
        self._latest = (sample, odometry, header.sequence)
        self._available = True

    @property
    def map_generation(self) -> str | None:
        if not self.enabled:
            return getattr(self.fallback, "map_generation", None) or (
                self.fallback.grid().grid_id if self.fallback and self.fallback.grid() else None
            )
        return self._generation

    def set_map_owner(self, check: Callable[[], bool]) -> None:
        self._map_owner = check

    def map_to_odom(self) -> Pose2D | None:
        if self.pose() is None or self._map_basis is None or self._odom_basis is None:
            return None
        return rebase_pose(Pose2D(0.0, 0.0, 0.0), self._odom_basis, self._map_basis)

    def grid(self) -> WorkspaceGrid | None:
        if not self.enabled:
            return self.fallback.grid() if self.fallback is not None else None
        if self._map_owner is not None and not self._map_owner():
            return None
        return self._grid

    def pose(self) -> Pose2D | None:
        if not self.enabled:
            return self.fallback.pose() if self.fallback is not None else None
        if (
            not self._available
            or self._latest is None
            or self.grid() is None
            or self._map_basis is None
            or self._odom_basis is None
        ):
            return None
        sample, odometry, _ = self._latest
        now = time.monotonic_ns()
        if now < sample.received_steady_ns:
            return None
        age = default_policy().sample_age(sample, now_steady_ns=now)
        if age.band == "invalid" or age.seconds > 0.5:
            return None
        return rebase_pose(odometry, self._odom_basis, self._map_basis)

    def qualified(self, ref: MapRef, generation: str) -> bool:
        if not self.enabled:
            return self.fallback is not None and bool(self.fallback.qualified(ref, generation))
        return (
            self._grid is not None
            and ref == self._grid.map_ref
            and generation == self._generation
            and self.pose() is not None
            and self._coverage
        )

    def state(
        self, mode: str, execution_status: str = "idle", *, navigation_ready: bool = False
    ) -> dict[str, Any]:
        if not self.enabled and self.fallback is not None:
            return dict(
                self.fallback.state(mode, execution_status, navigation_ready=navigation_ready)
            )
        pose, grid = self.pose(), self.grid()
        return {
            "v": 1,
            "ok": True,
            "sampled_at": time.monotonic(),
            "localized": pose is not None,
            "safety_clear": self.node.task_motion_allowed(continuing=False),
            "manual_driving": mode == "manual",
            "execution_status": execution_status,
            "map_id": grid.map_ref.map_id if grid else None,
            "reference_revision": grid.map_ref.reference_revision if grid else None,
            "map_generation": self._generation if grid else None,
            "localization_generation": self._generation if pose else None,
            "map_grid_id": grid.grid_id if grid else None,
            "navigation_ready": navigation_ready and pose is not None and self._coverage,
            "current_coverage_reason": self.coverage_reason,
            "robot_pose": None if pose is None else {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
            "scan_points": [],
        }

    def pose_reply(self) -> dict[str, Any]:
        if not self.enabled:
            return (
                dict(self.fallback.pose_reply())
                if self.fallback
                else {"ok": False, "error": "unavailable"}
            )
        pose = self.pose()
        if pose is None or self._grid is None or self._latest is None or self._map_basis is None:
            return {"ok": False, "error": "unavailable"}
        anchor = self._map_basis
        return {
            "v": 1,
            "ok": True,
            "healthy": True,
            "sampled_at": self._latest[0].sample_steady_ns / 1e9,
            "map_id": self._grid.map_ref.map_id,
            "reference_revision": self._grid.map_ref.reference_revision,
            "map_generation": self._generation,
            "anchor_id": self._anchor_id,
            "anchor_x": anchor.x,
            "anchor_y": anchor.y,
            "anchor_yaw": anchor.yaw,
            "x": pose.x,
            "y": pose.y,
            "yaw": pose.yaw,
        }
