"""Navigation reference installed only by the owned native SLAM verifier.

Raster cuts and anchor/odometry pairs come from that verifier's private services.
A fresh original scan/odometry receipt advances the body between graph keyframes;
a Home marker or operator seed never supplies a vehicle pose here.
"""

import hashlib
import json
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from senior_mapping.references import rebase_pose
from senior_mapping.shared_scan import CurrentLayerCoverage, ReferenceAuthorization, ScanKey
from senior_mapping.workspace import WorkspaceGrid

from vehicle_core.sample_time import SampleTime
from vehicle_core.sensor_policy import default_policy
from vehicle_core.values import MapRef, Pose2D


@dataclass(frozen=True)
class DisplayObservation:
    key: ScanKey
    source_id: str
    sampled_at: float
    expires_at: float
    beam_count: int
    # Original beam index, measured range, already projected map hit. No reprojection.
    hits: tuple[tuple[int, float, tuple[float, float]], ...]

    def preview(self, budget: int) -> list[dict[str, float]]:
        buckets: dict[int, tuple[float, tuple[float, float]]] = {}
        if budget > 0:
            for index, distance, point in self.hits:
                bucket = min(budget - 1, index * budget // self.beam_count)
                if bucket not in buckets or distance < buckets[bucket][0]:
                    buckets[bucket] = (distance, point)
        return [{"x": buckets[b][1][0], "y": buckets[b][1][1]} for b in sorted(buckets)]


class LiveMappingReference:
    def __init__(self, node: Any, fallback: Any = None) -> None:
        self.node, self.fallback = node, fallback
        self._display: DisplayObservation | None = None
        self.display_reason: str | None = "current_scan_unavailable"
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
        self._authorization: ReferenceAuthorization | None = None
        self._last_authorization: ReferenceAuthorization | None = None
        self._permits: dict[str, tuple[str, str, str]] = {}
        self._accepted_key = None
        self._costmaps: dict[str, CurrentLayerCoverage] = {}
        self._instances: dict[str, str] = {}
        self._cycles: dict[str, int] = {}
        self._coverage = False
        self.coverage_reason = "current_scan_unavailable"
        self._map_owner: Callable[[], bool] | None = None

    def begin(self, boot: str) -> None:
        self.enabled, self.boot = True, boot
        self.withdraw()
        self._source = self._latest = None

    def withdraw(self) -> None:
        self._display = None
        self._authorization = None
        self._costmaps.clear()
        self._accepted_key = None
        self._grid = self._map_basis = self._odom_basis = None
        self._generation = None
        self._available = False
        self.set_current_coverage(False, "reference_unavailable")

    def invalidate_pose(self) -> None:
        self._display = None
        self._available = False
        self.set_current_coverage(False, "current_scan_unavailable")

    def observe_display(
        self,
        key: ScanKey,
        sampled_at: float,
        expires_at: float,
        beam_count: int,
        hits: tuple[tuple[int, float, tuple[float, float]], ...],
    ) -> None:
        # Only the qualified bridge produces this immutable observation.
        if (
            key.sequence > 2**53 - 1
            or self._grid is None
            or self._grid.map_ref.reference_revision > 2**53 - 1
        ):
            self._display = None
            self.display_reason = "display_integer_out_of_range"
            return
        source = dict(zip(tuple(ScanKey.__dataclass_fields__)[:-1], key.source, strict=True))
        source_id = hashlib.sha256(
            json.dumps(source, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode(
                "utf-8"
            )
        ).hexdigest()
        self._display = DisplayObservation(key, source_id, sampled_at, expires_at, beam_count, hits)
        self.display_reason = None

    def resize_preview(self, result: dict[str, Any], budget: int) -> None:
        """Final reply owner may reduce the UI budget, preserving original angular coverage."""
        observation, layer = self._display, result.get("scan_layer")
        if observation is None or layer is None or not layer["valid"]:
            return
        if (layer["source_id"], layer["sequence"]) != (
            observation.source_id,
            observation.key.sequence,
        ):
            result["scan_points"] = []
            layer.update(valid=False, remaining_s=0.0, preview_count=0)
            return
        result["scan_points"] = observation.preview(budget)
        layer["preview_count"] = len(result["scan_points"])

    def set_current_coverage(self, valid: bool, reason: str | None = None) -> None:
        self._coverage = valid
        self.coverage_reason = reason

    def set_navigation_authorization(
        self, record: ReferenceAuthorization, *, accepted_key: ScanKey | None = None
    ) -> None:
        previous = self._last_authorization
        if (
            previous is not None
            and record.owner_instance == previous.owner_instance
            and (
                record.revision <= previous.revision
                or record.authority_epoch < previous.authority_epoch
            )
        ):
            return
        if previous is None or any(
            getattr(previous, name) != getattr(record, name)
            for name in (
                "run_nonce",
                "owner_instance",
                "authority_epoch",
                "coverage_domain_epoch",
                "geometry",
            )
        ):
            self._costmaps.clear()
        for role in ("global", "local"):
            permit = role + "_instance_permit"
            scope = (record.run_nonce, record.host_boot_id, getattr(record, permit))
            if self._permits.get(role) != scope:
                self._permits[role] = scope
                self._costmaps.pop(role, None)
                self._instances.pop(role, None)
                self._cycles.pop(role, None)
        self._authorization = self._last_authorization = record
        self._accepted_key = accepted_key
        if record.status != "selected":
            self._costmaps.clear()

    def observe_costmap_coverage(self, role: str, record: CurrentLayerCoverage) -> None:
        auth = self._authorization
        if role not in ("global", "local"):
            raise ValueError("unknown costmap role")
        if auth is None:
            return
        # Foreign/stale role records cannot supersede a newly rotated permit.
        if (
            any(
                getattr(record, name) != getattr(auth, name)
                for name in (
                    "run_nonce",
                    "host_boot_id",
                    "owner_instance",
                    "authority_epoch",
                )
            )
            or record.role != role
            or record.role_instance_permit != getattr(auth, role + "_instance_permit")
        ):
            return
        instance = self._instances.get(role)
        if instance is not None and record.plugin_instance != instance:
            self._costmaps.pop(role, None)
            return
        if record.cycle <= self._cycles.get(role, -1):
            return
        self._instances[role] = record.plugin_instance
        self._cycles[role] = record.cycle
        self._costmaps[role] = record

    def invalidate_costmap_coverage(self, role: str) -> None:
        self._costmaps.pop(role, None)

    def _navigation_covered(self) -> bool:
        auth, key = self._authorization, self._accepted_key
        now = time.monotonic_ns()
        if (
            not self._coverage
            or auth is None
            or auth.status != "selected"
            or key is None
            or auth.source
            != dict(zip(tuple(ScanKey.__dataclass_fields__)[:-1], key.source, strict=True))
            or not now < auth.lease_until_steady_ns
            or self._grid is None
            or auth.map_id != self._grid.map_ref.map_id
            or auth.reference_revision != self._grid.map_ref.reference_revision
            or auth.reference_generation != self._generation
            or self._latest is None
            or self._latest[0].sample_steady_ns <= auth.navigation_floor_steady_ns
        ):
            return False
        for role in ("global", "local"):
            row = self._costmaps.get(role)
            if (
                row is None
                or row.status != "ready"
                or row.key is None
                or any(
                    getattr(row, name) != getattr(auth, name)
                    for name in (
                        "map_id",
                        "reference_revision",
                        "reference_generation",
                        "backend_boot",
                        "coverage_domain_epoch",
                        "geometry",
                    )
                )
                or tuple(row.key.source) != tuple(key.source)
                or row.key.sequence > key.sequence
            ):
                return False
            if (
                not max(auth.acceptance_floor_steady_ns, auth.navigation_floor_steady_ns)
                < row.sample_steady_ns
                <= row.cycle_completed_steady_ns
                <= now
                or not now < min(row.valid_until_steady_ns, auth.lease_until_steady_ns)
                or row.master_frame not in (("map",) if role == "global" else ("map", "odom"))
                or not row.transform_valid
                or not row.cleanup_complete
                or not getattr(
                    row,
                    role + ("_all_hits_covered" if role == "global" else "_in_window_hits_covered"),
                )
            ):
                return False
        return True

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
            and self._navigation_covered()
        )

    def state(
        self, mode: str, execution_status: str = "idle", *, navigation_ready: bool = False
    ) -> dict[str, Any]:
        if not self.enabled and self.fallback is not None:
            return dict(
                self.fallback.state(mode, execution_status, navigation_ready=navigation_ready)
            )
        pose, grid = self.pose(), self.grid()
        now = time.monotonic()
        observation = self._display
        points: list[dict[str, float]] = []
        layer = None
        if observation is not None and grid is not None:
            remaining = max(0.0, min(0.5, observation.expires_at - now))
            valid = pose is not None and remaining > 0
            points = observation.preview(96) if valid else []
            layer = dict(
                map_ref=dict(
                    map_id=grid.map_ref.map_id, reference_revision=grid.map_ref.reference_revision
                ),
                map_generation=self._generation,
                backend_boot=observation.key.backend_boot,
                source_id=observation.source_id,
                sequence=observation.key.sequence,
                sampled_at=observation.sampled_at,
                expires_at=observation.expires_at,
                remaining_s=remaining if valid else 0.0,
                hit_count=len(observation.hits),
                preview_count=len(points),
                display_mode="preview",
                valid=valid,
            )
        return {
            "v": 1,
            "ok": True,
            "sampled_at": now,
            "localized": pose is not None,
            "safety_clear": self.node.task_motion_allowed(continuing=False),
            "manual_driving": mode == "manual",
            "execution_status": execution_status,
            "map_id": grid.map_ref.map_id if grid else None,
            "reference_revision": grid.map_ref.reference_revision if grid else None,
            "map_generation": self._generation if grid else None,
            "localization_generation": self._generation if pose else None,
            "map_grid_id": grid.grid_id if grid else None,
            "navigation_ready": navigation_ready
            and pose is not None
            and self._navigation_covered(),
            "current_coverage_reason": self.coverage_reason,
            "robot_pose": None if pose is None else {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
            "scan_points": points,
            "scan_layer": layer,
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
