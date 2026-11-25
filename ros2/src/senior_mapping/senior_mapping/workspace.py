"""Bounded map reads from an authoritative, immutable occupancy grid."""

import hashlib
import json
import math
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass
from typing import Any

from senior_mapping.evidence import (
    FIXED_PIXELS,
    OccupancySnapshot,
    fixed_pose,
    fixed_real,
    fixed_uint,
)
from senior_mapping.references import GridExtent
from vehicle_core.values import MapRef, Pose2D

# //Compatible: all map consumers share the same original limit values.
from vehicle_core.workspace_limits import MAX_GRID_CELLS as MAX_GRID_CELLS
from vehicle_core.workspace_limits import MAX_GRID_SIDE as MAX_GRID_SIDE
from vehicle_core.workspace_limits import TILE_SIZE as TILE_SIZE


@dataclass(frozen=True)
class WorkspaceGrid:
    """One map raster cut. Its id changes whenever its reference, geometry or cells change."""

    map_ref: MapRef
    grid_id: str
    width: int
    height: int
    resolution: float
    origin: Pose2D
    # Dense archives use bytes (255 encodes unknown) to bound memory use.
    cells: tuple[int, ...] | bytes
    display_only: bool = False
    waypoint_writes: bool = False

    def __post_init__(self) -> None:
        size = self.width * self.height
        if (
            not self.grid_id.strip()
            or len(self.grid_id) > 128
            or not 0 < self.width <= MAX_GRID_SIDE
            or not 0 < self.height <= MAX_GRID_SIDE
            or size > MAX_GRID_CELLS
            or len(self.cells) != size
            or not math.isfinite(self.resolution)
            or self.resolution <= 0
            or (
                any(cell > 100 and cell != 255 for cell in self.cells)
                if isinstance(self.cells, bytes)
                else any(type(cell) is not int or not -1 <= cell <= 100 for cell in self.cells)
            )
        ):
            raise ValueError("invalid occupancy grid")


# //Compatible: C3 absolute materialization uses the original public grid class.
AbsoluteGrid = WorkspaceGrid


def render_stable_grid(
    map_ref: MapRef,
    grid_id: str,
    extent: GridExtent,
    cell_size_m: float,
    occupancy: OccupancySnapshot,
) -> WorkspaceGrid:
    """Render confirmed occupancy only; absence never proves free space.

    Geometry is bounded before allocating the dense raster. This is a display
    projection, not a localization result or a navigation permission.
    """
    width, height = extent.width, extent.height
    if (
        type(width) is not int
        or type(height) is not int
        or not 0 < width <= MAX_GRID_SIDE
        or not 0 < height <= MAX_GRID_SIDE
        or width * height > MAX_GRID_CELLS
        or not math.isfinite(cell_size_m)
        or cell_size_m <= 0
    ):
        raise ValueError("invalid exported grid geometry")
    origin = Pose2D(extent.origin_x, extent.origin_y, 0.0)
    x0, y0 = math.floor(origin.x / cell_size_m), math.floor(origin.y / cell_size_m)
    pixels = bytearray(b"\xff") * (width * height)
    for evidence in occupancy.cells:
        x, y = evidence.cell[0] - x0, evidence.cell[1] - y0
        if 0 <= x < width and 0 <= y < height:
            pixels[y * width + x] = 100
    return WorkspaceGrid(
        map_ref, grid_id, width, height, cell_size_m, origin, bytes(pixels), display_only=True
    )


class WorkspaceMap:
    """Serve a trusted map cut. Never infer a reference from an untagged ROS topic."""

    def __init__(self, active_grid: Callable[[], WorkspaceGrid | None]) -> None:
        self._active_grid = active_grid

    def read(self, kind: str, fields: Mapping[str, Any]) -> dict[str, Any]:
        grid = self._active_grid()
        if grid is None:
            return {"ok": False, "error": "unavailable"}
        if kind == "map.active":
            return {"ok": True, "map_ref": asdict(grid.map_ref), "grid_id": grid.grid_id}
        if (
            fields.get("map_id") != grid.map_ref.map_id
            or type(fields.get("reference_revision")) is not int
            or fields["reference_revision"] != grid.map_ref.reference_revision
        ):
            return {"ok": False, "error": "conflict"}
        identity = {"ok": True, "map_ref": asdict(grid.map_ref), "grid_id": grid.grid_id}
        if kind == "map.manifest":
            return {
                **identity,
                "width": grid.width,
                "height": grid.height,
                "resolution": grid.resolution,
                "origin": asdict(grid.origin),
                "tile_size": TILE_SIZE,
                **({"display_only": True} if grid.display_only else {}),
                **({"waypoint_writes": True} if grid.waypoint_writes else {}),
            }
        if kind != "map.tile":
            return {"ok": False, "error": "validation"}
        if fields.get("grid_id") != grid.grid_id:
            return {"ok": False, "error": "conflict"}
        tx, ty = fields.get("tile_x"), fields.get("tile_y")
        if (
            type(tx) is not int
            or type(ty) is not int
            or not 0 <= tx < (grid.width + TILE_SIZE - 1) // TILE_SIZE
            or not 0 <= ty < (grid.height + TILE_SIZE - 1) // TILE_SIZE
        ):
            return {"ok": False, "error": "validation"}
        x, y = tx * TILE_SIZE, ty * TILE_SIZE
        width, height = min(TILE_SIZE, grid.width - x), min(TILE_SIZE, grid.height - y)
        cells = [
            -1 if cell == 255 else cell
            for row in range(y, y + height)
            for cell in grid.cells[row * grid.width + x : row * grid.width + x + width]
        ]
        return {
            **identity,
            "tile_x": tx,
            "tile_y": ty,
            "width": width,
            "height": height,
            "cells": cells,
        }


# C3 projection and incremental cache. The authoritative members remain in store.


def fixed_grid_id(map_ref, width, height, resolution, origin, cells) -> str:
    geometry = dict(
        map_ref=asdict(map_ref),
        width=width,
        height=height,
        resolution_m=float(resolution),
        origin=[
            0.0 if origin.x == 0 else float(origin.x),
            0.0 if origin.y == 0 else float(origin.y),
            0.0,
        ],
    )
    return hashlib.sha256(
        json.dumps(
            geometry, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
        ).encode()
        + cells
    ).hexdigest()


def validate_absolute_grid(grid: WorkspaceGrid) -> None:
    for v in (grid.width, grid.height):
        fixed_uint(v)
    fixed_real(grid.resolution, positive=True)
    fixed_pose(grid.origin)
    if (
        not 0 < grid.width <= 65536
        or not 0 < grid.height <= 65536
        or grid.width * grid.height > FIXED_PIXELS
        or grid.origin.yaw != 0
        or type(grid.cells) is not bytes
        or len(grid.cells) != grid.width * grid.height
        or any(v not in (0, 100, 255) for v in grid.cells)
        or type(grid.display_only) is not bool
        or type(grid.waypoint_writes) is not bool
    ):
        raise ValueError("invalid absolute OFU grid")


def _transform(point, pose):
    x, y = point
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + c * x - s * y, pose.y + s * x + c * y


def _inverse(point, pose):
    x, y = point[0] - pose.x, point[1] - pose.y
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return c * x + s * y, -s * x + c * y


def _quad(cell, r, pose):
    x, y = cell[0] * r, cell[1] * r
    return tuple(_transform(p, pose) for p in ((x, y), (x + r, y), (x + r, y + r), (x, y + r)))


def _clip(poly, axis, edge, lower):
    result = []
    if not poly:
        return result
    previous = poly[-1]
    before = previous[axis] >= edge if lower else previous[axis] <= edge
    for current in poly:
        inside = current[axis] >= edge if lower else current[axis] <= edge
        if inside != before:
            f = (edge - previous[axis]) / (current[axis] - previous[axis])
            result.append(tuple(previous[i] + f * (current[i] - previous[i]) for i in range(2)))
        if inside:
            result.append(current)
        previous, before = current, inside
    return result


def _area_in(poly, x, y, r):
    for axis, edge, lower in ((0, x, True), (0, x + r, False), (1, y, True), (1, y + r, False)):
        poly = _clip(poly, axis, edge, lower)
    if not poly:
        return 0.0
    # Translate before shoelace to avoid large-world cancellation.
    p = [(a - x, b - y) for a, b in poly]
    return abs(sum(a[0] * b[1] - a[1] * b[0] for a, b in zip(p, p[1:] + p[:1], strict=False))) * 0.5


@dataclass(frozen=True)
class _Projection:
    occupied: frozenset = frozenset()
    free: frozenset = frozenset()
    # Broad-phase overlap retains partially covered free bits as well as rendered F.
    free_bits: tuple = ()


@dataclass
class RasterPatch:
    records: dict
    reverse: dict
    bit_reverse: dict
    grid: WorkspaceGrid
    extent: tuple
    dirty_tiles: frozenset


class FixedRaster:
    """Rebuildable sparse indices; stage only changed members and dirty cells.

    Immutable dense publication and SHA256 G necessarily read O(N) bytes when
    pixels/extent change. No member reprojection or old scan replay accompanies it.
    Evidence-only updates return the identical dense grid object.
    """

    def __init__(self, map_ref, resolution, origin, width, height, display_only, waypoint_writes):
        self.map_ref, self.r = map_ref, resolution
        self.origin = origin  # persistent lattice origin; extent may grow negative
        self.extent = (0, 0, width, height)
        self.display_only, self.waypoint_writes = display_only, waypoint_writes
        self.records, self.reverse, self.bit_reverse = {}, {}, {}
        self._grid = None
        self.legacy = None
        self.legacy_bound = {}
        self.last_dirty_tiles = frozenset()

    def _cell(self, point):
        return math.floor((point[0] - self.origin.x) / self.r), math.floor(
            (point[1] - self.origin.y) / self.r
        )

    def _candidates(self, quad):
        lo = self._cell((min(p[0] for p in quad), min(p[1] for p in quad)))
        hi = self._cell((max(p[0] for p in quad), max(p[1] for p in quad)))
        if (hi[0] - lo[0] + 1) * (hi[1] - lo[1] + 1) > FIXED_PIXELS:
            raise ValueError("fixed_capacity: projected region")
        for y in range(lo[1], hi[1] + 1):
            for x in range(lo[0], hi[0] + 1):
                area = _area_in(
                    quad, self.origin.x + x * self.r, self.origin.y + y * self.r, self.r
                )
                if area > self.r * self.r * 1e-12:
                    yield (x, y), area

    def support_projection(self, support, pose):
        cells = set()
        for cell in support.local_cells:
            cells.update(c for c, area in self._candidates(_quad(cell, support.resolution_m, pose)))
        cells.update(self._cell(_transform(p, pose)) for p in support.local_endpoints)
        if len(cells) > FIXED_PIXELS:
            raise ValueError("fixed_capacity: occupied projection")
        return _Projection(occupied=frozenset(cells))

    def free_projection(self, tile, pose):
        areas, bits = {}, []
        for local in tile.cells():
            for cell, area in self._candidates(_quad(local, tile.resolution_m, pose)):
                areas[cell] = areas.get(cell, 0.0) + area
                bits.append((cell, local))
        # Area union is exact for disjoint local cells. Require the inverse target
        # corners inside the active cell union too; uncertain boundaries stay U.
        free = set()
        active = set(tile.cells())
        for cell, area in areas.items():
            if abs(area - self.r * self.r) > self.r * self.r * 1e-12:
                continue
            x, y = self.origin.x + cell[0] * self.r, self.origin.y + cell[1] * self.r
            points = ((x, y), (x + self.r, y), (x + self.r, y + self.r), (x, y + self.r))
            # Small inward samples avoid assigning a mathematically exact upper
            # edge to an adjacent half-open cell. No outward coverage is added.
            cx, cy = x + self.r * 0.5, y + self.r * 0.5
            if all(
                (math.floor(lx / tile.resolution_m), math.floor(ly / tile.resolution_m)) in active
                for lx, ly in (
                    _inverse((px + (cx - px) * 1e-12, py + (cy - py) * 1e-12), pose)
                    for px, py in points
                )
            ):
                free.add(cell)
        return _Projection(free=frozenset(free), free_bits=tuple(bits))

    def occupied_cells(self, updates):
        return (
            set().union(*(p.occupied for p in updates.values() if p is not None))
            if updates
            else set()
        )

    def overlapping_free(self, occupied, updates):
        found = {}
        changed = set(updates)
        for cell in occupied:
            for key, local in self.bit_reverse.get(cell, ()):
                if key not in changed:
                    found.setdefault(key[1], set()).add(local)
        for key, projection in updates.items():
            if projection is not None:
                for cell, local in projection.free_bits:
                    if cell in occupied:
                        found.setdefault(key[1], set()).add(local)
        return found

    def install(self, anchors, supports, tiles, legacy):
        poses = {a.anchor_id: a.pose for a in anchors if a.available}
        updates = {
            ("s", s.support_id): self.support_projection(s, poses[s.anchor_id]) for s in supports
        }
        updates.update(
            {("f", t.tile_key): self.free_projection(t, poses[t.anchor_id]) for t in tiles}
        )
        self.legacy = legacy
        if legacy is not None:
            for b in legacy.anchor_bindings:
                for index, quad in zip(b.base_cell_indices, b.local_quads, strict=False):
                    self.legacy_bound[index] = (b.anchor_id, quad)
            updates.update(self.legacy_projections(legacy, poses, self.legacy_bound))
        self.publish(self.prepare(updates))

    def legacy_projections(self, base, poses, indices):
        result = {}
        for index in indices:
            key = ("l", index)
            if not base.live_mask[index // 8] & (1 << (index % 8)):
                result[key] = None
                continue
            anchor, quad = self.legacy_bound[index]
            poly = tuple(_transform((quad[i], quad[i + 1]), poses[anchor]) for i in range(0, 8, 2))
            covered = list(self._candidates(poly))
            value = base.grid.cells[index]
            result[key] = _Projection(
                occupied=frozenset(c for c, a in covered) if value == 100 else frozenset(),
                free=frozenset(
                    c for c, a in covered if abs(a - self.r * self.r) <= self.r * self.r * 1e-12
                )
                if value == 0
                else frozenset(),
            )
        return result

    def legacy_indices(self, cell, legacy):
        # Frozen unbound base is axis aligned but need not share a lattice origin.
        x = self.origin.x + cell[0] * self.r
        y = self.origin.y + cell[1] * self.r
        g = legacy.grid
        x0, y0 = math.floor((x - g.origin.x) / self.r), math.floor((y - g.origin.y) / self.r)
        result = []
        for iy in range(y0, y0 + 2):
            for ix in range(x0, x0 + 2):
                if not 0 <= ix < g.width or not 0 <= iy < g.height:
                    continue
                bx, by = g.origin.x + ix * self.r, g.origin.y + iy * self.r
                if (
                    min(x + self.r, bx + self.r) - max(x, bx) > self.r * 1e-12
                    and min(y + self.r, by + self.r) - max(y, by) > self.r * 1e-12
                ):
                    result.append(iy * g.width + ix)
        return result

    def _legacy_value(self, cell, legacy):
        if legacy is None:
            return 255
        g = legacy.grid
        x = self.origin.x + cell[0] * self.r
        y = self.origin.y + cell[1] * self.r
        area = 0.0
        for index in self.legacy_indices(cell, legacy):
            if index in self.legacy_bound or not legacy.live_mask[index // 8] & (1 << (index % 8)):
                continue
            value = g.cells[index]
            if value == 100:
                return 100
            if value == 0:
                bx = g.origin.x + (index % g.width) * self.r
                by = g.origin.y + (index // g.width) * self.r
                area += (min(x + self.r, bx + self.r) - max(x, bx)) * (
                    min(y + self.r, by + self.r) - max(y, by)
                )
        return 0 if abs(area - self.r * self.r) <= self.r * self.r * 1e-12 else 255

    def prepare(self, updates, *, extent=None, legacy=None, legacy_dirty=()):
        extent = self.extent if extent is None else extent
        x0, y0, x1, y1 = extent
        w, h = x1 - x0, y1 - y0
        if not 0 < w <= 65536 or not 0 < h <= 65536 or w * h > FIXED_PIXELS:
            raise ValueError("coverage_limit: fixed extent")
        reverse, bit_reverse, dirty = {}, {}, set(legacy_dirty)
        for key, new in updates.items():
            old = self.records.get(key, _Projection())
            for cell in (
                old.occupied | old.free | (frozenset() if new is None else new.occupied | new.free)
            ):
                reverse.setdefault(cell, set(self.reverse.get(cell, ())))
                reverse[cell].discard(key)
                if new is not None and cell in new.occupied | new.free:
                    reverse[cell].add(key)
                dirty.add(cell)
            for cell, local in old.free_bits:
                bit_reverse.setdefault(cell, set(self.bit_reverse.get(cell, ()))).discard(
                    (key, local)
                )
            if new is not None:
                for cell, local in new.free_bits:
                    bit_reverse.setdefault(cell, set(self.bit_reverse.get(cell, ()))).add(
                        (key, local)
                    )
        old_grid = self._grid
        changed_extent = extent != self.extent or old_grid is None
        base = self.legacy if legacy is None else legacy
        if changed_extent:
            pixels = bytearray(b"\xff") * (w * h)
            if old_grid is not None:
                ox, oy, ox1, oy1 = self.extent
                for y in range(oy, oy1):
                    start = (y - y0) * w + (ox - x0)
                    pixels[start : start + old_grid.width] = old_grid.cells[
                        (y - oy) * old_grid.width : (y - oy + 1) * old_grid.width
                    ]
            else:
                # Initial restore may include a dense unbound legacy layer.
                for y in range(y0, y1):
                    for x in range(x0, x1):
                        pixels[(y - y0) * w + x - x0] = self._legacy_value((x, y), base)
        else:
            pixels = None
        changes = {}
        for cell in dirty:
            x, y = cell
            if not x0 <= x < x1 or not y0 <= y < y1:
                raise ValueError("fixed evidence falls outside accepted extent")
            value = self._legacy_value(cell, base)
            for key in reverse.get(cell, self.reverse.get(cell, ())):
                p = updates.get(key, self.records.get(key))
                if p is not None:
                    if cell in p.occupied:
                        value = 100
                        break
                    if cell in p.free and value != 100:
                        value = 0
            i = (y - y0) * w + x - x0
            current = pixels[i] if pixels is not None else old_grid.cells[i]
            if value != current:
                changes[i] = value
        if changes and pixels is None:
            pixels = bytearray(old_grid.cells)
        if pixels is None:
            grid = old_grid
        else:
            for i, v in changes.items():
                pixels[i] = v
            cells = bytes(pixels)
            origin = Pose2D(self.origin.x + x0 * self.r, self.origin.y + y0 * self.r, 0.0)
            identity = fixed_grid_id(self.map_ref, w, h, self.r, origin, cells)
            grid = WorkspaceGrid(
                self.map_ref,
                identity,
                w,
                h,
                self.r,
                origin,
                cells,
                self.display_only,
                self.waypoint_writes,
            )
        return RasterPatch(
            updates,
            reverse,
            bit_reverse,
            grid,
            extent,
            frozenset((x // 32, y // 32) for x, y in dirty),
        )

    def publish(self, patch):
        for key, projection in patch.records.items():
            if projection is None:
                self.records.pop(key, None)
            else:
                self.records[key] = projection
        for target, edits in ((self.reverse, patch.reverse), (self.bit_reverse, patch.bit_reverse)):
            for key, value in edits.items():
                if value:
                    target[key] = value
                else:
                    target.pop(key, None)
        self._grid, self.extent = patch.grid, patch.extent
        self.last_dirty_tiles = patch.dirty_tiles

    def grid(self):
        return self._grid
