"""Serial C2→C3 assembly owned by NativeSession; no transport or archive authority."""

import copy
import math
from collections import Counter
from dataclasses import asdict, replace

from senior_mapping.evidence import (
    FixedDelta,
    FreeTile,
    LegacyMaskEdit,
    ProfileProvenance,
    SourceIdentity,
    StableSupport,
    SupportCellRemoval,
    SupportRevocation,
)
from senior_mapping.occupancy import FreeSpaceAccumulator
from senior_mapping.references import rebase_pose
from senior_mapping.scan_geometry import GeometryTracker
from senior_mapping.workspace import FixedRaster, _inverse, _transform
from vehicle_core.values import Pose2D


class FixedEvidenceConsumer:
    def __init__(self, thresholds, *, profile, clock):
        self.profile = profile
        self.clock = clock
        self.tracker = GeometryTracker(clock_ns=clock)
        self.free = FreeSpaceAccumulator(thresholds)
        self.reason = "profile_missing" if profile is None else None

    def reset(self):
        self.tracker.reset()
        self.free = FreeSpaceAccumulator(self.thresholds)

    @property
    def thresholds(self):
        from senior_mapping.evidence import OccupancyThresholds

        limits = self.free.limits
        return (
            OccupancyThresholds(**asdict(limits)) if limits is not None else OccupancyThresholds()
        )

    def profiles(self):
        if self.profile is None:
            return ()
        values = asdict(self.profile)
        values.pop("profile_id")
        values.pop("evidence_digest")
        return (ProfileProvenance(self.profile.profile_id, self.profile.evidence_digest, values),)

    def consume(self, active, source, scan, basis):
        """Stage bounded transient windows too: failed capacity cannot lose bindings."""
        tracker = copy.deepcopy(self.tracker)
        free = copy.deepcopy(self.free)
        geometry = tracker.update(scan, profile=self.profile)
        cut = active.fixed_snapshot(source)
        revision = cut.watermark.evidence_revision + 1
        poses = {a.anchor_id: a.pose for a in cut.anchors if a.available}
        alignment = rebase_pose(Pose2D(0, 0, 0), basis.odom_basis, basis.map_basis)
        supports = {s.support_id: s for s in cut.supports}
        revoked = tuple(
            SupportRevocation(
                s,
                supports[s].updated_evidence_revision,
                "motion_evidenced",
                scan.key,
                basis.anchor_id,
            )
            for s in geometry.moved_support_ids
            if s in supports
        )
        upserts, bindings, issued = [], [], []
        try:
            upserts, bindings, association_unresolved = self._prepare_supports(
                active,
                source,
                scan,
                basis,
                cut,
                geometry,
                poses,
                alignment,
                revision,
                issued,
                tracker.bound_support_ids,
            )
            tiles, free_world, raster = self._prepare_free_tiles(scan, basis, cut, free, revision)
            removals = self._free_removals(scan, basis, cut, geometry, poses, raster, free_world)
            edits = self._legacy_edits(scan, basis, cut, poses, raster, free_world, upserts)
            delta = FixedDelta(
                cut.watermark,
                scan.key,
                revision,
                basis.anchor_id,
                tuple(upserts),
                revoked,
                tuple(removals),
                tuple(tiles),
                tuple(edits),
            )
            result = active.apply_evidence(
                source,
                expected_graph_sequence=basis.graph_sequence,
                expected_anchor_digest=basis.anchor_digest,
                delta=delta,
            )
        except BaseException:
            active.discard_support_ids(source, tuple(issued))
            raise
        live_ids = {s.support_id for s in result.supports}
        for contour in geometry.stable:
            if contour.support_ids and not set(contour.support_ids) <= live_ids:
                bindings.append(
                    (contour.local_track_id, tuple(s for s in contour.support_ids if s in live_ids))
                )
        for track, identities in bindings:
            tracker.bind_support(track, tuple(s for s in identities if s in live_ids))
        self.tracker, self.free = tracker, free
        self.reason = geometry.unresolved_reason or (
            "persistent_association_unresolved" if association_unresolved else None
        )
        return result

    @staticmethod
    def _same_surface(world, summary, tolerance):
        samples = world[:: max(1, math.ceil(len(world) / 32))]
        return (
            max(min(math.dist(p, q) for q in summary) for p in samples) <= tolerance
            and max(min(math.dist(q, p) for p in world) for q in summary) <= tolerance
        )

    def _prepare_supports(
        self,
        active,
        source,
        scan,
        basis,
        cut,
        geometry,
        poses,
        alignment,
        revision,
        issued,
        reserved,
    ):
        upserts, bindings = [], []
        unresolved = False
        for contour in geometry.stable:
            if contour.support_ids:
                continue
            world = tuple(_transform(p, alignment) for p in contour.odom_surface)
            # Restore association consults only current compact persistent
            # surfaces in this local aperture. Ambiguity creates no alias or
            # revocation and cannot be escaped by assigning a new support ID.
            xs, ys = tuple(p[0] for p in world), tuple(p[1] for p in world)
            aperture = (
                math.hypot(max(xs) - min(xs), max(ys) - min(ys)) + self.profile.motion_residual_m
            )
            nearby = active.local_support_candidates(
                source,
                (min(xs) - aperture, min(ys) - aperture, max(xs) + aperture, max(ys) + aperture),
            )
            if nearby is None:
                unresolved = True
                continue
            candidates, displaced = [], False
            for support in nearby:
                summary = tuple(
                    _transform(p, poses[support.anchor_id]) for p in support.local_surface
                )
                if not summary:
                    continue
                tolerance = self.profile.stable_residual_m
                if self._same_surface(world, summary, tolerance):
                    candidates.append(support.support_id)
                else:
                    # A plausible displaced persistent surface cannot be
                    # bypassed by issuing a fresh ID after restart. Without
                    # the old transient motion window it remains unresolved.
                    wc = ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2)
                    sx, sy = tuple(p[0] for p in summary), tuple(p[1] for p in summary)
                    sc = ((min(sx) + max(sx)) / 2, (min(sy) + max(sy)) / 2)
                    shifted = tuple((x + wc[0] - sc[0], y + wc[1] - sc[1]) for x, y in summary)
                    displaced |= self._same_surface(world, shifted, tolerance)
            if displaced:
                unresolved = True
                continue
            if len(candidates) == 1:
                bindings.append((contour.local_track_id, tuple(candidates)))
                continue
            if candidates:
                unresolved = True
                continue
            local = tuple(_inverse(p, basis.map_basis) for p in world)
            cells = tuple(
                sorted(
                    {
                        (
                            math.floor(x / cut.grid.resolution),
                            math.floor(y / cut.grid.resolution),
                        )
                        for x, y in local
                    }
                )
            )
            identity = active.allocate_support_id(source)
            issued.append(identity)
            upserts.append(
                StableSupport(
                    identity,
                    basis.anchor_id,
                    cut.grid.resolution,
                    local[:: max(1, math.ceil(len(local) / 32))],
                    local,
                    cells,
                    contour.first_key,
                    contour.last_key,
                    SourceIdentity.from_key(scan.key),
                    "geometric",
                    self.profile.profile_id,
                    self.profile.evidence_digest,
                    revision,
                    revision,
                )
            )
            bindings.append((contour.local_track_id, (identity,)))
        counts = Counter(support for _track, ids in bindings for support in ids)
        unique = [
            (track, ids)
            for track, ids in bindings
            if all(counts[s] == 1 and s not in reserved for s in ids)
        ]
        unresolved |= len(unique) != len(bindings)
        return upserts, unique, unresolved

    def _prepare_free_tiles(self, scan, basis, cut, free, revision):
        legacy_beliefs = {}
        if cut.legacy_base is not None:
            for binding in cut.legacy_base.anchor_bindings:
                legacy_beliefs.update(
                    (i, original.evidence.belief)
                    for i, original in zip(
                        binding.base_cell_indices, binding.original_bindings, strict=True
                    )
                )

        def initial_belief(cell):
            r = cut.grid.resolution
            point = _transform(((cell[0] + 0.5) * r, (cell[1] + 0.5) * r), basis.map_basis)
            x = math.floor((point[0] - cut.grid.origin.x) / r)
            y = math.floor((point[1] - cut.grid.origin.y) / r)
            if not (
                0 <= x < cut.grid.width
                and 0 <= y < cut.grid.height
                and cut.grid.cells[y * cut.grid.width + x] == 100
            ):
                return 0.0
            base = cut.legacy_base
            if base is not None:
                bx = math.floor((point[0] - base.grid.origin.x) / r)
                by = math.floor((point[1] - base.grid.origin.y) / r)
                if 0 <= bx < base.grid.width and 0 <= by < base.grid.height:
                    prior = legacy_beliefs.get(by * base.grid.width + bx)
                    if prior is not None:
                        return prior
            # A restored geometric support has no transient belief window;
            # preserve the original strongest bounded occupied prior.
            return free.limits.belief_max

        qualified = free.update(scan, basis, initial_belief=initial_belief)
        old_tiles = {t.tile_key: t for t in cut.free_tiles}
        grouped = {}
        for x, y in qualified:
            tx, ty = x // 32, y // 32
            grouped.setdefault((tx, ty), set()).add((x, y))
        tiles = []
        for (tx, ty), cells in sorted(grouped.items()):
            key = (basis.anchor_id, cut.grid.resolution, tx, ty)
            old = old_tiles.get(key)
            mask = bytearray(128 if old is None else old.active_mask)
            for x, y in cells:
                bit = (y % 32) * 32 + x % 32
                mask[bit // 8] |= 1 << (bit % 8)
            tiles.append(
                FreeTile(
                    basis.anchor_id,
                    cut.grid.resolution,
                    tx,
                    ty,
                    bytes(mask),
                    1 if old is None else old.tile_revision + 1,
                    scan.key,
                    SourceIdentity.from_key(scan.key),
                    revision,
                )
            )
        raster = FixedRaster(
            cut.grid.map_ref,
            cut.grid.resolution,
            cut.grid.origin,
            cut.grid.width,
            cut.grid.height,
            cut.grid.display_only,
            cut.grid.waypoint_writes,
        )
        free_world = set()
        for tile in tiles:
            mask = bytearray(128)
            for x, y in grouped[tile.tile_x, tile.tile_y]:
                bit = y % 32 * 32 + x % 32
                mask[bit // 8] |= 1 << (bit % 8)
            observed = replace(tile, active_mask=bytes(mask))
            free_world.update(raster.free_projection(observed, basis.map_basis).free)
        return tiles, free_world, raster

    def _free_removals(self, scan, basis, cut, geometry, poses, raster, free_world):
        removals = []
        for s in cut.supports:
            if s.support_id in geometry.moved_support_ids:
                continue
            cells = []
            for cell in s.local_cells:
                # Full cell coverage is checked with the same renderer used
                # by C3, including endpoint quantization at corrected borders.
                endpoints = tuple(
                    p
                    for p in s.local_endpoints
                    if (math.floor(p[0] / s.resolution_m), math.floor(p[1] / s.resolution_m))
                    == cell
                )
                fragment = replace(
                    s, local_cells=(cell,), local_surface=(), local_endpoints=endpoints
                )
                if raster.support_projection(fragment, poses[s.anchor_id]).occupied <= free_world:
                    cells.append(cell)
            if cells:
                removals.append(
                    SupportCellRemoval(
                        s.support_id,
                        s.updated_evidence_revision,
                        tuple(cells),
                        "observed_free_space",
                        scan.key,
                        basis.anchor_id,
                    )
                )
        return removals

    def _legacy_edits(self, scan, basis, cut, poses, raster, free_world, upserts):
        edits = []
        if cut.legacy_base is not None:
            base = cut.legacy_base
            raster.install(cut.anchors, (), (), base)
            selected = set()
            occupied = set()
            for s in upserts:
                occupied.update(raster.support_projection(s, poses[s.anchor_id]).occupied)
            for cell in free_world | occupied:
                for index in raster.legacy_indices(cell, base):
                    if not base.live_mask[index // 8] & (1 << (index % 8)):
                        continue
                    coverage = {c for c, _ in raster.legacy_coverage(index, base, poses)}
                    if cell in occupied or coverage <= free_world:
                        selected.add(index)
            if selected:
                indices = tuple(sorted(selected))
                edits.append(
                    LegacyMaskEdit(
                        base.base_id,
                        indices,
                        tuple(base.grid.cells[i] for i in indices),
                        "observed_free_space",
                        scan.key,
                        basis.anchor_id,
                    )
                )
        return edits
