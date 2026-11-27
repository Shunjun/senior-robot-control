"""Actual TF geometry used by the RGB-D algorithm, without source credentials."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Edge:
    parent: str
    child: str
    stamp_ns: int
    translation: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


@dataclass(frozen=True)
class TransformSnapshot:
    edges: tuple[Edge, ...]

    @property
    def geometry(self) -> tuple:
        return tuple((e.parent, e.child, e.translation, e.quaternion) for e in self.edges)


def _valid(snapshot: TransformSnapshot, base: str, camera: str) -> bool:
    if not snapshot.edges:
        return False
    edges = {}
    for edge in snapshot.edges:
        if (
            not edge.parent
            or not edge.child
            or edge.parent == edge.child
            or edge.child in edges
            or edge.stamp_ns < 0
            or len(edge.translation) != 3
            or len(edge.quaternion) != 4
            or not all(math.isfinite(v) for v in (*edge.translation, *edge.quaternion))
            or abs(sum(v * v for v in edge.quaternion) - 1) > 1e-6
        ):
            return False
        edges[edge.child] = edge.parent
    for child in edges:
        seen = set()
        node = child
        while node in edges:
            if node in seen:
                return False
            seen.add(node)
            node = edges[node]
    node = camera
    while node != base and node in edges:
        node = edges[node]
    return node == base


class StaticBinding:
    """Local geometry revision for outstanding algorithm work, not sensor generation."""

    def __init__(self, base: str, camera: str) -> None:
        if not base or not camera or base == camera:
            raise ValueError("invalid_frames")
        self.base, self.camera = base, camera
        self.current: TransformSnapshot | None = None
        self.generation = 0

    @property
    def first_receipt(self) -> TransformSnapshot | None:
        # //Compatible: coordinator's historical name now refers only to TF geometry.
        return self.current

    def observe(self, snapshot: TransformSnapshot) -> bool:
        if not _valid(snapshot, self.base, self.camera):
            self.revoke()
            return False
        if self.current is None or self.current.geometry != snapshot.geometry:
            self.generation += 1
            self.current = snapshot
        return True

    def revoke(self) -> None:
        if self.current is not None:
            self.current = None
            self.generation += 1

    def pair_valid(
        self, first_measurement_steady: float, second_measurement_steady: float, *, generation: int
    ) -> bool:
        return (
            self.current is not None
            and generation == self.generation
            and math.isfinite(first_measurement_steady)
            and math.isfinite(second_measurement_steady)
            and first_measurement_steady < second_measurement_steady
        )


def body_from_camera(
    edges: tuple[Edge, ...], base: str, camera: str
) -> tuple[tuple[float, ...], ...]:
    """Compose body<-camera from the actual transform edges."""
    if not _valid(TransformSnapshot(edges), base, camera):
        raise ValueError("invalid_tf_geometry")
    by_child = {edge.child: edge for edge in edges}
    result = (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    node = camera
    while node != base:
        edge = by_child[node]
        x, y, z, w = edge.quaternion
        tx, ty, tz = edge.translation
        transform = (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), tx),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), ty),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), tz),
            (0.0, 0.0, 0.0, 1.0),
        )
        result = tuple(
            tuple(sum(transform[i][k] * result[k][j] for k in range(4)) for j in range(4))
            for i in range(4)
        )
        node = edge.parent
    return result
