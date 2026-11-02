"""SIM-only static calibration lifetime; native receipt supplies source proof.

This contract does not authenticate DDS messages. Its caller must bind the
receipt to the current native relay and inspect the graph independently.
"""

import math
from dataclasses import dataclass
from typing import cast


@dataclass(frozen=True)
class Edge:
    parent: str
    child: str
    stamp_ns: int
    translation: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


@dataclass(frozen=True)
class Receipt:
    boot: str
    source_gid: str
    generation: int
    sequence: int
    received_ros_ns: int
    received_steady: float
    edges: tuple[Edge, ...]
    relay_gid: str


def _text(value: str, limit: int) -> bool:
    try:
        return bool(value) and len(value.encode("utf-8")) <= limit
    except (AttributeError, UnicodeError):
        return False


def _finite(value: object) -> bool:
    try:
        return type(value) in (int, float) and math.isfinite(cast(float, value))
    except OverflowError:
        return False


def _valid(receipt: Receipt, base: str, camera: str) -> bool:
    if (
        not _text(receipt.boot, 64)
        or not _text(receipt.source_gid, 128)
        or not _text(receipt.relay_gid, 128)
        or type(receipt.generation) is not int
        or type(receipt.sequence) is not int
        or type(receipt.received_ros_ns) is not int
        or receipt.generation <= 0
        or receipt.sequence <= 0
        or receipt.received_ros_ns < 0
        or not _finite(receipt.received_steady)
        or receipt.received_steady < 0
        or type(receipt.edges) is not tuple
        or not 0 < len(receipt.edges) <= 32
    ):
        return False
    edges = {}
    byte_count = 0
    for edge in receipt.edges:
        if (
            type(edge) is not Edge
            or type(edge.translation) is not tuple
            or type(edge.quaternion) is not tuple
            or not _text(edge.parent, 128)
            or not _text(edge.child, 128)
            or edge.parent == edge.child
            or edge.child in edges
            or type(edge.stamp_ns) is not int
            or edge.stamp_ns < 0
            or len(edge.translation) != 3
            or len(edge.quaternion) != 4
            or not all(_finite(v) for v in (*edge.translation, *edge.quaternion))
            or abs(sum(v * v for v in edge.quaternion) - 1.0) > 1e-6
        ):
            return False
        byte_count += len(edge.parent.encode("utf-8")) + len(edge.child.encode("utf-8")) + 64
        edges[edge.child] = edge.parent
    if byte_count > 16384:
        return False
    # Check every branch, not just the requested chain, for cycles.
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
    """Single static source with explicit lifetime and dual-clock admission."""

    def __init__(self, base: str, camera: str) -> None:
        if not _text(base, 128) or not _text(camera, 128) or base == camera:
            raise ValueError("invalid_frames")
        self.base, self.camera = base, camera
        self.current: Receipt | None = None
        self.first_receipt: Receipt | None = None
        self._last: Receipt | None = None
        self.generation = 0
        self._admitted: tuple[int, float] | None = None
        self._retired_raw: set[str] = set()
        self._retired_relays: set[str] = set()
        self._retired_boots: set[str] = set()
        self._closed = False

    @property
    def admitted_from(self) -> tuple[int, float] | None:
        return self._admitted

    def is_retired_source(self, *, boot: str, source_gid: str, relay_gid: str) -> bool:
        return (source_gid in self._retired_raw or relay_gid in self._retired_relays
                or boot in self._retired_boots)

    def revoke(self) -> None:
        """Revoke a lifetime on topology or clock loss; new native capture required."""
        if self.current is not None:
            self._last = self.current
        self.current = None
        self.first_receipt = None
        self._admitted = None
        self.generation += 1

    def observe(self, receipt: Receipt, *, current_gid: str, current_relay_gid: str) -> bool:
        if self._closed:
            return False
        if (
            receipt.source_gid in self._retired_raw
            or receipt.relay_gid in self._retired_relays
            or receipt.boot in self._retired_boots
        ):
            return False
        # The caller authenticates the native relay using actual MessageInfo and
        # graph state. Old/foreign queued relay messages never revoke the owner.
        if receipt.relay_gid != current_relay_gid:
            return False
        restarting = self.current is None
        old = self.current or self._last
        if current_gid != receipt.source_gid or not _valid(receipt, self.base, self.camera):
            self.revoke()
            return False
        if old is not None:
            relay_changed = receipt.relay_gid != old.relay_gid
            raw_changed = receipt.source_gid != old.source_gid
            if relay_changed:
                if receipt.boot == old.boot:
                    self.revoke()
                    return False
                if len(self._retired_relays) >= 64 or len(self._retired_boots) >= 64:
                    self._closed = True
                    self.revoke()
                    return False
                self._retired_relays.add(old.relay_gid)
                self._retired_boots.add(old.boot)
            elif receipt.boot != old.boot:
                self.revoke()
                return False
            if raw_changed:
                if len(self._retired_raw) >= 64:
                    self._closed = True
                    self.revoke()
                    return False
                self._retired_raw.add(old.source_gid)
            if not relay_changed and not raw_changed:
                if receipt.generation < old.generation or (
                    receipt.generation == old.generation and receipt.sequence <= old.sequence
                ):
                    return False
            if (
                restarting
                or relay_changed
                or raw_changed
                or receipt.generation != old.generation
                or receipt.edges != old.edges
            ):
                self._admitted = None
                self.first_receipt = receipt
                self.generation += 1
        else:
            self.first_receipt = receipt
            self.generation += 1
        self.current = receipt
        self._last = receipt
        return True

    def admit(self, ros_ns: int, steady: float) -> bool:
        if self.current is None or ros_ns <= 0 or not math.isfinite(steady) or steady <= 0:
            return False
        if self._admitted is None:
            self._admitted = (ros_ns, steady)
        return True

    def pair_valid(
        self,
        first_ros_ns: int,
        second_ros_ns: int,
        first_steady: float,
        second_steady: float,
        *,
        generation: int,
        first_received_ros_ns: int,
        second_received_ros_ns: int,
    ) -> bool:
        if self.current is None or self._admitted is None or generation != self.generation:
            return False
        if (
            not all(
                type(value) is int
                for value in (
                    first_ros_ns,
                    second_ros_ns,
                    first_received_ros_ns,
                    second_received_ros_ns,
                )
            )
            or not _finite(first_steady)
            or not _finite(second_steady)
        ):
            return False
        ros, steady = self._admitted
        return (
            first_ros_ns >= ros
            and second_ros_ns > first_ros_ns
            and first_received_ros_ns >= ros
            and second_received_ros_ns >= ros
            and first_steady >= steady
            and second_steady >= first_steady
        )


def body_from_camera(
    edges: tuple[Edge, ...], base: str, camera: str
) -> tuple[tuple[float, ...], ...]:
    """Compose base<-camera from a validated static tree."""
    sample = Receipt("validation", "validation", 1, 1, 0, 0.0, edges, "validation")
    if not _valid(sample, base, camera):
        raise ValueError("invalid_static_tree")
    by_child = {edge.child: edge for edge in edges}
    result: tuple[tuple[float, ...], ...] = (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    node = camera
    while node != base:
        edge = by_child[node]
        norm = math.sqrt(sum(component * component for component in edge.quaternion))
        x, y, z, w = (component / norm for component in edge.quaternion)
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
