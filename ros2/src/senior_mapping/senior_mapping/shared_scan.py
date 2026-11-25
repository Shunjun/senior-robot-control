"""Immutable qualified facts and the private, replace-only current-scan mailbox.

The basis is a copy of the graph owner's verified cut, never a localization owner.
Readers compare revision (not ScanKey): invalidation and retirement may have no scan.
Steady coordinates are meaningful only on the recorded host boot.
"""

import json
import math
import os
import stat
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

from senior_perception.external_motion_features import external_feature_ranges
from senior_perception.learning_view import LearningFilterView
from senior_perception.self_filter import ScanObservation, filter_self
from senior_perception.vision.raw_capture import RawCaptureHeader, capture_from_message

from vehicle_core.config import DeviceProfile
from vehicle_core.sample_time import SampleTime
from vehicle_core.values import Pose2D

# 320 encoded bytes/beam includes four worst-case binary64 coordinates, index,
# kind and optional limit, field names and punctuation. 32KiB bounds the envelope.
MAX_BEAMS = 2048
MAX_SLOT_BYTES = MAX_BEAMS * 320 + 32768


def _text(value: str, limit: int) -> None:
    if not isinstance(value, str) or not 1 <= len(value.encode("utf-8")) <= limit:
        raise ValueError("scan metadata text exceeds bound")


def _u64(value: int) -> None:
    if type(value) is not int or not 0 <= value < 1 << 64:
        raise ValueError("scan metadata is not uint64")


@dataclass(frozen=True)
class ScanKey:
    backend_boot: str
    bridge_boot_id: str
    source_epoch: str
    capture_generation: int
    clock_generation: int
    raw_gid: str
    bridge_gid: str
    sequence: int

    def __post_init__(self) -> None:
        _text(self.backend_boot, 36)
        for value in (self.bridge_boot_id, self.source_epoch):
            _text(value, 64)
        for value in (self.raw_gid, self.bridge_gid):
            _text(value, 128)
        for value in (self.capture_generation, self.clock_generation, self.sequence):
            _u64(value)

    @classmethod
    def of(cls, boot: str, capture: RawCaptureHeader) -> "ScanKey":
        return cls(
            boot,
            capture.bridge_boot_id,
            capture.source_epoch,
            capture.capture_generation,
            capture.time.clock_generation,
            capture.raw_gid,
            capture.bridge_gid,
            capture.sequence,
        )

    @property
    def source(self) -> tuple:
        return tuple(asdict(self).values())[:-1]


@dataclass(frozen=True)
class Beam:
    index: int
    origin_odom: tuple[float, float]
    endpoint_odom: tuple[float, float] | None
    kind: Literal["hit", "self", "invalid", "clear"]
    clear_limit_m: float | None = None

    def __post_init__(self) -> None:
        if (
            type(self.index) is not int
            or not 0 <= self.index < MAX_BEAMS
            or self.kind not in ("hit", "self", "invalid", "clear")
            or len(self.origin_odom) != 2
            or not all(math.isfinite(v) for v in self.origin_odom)
            or (self.kind == "hit" and self.endpoint_odom is None)
        ):
            raise ValueError("invalid beam")
        if self.endpoint_odom is not None and (
            len(self.endpoint_odom) != 2 or not all(math.isfinite(v) for v in self.endpoint_odom)
        ):
            raise ValueError("invalid endpoint")
        if self.clear_limit_m is not None and not math.isfinite(self.clear_limit_m):
            raise ValueError("invalid clear limit")


@dataclass(frozen=True)
class QualifiedScan:
    key: ScanKey
    capture: RawCaptureHeader
    geometry_id: str
    odom_base: Pose2D
    beams: tuple[Beam, ...]
    valid_until_steady_ns: int
    host_boot_id: str

    def __post_init__(self) -> None:
        _text(self.host_boot_id, 36)
        _text(self.geometry_id, 128)
        _text(self.capture.frame_id, 128)
        if (
            self.key != ScanKey.of(self.key.backend_boot, self.capture)
            or not self.capture.valid
            or len(self.beams) > MAX_BEAMS
            or tuple(beam.index for beam in self.beams) != tuple(range(len(self.beams)))
            or self.valid_until_steady_ns != self.capture.time.sample_steady_ns + 500_000_000
        ):
            raise ValueError("qualified scan identity or deadline changed")


def _point(point: tuple[float, float], pose: Pose2D) -> tuple[float, float]:
    x, y = point
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return pose.x + c * x - s * y, pose.y + s * x + c * y


def decode_qualified_scan(
    message: object,
    profile: DeviceProfile,
    *,
    learning_view: LearningFilterView | None,
    host_boot_id: str,
) -> QualifiedScan:
    raw = message.scan
    capture = capture_from_message(raw.capture)
    if (
        message.status != 1
        or not capture.valid
        or not message.static_receipt_valid
        or message.raw_ray_angle_sign not in (-1, 1)
        or not 1 <= len(raw.ranges) <= MAX_BEAMS
        or not 1 <= len(message.geometry_id) <= 128
    ):
        raise ValueError("unqualified scan")
    mount, odom = message.base_lidar, message.odom_base
    q = mount.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    base = Pose2D(odom.x, odom.y, odom.theta)
    if not all(
        math.isfinite(v)
        for v in (base.x, base.y, base.yaw, mount.translation.x, mount.translation.y, yaw)
    ):
        raise ValueError("nonfinite qualified geometry")
    # filter_self requires positive angular order. Reverse only for filtering, then
    # map every result back to the original raw beam index, including inverted mounts.
    inverted = message.raw_ray_angle_sign == -1
    ranges = tuple(
        float(v) if math.isfinite(v) and v < raw.range_max else math.nan for v in raw.ranges
    )  # no protocol proof for finite max or no-return
    first = (
        -(raw.angle_min + (len(ranges) - 1) * raw.angle_increment) if inverted else raw.angle_min
    )
    seconds = capture.time.stamp_ns / 1e9
    observation = ScanObservation(
        ranges[::-1] if inverted else ranges,
        first,
        raw.angle_increment,
        raw.range_min,
        raw.range_max,
        seconds,
        "ros",
        Pose2D(mount.translation.x, mount.translation.y, yaw),
        seconds,
        "base_link",
        seconds,
        transform_static=True,
    )
    filtered = filter_self(observation, profile)
    external = external_feature_ranges(filtered, learning_view)
    beams = []
    for ray in filtered.rays:
        kind = ray.kind
        if kind == "hit" and not math.isfinite(external[ray.index]):
            kind = "self"
        index = len(ranges) - 1 - ray.index if inverted else ray.index
        beams.append(
            Beam(
                index,
                _point(ray.origin, base),
                None if ray.endpoint is None else _point(ray.endpoint, base),
                kind,
            )
        )
    return QualifiedScan(
        ScanKey.of(message.backend_boot, capture),
        capture,
        message.geometry_id,
        base,
        tuple(sorted(beams, key=lambda b: b.index)),
        capture.time.sample_steady_ns + 500_000_000,
        host_boot_id,
    )


def project_hits(scan: QualifiedScan, map_to_odom: Pose2D) -> tuple[tuple[float, float], ...]:
    return tuple(
        _point(b.endpoint_odom, map_to_odom)
        for b in scan.beams
        if b.kind == "hit" and b.endpoint_odom is not None
    )


@dataclass(frozen=True)
class ReferenceBasis:
    map_id: str
    reference_revision: int
    reference_generation: str
    graph_sequence: int
    anchor_digest: str
    anchor_id: str
    map_basis: Pose2D
    odom_basis: Pose2D

    def __post_init__(self) -> None:
        for value in (self.map_id, self.reference_generation, self.anchor_id):
            _text(value, 128)
        if (
            not isinstance(self.anchor_digest, str)
            or len(self.anchor_digest) != 64
            or any(char not in "0123456789abcdef" for char in self.anchor_digest)
        ):
            raise ValueError("anchor digest must be SHA256 lowercase hex")
        _u64(self.reference_revision)
        _u64(self.graph_sequence)

    @property
    def token(self) -> tuple:
        return self.map_id, self.reference_revision, self.reference_generation


@dataclass(frozen=True)
class ScanEnvelope:
    schema: int
    revision: int
    status: Literal["valid", "invalid", "retired"]
    reason: str
    session_nonce: str
    host_boot_id: str
    backend_boot: str
    key: ScanKey | None
    acceptance_floor: ScanKey | None
    acceptance_floor_steady_ns: int
    basis: ReferenceBasis | None
    valid_until_steady_ns: int
    scan: QualifiedScan | None


@dataclass(frozen=True)
class SlotRead:
    changed: bool
    envelope: ScanEnvelope | None = None


def _decode(data: dict) -> ScanEnvelope:
    def key(row):
        return None if row is None else ScanKey(**row)

    basis = data["basis"]
    if basis is not None:
        basis = ReferenceBasis(
            **{
                **basis,
                "map_basis": Pose2D(**basis["map_basis"]),
                "odom_basis": Pose2D(**basis["odom_basis"]),
            }
        )
    scan = data["scan"]
    if scan is not None:
        capture = scan["capture"]
        capture = RawCaptureHeader(**{**capture, "time": SampleTime(**capture["time"])})
        beams = tuple(
            Beam(
                **{
                    **b,
                    "origin_odom": tuple(b["origin_odom"]),
                    "endpoint_odom": None
                    if b["endpoint_odom"] is None
                    else tuple(b["endpoint_odom"]),
                }
            )
            for b in scan["beams"]
        )
        if len(beams) > MAX_BEAMS:
            raise ValueError("scan capacity")
        scan = QualifiedScan(
            **{
                **scan,
                "key": key(scan["key"]),
                "capture": capture,
                "odom_base": Pose2D(**scan["odom_base"]),
                "beams": beams,
            }
        )
    return ScanEnvelope(
        **{
            **data,
            "key": key(data["key"]),
            "acceptance_floor": key(data["acceptance_floor"]),
            "basis": basis,
            "scan": scan,
        }
    )


class LatestScanSlot:
    """One encoder/writer and one replaceable candidate; no scan RPC or archive fsync.

    The serial writer cannot replace a published retirement with an older in-flight
    write. A reference transition publishes invalid and pins the last admitted key
    as its floor; it never reprojects the previous scan under a new R.
    """

    def __init__(self, path: Path, *, nonce: str, backend_boot: str, host_boot_id: str):
        for value in (nonce, backend_boot, host_boot_id):
            _text(value, 36)
        self.path = path
        directory = path.parent.lstat()
        if (
            not stat.S_ISDIR(directory.st_mode)
            or directory.st_uid != os.getuid()
            or directory.st_mode & 0o077
        ):
            raise ValueError("scan directory must be private and non-symlink")
        self.nonce, self.boot, self.host = nonce, backend_boot, host_boot_id
        self._condition = threading.Condition()
        self._revision = 0
        self._basis = None
        self._last = self._floor = None
        self._floor_steady_ns = 0
        self._retired = self._closed = self._busy = False
        self._pending = None
        self.error = None
        self._thread = threading.Thread(target=self._run, name="latest-mapping-scan")
        self._thread.start()
        self.invalidate("starting")

    def _offer(self, status, scan, reason):
        if len(reason.encode("utf-8")) > 64:
            raise ValueError("scan reason exceeds bound")
        self._revision += 1
        self._pending = ScanEnvelope(
            1,
            self._revision,
            status,
            reason,
            self.nonce,
            self.host,
            self.boot,
            None if scan is None else scan.key,
            self._floor,
            self._floor_steady_ns,
            self._basis,
            0 if scan is None else scan.valid_until_steady_ns,
            scan,
        )
        self._condition.notify()

    def install_reference(
        self, basis: ReferenceBasis, *, floor_steady_ns: int | None = None
    ) -> None:
        with self._condition:
            if self._closed:
                return
            if self._basis is None or self._basis.token != basis.token:
                self._floor = self._last
                self._floor_steady_ns = (
                    time.monotonic_ns() if floor_steady_ns is None else floor_steady_ns
                )
                self._retired = False
                self._basis = basis
                self._offer("invalid", None, "reference_changed")
            else:
                self._basis = basis

    def write(self, scan: QualifiedScan) -> bool:
        with self._condition:
            if (
                self._closed
                or self._retired
                or scan.key.backend_boot != self.boot
                or scan.host_boot_id != self.host
                or scan.capture.time.sample_steady_ns <= self._floor_steady_ns
            ):
                return False
            if self._last is not None and (
                scan.key.source != self._last.source or scan.key.sequence <= self._last.sequence
            ):
                return False
            self._last = scan.key
            self._offer("valid", scan, "")
            return True

    def invalidate(self, reason="invalid") -> None:
        with self._condition:
            if not self._closed and not self._retired:
                self._offer("invalid", None, reason)

    def retire(self) -> None:
        with self._condition:
            if not self._closed and not self._retired:
                self._retired = True
                self._floor = self._last
                self._offer("retired", None, "retired")

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._pending is not None or self._closed)
                if self._pending is None:
                    return
                row, self._pending, self._busy = self._pending, None, True
            temporary = self.path.with_suffix(".tmp")
            try:
                payload = json.dumps(asdict(row), allow_nan=False, separators=(",", ":")).encode()
                if len(payload) > MAX_SLOT_BYTES:
                    raise ValueError("scan encoding exceeds calculated bound")
                fd = os.open(
                    temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600
                )
                with os.fdopen(fd, "wb") as stream:
                    os.fchmod(stream.fileno(), 0o600)
                    stream.write(payload)
                # Only this worker replaces the inode; later revisions always win.
                os.replace(temporary, self.path)
                self.error = None
            except (OSError, ValueError) as exc:
                self.error = str(exc)
            finally:
                with self._condition:
                    self._busy = False
                    self._condition.notify_all()

    def flush(self):
        """Lifecycle/test barrier, never called from the scan callback."""
        with self._condition:
            self._condition.wait_for(lambda: self._pending is None and not self._busy)
        if self.error:
            raise OSError(self.error)

    def read_after(self, revision: int) -> SlotRead:
        return read_latest_scan(
            self.path, revision, nonce=self.nonce, backend_boot=self.boot, host_boot_id=self.host
        )

    def close(self):
        self.retire()
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        self._thread.join()


def read_latest_scan(
    path: Path, revision: int, *, nonce: str, backend_boot: str, host_boot_id: str
) -> SlotRead:
    """Open one bounded inode; consumers also fence basis against their graph cut."""
    directory_fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    try:
        directory = os.fstat(directory_fd)
        if directory.st_uid != os.getuid() or directory.st_mode & 0o077:
            raise ValueError("untrusted scan directory")
        try:
            fd = os.open(
                path.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory_fd
            )
        except FileNotFoundError:
            return SlotRead(False)
    finally:
        os.close(directory_fd)
    with os.fdopen(fd, "rb") as stream:
        info = os.fstat(stream.fileno())
        if (
            not stat.S_ISREG(info.st_mode)
            or info.st_uid != os.getuid()
            or info.st_mode & 0o077
            or info.st_size > MAX_SLOT_BYTES
        ):
            raise ValueError("untrusted scan inode")
        payload = stream.read(MAX_SLOT_BYTES + 1)
    if len(payload) > MAX_SLOT_BYTES:
        raise ValueError("scan inode grew beyond budget")
    row = _decode(json.loads(payload))
    if (
        row.schema != 1
        or row.session_nonce != nonce
        or row.backend_boot != backend_boot
        or row.host_boot_id != host_boot_id
        or row.status not in ("valid", "invalid", "retired")
    ):
        raise ValueError("scan envelope identity changed")
    _u64(row.revision)
    _u64(row.acceptance_floor_steady_ns)
    if row.status == "valid":
        if (
            row.scan is None
            or row.key != row.scan.key
            or row.key.backend_boot != backend_boot
            or row.scan.host_boot_id != host_boot_id
            or row.valid_until_steady_ns != row.scan.valid_until_steady_ns
            or row.scan.capture.time.sample_steady_ns <= row.acceptance_floor_steady_ns
        ):
            raise ValueError("scan envelope payload or reference floor changed")
    elif row.scan is not None or row.key is not None or row.valid_until_steady_ns != 0:
        raise ValueError("invalid scan envelope retained a payload")
    return SlotRead(row.revision > revision, row if row.revision > revision else None)
