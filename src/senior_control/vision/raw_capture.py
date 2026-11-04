"""Identity and first-receipt fences for a per-message-verified raw bridge."""

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class RawCaptureHeader:
    stamp_ns: int
    received_ros_ns: int
    received_steady: float
    frame_id: str
    raw_gid: str
    bridge_gid: str
    source_epoch: str
    bridge_boot_id: str
    capture_generation: int
    sequence: int
    valid: bool
    reason: str


def capture_from_message(message: Any) -> RawCaptureHeader:
    """Copy a typed bridge header without changing either original receipt."""

    def nanoseconds(stamp: Any) -> int:
        seconds, nanos = int(stamp.sec), int(stamp.nanosec)
        if seconds < 0 or not 0 <= nanos < 1_000_000_000:
            raise ValueError("invalid_capture_timestamp")
        return seconds * 1_000_000_000 + nanos

    return RawCaptureHeader(
        stamp_ns=nanoseconds(message.stamp),
        received_ros_ns=nanoseconds(message.received_ros),
        received_steady=float(message.received_steady),
        frame_id=str(message.frame_id),
        raw_gid=str(message.raw_gid),
        bridge_gid=str(message.bridge_gid),
        source_epoch=str(message.source_epoch),
        bridge_boot_id=str(message.bridge_boot_id),
        capture_generation=int(message.capture_generation),
        sequence=int(message.sequence),
        valid=bool(message.valid),
        reason=str(message.reason),
    )


def _gid(value: str) -> bool:
    try:
        raw = bytes.fromhex(value)
        return 1 <= len(raw) <= 64 and any(raw) and value == raw.hex()
    except (TypeError, ValueError):
        return False


def _text(value: str, maximum: int) -> bool:
    try:
        return bool(value) and len(value.encode("utf-8")) <= maximum
    except (AttributeError, UnicodeError):
        return False


class RawCaptureAdmission:
    """Only accept a bridge payload naming the currently unique bridge endpoint.

    Source epochs are retired on replacement, and sequence frontiers survive
    temporary graph loss. Measurements with a future stamp can be *queued* by
    a bounded synchronizer but cannot be projected until ROS clock catch-up.
    """

    def __init__(self, *, max_age_s: float, hard_limit_s: float = 1.0) -> None:
        if (
            not math.isfinite(max_age_s)
            or not math.isfinite(hard_limit_s)
            or not 0 < hard_limit_s <= 5.0
            or not 0 < max_age_s <= hard_limit_s
        ):
            raise ValueError("invalid_raw_age")
        self.max_age_s = max_age_s
        self.latest: RawCaptureHeader | None = None
        self.generation = 0
        self._bridge_gid = ""
        self._bridge_boot = ""
        self._capture_generation = 0
        self._available = False
        self._source_epoch = ""
        self._raw_gid = ""
        self._sequence = 0
        self._retired: set[str] = set()
        self._closed = False

    def _retire(self) -> None:
        if self._source_epoch:
            self._retired.add(self._source_epoch)
            if len(self._retired) >= 256:
                self._closed = True
        self._source_epoch = self._raw_gid = ""
        self._sequence = 0
        self.latest = None
        self.generation += 1

    def observe_bridge(self, bridge_gid: str) -> bool:
        if self._closed:
            return False
        if not _gid(bridge_gid):
            if self._available:
                self.latest = None
                self.generation += 1
            self._available = False
            return False
        if self._bridge_gid and bridge_gid != self._bridge_gid:
            self._retire()
            self._bridge_boot = ""
            self._capture_generation = 0
        if self._closed:
            return False
        self._bridge_gid, self._available = bridge_gid, True
        return True

    def offer(
        self, header: RawCaptureHeader, *, bridge_gid: str, now_ros_ns: int, now_steady: float
    ) -> bool:
        if not self.observe_bridge(bridge_gid):
            return False
        if header.bridge_gid != bridge_gid or header.source_epoch in self._retired:
            return False
        if not _text(header.bridge_boot_id, 64) or header.capture_generation <= 0:
            self.latest = None
            self.generation += 1
            return False
        if self._bridge_boot and header.bridge_boot_id != self._bridge_boot:
            self.latest = None
            self.generation += 1
            return False
        self._bridge_boot = header.bridge_boot_id
        if header.capture_generation < self._capture_generation:
            return False
        if header.capture_generation > self._capture_generation:
            self.latest = None
            self.generation += 1
            self._capture_generation = header.capture_generation
        if not _text(header.source_epoch, 64) or header.sequence <= 0:
            self.latest = None
            self.generation += 1
            return False
        if self._source_epoch != header.source_epoch:
            if self._source_epoch:
                self._retire()
            if self._closed:
                return False
            self._source_epoch = header.source_epoch
        if header.sequence <= self._sequence:
            return False
        self._sequence = header.sequence
        if not header.valid:
            self.latest = None
            self.generation += 1
            return False
        if self._raw_gid and header.raw_gid != self._raw_gid:
            self._retire()
            return False
        if (
            not _gid(header.raw_gid)
            or not _text(header.frame_id, 128)
            or header.stamp_ns <= 0
            or header.received_ros_ns <= 0
            or not math.isfinite(header.received_steady)
            or not math.isfinite(now_steady)
            or not 0 <= now_ros_ns - header.received_ros_ns <= self.max_age_s * 1e9
            or not 0 <= now_steady - header.received_steady <= self.max_age_s
        ):
            self.latest = None
            self.generation += 1
            return False
        self._raw_gid = header.raw_gid
        self.latest = header
        self.generation += 1
        return True


def compatible_captures(*headers: RawCaptureHeader) -> bool:
    """An exact frame may not span bridge lifetimes or source generations."""
    if not headers:
        return False
    first = headers[0]
    return (
        first.stamp_ns > 0
        and _text(first.bridge_boot_id, 64)
        and first.capture_generation > 0
        and all(
            header.valid
            and header.stamp_ns == first.stamp_ns
            and header.bridge_boot_id == first.bridge_boot_id
            and header.capture_generation == first.capture_generation
            for header in headers
        )
    )


class RawCaptureGroup:
    """Keep a shared capture generation across separately verified streams."""

    def __init__(self, streams: tuple[str, ...], *, max_age_s: float) -> None:
        if not streams or len(set(streams)) != len(streams):
            raise ValueError("invalid_capture_streams")
        self._admissions = {name: RawCaptureAdmission(max_age_s=max_age_s) for name in streams}
        self._latest: dict[str, RawCaptureHeader] = {}
        self._boot = ""
        self._retired_boots: set[str] = set()
        self._bridge_gids: dict[str, str] = {}
        self._capture_generation = 0
        self.generation = 0

    def offer(
        self,
        stream: str,
        header: RawCaptureHeader,
        *,
        bridge_gid: str,
        now_ros_ns: int,
        now_steady: float,
    ) -> bool:
        admission = self._admissions[stream]
        if self._boot and header.bridge_boot_id != self._boot:
            previous_gid = self._bridge_gids.get(stream, "")
            if (
                header.bridge_boot_id in self._retired_boots
                or not previous_gid
                or not _gid(bridge_gid)
                or header.bridge_gid != bridge_gid
                or bridge_gid == previous_gid
                or not _text(header.bridge_boot_id, 64)
                or len(self._retired_boots) >= 255
            ):
                return False
            self._retired_boots.add(self._boot)
            self._boot = header.bridge_boot_id
            self._capture_generation = 0
            self._latest.clear()
            self.generation += 1
        if header.capture_generation < self._capture_generation:
            return False
        prior = admission.generation
        accepted = admission.offer(
            header, bridge_gid=bridge_gid, now_ros_ns=now_ros_ns, now_steady=now_steady
        )
        if admission.generation == prior:
            return False
        if header.bridge_gid != bridge_gid or not _text(header.bridge_boot_id, 64):
            self._latest.clear()
            self.generation += 1
            return False
        self._boot = header.bridge_boot_id
        self._bridge_gids[stream] = bridge_gid
        if header.capture_generation > self._capture_generation:
            self._capture_generation = header.capture_generation
            self._latest.clear()
        if accepted:
            self._latest[stream] = header
        else:
            self._latest.clear()
        self.generation += 1
        return accepted

    def observe_bridge(self, stream: str, bridge_gid: str) -> bool:
        """Revoke the group as soon as a bridge endpoint disappears or changes."""
        admission = self._admissions[stream]
        prior = admission.generation
        ready = admission.observe_bridge(bridge_gid)
        if admission.generation != prior:
            self._latest.clear()
            self.generation += 1
        return ready

    def paired(self, *streams: str) -> bool:
        """Return true only for retained exact-stamp captures in this generation."""
        return all(stream in self._latest for stream in streams) and compatible_captures(
            *(self._latest[stream] for stream in streams)
        )
