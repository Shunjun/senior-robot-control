"""Consumer-side native relay and producer ownership frontiers."""

import math
from dataclasses import dataclass
from typing import Any

from senior_control.vision.raw_capture import (
    RawCaptureHeader,
    _gid,
    _text,
    capture_from_message,
    compatible_captures,
)


@dataclass(frozen=True)
class EgressHeader:
    relay_gid: str
    relay_boot: str
    owner_gid: str
    owner_epoch: str
    owner_generation: int
    owner_sequence: int
    relay_sequence: int
    valid: bool
    reason: str
    owner_lost: bool = False
    upstream_invalid: bool = False


def egress_from_message(message: Any) -> EgressHeader:
    """Copy native ownership metadata without inventing receipt times."""
    return EgressHeader(
        relay_gid=str(message.relay_gid),
        relay_boot=str(message.relay_boot),
        relay_sequence=int(message.relay_sequence),
        owner_gid=str(message.owner_gid),
        owner_epoch=str(message.owner_epoch),
        owner_generation=int(message.owner_generation),
        owner_sequence=int(message.owner_sequence),
        valid=bool(message.valid),
        owner_lost=bool(message.owner_lost),
        upstream_invalid=bool(message.upstream_invalid),
        reason=str(message.reason),
    )


def depth_capture_headers(
    body: Any,
    *,
    depth_gid: str,
    info_gid: str,
    now_ros_ns: int,
    now_steady: float,
    max_age_s: float,
) -> tuple[RawCaptureHeader, RawCaptureHeader] | None:
    """Verify both original bridge receipts and their duplicated body fields."""
    try:
        depth = capture_from_message(body.depth_capture)
        info = capture_from_message(body.camera_info_capture)

        def stamp(value: Any) -> int:
            sec, nano = int(value.sec), int(value.nanosec)
            if sec < 0 or not 0 <= nano < 1_000_000_000:
                raise ValueError("invalid_stamp")
            return sec * 1_000_000_000 + nano

        if (
            not compatible_captures(depth, info)
            or not _gid(depth_gid)
            or not _gid(info_gid)
            or depth.bridge_gid != depth_gid
            or info.bridge_gid != info_gid
            or not _gid(depth.raw_gid)
            or not _gid(info.raw_gid)
            or not _text(depth.source_epoch, 64)
            or not _text(info.source_epoch, 64)
            or depth.sequence <= 0
            or info.sequence <= 0
            or depth.frame_id != body.frame_id
            or info.frame_id != body.frame_id
            or depth.raw_gid != body.raw_source
            or info.raw_gid != body.camera_info_source
            or depth.stamp_ns != stamp(body.stamp)
            or info.stamp_ns != stamp(body.camera_info_stamp)
            or depth.received_ros_ns != stamp(body.received_ros)
            or info.received_ros_ns != stamp(body.camera_info_received_ros)
            or depth.received_steady != body.received_steady
            or info.received_steady != body.camera_info_received_steady
            or not math.isfinite(now_steady)
            or not math.isfinite(max_age_s)
            or not 0 < max_age_s <= 1
            or any(
                not 0 <= now_ros_ns - h.received_ros_ns <= max_age_s * 1e9
                or not math.isfinite(h.received_steady)
                or not 0 <= now_steady - h.received_steady <= max_age_s
                for h in (depth, info)
            )
        ):
            return None
        return depth, info
    except (AttributeError, TypeError, ValueError, OverflowError, UnicodeError):
        return None


def safety_scan_capture(
    body: Any,
    *,
    bridge_gid: str,
    now_ros_ns: int,
    now_steady: float,
    max_age_s: float,
) -> RawCaptureHeader | None:
    """Validate the original scan source, native endpoint and first receipt clocks."""
    try:
        capture = capture_from_message(body.raw_capture)
        sec, nano = int(body.stamp.sec), int(body.stamp.nanosec)
        if (
            sec < 0
            or not 0 <= nano < 1_000_000_000
            or not capture.valid
            or not _gid(bridge_gid)
            or capture.bridge_gid != bridge_gid
            or not _gid(capture.raw_gid)
            or not _text(capture.source_epoch, 64)
            or not _text(capture.bridge_boot_id, 64)
            or not _text(capture.frame_id, 128)
            or capture.capture_generation <= 0
            or capture.sequence <= 0
            or capture.stamp_ns != sec * 1_000_000_000 + nano
            or capture.stamp_ns <= 0
            or capture.frame_id != body.frame_id
            or capture.raw_gid != body.raw_source
            or capture.received_ros_ns <= 0
            or capture.received_ros_ns / 1e9 != body.ingress_ros
            or capture.received_steady != body.ingress_steady
            or not math.isfinite(now_steady)
            or not math.isfinite(capture.received_steady)
            or not math.isfinite(max_age_s)
            or not 0 < max_age_s <= 5
            or not 0 <= now_ros_ns - capture.received_ros_ns <= max_age_s * 1e9
            or not 0 <= now_steady - capture.received_steady <= max_age_s
        ):
            return None
        return capture
    except (AttributeError, TypeError, ValueError, OverflowError, UnicodeError):
        return None


class EgressAdmission:
    """Fence relay and owner identities; body freshness is checked separately."""

    def __init__(self, *, max_age_s: float, hard_limit_s: float = 1.0) -> None:
        if (
            not math.isfinite(max_age_s)
            or not math.isfinite(hard_limit_s)
            or not 0 < hard_limit_s <= 5.0
            or not 0 < max_age_s <= hard_limit_s
        ):
            raise ValueError("invalid_egress_age")
        self.max_age_s = max_age_s
        self.latest: EgressHeader | None = None
        self.generation = 0
        self._relay_gid = ""
        self._relay_boot = ""
        self._relay_sequence = 0
        self._owner_generation = 0
        self._owner: tuple[str, str] | None = None
        self._owner_sequence = 0
        self._retired: set[tuple[str, str]] = set()
        self._retired_boots: set[str] = set()
        self._available = False
        self._closed = False

    def _invalidate(self) -> None:
        if self.latest is not None:
            self.latest = None
            self.generation += 1

    def observe_relay(self, relay_gid: str) -> bool:
        """Invalidate once on graph loss and fence a changed relay endpoint."""
        if self._closed:
            return False
        if not _gid(relay_gid):
            self._invalidate()
            self._available = False
            return False
        if self._relay_gid and relay_gid != self._relay_gid:
            self._invalidate()
            if self._relay_boot:
                self._retired_boots.add(self._relay_boot)
            if len(self._retired_boots) >= 256:
                self._closed = True
                return False
            self._relay_boot = ""
            self._relay_sequence = 0
            self._owner_generation = 0
            self._owner = None
            self._owner_sequence = 0
        self._relay_gid, self._available = relay_gid, True
        return True

    def offer(self, header: EgressHeader, *, relay_gid: str) -> bool:
        """Admit only a current native relay and non-retired owner frontier."""
        if not self.observe_relay(relay_gid) or header.relay_gid != relay_gid:
            return False
        if not _text(header.relay_boot, 64) or header.relay_boot in self._retired_boots:
            return False
        if self._relay_boot and header.relay_boot != self._relay_boot:
            self._invalidate()
            return False
        if header.owner_generation < self._owner_generation:
            return False
        owner = (header.owner_gid, header.owner_epoch)
        if header.upstream_invalid:
            if (
                header.valid
                or header.owner_lost
                or owner != self._owner
                or header.owner_generation != self._owner_generation
                or header.owner_sequence != self._owner_sequence
                or header.relay_sequence <= self._relay_sequence
            ):
                return False
            self._relay_sequence = header.relay_sequence
            self._invalidate()
            return False
        if header.owner_lost:
            if (
                header.valid
                or header.owner_sequence != 0
                or owner != self._owner
                or header.owner_generation <= self._owner_generation
                or header.relay_sequence <= self._relay_sequence
            ):
                return False
            self._retired.add(owner)
            if len(self._retired) >= 256:
                self._closed = True
            self._invalidate()
            self._owner = None
            self._owner_generation = header.owner_generation
            self._owner_sequence = 0
            self._relay_sequence = header.relay_sequence
            return False
        if owner in self._retired:
            return False
        if (
            not _gid(header.owner_gid)
            or not _text(header.owner_epoch, 64)
            or header.owner_generation <= 0
            or header.owner_sequence <= 0
            or header.relay_sequence <= 0
        ):
            self._invalidate()
            return False
        if header.relay_sequence <= self._relay_sequence:
            return False
        if header.owner_generation == self._owner_generation and self._owner not in (None, owner):
            self._invalidate()
            return False
        if header.owner_generation > self._owner_generation:
            if self._owner is not None:
                self._retired.add(self._owner)
                if len(self._retired) >= 256:
                    self._closed = True
                    self._invalidate()
                    return False
            self._invalidate()
            self._owner_generation = header.owner_generation
            self._owner = owner
            self._owner_sequence = 0
        if header.owner_sequence <= self._owner_sequence:
            return False
        self._relay_boot = header.relay_boot
        self._relay_sequence = header.relay_sequence
        self._owner_sequence = header.owner_sequence
        if not header.valid:
            self._invalidate()
            return False
        self.latest = header
        self.generation += 1
        return True
