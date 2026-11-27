"""Consumer-side native relay and producer ownership frontiers."""

import math
from dataclasses import dataclass
from typing import Any

from senior_perception.vision.raw_capture import (
    RawCaptureHeader,
    capture_from_message,
    compatible_captures,
)
from vehicle_core.source_order import SourceOrder


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
    max_age_ms: float,
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
            or depth.frame_id != body.frame_id
            or info.frame_id != body.frame_id
            or depth.stamp_ns != stamp(body.stamp)
            or info.stamp_ns != stamp(body.camera_info_stamp)
            or depth.received_ros_ns != stamp(body.received_ros)
            or info.received_ros_ns != stamp(body.camera_info_received_ros)
            or depth.received_steady != body.received_steady
            or info.received_steady != body.camera_info_received_steady
            or any(not 0 <= h.measurement_steady <= h.received_steady for h in (depth, info))
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
    max_age_ms: float,
) -> RawCaptureHeader | None:
    """Validate the original scan source, native endpoint and first receipt clocks."""
    try:
        capture = capture_from_message(body.raw_capture)
        sec, nano = int(body.stamp.sec), int(body.stamp.nanosec)
        if (
            sec < 0
            or not 0 <= nano < 1_000_000_000
            or not capture.valid
            or not capture.bridge_boot_id
            or not capture.frame_id
            or capture.capture_generation <= 0
            or capture.stamp_ns != sec * 1_000_000_000 + nano
            or capture.stamp_ns < 0
            or capture.frame_id != body.frame_id
            or capture.received_ros_ns < 0
            or capture.received_ros_ns / 1e9 != body.ingress_ros
            or capture.received_steady != body.ingress_steady
            or not math.isfinite(capture.received_steady)
            or not 0 <= capture.measurement_steady <= capture.received_steady
        ):
            return None
        return capture
    except (AttributeError, TypeError, ValueError, OverflowError, UnicodeError):
        return None


class EgressAdmission:
    """Relay lifetime status only; the body consumer orders capture timestamps."""

    def __init__(self, *, max_age_ms: float, hard_limit_ms: float = 1000.0) -> None:
        # //Compatible: old settings remain accepted; raw transport has no TTL.
        self.max_age_ms = max_age_ms
        self.latest: EgressHeader | None = None
        self.generation = 0
        self._order = SourceOrder()

    def _invalidate(self) -> None:
        if self.latest is not None:
            self.latest = None
            self.generation += 1

    def confirm_boot(self, boot: str) -> None:
        """The managed subscription/session owner confirms a process replacement."""
        if self._order.boot_id != boot:
            self._order.confirm_boot(boot)
            self._invalidate()

    def observe_relay(self, relay_gid: str) -> bool:
        # //Compatible: this argument represents the caller's configured endpoint
        # availability. Its bytes are neither parsed nor compared with a claim.
        if not relay_gid:
            self._invalidate()
            return False
        return True

    def offer(self, header: EgressHeader, *, relay_gid: str) -> bool:
        if not self.observe_relay(relay_gid):
            return False
        previous = self._order.generation
        if not self._order.observe_generation(header.relay_boot, header.owner_generation):
            return False
        if self._order.generation != previous:
            self._invalidate()
        if not header.valid or header.owner_lost or header.upstream_invalid:
            self._invalidate()
            return False
        self.latest = header
        self.generation += 1
        return True
