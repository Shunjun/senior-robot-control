"""Bounded diagnostic-only producer engine; caller owns graph authentication.

Call only on one dedicated coordinator thread. A native receiver independently
checks MessageInfo, source bodies, original times and TF before fenced egress.
"""

from typing import Any

from senior_control.vision.independent_motion import Profile
from senior_control.vision.raw_capture import RawCaptureHeader
from senior_control.vision.rgbd_coordinator import RgbdCoordinator
from senior_control.vision.rgbd_joiner import CaptureJoiner
from senior_control.vision.rgbd_spawn_worker import CompactResult
from senior_control.vision.static_tf_binding import StaticBinding


class RgbdProducerEngine:
    def __init__(
        self,
        profile: Profile,
        binding: StaticBinding,
        *,
        registration: str,
        fresh_after_ros_ns: int,
        fresh_after_steady: float,
        generation: int,
        sequence_floor: int,
    ) -> None:
        self.joiner = CaptureJoiner(capacity=4)
        self.coordinator = RgbdCoordinator(
            profile,
            binding,
            registration=registration,
            fresh_after_ros_ns=fresh_after_ros_ns,
            fresh_after_steady=fresh_after_steady,
            generation=generation,
            sequence_floor=sequence_floor,
        )
        self._pending_invalid = ""

    def revoke(self, reason: str) -> None:
        self.joiner.invalidate()
        self.coordinator.ledger.invalidate()
        self.coordinator.worker.invalidate()
        self._pending_invalid = reason

    def offer(
        self,
        stream: int,
        header: RawCaptureHeader,
        body: Any,
        *,
        bridge_gid: str,
        now_ros_ns: int,
        now_steady: float,
    ) -> None:
        joined = self.joiner.offer(stream, header, body, bridge_gid=bridge_gid)
        if joined.kind == "invalid":
            # A valid first item of a new generation remains in the joiner;
            # only old retained result tickets and work are revoked here.
            self.coordinator.ledger.invalidate()
            self.coordinator.worker.invalidate()
            self._pending_invalid = "capture_unavailable"
            return
        if joined.kind != "ready":
            return
        rgb, depth, rgb_info, depth_info = joined.bodies
        if not self.coordinator.profile.static_background_declared:
            self.revoke("static_domain_unavailable")
            return
        try:
            width, height = int(rgb.width), int(rgb.height)
            if not 80 <= width <= 640 or not 80 <= height <= 480:
                raise ValueError("image_size")
            exclusion = bytes(width * height)
        except (AttributeError, TypeError, ValueError, OverflowError):
            self.revoke("invalid_frame")
            return
        self.coordinator.offer(
            joined.headers,
            rgb,
            depth,
            rgb_info,
            depth_info,
            bridges=joined.bridges,
            exclusion=exclusion,
            now_ros_ns=now_ros_ns,
            now_steady=now_steady,
        )

    def poll(self, *, now_ros_ns: int, now_steady: float) -> CompactResult | None:
        if self._pending_invalid:
            reason, self._pending_invalid = self._pending_invalid, ""
            return CompactResult(
                self.coordinator.worker.generation,
                self.coordinator.worker.sequence_frontier,
                reason,
                (),
                None,
                0,
                0,
                None,
                None,
            )
        return self.coordinator.poll(now_ros_ns=now_ros_ns, now_steady=now_steady)

    def close(self) -> None:
        self.coordinator.close()
