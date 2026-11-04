"""Diagnostic coordinator; run outside ROS control and safety executors.

All input bridge and static relay GIDs must already have native MessageInfo and
current graph authentication. This coordinator is not a DDS admission fence.
"""

from dataclasses import replace
from typing import Any

from senior_control.vision.independent_motion import Profile
from senior_control.vision.raw_capture import RawCaptureHeader
from senior_control.vision.rgbd_frame_builder import (
    CameraInfoSnapshot,
    ImageLayoutSnapshot,
    build_frame,
)
from senior_control.vision.rgbd_parent_ledger import CaptureLedger, CaptureTicket
from senior_control.vision.rgbd_spawn_worker import CompactResult, SpawnWorker
from senior_control.vision.static_tf_binding import StaticBinding


class RgbdCoordinator:
    def __init__(
        self,
        profile: Profile,
        binding: StaticBinding,
        *,
        generation: int,
        sequence_floor: int,
        registration: str,
        fresh_after_ros_ns: int,
        fresh_after_steady: float,
        capacity: int = 4,
    ) -> None:
        self._binding = binding
        self._profile = profile
        self._registration = registration
        self.ledger = CaptureLedger(
            capacity=capacity,
            fresh_after_ros_ns=fresh_after_ros_ns,
            fresh_after_steady=fresh_after_steady,
        )
        self.worker = SpawnWorker(profile, generation=generation, sequence_floor=sequence_floor)
        self._binding_generation = binding.generation
        self._tickets: list[CaptureTicket] = []
        self._preprocessing: dict[
            int,
            tuple[
                str,
                str,
                CameraInfoSnapshot,
                CameraInfoSnapshot,
                ImageLayoutSnapshot,
                ImageLayoutSnapshot,
            ],
        ] = {}
        self._capacity = capacity
        self._pending_invalid = ""
        self._tf_missing = False
        self._clock_failed = False

    @property
    def profile(self) -> Profile:
        return self._profile

    @property
    def registration(self) -> str:
        return self._registration

    @property
    def binding(self) -> StaticBinding:
        return self._binding

    def _sync_binding(self) -> bool:
        if self.binding.current is None:
            if not self._tf_missing:
                self.ledger.invalidate()
                self.worker.invalidate()
                self._tickets.clear()
                self._preprocessing.clear()
                self._pending_invalid = "static_tf_unavailable"
                self._tf_missing = True
            return False
        self._tf_missing = False
        if self.binding.generation != self._binding_generation:
            self._pending_invalid = "static_tf_unavailable"
            self._binding_generation = self.binding.generation
            self.ledger.observe_binding(self._binding_generation)
            self.worker.invalidate()
            self._tickets.clear()
            self._preprocessing.clear()
        return True

    def offer(
        self,
        headers: tuple[RawCaptureHeader, ...],
        rgb: Any,
        depth: Any,
        rgb_info: Any,
        depth_info: Any,
        *,
        bridges: tuple[str, ...],
        exclusion: bytes,
        now_ros_ns: int,
        now_steady: float,
    ) -> bool:
        if not self._sync_binding():
            return False
        old_generation = self.ledger.generation
        ticket = self.ledger.offer(
            headers,
            bridges=bridges,
            binding_generation=self.binding.generation,
            now_ros_ns=now_ros_ns,
            now_steady=now_steady,
        )
        if self.ledger.generation != old_generation:
            self._pending_invalid = "capture_source_changed"
            self.worker.invalidate()
            self._tickets.clear()
            self._preprocessing.clear()
        if ticket is None:
            return False
        first = self.binding.first_receipt
        if first is None:
            self.ledger.invalidate()
            self.worker.invalidate()
            return False
        epoch = f"{first.boot}:{first.relay_gid}:{self.binding.generation}"
        try:
            built = build_frame(
                headers,
                rgb,
                depth,
                rgb_info,
                depth_info,
                calibration_epoch=self.profile.calibration_epoch,
                registration=self.registration,
                tf_epoch=epoch,
                dependencies=(epoch,),
                exclusion=exclusion,
                static_sim_declared=self.profile.static_background_declared,
            )
        except ValueError:
            self._pending_invalid = "invalid_frame"
            self.ledger.invalidate()
            self.worker.invalidate()
            self._tickets.clear()
            self._preprocessing.clear()
            return False
        ros_deadline = min(min(h.stamp_ns, h.received_ros_ns) + 300_000_000 for h in headers)
        steady_deadline = min(h.received_steady + 0.3 for h in headers)
        if not self.worker.submit(
            built.frame,
            now_ros_ns=now_ros_ns,
            now_steady=now_steady,
            ros_deadline_ns=ros_deadline,
            steady_deadline=steady_deadline,
        ):
            self.ledger.invalidate()
            self.worker.invalidate()
            self._tickets.clear()
            self._preprocessing.clear()
            return False
        self._tickets.append(ticket)
        self._preprocessing[ticket.number] = (
            built.raw_rgb_encoding,
            built.preprocessing_version,
            built.rgb_info,
            built.depth_info,
            built.rgb_layout,
            built.depth_layout,
        )
        if len(self._tickets) > self._capacity:
            old = self._tickets.pop(0)
            self._preprocessing.pop(old.number, None)
        return True

    def poll(self, *, now_ros_ns: int, now_steady: float) -> CompactResult | None:
        if not self.ledger.observe_clock(now_ros_ns, now_steady):
            if self._clock_failed:
                return None
            self._clock_failed = True
            self.worker.invalidate()
            self._tickets.clear()
            self._preprocessing.clear()
            return CompactResult(
                self.worker.generation,
                self.worker.sequence_frontier,
                "clock_invalid",
                (),
                None,
                0,
                0,
                None,
                None,
            )
        ready = self._sync_binding()
        if self._pending_invalid:
            reason, self._pending_invalid = self._pending_invalid, ""
            return CompactResult(
                self.worker.generation,
                self.worker.sequence_frontier,
                reason,
                (),
                None,
                0,
                0,
                None,
                None,
            )
        if not ready:
            return None
        result = self.worker.poll(now_ros_ns=now_ros_ns, now_steady=now_steady)
        if result is None or result.reason != "candidate":
            return result
        if len(result.headers) != 2:
            self.ledger.invalidate()
            self.worker.invalidate()
            return CompactResult(
                self.worker.generation,
                result.sequence,
                "malformed_result",
                (),
                None,
                0,
                0,
                None,
                None,
            )
        static_receipt = self.binding.first_receipt
        if static_receipt is None:
            self.ledger.invalidate()
            self.worker.invalidate()
            return CompactResult(
                self.worker.generation,
                result.sequence,
                "tf_lifetime_invalid",
                result.headers,
                None,
                0,
                0,
                None,
                None,
            )
        expected_epoch = (
            f"{static_receipt.boot}:{static_receipt.relay_gid}:{self.binding.generation}"
        )
        if result.tf_epochs != (expected_epoch, expected_epoch) or result.dependencies != (
            (expected_epoch,),
            (expected_epoch,),
        ):
            self.ledger.invalidate()
            self.worker.invalidate()
            return CompactResult(
                self.worker.generation,
                result.sequence,
                "tf_lifetime_invalid",
                result.headers,
                None,
                0,
                0,
                None,
                None,
            )
        first = next((t for t in self._tickets if t.headers == result.headers[0]), None)
        second = next((t for t in self._tickets if t.headers == result.headers[1]), None)
        if (
            first is None
            or second is None
            or not self.ledger.pair_current(
                first,
                second,
                binding_generation=self.binding.generation,
                now_ros_ns=now_ros_ns,
                now_steady=now_steady,
            )
        ):
            return CompactResult(
                self.worker.generation,
                result.sequence,
                "dependency_unavailable",
                result.headers,
                None,
                0,
                0,
                None,
                None,
            )
        for a, b in zip(first.headers, second.headers, strict=True):
            if not self.binding.pair_valid(
                a.stamp_ns,
                b.stamp_ns,
                a.received_steady,
                b.received_steady,
                generation=self.binding.generation,
                first_received_ros_ns=a.received_ros_ns,
                second_received_ros_ns=b.received_ros_ns,
            ):
                self.ledger.invalidate()
                self.worker.invalidate()
                return CompactResult(
                    self.worker.generation,
                    result.sequence,
                    "tf_lifetime_invalid",
                    result.headers,
                    None,
                    0,
                    0,
                    None,
                    None,
                )
        a_meta, b_meta = self._preprocessing[first.number], self._preprocessing[second.number]
        receipt, admitted = self.binding.first_receipt, self.binding.admitted_from
        if receipt is None or admitted is None:
            self.ledger.invalidate()
            self.worker.invalidate()
            return CompactResult(
                self.worker.generation,
                result.sequence,
                "tf_lifetime_invalid",
                result.headers,
                None,
                0,
                0,
                None,
                None,
            )
        return replace(
            result,
            profile_name=self.profile.name,
            calibration_epoch=self.profile.calibration_epoch,
            registration=self.registration,
            raw_rgb_encodings=(a_meta[0], b_meta[0]),
            preprocessing_versions=(a_meta[1], b_meta[1]),
            camera_info=((a_meta[2], a_meta[3]), (b_meta[2], b_meta[3])),
            image_layouts=((a_meta[4], a_meta[5]), (b_meta[4], b_meta[5])),
            tf_receipts=(receipt, receipt),
            tf_admitted_from=(admitted, admitted),
        )

    def close(self) -> None:
        self.worker.close()
