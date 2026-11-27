"""Identity and first-receipt fences for a per-message-verified raw bridge."""

from dataclasses import dataclass
from typing import Any

from vehicle_core.sample_time import SampleTime
from vehicle_core.source_order import SourceOrder


@dataclass(frozen=True)
class RawCaptureHeader:
    time: SampleTime
    frame_id: str
    raw_gid: str
    bridge_gid: str
    source_epoch: str
    bridge_boot_id: str
    capture_generation: int
    sequence: int
    valid: bool
    reason: str

    @property
    def stamp_ns(self) -> int:
        return self.time.stamp_ns

    @property
    def received_ros_ns(self) -> int:
        return self.time.received_ros_ns

    @property
    def received_steady(self) -> float:
        return self.time.received_steady_ns / 1e9

    @property
    def measurement_steady(self) -> float:
        return self.time.sample_steady_ns / 1e9

    @property
    def start_steady(self) -> float:
        return self.measurement_steady

    @property
    def end_steady(self) -> float:
        return self.measurement_steady


def capture_from_message(message: Any) -> RawCaptureHeader:
    """Copy a typed bridge header without changing either original receipt."""

    def nanoseconds(stamp: Any) -> int:
        seconds, nanos = int(stamp.sec), int(stamp.nanosec)
        if seconds < 0 or not 0 <= nanos < 1_000_000_000:
            raise ValueError("invalid_capture_timestamp")
        return seconds * 1_000_000_000 + nanos

    return RawCaptureHeader(
        time=SampleTime(
            nanoseconds(message.stamp),
            nanoseconds(message.received_ros),
            int(message.received_steady_ns),
            int(message.sample_steady_ns),
            int(message.clock_generation),
        ),
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


class RawCaptureAdmission:
    """One consumer's source frontier. generation is a local cache revision."""

    def __init__(self, *, max_age_ms: float, hard_limit_ms: float = 1000.0) -> None:
        # //Compatible: legacy age settings remain accepted; raw inputs have no TTL.
        self.max_age_ms = max_age_ms
        self.latest: RawCaptureHeader | None = None
        self.generation = 0
        self._order = SourceOrder()
        self._available = False
        self.reason = ""

    def confirm_boot(self, boot_id: str) -> None:
        """Only the subscription/session owner confirms process replacement."""
        if boot_id != self._order.boot_id:
            self._order.confirm_boot(boot_id)
            self.latest = None
            self.generation += 1

    @property
    def boot_id(self) -> str | None:
        return self._order.boot_id

    @property
    def source_generation(self) -> int:
        return self._order.generation

    def observe_bridge(self, bridge_gid: str) -> bool:
        # //Compatible: empty lookup means no configured unique endpoint;
        # the actual GID value is never compared or validated.
        available = bool(bridge_gid)
        if available != self._available:
            self._available = available
            if not available:
                self.latest = None
            self.generation += 1
        return available

    def offer(
        self, header: RawCaptureHeader, *, bridge_gid: str, now_ros_ns: int, now_steady: float
    ) -> bool:
        # //Compatible: keep receipt arguments without remapping or ageing original time.
        if not self.observe_bridge(bridge_gid):
            self.reason = "source_unavailable"
            return False
        previous = (self._order.boot_id, self._order.generation)
        if not header.valid:
            if not self._order.observe_generation(header.bridge_boot_id, header.capture_generation):
                self.reason = self._order.reason
                return False
            self.latest = None
            self.reason = header.reason or "invalid_raw_capture"
            self.generation += 1
            return False
        if not header.frame_id:
            self.latest = None
            self.reason = "missing_capture_frame"
            self.generation += 1
            return False
        if not self._order.accept_capture(
            header.bridge_boot_id,
            header.capture_generation,
            header.stamp_ns,
            header.time.received_steady_ns,
        ):
            self.reason = self._order.reason
            return False
        if (self._order.boot_id, self._order.generation) != previous:
            self.latest = None
            self.generation += 1
        self.latest = header
        self.reason = ""
        self.generation += 1
        return True


def compatible_captures(*headers: RawCaptureHeader) -> bool:
    """Exact-frame pairing remains within one producer boot and source batch."""
    if not headers:
        return False
    first = headers[0]
    return (
        first.stamp_ns >= 0
        and bool(first.bridge_boot_id)
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
    """Retain exact-frame inputs within an explicitly bound producer lifetime."""

    def __init__(self, streams: tuple[str, ...], *, max_age_ms: float) -> None:
        if not streams or len(set(streams)) != len(streams):
            raise ValueError("invalid_capture_streams")
        self._admissions = {name: RawCaptureAdmission(max_age_ms=max_age_ms) for name in streams}
        self._latest: dict[str, RawCaptureHeader] = {}
        self._boot = ""
        self._capture_generation = 0
        self._last_receipt_ns = -1
        self.generation = 0

    def confirm_boot(self, boot_id: str) -> None:
        if not isinstance(boot_id, str) or not boot_id:
            raise ValueError("source boot must be nonempty")
        if boot_id != self._boot:
            for admission in self._admissions.values():
                admission.confirm_boot(boot_id)
            self._boot = boot_id
            self._capture_generation = 0
            self._latest.clear()
            self.generation += 1

    def offer(
        self,
        stream: str,
        header: RawCaptureHeader,
        *,
        bridge_gid: str,
        now_ros_ns: int,
        now_steady: float,
    ) -> bool:
        replacing = bool(self._boot and header.bridge_boot_id != self._boot)
        if replacing and (
            not header.valid or header.time.received_steady_ns <= self._last_receipt_ns
        ):
            return False
        if not header.bridge_boot_id or header.capture_generation <= 0:
            return False
        if not replacing and header.capture_generation < self._capture_generation:
            return False
        admission = self._admissions[stream]
        previous = admission.generation
        accepted = admission.offer(
            header, bridge_gid=bridge_gid, now_ros_ns=now_ros_ns, now_steady=now_steady
        )
        if admission.generation == previous or admission.boot_id is None:
            return False
        if replacing and accepted:
            self.confirm_boot(admission.boot_id)
        self._boot = admission.boot_id
        if accepted:
            self._last_receipt_ns = max(self._last_receipt_ns, header.time.received_steady_ns)
        if admission.source_generation > self._capture_generation:
            self._capture_generation = admission.source_generation
            self._latest.clear()
        if accepted:
            self._latest[stream] = header
        else:
            self._latest.clear()
        self.generation += 1
        return accepted

    def observe_bridge(self, stream: str, bridge_gid: str) -> bool:
        """Observe configured endpoint availability without comparing GID identity."""
        admission = self._admissions[stream]
        previous = admission.generation
        ready = admission.observe_bridge(bridge_gid)
        if admission.generation != previous:
            self._latest.clear()
            self.generation += 1
        return ready

    def paired(self, *streams: str) -> bool:
        return all(stream in self._latest for stream in streams) and compatible_captures(
            *(self._latest[stream] for stream in streams)
        )
