"""Same-host producer proof established before parsing, computation or publication."""

import secrets
from pathlib import Path
from uuid import UUID

from senior_motion_msgs.msg import SourceTime


class DriverClock:
    def __init__(self):
        self.domain = str(UUID(Path("/proc/sys/kernel/random/boot_id").read_text().strip()))
        self.epoch = secrets.token_hex(16)
        self.sequence = 0

    def sample(
        self,
        start_ns,
        end_ns=None,
        *,
        time_basis="driver_read",
        measurement_epoch="",
        measurement_tick_ns=None,
    ):
        if end_ns is None:
            end_ns = start_ns
        if not 0 < start_ns <= end_ns or time_basis not in {
            "driver_read",
            "sim_step",
            "control_compute",
        }:
            raise ValueError("invalid source monotonic support")
        if time_basis == "sim_step" and (
            not measurement_epoch
            or type(measurement_tick_ns) is not int
            or not 0 <= measurement_tick_ns < 2**64
        ):
            raise ValueError("missing simulation physical measurement proof")
        if self.sequence == 2**64 - 1:
            raise OverflowError("source sequence exhausted")
        self.sequence += 1
        return SourceTime(
            clock_domain=self.domain,
            source_epoch=self.epoch,
            sequence=self.sequence,
            capture_start_steady_ns=start_ns,
            capture_end_steady_ns=end_ns,
            time_basis=time_basis,
            measurement_epoch=measurement_epoch if time_basis == "sim_step" else "",
            measurement_tick_ns=measurement_tick_ns if time_basis == "sim_step" else 0,
        )
