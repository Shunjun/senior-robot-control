"""Interpret an already native-owner-authenticated bounded static TF status.

This helper is not an rclcpp MessageInfo or graph check. The receiver must prove
that the current relay actually delivered this message before invoking it.
"""

import math
from typing import Any

from senior_control.vision.static_tf_binding import Edge, Receipt, StaticBinding


def _time(value: Any) -> int:
    sec, nano = value.sec, value.nanosec
    if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 1_000_000_000:
        raise ValueError("invalid_time")
    return sec * 1_000_000_000 + nano


def observe_verified_tf(binding: StaticBinding, message: Any, *, current_relay_gid: str) -> bool:
    """Apply original receipt and separate admission without relabeling either."""
    try:
        if message.bridge_gid != current_relay_gid:
            return False
        if binding.is_retired_source(
            boot=message.bridge_boot_id, source_gid=message.raw_gid, relay_gid=message.bridge_gid
        ):
            return False
        if not message.valid:
            current = binding.current
            if (
                current is not None
                and message.bridge_gid == current.relay_gid
                and message.bridge_boot_id == current.boot
                and message.raw_gid == current.source_gid
                and type(message.generation) is int
                and message.generation >= current.generation
            ):
                binding.revoke()
            return False
        edges = tuple(
            Edge(
                e.parent_frame,
                e.child_frame,
                _time(e.stamp),
                tuple(e.translation),
                tuple(e.rotation_xyzw),
            )
            for e in message.edges
        )
        first = Receipt(
            message.bridge_boot_id,
            message.raw_gid,
            message.generation,
            message.first_sequence,
            _time(message.first_received_ros),
            float(message.first_received_steady),
            edges,
            message.bridge_gid,
        )
        latest = Receipt(
            message.bridge_boot_id,
            message.raw_gid,
            message.generation,
            message.sequence,
            _time(message.received_ros),
            float(message.received_steady),
            edges,
            message.bridge_gid,
        )
        if (
            first.sequence <= 0
            or latest.sequence < first.sequence
            or latest.received_ros_ns < first.received_ros_ns
            or latest.received_steady < first.received_steady
            or not math.isfinite(first.received_steady)
            or not math.isfinite(latest.received_steady)
            or (latest.sequence == first.sequence and latest != first)
        ):
            binding.revoke()
            return False
        old_first = binding.first_receipt
        old_current = binding.current
        same_lifetime = (
            old_first is not None
            and old_current is not None
            and old_first.boot == first.boot
            and old_first.relay_gid == first.relay_gid
            and old_first.source_gid == first.source_gid
            and old_first.generation == first.generation
        )
        if same_lifetime:
            if old_first != first:
                binding.revoke()
                return False
            assert old_current is not None
            if latest.sequence > old_current.sequence:
                if not binding.observe(
                    latest, current_gid=latest.source_gid, current_relay_gid=current_relay_gid
                ):
                    return False
            elif latest.sequence == old_current.sequence and latest != old_current:
                binding.revoke()
                return False
            elif latest.sequence < old_current.sequence and latest != first:
                return False
        else:
            if not binding.observe(
                first, current_gid=first.source_gid, current_relay_gid=current_relay_gid
            ):
                return False
            if latest.sequence > first.sequence:
                if not binding.observe(
                    latest, current_gid=latest.source_gid, current_relay_gid=current_relay_gid
                ):
                    return False
        if message.admitted:
            ros, steady = _time(message.admitted_from_ros), float(message.admitted_from_steady)
            if (
                ros < first.received_ros_ns
                or not math.isfinite(steady)
                or steady < first.received_steady
                or (binding.admitted_from is not None and binding.admitted_from != (ros, steady))
                or not binding.admit(ros, steady)
            ):
                binding.revoke()
                return False
        return True
    except (AttributeError, TypeError, ValueError, OverflowError, UnicodeError):
        binding.revoke()
        return False
