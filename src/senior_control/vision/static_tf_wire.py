"""Interpret an already native-owner-authenticated bounded static TF status.

This helper is not an rclcpp MessageInfo or graph check. The receiver must prove
that the current relay actually delivered this message before invoking it.
"""

import math
from numbers import Real
from typing import Any, Protocol, cast

from senior_control.vision.static_tf_binding import Edge, Receipt, StaticBinding


class _WireFrontier(Protocol):
    _wire_invalid_frontier: tuple[str, str, int]


class _BindingHistory(Protocol):
    _last: Receipt | None


def _time(value: Any) -> int:
    sec, nano = value.sec, value.nanosec
    if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 1_000_000_000:
        raise ValueError("invalid_time")
    return sec * 1_000_000_000 + nano


def _real_tuple(values: Any) -> tuple[float, ...]:
    # //Compatible: ROS fixed float64 arrays contain NumPy scalar values.
    values = tuple(values)
    if any(not isinstance(value, Real) or isinstance(value, bool) for value in values):
        raise ValueError("invalid_transform_value")
    return tuple(float(value) for value in values)


def observe_verified_tf(binding: StaticBinding, message: Any, *, current_relay_gid: str) -> bool:
    """Apply original receipt and separate admission without relabeling either."""
    try:
        if message.bridge_gid != current_relay_gid:
            return False
        # The caller authenticates the actual current relay; body raw_gid is
        # not proof of the current raw source and may be empty on invalid.
        if (
            type(message.bridge_gid) is not str
            or not message.bridge_gid
            or len(message.bridge_gid.encode("utf-8")) > 128
            or type(message.bridge_boot_id) is not str
            or not message.bridge_boot_id
            or len(message.bridge_boot_id.encode("utf-8")) > 64
        ):
            return False
        if binding.is_retired_source(
            boot=message.bridge_boot_id, source_gid="", relay_gid=message.bridge_gid
        ):
            return False
        frontier = cast(
            tuple[str, str, int] | None,
            getattr(binding, "_wire_invalid_frontier", None),
        )
        current = binding.current
        last = cast(_BindingHistory, binding)._last
        relay = (message.bridge_gid, message.bridge_boot_id)
        # One relay GID cannot change boot even after its lifetime is revoked.
        if last is not None and relay[0] == last.relay_gid and relay[1] != last.boot:
            return False
        if frontier is not None and relay[0] == frontier[0] and relay[1] != frontier[1]:
            return False
        if message.valid is False:
            if type(message.generation) is not int or message.generation <= 0:
                return False
            floor = 0
            if frontier is not None and relay == frontier[:2]:
                floor = frontier[2]
            if current is not None and relay == (current.relay_gid, current.boot):
                floor = max(floor, current.generation)
            if message.generation >= floor:
                cast(_WireFrontier, binding)._wire_invalid_frontier = (*relay, message.generation)
                if current is not None:
                    binding.revoke()
            return False
        if message.valid is not True:
            raise ValueError("invalid_status")
        if binding.is_retired_source(
            boot=message.bridge_boot_id, source_gid=message.raw_gid, relay_gid=message.bridge_gid
        ):
            return False
        if (
            frontier is not None
            and relay == frontier[:2]
            and type(message.generation) is int
            and message.generation <= frontier[2]
        ):
            return False
        edges = tuple(
            Edge(
                e.parent_frame,
                e.child_frame,
                _time(e.stamp),
                cast(tuple[float, float, float], _real_tuple(e.translation)),
                cast(tuple[float, float, float, float], _real_tuple(e.rotation_xyzw)),
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
