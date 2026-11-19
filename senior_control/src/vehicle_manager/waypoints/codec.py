"""Original SQLite row decoding, using the current store's concrete constructors."""

import math
from collections.abc import Callable

import aiosqlite
from senior_contracts.errors import ErrorCode

from vehicle_manager.waypoints.ports import PoseT, StoredT, WaypointStorePorts, WaypointT


def _stored(
    row: aiosqlite.Row,
    *,
    adopted: bool | None,
    link_status: str | None,
    ports: WaypointStorePorts[WaypointT, StoredT, PoseT],
) -> StoredT:
    latitude = _optional_real(row[10], error=ports.error)
    longitude = _optional_real(row[11], error=ports.error)
    waypoint = ports.waypoint(
        _text(row[0], error=ports.error),
        _text(row[1], error=ports.error),
        ports.pose(
            _real(row[2], error=ports.error),
            _real(row[3], error=ports.error),
            _real(row[4], error=ports.error),
        ),
        ports.map_reference(_text(row[5], error=ports.error), _whole(row[6], error=ports.error)),
        _text(row[7], error=ports.error),
        _text(row[8], error=ports.error),
        _text(row[9], error=ports.error),
        latitude,
        longitude,
    )
    return ports.stored(
        waypoint,
        ports.pose(
            _real(row[12], error=ports.error),
            _real(row[13], error=ports.error),
            _real(row[14], error=ports.error),
        ),
        adopted,
        link_status,
    )


def _text(value: object, *, error: Callable[[ErrorCode, str], Exception]) -> str:
    if not isinstance(value, str):
        raise error(ErrorCode.VALIDATION, "expected text")
    return value


def _whole(value: object, *, error: Callable[[ErrorCode, str], Exception]) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise error(ErrorCode.VALIDATION, "expected integer")
    return value


def _real(value: object, *, error: Callable[[ErrorCode, str], Exception]) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise error(ErrorCode.VALIDATION, "expected number")
    number = float(value)
    if not math.isfinite(number):
        raise error(ErrorCode.VALIDATION, "expected finite number")
    return number


def _optional_real(value: object, *, error: Callable[[ErrorCode, str], Exception]) -> float | None:
    if value is None:
        return None
    return _real(value, error=error)


def _flag(value: object, *, error: Callable[[ErrorCode, str], Exception]) -> bool:
    if isinstance(value, bool):
        return value
    if value == 0:
        return False
    if value == 1:
        return True
    raise error(ErrorCode.VALIDATION, "expected flag")
