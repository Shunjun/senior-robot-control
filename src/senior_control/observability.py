"""Allowlisted structured logging: arbitrary log messages are never emitted."""

import json
import logging
import re
import threading

_EVENTS = frozenset(
    {
        "operation_accepted",
        "operation_failed",
        "operation_completed",
        "connection_opened",
        "connection_closed",
        "safety_rejected",
        "migration_failed",
        "startup",
        "shutdown",
    }
)
_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.:-]{1,128}$")


class JsonFormatter(logging.Formatter):
    """Emit only approved event names and correlation identifiers."""

    def format(self, record: logging.LogRecord) -> str:
        event = getattr(record, "event", None)
        data = {
            "level": record.levelname,
            "event": event if event in _EVENTS else "unstructured_log",
        }
        for key in ("robot_id", "request_id", "operation_id", "execution_id"):
            value = getattr(record, key, None)
            if isinstance(value, str) and _IDENTIFIER.fullmatch(value):
                data[key] = value
        return json.dumps(data, ensure_ascii=False)


_METRIC_NAMES = frozenset({"agent_operations_total", "agent_reconnects_total"})
_OPERATION_LABELS = frozenset(
    {
        "task.create",
        "task.update",
        "task.delete",
        "task.enable",
        "home.set",
        "map.export",
        "map.import",
        "map.clear",
        "operation.query",
        "drive.velocity",
        "drive.stop",
        "safety.estop",
        "control",
        "none",
    }
)
_RESULT_LABELS = frozenset(
    {
        "accepted",
        "dispatching",
        "running",
        "succeeded",
        "failed",
        "needs_review",
        "validation",
        "conflict",
        "offline",
        "localization_unavailable",
        "safety_rejected",
        "timeout",
        "operation_unknown",
        "none",
    }
)
OTHER = "other"


class Metrics:
    """Counters whose labels come from closed sets: a caller cannot grow the series count."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._series: dict[tuple[str, str, str], int] = {}

    def inc(self, name: str, *, operation: str = "none", result: str = "none") -> None:
        if name not in _METRIC_NAMES:
            raise ValueError("unknown metric")
        key = (
            name,
            operation if operation in _OPERATION_LABELS else OTHER,
            result if result in _RESULT_LABELS else OTHER,
        )
        with self._lock:
            self._series[key] = self._series.get(key, 0) + 1

    def series(self) -> dict[tuple[str, str, str], int]:
        with self._lock:
            return dict(self._series)
