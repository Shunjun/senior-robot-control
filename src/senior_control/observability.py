"""Allowlisted structured logging: arbitrary log messages are never emitted."""

import json
import logging
import re

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
