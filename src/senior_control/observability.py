"""Allowlisted structured logging: arbitrary log messages are never emitted."""

import json
import logging
import os
import re
import threading
import traceback
from collections import deque
from typing import Any, TextIO

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
# Only source-controlled message templates become diagnostic events. Never format
# their arguments, which can contain credentials, payloads or exception text.
_DIAGNOSTICS = {
    "motion revoked: %s": "motion_revoked",
    "heartbeat failed": "heartbeat_failed",
    "drive sink failed": "drive_sink_failed",
    "background task %s failed": "background_task_failed",
    "uplink failed": "uplink_failed",
    "could not initialize persistent agent safety state": "safety_state_initialize_failed",
    "ROS adapter identity check could not reach the adapter": "adapter_unreachable",
    "ROS adapter identity check failed": "adapter_identity_failed",
    "configured workspace display map is unavailable": "display_map_unavailable",
    "simulation video disabled because the video extra is unavailable": "video_dependency_missing",
    "could not read persistent safety state": "safety_state_read_failed",
    "unresolved safety intent: status-only recovery channel": "safety_recovery_required",
    "inflight work did not stop before shutdown deadline": "shutdown_work_timeout",
    "shutdown step exceeded its deadline": "shutdown_step_timeout",
    "shutdown step failed": "shutdown_step_failed",
    "initial snapshot unavailable": "initial_snapshot_failed",
    "periodic snapshot collect failed": "snapshot_collect_failed",
    "periodic snapshot send failed": "snapshot_send_failed",
    "could not resolve rejected safety intent": "safety_intent_resolve_failed",
    "could not retain safety intent after interrupted clear": "safety_intent_retain_failed",
    "safety command did not complete; restart is disabled": "safety_command_incomplete",
    "snapshot after edit failed": "snapshot_after_edit_failed",
    "workspace read failed": "workspace_read_failed",
    "response could not be sent": "response_send_failed",
    "operation port failed": "operation_port_failed",
    "video signaling failed": "video_signaling_failed",
    "video peer cleanup failed": "video_cleanup_failed",
    "video close could not be sent": "video_close_failed",
    "peer stats failed": "video_stats_failed",
    "video push failed": "video_push_failed",
    "video close could not be queued for robot %s": "video_close_queue_failed",
}
_REVOCATION_REASONS = frozenset(
    {
        "drive sink failed",
        "device credential rejected",
        "shutdown requested",
        "shutting down",
        "background task tasks failed",
        "background task tasks ended",
    }
)


def _exception_details(error: BaseException) -> list[dict[str, Any]]:
    """Bounded exception chain with code locations, never messages or locals."""
    details: list[dict[str, Any]] = []
    seen: set[int] = set()
    while id(error) not in seen and len(details) < 4:
        seen.add(id(error))
        item: dict[str, Any] = {"type": type(error).__name__[:128]}
        frames: deque[dict[str, Any]] = deque(maxlen=8)
        for frame, line in traceback.walk_tb(error.__traceback__):
            frames.append(
                {
                    "file": os.path.basename(frame.f_code.co_filename),
                    "function": frame.f_code.co_name,
                    "line": line,
                }
            )
        item["frames"] = list(frames)
        if isinstance(error, OSError) and isinstance(error.errno, int):
            item["errno"] = error.errno
        status = getattr(getattr(error, "response", None), "status_code", None)
        if type(status) is int and 100 <= status <= 599:
            item["http_status"] = status
        details.append(item)
        next_error = error.__cause__
        if next_error is None and not error.__suppress_context__:
            next_error = error.__context__
        if next_error is None:
            break
        error = next_error
    return details


_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.:-]{1,128}$")


class JsonFormatter(logging.Formatter):
    """Emit approved events, identifiers and exception metadata without payloads."""

    def format(self, record: logging.LogRecord) -> str:
        event = getattr(record, "event", None)
        diagnostic = (
            _DIAGNOSTICS.get(record.msg)
            if isinstance(record.msg, str) and record.name.startswith("senior_control.")
            else None
        )
        data: dict[str, Any] = {
            "level": record.levelname,
            "event": event if event in _EVENTS else diagnostic or "unstructured_log",
        }
        if _IDENTIFIER.fullmatch(record.name):
            data["logger"] = record.name
        if diagnostic is not None:
            data["source"] = {
                "file": os.path.basename(record.pathname),
                "function": record.funcName,
                "line": record.lineno,
            }
        if record.exc_info is not None and record.exc_info[1] is not None:
            data["exceptions"] = _exception_details(record.exc_info[1])
        if diagnostic == "motion_revoked" and isinstance(record.args, tuple) and record.args:
            reason = record.args[0]
            if isinstance(reason, str) and reason in _REVOCATION_REASONS:
                data["reason"] = reason
        for key in ("robot_id", "request_id", "operation_id", "execution_id"):
            value = getattr(record, key, None)
            if isinstance(value, str) and _IDENTIFIER.fullmatch(value):
                data[key] = value
        return json.dumps(data, ensure_ascii=False)


def configure_logging(stream: TextIO | None = None) -> None:
    """Configure only this application's namespace when starting a process."""
    parent = logging.getLogger("senior_control")
    handler = logging.StreamHandler(stream)
    handler.setFormatter(JsonFormatter())
    parent.handlers[:] = [handler]
    parent.setLevel(logging.INFO)
    parent.propagate = False
    parent.disabled = False
    for name, child in logging.root.manager.loggerDict.items():
        if name.startswith("senior_control.") and isinstance(child, logging.Logger):
            child.handlers.clear()
            child.setLevel(logging.NOTSET)
            child.propagate = True
            child.disabled = False


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
