"""Bounded, one-way local lifecycle facts; never a motion or recovery transport."""

import asyncio
import json
import logging
import os
from collections.abc import Callable, Mapping
from typing import Any

# //Compatible: preserve the existing lifecycle logger and its filters.
logger = logging.getLogger("vehicle_core.lifecycle_events")
READ_FD = "SENIOR_RUNTIME_EVENTS_READ_FD"
WRITE_FD = "SENIOR_RUNTIME_EVENTS_WRITE_FD"
MAX_EVENT_BYTES = 2048  # Below POSIX PIPE_BUF: concurrent lifecycle writers stay atomic.


def emit(fd: int, module: str, phase: str, detail: str = "", **facts: Any) -> bool:
    value = {"module": module[:64], "phase": phase[:64], "detail": detail[:256], **facts}
    raw = json.dumps(value, ensure_ascii=True).encode() + b"\n"
    if len(raw) > MAX_EVENT_BYTES:
        return False
    try:
        return os.write(fd, raw) == len(raw)
    except OSError:
        # Full pipe or closed observer must never delay the process owner.
        return False


class LifecycleEvents:
    def __init__(self) -> None:
        self.reader, self.writer = os.pipe()
        os.set_blocking(self.writer, False)
        self.lost = 0

    def agent_environment(self, environment: Mapping[str, str]) -> dict[str, str]:
        result = dict(environment)
        result.pop(WRITE_FD, None)
        result[READ_FD] = str(self.reader)
        return result

    def child_environment(self, environment: Mapping[str, str]) -> dict[str, str]:
        result = dict(environment)
        result.pop(READ_FD, None)
        result[WRITE_FD] = str(self.writer)
        return result

    def report(self, module: str, phase: str, detail: str = "", **facts: Any) -> None:
        logger.warning("runtime lifecycle %s %s: %s %s", module, phase, detail, facts)
        if emit(self.writer, module, phase, detail, lost=self.lost, **facts):
            self.lost = 0
        else:
            self.lost += 1

    def close_reader(self) -> None:
        if self.reader >= 0:
            os.close(self.reader)
            self.reader = -1

    def close(self) -> None:
        self.close_reader()
        if self.writer >= 0:
            os.close(self.writer)
            self.writer = -1


def report_ros_exit(event: Any, context: Any) -> list[Any]:
    """Launch sees its own children; record facts without a global Shutdown event."""
    detail = f"pid={event.pid} returncode={event.returncode}"
    logger.error("ROS process exited: %s", detail)
    raw = os.environ.get(WRITE_FD)
    if raw is not None:
        emit(int(raw), "ros_child", "exited", detail, pid=event.pid, returncode=event.returncode)
    return []


async def observe(fd: int, receive: Callable[[dict[str, Any]], None]) -> None:
    reader = asyncio.StreamReader(limit=MAX_EVENT_BYTES)
    protocol = asyncio.StreamReaderProtocol(reader)
    transport, _ = await asyncio.get_running_loop().connect_read_pipe(
        lambda: protocol, os.fdopen(fd, "rb", buffering=0)
    )
    try:
        while True:
            try:
                raw = await reader.readline()
            except ValueError:
                receive({"module": "observer", "phase": "unavailable", "detail": "oversize frame"})
                return
            if not raw:
                receive({"module": "observer", "phase": "unavailable", "detail": "lifecycle EOF"})
                return
            try:
                value = json.loads(raw)
                if (
                    not isinstance(value, dict)
                    or not isinstance(value.get("module"), str)
                    or not isinstance(value.get("phase"), str)
                    or not isinstance(value.get("detail"), str)
                    or len(value["module"]) > 64
                    or len(value["phase"]) > 64
                    or len(value["detail"]) > 256
                ):
                    raise ValueError("invalid lifecycle frame")
            except (ValueError, UnicodeError):
                receive({"module": "observer", "phase": "unavailable", "detail": "invalid frame"})
                continue
            receive(value)
    finally:
        transport.close()
