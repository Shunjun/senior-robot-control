import errno
import json
import logging
import sys

from senior_control.observability import JsonFormatter


def test_uplink_error_keeps_cause_and_location_without_exception_secrets():
    try:
        try:
            raise OSError(
                errno.ECONNREFUSED, "Bearer private-token /private/device/credentials.json"
            )
        except OSError as cause:
            raise ConnectionError("wss://user:private-token@host/?token=private-token") from cause
    except ConnectionError:
        record = logging.LogRecord(
            "senior_control.runtime.supervisor",
            logging.WARNING,
            __file__,
            12,
            "uplink failed",
            (),
            sys.exc_info(),
        )
    result = json.loads(JsonFormatter().format(record))
    assert result["event"] == "uplink_failed"
    assert result["logger"] == "senior_control.runtime.supervisor"
    assert result["exceptions"][0]["type"] == "ConnectionError"
    assert result["exceptions"][1]["type"] == "ConnectionRefusedError"
    assert result["exceptions"][1]["errno"] == errno.ECONNREFUSED
    assert result["exceptions"][0]["frames"][-1]["file"] == "test_logging_diagnostics.py"
    assert "private-token" not in str(result)
    assert "/private/" not in str(result)
    assert "wss://" not in str(result)


def test_http_handshake_error_keeps_status_without_headers_or_body():
    from websockets.datastructures import Headers
    from websockets.exceptions import InvalidStatus
    from websockets.http11 import Response

    try:
        raise InvalidStatus(Response(403, "private-token", Headers({"secret": "private-token"})))
    except InvalidStatus:
        record = logging.LogRecord(
            "senior_control.runtime.supervisor",
            logging.WARNING,
            __file__,
            12,
            "uplink failed",
            (),
            sys.exc_info(),
        )
    result = json.loads(JsonFormatter().format(record))
    assert result["exceptions"][0]["http_status"] == 403
    assert "private-token" not in str(result)


def test_revocation_reason_is_allowlisted():
    for reason, expected in (
        ("background task tasks failed", "background task tasks failed"),
        ("shutdown requested", "shutdown requested"),
        ("Bearer private-token", None),
    ):
        record = logging.LogRecord(
            "senior_control.runtime.supervisor",
            logging.ERROR,
            __file__,
            12,
            "motion revoked: %s",
            (reason,),
            None,
        )
        result = json.loads(JsonFormatter().format(record))
        assert result["event"] == "motion_revoked"
        assert result.get("reason") == expected
        assert "private-token" not in str(result)


def test_unknown_message_and_arguments_stay_private():
    record = logging.LogRecord(
        "senior_control.agent",
        logging.ERROR,
        __file__,
        12,
        "Bearer %s",
        ("private-token",),
        None,
    )
    result = json.loads(JsonFormatter().format(record))
    assert result["event"] == "unstructured_log"
    assert "Bearer" not in str(result)
    assert "private-token" not in str(result)


def test_exception_chain_and_frames_are_bounded():
    def fail(depth):
        if depth:
            return fail(depth - 1)
        raise RuntimeError("private-token")

    try:
        fail(12)
    except RuntimeError:
        record = logging.LogRecord(
            "senior_control.agent",
            logging.ERROR,
            __file__,
            12,
            "workspace read failed",
            (),
            sys.exc_info(),
        )
    result = json.loads(JsonFormatter().format(record))
    assert len(result["exceptions"][0]["frames"]) == 8
    error = record.exc_info[1]
    for _ in range(6):
        parent = RuntimeError("private-token")
        error.__cause__ = parent
        error = parent
    # A cycle must not hang the formatter.
    error.__cause__ = record.exc_info[1]
    result = json.loads(JsonFormatter().format(record))
    assert len(result["exceptions"]) == 4
    assert "private-token" not in str(result)


async def test_supervisor_failure_logs_diagnostic_and_revocation():
    import asyncio
    import io

    from senior_control.runtime.supervisor import Supervisor

    output = io.StringIO()
    handler = logging.StreamHandler(output)
    handler.setFormatter(JsonFormatter())
    logger = logging.getLogger("senior_control.runtime.supervisor")
    saved = list(logger.handlers), logger.level, logger.propagate, logger.disabled
    logger.handlers = [handler]
    logger.setLevel(logging.INFO)
    logger.propagate = False
    logger.disabled = False
    fault = asyncio.Event()

    async def noop(*args):
        pass

    async def task_loop():
        raise RuntimeError("Bearer private-token")

    try:
        async with Supervisor(
            send_heartbeat=noop,
            call_ros=noop,
            drive_sink=noop,
            publish_cmd_vel=noop,
            background={"tasks": task_loop},
            on_fault=lambda reason: fault.set(),
        ):
            await asyncio.wait_for(fault.wait(), timeout=1)
    finally:
        logger.handlers, logger.level, logger.propagate, logger.disabled = saved
    records = [json.loads(line) for line in output.getvalue().splitlines()]
    assert records[0]["event"] == "background_task_failed"
    assert records[0]["exceptions"][0]["type"] == "RuntimeError"
    assert records[0]["exceptions"][0]["frames"][-1]["function"] == "task_loop"
    assert records[1]["event"] == "motion_revoked"
    assert records[1]["reason"] == "background task tasks failed"
    assert "private-token" not in output.getvalue()
