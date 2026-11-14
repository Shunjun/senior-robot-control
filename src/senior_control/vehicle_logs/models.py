"""Bounded log values and redaction, independent of transport/authentication."""

import json
import math
import re
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

from senior_control.settings.models import Level

QUEUE_LIMIT = 1024
DETAIL_BYTES = 4096
PAGE_BYTES = 256 * 1024
DOWNLOAD_BYTES = 2 * 1024 * 1024
DISK_BYTES = 64 * 1024 * 1024
LEVEL_WEIGHT = {"DEBUG": 10, "INFO": 20, "WARNING": 30, "ERROR": 40, "CRITICAL": 50}
_IDENTIFIER = re.compile(r"^[A-Za-z0-9_.:-]{1,128}$")
_SECRET_KEY = re.compile(
    r"password|passwd|secret|token|credential|authorization|cookie|api.?key|device.?key|pairing.?(key|code)",
    re.IGNORECASE,
)
_SECRET_TEXT = re.compile(
    r"(?i)(\b(?:password|passwd|[\w-]*secret|[\w-]*token|[\w-]*credential|"
    r"api[_-]?key|device[_-]?key|pairing[ _-]?(?:key|code)|authorization|cookie)"
    r"[\"']?\s*[:=]\s*)"
    r"(\[REDACTED\]|\"[^\"\n]*\"|'[^'\n]*'|[^\s,;&}\]]+)"
)

_HEADER = re.compile(r"(?im)(\b(?:authorization|set-cookie|cookie)\s*:\s*)[^\r\n]+")
_BEARER = re.compile(r"(?i)\b(Bearer|Basic)\s+[A-Za-z0-9+/_.=:-]+")
_URL_PASSWORD = re.compile(r"(https?://)[^\s/@:]+:[^\s/@]+@", re.IGNORECASE)
LossReason = Literal["queue_full", "oversize", "invalid", "storage_error", "restart_unknown"]


@dataclass(frozen=True)
class LogInput:
    level: Level
    module: str
    summary: str
    detail: str = ""
    fields: Mapping[str, object] = field(default_factory=dict)
    source_utc: float = 0.0
    request_id: str | None = None
    task_id: str | None = None


class FrozenModel(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", strict=True, allow_inf_nan=False)


class LogQuery(FrozenModel):
    after: int = Field(default=0, ge=0, le=2**63 - 1)
    limit: int = Field(default=200, ge=1, le=200)
    levels: tuple[Level, ...] = Field(default=(), max_length=5)
    module: str | None = Field(default=None, max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    task_id: str | None = Field(default=None, max_length=128)
    since_utc: float | None = None
    until_utc: float | None = None

    def matches(self, entry: "LogEntry") -> bool:
        # Gaps are unconditional: filters must not hide lost records.
        if entry.kind == "gap":
            return True
        return (
            (not self.levels or entry.level in self.levels)
            and (self.module is None or self.module == entry.module)
            and (self.request_id is None or self.request_id == entry.request_id)
            and (self.task_id is None or self.task_id == entry.task_id)
            and (self.since_utc is None or entry.source_utc >= self.since_utc)
            and (self.until_utc is None or entry.source_utc <= self.until_utc)
        )


class LogEntry(FrozenModel):
    sequence: int = 0
    kind: Literal["event", "gap"] = "event"
    level: Level = "INFO"
    module: str = "vehicle_logs"
    summary: str
    detail: str = ""
    fields_json: str = "{}"
    source_utc: float
    received_utc: float
    request_id: str | None = None
    task_id: str | None = None
    loss_reason: LossReason | None = None
    loss_count: int | None = 0


class LogGap(FrozenModel):
    reason: str
    count: int | None = None
    after: int | None = None
    through: int | None = None
    durable: bool = False


class LogPage(FrozenModel):
    entries: tuple[LogEntry, ...] = ()
    gaps: tuple[LogGap, ...] = ()
    next_cursor: int
    high_water: int
    has_more: bool
    scanned: int = 0


@dataclass(frozen=True)
class LogDownload:
    data: bytes
    next_cursor: int
    has_more: bool
    gaps: tuple[LogGap, ...]


def _bounded_text(text: str, limit: int) -> str:
    if not isinstance(text, str) or len(text) > limit or len(text.encode("utf-8")) > limit:
        raise OverflowError("log text exceeds byte limit")
    return text


def redact_text(text: str) -> str:
    text = _HEADER.sub(lambda match: match[1] + "[REDACTED]", text)
    text = _BEARER.sub(lambda match: match[1] + " [REDACTED]", text)
    text = _SECRET_TEXT.sub(lambda match: match[1] + "[REDACTED]", text)
    return _URL_PASSWORD.sub(r"\1[REDACTED]@", text)


def _redact_fields(value: object, budget: list[int], depth: int = 0) -> object:
    budget[0] -= 1
    if budget[0] < 0 or depth > 4:
        raise OverflowError("log context exceeds structural budget")
    if isinstance(value, str):
        return redact_text(_bounded_text(value, DETAIL_BYTES))
    if value is None or type(value) in (bool, int):
        if type(value) is int and not -(2**63) <= value < 2**63:
            raise ValueError("integer outside log range")
        return value
    if isinstance(value, float) and math.isfinite(value):
        return value
    if isinstance(value, Mapping):
        if len(value) > 16:
            raise OverflowError("too many log fields")
        result: dict[str, object] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError("log keys must be strings")
            key = _bounded_text(key, 128)
            result[redact_text(key)] = (
                "[REDACTED]" if _SECRET_KEY.search(key) else _redact_fields(item, budget, depth + 1)
            )
        return result
    if isinstance(value, (list, tuple)) and len(value) <= 16:
        return [_redact_fields(item, budget, depth + 1) for item in value]
    raise ValueError("unsupported log field")


def prepare_entry(value: LogInput, received_utc: float) -> LogEntry:
    if (
        value.level not in LEVEL_WEIGHT
        or not _IDENTIFIER.fullmatch(value.module)
        or redact_text(value.module) != value.module
    ):
        raise ValueError("invalid log level/module")
    for identifier in (value.request_id, value.task_id):
        if identifier is not None and (
            not _IDENTIFIER.fullmatch(identifier) or redact_text(identifier) != identifier
        ):
            raise ValueError("invalid log correlation ID")
    summary = _bounded_text(redact_text(_bounded_text(value.summary, 512)), 512)
    detail = _bounded_text(redact_text(_bounded_text(value.detail, DETAIL_BYTES)), DETAIL_BYTES)
    fields = _redact_fields(value.fields, [64])
    encoded = json.dumps(fields, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    _bounded_text(encoded, DETAIL_BYTES)
    return LogEntry(
        level=value.level,
        module=value.module,
        summary=summary,
        detail=detail,
        fields_json=encoded,
        source_utc=value.source_utc,
        received_utc=received_utc,
        request_id=value.request_id,
        task_id=value.task_id,
    )


def redact_fields_json(encoded: str) -> str:
    _bounded_text(encoded, DETAIL_BYTES)
    value: object = json.loads(encoded)
    redacted = _redact_fields(value, [64])
    return json.dumps(redacted, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
