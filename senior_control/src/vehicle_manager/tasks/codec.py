"""Original task row/value parsing with caller-owned map and error constructors."""

import json
from collections.abc import Callable
from datetime import datetime, timezone

import aiosqlite
from senior_contracts.errors import ErrorCode

from vehicle_manager.tasks.models import (
    ExecutionRecord,
    ExecutionStatus,
    SkipRecord,
    TaskDefinition,
)
from vehicle_manager.tasks.ports import MapReference
from vehicle_manager.tasks.schedule import Schedule


class TaskCodec:
    def __init__(
        self,
        map_reference: Callable[[str, int], MapReference],
        error: Callable[[ErrorCode, str], Exception],
    ) -> None:
        self._map_reference = map_reference
        self._error = error

    def task(self, row: aiosqlite.Row) -> TaskDefinition:
        created = self.moment(self.text(row[11]))
        kind = self.text(row[5])
        timezone_name = self.text(row[8])
        every = self.optional_whole(row[6])
        expression = self.optional_text(row[7])
        if kind == "interval":
            if every is None:
                raise self._error(ErrorCode.VALIDATION, "interval is incomplete")
            schedule = Schedule.interval(created, every, timezone_name)
        elif kind == "cron":
            if expression is None:
                raise self._error(ErrorCode.VALIDATION, "cron is incomplete")
            schedule = Schedule.cron(expression, timezone_name)
        else:
            raise self._error(ErrorCode.VALIDATION, "unknown schedule")
        return TaskDefinition(
            self.text(row[0]),
            self.text(row[1]),
            self._map_reference(self.text(row[2]), self.whole(row[3])),
            self.waypoint_ids(self.text(row[4])),
            schedule,
            timezone_name,
            self.flag(row[9]),
            self.whole(row[10]),
            created,
        )

    def execution(self, row: aiosqlite.Row) -> ExecutionRecord:
        return ExecutionRecord(
            self.text(row[0]),
            self.text(row[1]),
            self.moment(self.text(row[2])),
            ExecutionStatus(self.text(row[3])),
            self.flag(row[4]),
            self.flag(row[5]),
        )

    def skip(self, row: aiosqlite.Row) -> SkipRecord:
        return SkipRecord(
            self.text(row[0]),
            self.text(row[1]),
            self.moment(self.text(row[2])),
            self.moment(self.text(row[3])),
            self.text(row[4]),
        )

    def points(self, task: TaskDefinition) -> str:
        return json.dumps(list(task.waypoint_ids), separators=(",", ":"))

    def waypoint_ids(self, raw: str) -> tuple[str, ...]:
        parsed = json.loads(raw)
        if not isinstance(parsed, list) or any(not isinstance(item, str) for item in parsed):
            raise self._error(ErrorCode.VALIDATION, "waypoints are invalid")
        return tuple(parsed)

    def stamp(self, moment: datetime) -> str:
        return self.utc(moment).isoformat()

    def moment(self, value: str) -> datetime:
        parsed = datetime.fromisoformat(value)
        if parsed.tzinfo is None:
            raise self._error(ErrorCode.VALIDATION, "timestamp must be timezone-aware")
        return parsed.astimezone(timezone.utc)

    def utc(self, moment: datetime) -> datetime:
        if moment.tzinfo is None or moment.utcoffset() is None:
            raise self._error(ErrorCode.VALIDATION, "timestamp must be timezone-aware")
        return moment.astimezone(timezone.utc)

    def text(self, value: object) -> str:
        if not isinstance(value, str):
            raise self._error(ErrorCode.VALIDATION, "expected text")
        return value

    def optional_text(self, value: object) -> str | None:
        if value is None:
            return None
        return self.text(value)

    def whole(self, value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise self._error(ErrorCode.VALIDATION, "expected integer")
        return value

    def optional_whole(self, value: object) -> int | None:
        if value is None:
            return None
        return self.whole(value)

    def flag(self, value: object) -> bool:
        if isinstance(value, bool):
            return value
        if value == 0:
            return False
        if value == 1:
            return True
        raise self._error(ErrorCode.VALIDATION, "expected flag")
