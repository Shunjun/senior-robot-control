"""Interval and five-field cron schedules.

Day and week fields use the schedule timezone. The default is Asia/Shanghai.
An interval is measured from its UTC creation instant. When both day-of-month
and day-of-week are restricted, a date matches if either field matches.
A wall time that does not exist is skipped. A repeated wall time runs once,
at the earlier offset.
"""

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError


@dataclass(frozen=True)
class _Field:
    expression: str
    values: frozenset[int]

    @property
    def unrestricted(self) -> bool:
        return self.expression == "*"


@dataclass(frozen=True)
class _Cron:
    minute: _Field
    hour: _Field
    dom: _Field
    month: _Field
    dow: _Field

    def matches(self, wall: datetime) -> bool:
        if wall.minute not in self.minute.values or wall.hour not in self.hour.values:
            return False
        if wall.month not in self.month.values:
            return False
        dom_ok = wall.day in self.dom.values
        cron_dow = (wall.weekday() + 1) % 7
        dow_ok = cron_dow in self.dow.values
        if not self.dom.unrestricted and not self.dow.unrestricted:
            return dom_ok or dow_ok
        if self.dom.unrestricted and self.dow.unrestricted:
            return True
        if self.dom.unrestricted:
            return dow_ok
        return dom_ok


@dataclass(frozen=True)
class Schedule:
    kind: str
    timezone: str
    created_at: datetime | None = None
    every_seconds: int | None = None
    expression: str | None = None

    def __post_init__(self) -> None:
        if not self.timezone.strip():
            raise ValueError("timezone is required")
        try:
            ZoneInfo(self.timezone)
        except ZoneInfoNotFoundError as exc:
            raise ValueError("unknown timezone") from exc
        if self.kind == "interval":
            if self.created_at is None or self.created_at.tzinfo is None:
                raise ValueError("interval anchor must be timezone-aware")
            if (
                isinstance(self.every_seconds, bool)
                or not isinstance(self.every_seconds, int)
                or self.every_seconds <= 0
            ):
                raise ValueError("interval must be positive")
            object.__setattr__(self, "created_at", self.created_at.astimezone(timezone.utc))
            object.__setattr__(self, "expression", None)
            return
        if self.kind == "cron":
            if self.expression is None:
                raise ValueError("cron expression is required")
            compiled = _compile(self.expression)
            object.__setattr__(self, "expression", _canonical(compiled))
            object.__setattr__(self, "created_at", None)
            object.__setattr__(self, "every_seconds", None)
            return
        raise ValueError("unknown schedule")

    @staticmethod
    def interval(
        created_at: datetime,
        every_seconds: int,
        timezone: str = "Asia/Shanghai",
    ) -> "Schedule":
        return Schedule("interval", timezone, created_at, every_seconds, None)

    @staticmethod
    def cron(expression: str, timezone: str = "Asia/Shanghai") -> "Schedule":
        return Schedule("cron", timezone, None, None, expression)

    def next_after(self, utc: datetime) -> datetime:
        moment = _aware_utc(utc)
        if self.kind == "interval":
            return _interval_after(self, moment)
        return _cron_after(self, moment)


def _aware_utc(moment: datetime) -> datetime:
    if moment.tzinfo is None or moment.utcoffset() is None:
        raise ValueError("timestamp must be timezone-aware")
    return moment.astimezone(timezone.utc)


def _interval_after(schedule: Schedule, moment: datetime) -> datetime:
    created = schedule.created_at
    every_seconds = schedule.every_seconds
    if created is None or every_seconds is None:
        raise ValueError("interval schedule is incomplete")
    if moment <= created:
        return created + timedelta(seconds=every_seconds)
    elapsed = moment - created
    completed = (elapsed.days * 86400 + elapsed.seconds) // every_seconds
    return created + timedelta(seconds=(completed + 1) * every_seconds)


def _cron_after(schedule: Schedule, moment: datetime) -> datetime:
    if schedule.expression is None:
        raise ValueError("cron expression is required")
    cron = _compile(schedule.expression)
    zone = ZoneInfo(schedule.timezone)
    local = moment.astimezone(zone)
    wall = datetime(local.year, local.month, local.day, local.hour, local.minute)
    fired = _wall_utc(wall, zone)
    if fired is None or fired <= moment:
        wall += timedelta(minutes=1)
    limit = 366 * 24 * 60 * 8
    for _ in range(limit):
        if cron.matches(wall):
            fired = _wall_utc(wall, zone)
            if fired is not None and fired > moment:
                return fired
        wall += timedelta(minutes=1)
    raise ValueError("no upcoming occurrence")


def _wall_utc(wall: datetime, zone: ZoneInfo) -> datetime | None:
    aware = wall.replace(tzinfo=zone)
    utc = aware.astimezone(timezone.utc)
    back = utc.astimezone(zone)
    observed = (back.year, back.month, back.day, back.hour, back.minute)
    wanted = (wall.year, wall.month, wall.day, wall.hour, wall.minute)
    if observed != wanted:
        return None
    return utc


def _compile(expression: str) -> _Cron:
    parts = expression.split()
    if len(parts) != 5:
        raise ValueError("cron must contain five fields and no seconds")
    minute = _field(parts[0], 0, 59)
    hour = _field(parts[1], 0, 23)
    dom = _field(parts[2], 1, 31)
    month = _field(parts[3], 1, 12)
    dow = _field(parts[4], 0, 7, sunday=True)
    return _Cron(minute, hour, dom, month, dow)


def _canonical(cron: _Cron) -> str:
    return " ".join(
        (
            cron.minute.expression,
            cron.hour.expression,
            cron.dom.expression,
            cron.month.expression,
            cron.dow.expression,
        )
    )


def _field(token: str, low: int, high: int, *, sunday: bool = False) -> _Field:
    if not token:
        raise ValueError("cron field is empty")
    values: set[int] = set()
    for part in token.split(","):
        step = 1
        body = part
        if "/" in part:
            body, raw_step = part.split("/", 1)
            if not raw_step.isdigit():
                raise ValueError("cron step is invalid")
            step = int(raw_step)
            if step <= 0:
                raise ValueError("cron step is invalid")
        if body == "*":
            start, end = low, high
        elif "-" in body:
            raw_start, raw_end = body.split("-", 1)
            start, end = _number(raw_start), _number(raw_end)
        else:
            start = end = _number(body)
        if start < low or end > high or start > end:
            raise ValueError("cron field is out of range")
        values.update(range(start, end + 1, step))
    if sunday and 7 in values:
        values.add(0)
    return _Field(token, frozenset(values))


def _number(token: str) -> int:
    if not token.isdigit():
        raise ValueError("cron field is invalid")
    return int(token)
