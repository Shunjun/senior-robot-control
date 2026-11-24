"""Read Linux counters without subprocesses, waits, or whole-system PID scans."""

import math
import os
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal


@dataclass(frozen=True)
class ModuleReading:
    name: str
    status: Literal["running", "inactive", "unavailable"]
    cpu_percent: float | None
    rss_bytes: int | None
    process_count: int


@dataclass(frozen=True)
class ResourceReading:
    sequence: int
    sampled_steady: float
    interval_s: float | None
    cpu_count: int
    cpu_percent: float | None
    memory_total_bytes: int | None
    memory_used_bytes: int | None
    modules: tuple[ModuleReading, ...]


@dataclass(frozen=True)
class _Process:
    birth: int
    ticks: int
    rss: int
    parent: int


def _read(path: Path, limit: int = 65536) -> str:
    with path.open("rb") as stream:
        raw = stream.read(limit + 1)
    if len(raw) > limit:
        raise ValueError("resource counter exceeds its bound")
    # Linux comm may contain UTF-8. Its text is not an ownership signal.
    return raw.decode("utf-8", errors="replace")


class ProcSampler:
    """CPU uses the aggregate machine counter as denominator, including all cores.

    Module RSS sums resident process pages; shared pages may be counted more than
    once. Process identities include kernel start time, not just a recyclable PID.
    The caller supplies the owned module/PID registry; this class never guesses
    ownership from process names. Missing counters are unknown, not zero usage.
    """

    def __init__(
        self,
        root: Path = Path("/proc"),
        *,
        page_size: int | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.root, self.clock = root, clock
        self.page_size = page_size if page_size is not None else os.sysconf("SC_PAGE_SIZE")
        if self.page_size <= 0:
            raise ValueError("page size must be positive")
        self._sequence = 0
        self._previous: tuple[float, int, int] | None = None
        self._processes: dict[int, _Process] = {}

    def _cpu(self) -> tuple[int, int, int]:
        rows = _read(self.root / "stat").splitlines()
        values = [int(value) for value in rows[0].split()[1:9]]
        if not rows[0].startswith("cpu ") or len(values) != 8 or min(values) < 0:
            raise ValueError("invalid CPU counters")
        # Guest time is already included in user/nice; do not count it twice.
        return (
            sum(values),
            values[3] + values[4],
            sum(row.split()[0][3:].isdigit() for row in rows if row.startswith("cpu")),
        )

    def _memory(self) -> tuple[int | None, int | None]:
        try:
            rows = dict(row.split(":", 1) for row in _read(self.root / "meminfo").splitlines())
            total = int(rows["MemTotal"].split()[0]) * 1024
            available = int(rows["MemAvailable"].split()[0]) * 1024
            if not 0 <= available <= total or total <= 0:
                raise ValueError("invalid memory counters")
            return total, total - available
        except (OSError, ValueError, KeyError, IndexError):
            return None, None

    def _process(self, pid: int) -> _Process:
        raw = _read(self.root / str(pid) / "stat", 8192)
        # comm may itself contain spaces and parentheses. Numeric fields follow
        # the last closing parenthesis; splitting the entire row corrupts indices.
        end = raw.rfind(")")
        if end < 0 or int(raw.split("(", 1)[0]) != pid:
            raise ValueError("invalid process identity")
        fields = raw[end + 1 :].split()
        ticks, birth, pages = int(fields[11]) + int(fields[12]), int(fields[19]), int(fields[21])
        if min(ticks, birth, pages) < 0:
            raise ValueError("invalid process counters")
        return _Process(birth, ticks, pages * self.page_size, int(fields[1]))

    def sample(
        self,
        owners: Mapping[str, tuple[int, ...]],
        *,
        expected_births: Mapping[int, int] | None = None,
    ) -> ResourceReading:
        if len(owners) > 16 or sum(len(pids) for pids in owners.values()) > 128:
            raise ValueError("resource registry exceeds its bound")
        if any(not name or len(name) > 64 for name in owners):
            raise ValueError("invalid resource module")
        if any(type(pid) is not int or pid < 1 for pids in owners.values() for pid in pids):
            raise ValueError("invalid resource PID")
        now = self.clock()
        if not math.isfinite(now):
            raise ValueError("invalid resource clock")
        previous = self._previous
        try:
            total, idle, count = self._cpu()
            if count < 1:
                raise ValueError("CPU count unavailable")
            self._previous = (now, total, idle)
        except (OSError, ValueError, IndexError):
            self._previous = None
            total, idle, count = 0, 0, 0
        interval = None if previous is None or now <= previous[0] else now - previous[0]
        delta = 0 if previous is None else total - previous[1]
        idle_delta = 0 if previous is None else idle - previous[2]
        valid = interval is not None and delta > 0 and 0 <= idle_delta <= delta
        cpu = 100 * (delta - idle_delta) / delta if valid else None
        current: dict[int, _Process] = {}
        failed = set()
        for pid in {pid for pids in owners.values() for pid in pids}:
            try:
                row = self._process(pid)
                if expected_births is not None and expected_births.get(pid) != row.birth:
                    raise ValueError("owned process identity changed during sample")
                current[pid] = row
            except (OSError, ValueError, IndexError):
                failed.add(pid)
        modules = []
        for name, pids in owners.items():
            members = set(pids)
            complete = not members & failed
            rows = [current[pid] for pid in members if pid in current]
            known = (
                complete
                and valid
                and all(
                    pid in self._processes
                    and current[pid].birth == self._processes[pid].birth
                    and current[pid].ticks >= self._processes[pid].ticks
                    for pid in members
                )
            )
            ticks = (
                sum(current[pid].ticks - self._processes[pid].ticks for pid in members)
                if known
                else 0
            )
            modules.append(
                ModuleReading(
                    name,
                    "inactive" if not members else "running" if complete else "unavailable",
                    100 * ticks / delta if members and known and ticks <= delta else None,
                    sum(row.rss for row in rows) if members and complete else None,
                    len(rows),
                )
            )
        self._processes = current
        self._sequence += 1
        memory_total, memory_used = self._memory()
        return ResourceReading(
            self._sequence, now, interval, count, cpu, memory_total, memory_used, tuple(modules)
        )
