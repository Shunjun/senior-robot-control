"""One bounded background sample; synchronous readers only access the cache."""

import asyncio
import time
from collections.abc import Mapping
from typing import Any, Literal, cast

from senior_contracts.wire import ModulePerformance, PerformancePayload

from vehicle_manager.performance.proc import ProcSampler, _read

MODULES = ("video", "navigation", "mapping")


class PerformanceMonitor:
    def __init__(self, sampler: ProcSampler | None = None, *, interval: float = 1.0) -> None:
        if interval < 1 or interval > 60:
            raise ValueError("performance interval must be between 1 and 60 seconds")
        self.sampler = sampler or ProcSampler()
        self.interval = interval
        self._roots: dict[str, dict[int, int]] = {name: {} for name in MODULES}
        self._known: set[str] = set()
        self._cached: PerformancePayload | None = None

    def event(self, value: Mapping[str, Any]) -> None:
        """Only process owners register identities; executable names never grant ownership."""
        name = value.get("resource_module")
        if name not in MODULES:
            return
        pid, birth = value.get("pid"), value.get("process_start")
        if type(pid) is not int or type(birth) is not int or pid < 1 or birth < 0:
            return
        roots = self._roots[name]
        if value.get("phase") == "resource_started" and len(roots) < 32:
            roots[pid] = birth
            self._known.add(name)
        elif value.get("phase") == "resource_exited" and roots.get(pid) == birth:
            del roots[pid]

    def read(self) -> PerformancePayload | None:
        return self._cached

    def _members(self, roots: Mapping[int, int]) -> dict[int, int]:
        """Walk only registered owned subtrees, bounded to 128 processes/256 tasks.

        Keeping a dead root makes the module unavailable rather than transferring
        its identity to a recycled PID. Child births are checked by ProcSampler.
        """
        members: dict[int, int] = {}
        pending: list[tuple[int, int | None]] = [(pid, None) for pid in roots]
        tasks_read = 0
        while pending:
            pid, parent = pending.pop()
            if pid in members:
                continue
            if len(members) >= 128:
                raise ValueError("owned process tree exceeds telemetry bound")
            current = self.sampler._process(pid)
            if pid in roots and current.birth != roots[pid]:
                raise ValueError("registered process identity changed")
            if parent is not None and current.parent != parent:
                raise ValueError("owned child changed parent")
            members[pid] = current.birth
            task_root = self.sampler.root / str(pid) / "task"
            tasks = task_root.iterdir()
            for task in tasks:
                tasks_read += 1
                if tasks_read > 256:
                    raise ValueError("owned task tree exceeds telemetry bound")
                if task.name.isdigit():
                    pending.extend((int(child), pid) for child in _read(task / "children").split())
                    if len(pending) + len(members) > 256:
                        raise ValueError("owned process discovery exceeds telemetry bound")
        return members

    def _sample(self, roots: dict[str, dict[int, int]], known: set[str]) -> PerformancePayload:
        owners = {}
        expected_births = {}
        unavailable = set(MODULES) - known
        for name, values in roots.items():
            try:
                members = self._members(values)
                owners[name] = tuple(members)
                expected_births.update(members)
            except (OSError, ValueError, IndexError):
                owners[name] = ()
                unavailable.add(name)
        # Bound across modules as well as per tree, before any counter read.
        if sum(len(pids) for pids in owners.values()) > 128:
            owners = dict.fromkeys(MODULES, ())
            unavailable.update(MODULES)
        reading = self.sampler.sample(owners, expected_births=expected_births)
        return PerformancePayload(
            sequence=reading.sequence,
            sampled_at=time.time(),
            interval_s=reading.interval_s,
            cpu_count=reading.cpu_count,
            cpu_percent=reading.cpu_percent,
            memory_total_bytes=reading.memory_total_bytes,
            memory_used_bytes=reading.memory_used_bytes,
            modules=[
                ModulePerformance(
                    # The sampler receives only this monitor's fixed MODULES registry.
                    name=cast(Literal["video", "navigation", "mapping"], row.name),
                    status="unavailable" if row.name in unavailable else row.status,
                    cpu_percent=None if row.name in unavailable else row.cpu_percent,
                    rss_bytes=None if row.name in unavailable else row.rss_bytes,
                    process_count=row.process_count,
                )
                for row in reading.modules
            ],
        )

    async def run(self) -> None:
        while True:
            started = time.monotonic()
            roots = {name: dict(values) for name, values in self._roots.items()}
            self._cached = await asyncio.to_thread(self._sample, roots, set(self._known))
            await asyncio.sleep(max(0, self.interval - (time.monotonic() - started)))
