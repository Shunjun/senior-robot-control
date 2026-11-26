"""Task-owned observation only: bounded JSONL, /proc identities and resource samples.

No process control, authorization writer, scheduler or DDS loss estimator lives here.
Receive/cycle observations are upper bounds; unavailable lock/queue data stays N/A.
"""

import json
import os
import queue
import threading
import time
from collections import Counter, OrderedDict, defaultdict
from pathlib import Path

SCHEMA = 1
KEY_FIELDS = (
    "backend_boot",
    "bridge_boot_id",
    "source_epoch",
    "capture_generation",
    "clock_generation",
    "raw_gid",
    "bridge_gid",
    "sequence",
)
MAX_ROW_BYTES = 262144


def identity(row):
    return (row["host_boot_id"], *(row["key"][name] for name in KEY_FIELDS))


class TraceWriter:
    """emit only queues immutable-by-convention small observations; writer owns IO."""

    def __init__(self, path: Path, metadata: dict, *, capacity: int = 4096):
        self.path, self.metadata = path, metadata
        self.queue = queue.Queue(maxsize=capacity)
        if capacity < 1:
            raise ValueError("positive trace capacity required")
        self.dropped = 0
        self.max_queue = 0
        self.emit_elapsed_ns = 0
        self.error = None
        self._closed = False
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._write, name="shared-layer-trace")

    def __enter__(self):
        self._thread.start()
        return self

    def emit(self, stage: str, **fields) -> None:
        started = time.monotonic_ns()
        row = dict(schema=SCHEMA, stage=stage, steady_ns=started, **fields)
        with self._lock:
            if self._closed:
                raise RuntimeError("trace already closed")
            if self.error is not None:
                raise RuntimeError("trace writer failed") from self.error
            try:
                self.queue.put_nowait(row)
                self.max_queue = max(self.max_queue, self.queue.qsize())
            except queue.Full:
                self.dropped += 1
            self.emit_elapsed_ns += time.monotonic_ns() - started

    def _write(self):
        started_cpu = time.thread_time_ns()
        rows_written = bytes_written = 0
        try:
            fd = os.open(self.path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, "w") as output:
                output.write(
                    json.dumps(
                        dict(
                            schema=SCHEMA,
                            stage="metadata",
                            steady_ns=time.monotonic_ns(),
                            **self.metadata,
                        ),
                        allow_nan=False,
                    )
                    + "\n"
                )
                while not self._stop.is_set() or not self.queue.empty():
                    try:
                        row = self.queue.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    encoded = json.dumps(row, allow_nan=False, separators=(",", ":"))
                    if len(encoded.encode()) > MAX_ROW_BYTES:
                        with self._lock:
                            self.dropped += 1
                    else:
                        output.write(encoded + "\n")
                        rows_written += 1
                        bytes_written += len(encoded.encode()) + 1
                output.write(
                    json.dumps(
                        dict(
                            schema=SCHEMA,
                            stage="trace_end",
                            steady_ns=time.monotonic_ns(),
                            dropped=self.dropped,
                            rows_written=rows_written,
                            bytes_written=bytes_written,
                            max_queue=self.max_queue,
                            emit_elapsed_ns=self.emit_elapsed_ns,
                            writer_cpu_ns=time.thread_time_ns() - started_cpu,
                        )
                    )
                    + "\n"
                )
        except BaseException as exc:
            self.error = exc

    def __exit__(self, *_):
        with self._lock:
            self._closed = True
        self._stop.set()
        self._thread.join(timeout=10)
        if self._thread.is_alive():
            raise RuntimeError("trace writer settlement unconfirmed")
        if self.error is not None:
            raise RuntimeError("trace writer failed") from self.error


def process_identity(pid: int) -> dict | None:
    try:
        text = Path(f"/proc/{pid}/stat").read_text()
        fields = text[text.rindex(")") + 2 :].split()
        return dict(
            pid=pid,
            starttime=int(fields[19]),
            ppid=int(fields[1]),
            pgid=int(fields[2]),
            state=fields[0],
            cpu_ticks=int(fields[11]) + int(fields[12]),
            cgroup=Path(f"/proc/{pid}/cgroup").read_text().strip(),
        )
    except (FileNotFoundError, ProcessLookupError):
        return None
    except (OSError, ValueError, IndexError) as exc:
        raise RuntimeError(f"process identity unconfirmed: {pid}") from exc


def alive(row: dict) -> bool:
    current = process_identity(row["pid"])
    # A zombie has not been reaped; it is deliberately still an obligation.
    return current is not None and current["starttime"] == row["starttime"]


class ResourceSampler:
    def __init__(self, writer: TraceWriter):
        self.writer = writer
        self.owned = {}
        self.files = []
        self._file_versions = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="shared-layer-resources")

    def register(self, pid: int):
        row = process_identity(pid)
        if row is None:
            raise RuntimeError("launched PID disappeared before identity capture")
        with self._lock:
            if len(self.owned) >= 512:
                raise RuntimeError("owned process observation capacity exceeded")
            self.owned[(pid, row["starttime"])] = row
        self.writer.emit("process_start", process=row)

    def start(self):
        self._thread.start()

    def close(self):
        self._stop.set()
        self._thread.join(timeout=5)
        if self._thread.is_alive():
            raise RuntimeError("resource sampler settlement unconfirmed")

    def snapshot(self):
        with self._lock:
            return tuple(dict(row) for row in self.owned.values())

    def _discover(self):
        # Descendants only. No arbitrary matching by executable or node name.
        rows = [process_identity(int(p.name)) for p in Path("/proc").iterdir() if p.name.isdigit()]
        rows = [row for row in rows if row]
        known = {row["pid"]: row for row in self.snapshot() if alive(row)}
        for _ in range(16):
            added = False
            for row in rows:
                parent = known.get(row["ppid"])
                if parent and row["pid"] not in known and row["starttime"] >= parent["starttime"]:
                    self.register(row["pid"])
                    known[row["pid"]] = row
                    added = True
            if not added:
                break

    def _resources(self, full):
        for expected in self.snapshot():
            row = process_identity(expected["pid"])
            if row is None or row["starttime"] != expected["starttime"]:
                continue
            if full:
                root = Path(f"/proc/{row['pid']}")
                for name in ("status", "smaps_rollup", "io"):
                    try:
                        row[name] = (root / name).read_text()[:32768]
                    except OSError:
                        row[name] = "N/A"
                try:
                    row["cmdline"] = (
                        (root / "cmdline")
                        .read_bytes()[:8192]
                        .replace(b"\0", b" ")
                        .decode(errors="replace")
                    )
                except OSError:
                    row["cmdline"] = "N/A"
            self.writer.emit("resource", process=row, full=full)
        groups = {row["cgroup"] for row in self.snapshot()}
        for group in groups:
            paths = [line[3:] for line in group.splitlines() if line.startswith("0::")]
            if not paths or paths[0] == "/":
                continue
            root = Path("/sys/fs/cgroup") / paths[0].lstrip("/")
            values = {}
            names = ("cpu.stat", "cpu.pressure", "cpu.max", "cpuset.cpus.effective")
            if full:
                names += (
                    "memory.current",
                    "memory.peak",
                    "memory.max",
                    "memory.events",
                    "memory.swap.current",
                    "io.stat",
                )
            for name in names:
                try:
                    values[name] = (root / name).read_text()[:32768]
                except OSError:
                    values[name] = "N/A"
            self.writer.emit("cgroup", cgroup=group, values=values)

    def _run(self):
        tick = 0
        while not self._stop.is_set():
            started = time.monotonic_ns()
            try:
                self._discover()
                self._resources(tick % 5 == 0)
                for path in tuple(self.files):
                    try:
                        with path.open("rb") as stream:
                            raw = stream.read(65537)
                        if len(raw) > 65536:
                            raise ValueError("contract observation exceeds bound")
                        if self._file_versions.get(path) != raw:
                            self._file_versions[path] = raw
                            self.writer.emit("file", path=str(path), record=json.loads(raw))
                    except FileNotFoundError:
                        pass
                    except (OSError, ValueError) as exc:
                        self.writer.emit("observation_error", path=str(path), reason=str(exc))
                self.writer.emit("sampler_overhead", elapsed_ns=time.monotonic_ns() - started)
            except (OSError, RuntimeError) as exc:
                self.writer.emit("observation_error", reason=str(exc))
            tick += 1
            elapsed = (time.monotonic_ns() - started) / 1e9
            if elapsed > 0.2:
                self.writer.emit("sample_gap", overrun_ns=int((elapsed - 0.2) * 1e9))
            self._stop.wait(max(0, 0.2 - elapsed))


def _bounded_lines(path: Path):
    with path.open() as stream:
        while True:
            line = stream.readline(MAX_ROW_BYTES + 2)
            if not line:
                return
            yield line


def summarize_system_trace(path: Path) -> dict:
    """Streaming decode; bounded capture window, no synthetic publish/control cause."""
    stages, captures, latencies = Counter(), OrderedDict(), defaultdict(list)
    dropped = evicted = unmatched = 0
    metadata = None
    complete = False
    process_peaks = {}
    previous_cpu = {}
    footer = {}
    latest_truth = None
    pose_errors = []
    reasons = Counter()
    sampler_ns = []
    phase = "bootstrap"
    phase_peaks = defaultdict(dict)
    phase_cgroup_current = defaultdict(dict)
    counters = {}
    cgroup_latest = {}
    for line_number, raw in enumerate(_bounded_lines(path), 1):
        try:
            if complete:
                raise ValueError("data after trace_end")
            if len(raw.encode()) > MAX_ROW_BYTES + 1:
                raise ValueError("oversized row")
            row = json.loads(raw)
            if row["schema"] != SCHEMA or type(row["steady_ns"]) is not int:
                raise ValueError("unsupported schema/time")
            stage = row["stage"]
            if stage not in stages and len(stages) >= 64:
                raise ValueError("stage observation capacity exceeded")
            stages[stage] += 1
            if row.get("reason"):
                reasons[str(row["reason"])[:256]] += 1
            if stage == "phase":
                phase = row["name"]
            if stage == "raw_input":
                latest_truth = row["pose"]
            if stage == "state" and latest_truth is not None:
                pose = row["record"].get("robot_pose")
                if isinstance(pose, dict) and all(name in pose for name in ("x", "y", "yaw")):
                    import math

                    pose_errors.append(
                        (
                            math.hypot(pose["x"] - latest_truth[0], pose["y"] - latest_truth[1]),
                            abs(math.remainder(pose["yaw"] - latest_truth[2], math.tau)),
                        )
                    )
                    if len(pose_errors) > 8192:
                        pose_errors.pop(0)
            if stage == "sampler_overhead" and len(sampler_ns) < 8192:
                sampler_ns.append(row["elapsed_ns"])
            if stage == "cgroup":
                cgroup_latest[row["cgroup"]] = row["values"]
                for label in ("memory.current", "memory.swap.current"):
                    value = row["values"].get(label, "N/A").strip()
                    if value.isdigit():
                        phase_cgroup_current[phase][label] = max(
                            phase_cgroup_current[phase].get(label, 0), int(value)
                        )
            if len(reasons) > 256 or len(cgroup_latest) > 128:
                raise ValueError("summary observation capacity exceeded")
            if stage == "metadata":
                if metadata is not None:
                    raise ValueError("duplicate metadata")
                metadata = row
            elif stage == "trace_end":
                complete, dropped = True, row["dropped"]
                if type(dropped) is not int or dropped < 0:
                    raise ValueError("invalid trace loss")
                footer = row
            elif stage == "resource":
                proc = row["process"]
                key = f"{proc['pid']}:{proc['starttime']}"
                if key not in process_peaks and len(process_peaks) >= 512:
                    raise ValueError("process observation capacity exceeded")
                peaks = process_peaks.setdefault(key, {})
                sample_metrics = {}
                for field, label in (("status", "VmRSS:"), ("smaps_rollup", "Pss:")):
                    for text in proc.get(field, "").splitlines():
                        if text.startswith(label):
                            sample_metrics[label] = int(text.split()[1]) * 1024
                            peaks[label] = max(peaks.get(label, 0), sample_metrics[label])
                old = previous_cpu.get(key)
                if old and row["steady_ns"] > old[0]:
                    percent = (
                        (proc["cpu_ticks"] - old[1])
                        / os.sysconf("SC_CLK_TCK")
                        / ((row["steady_ns"] - old[0]) / 1e9)
                        * 100
                    )
                    sample_metrics["cpu_percent_one_core"] = percent
                    peaks["cpu_percent_one_core"] = max(
                        peaks.get("cpu_percent_one_core", 0), percent
                    )
                previous_cpu[key] = (row["steady_ns"], proc["cpu_ticks"])
                per_process = phase_peaks[phase].setdefault(key, {})
                for label, value in sample_metrics.items():
                    per_process[label] = max(per_process.get(label, 0), value)
                current_counters = {}
                for text in (proc.get("io", "") + "\n" + proc.get("status", "")).splitlines():
                    name, separator, value = text.partition(":")
                    if (
                        separator
                        and value.strip().isdigit()
                        and name
                        in (
                            "read_bytes",
                            "write_bytes",
                            "rchar",
                            "wchar",
                            "syscr",
                            "syscw",
                            "voluntary_ctxt_switches",
                            "nonvoluntary_ctxt_switches",
                        )
                    ):
                        current_counters[name] = int(value.strip())
                if current_counters:
                    history = counters.setdefault(key, [current_counters, current_counters])
                    history[1] = current_counters
            if "key" in row and row["key"] is not None:
                key = identity(row)
                if stage in ("qualified", "q", "tf", "global", "local"):
                    if row["deadline_ns"] != row["sample_steady_ns"] + 500_000_000:
                        raise ValueError("capture renewed scan deadline")
                observations = captures.setdefault(key, {})
                existing = observations.get(stage)
                if existing is not None:
                    for name in ("sample_steady_ns", "deadline_ns", "stamp_ns"):
                        if name in row and name in existing and row[name] != existing[name]:
                            raise ValueError("duplicate capture changed original " + name)
                observations.setdefault(stage, row)
                captures.move_to_end(key)
                if len(captures) > 4096:
                    captures.popitem(last=False)
                    evicted += 1
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid trace line {line_number}: {exc}") from exc
    if metadata is None:
        raise ValueError("trace has no metadata")
    coverage_before_observer_q = 0
    expired = 0
    for observations in captures.values():
        q = observations.get("q")
        if ("global" in observations or "local" in observations) and q is None:
            unmatched += 1
        for stage in ("qualified", "q", "tf", "global", "local"):
            row = observations.get(stage)
            if row is None:
                continue
            sample = row["sample_steady_ns"]
            if row.get("deadline_ns") != sample + 500_000_000:
                raise ValueError("capture renewed scan deadline")
            endpoint = row.get("cycle_completed_steady_ns", row["steady_ns"])
            expired += endpoint >= row["deadline_ns"]
            if stage in ("global", "local"):
                if q is None:
                    continue
                if (
                    sample != q["sample_steady_ns"]
                    or row["scan_deadline_steady_ns"] != q["deadline_ns"]
                ):
                    raise ValueError("coverage does not match original Q")
                if not (
                    row["cycle_started_steady_ns"]
                    <= endpoint
                    < row["valid_until_steady_ns"]
                    <= q["deadline_ns"]
                ):
                    raise ValueError("coverage cycle/deadline changed")
                elapsed = endpoint - q["steady_ns"]
                if elapsed >= 0:
                    latencies["q_observed_to_" + stage].append(elapsed / 1e6)
                else:
                    coverage_before_observer_q += 1
            if endpoint >= sample:
                latencies[stage].append((endpoint - sample) / 1e6)

    def percentiles(values):
        ordered = sorted(values)
        return {
            name: ordered[min(len(ordered) - 1, int((len(ordered) - 1) * fraction))]
            for name, fraction in (
                ("p50_ms", 0.5),
                ("p95_ms", 0.95),
                ("p99_ms", 0.99),
                ("max_ms", 1),
            )
        }

    return dict(
        scope="observed_system_trace",
        metadata=metadata,
        stages=dict(stages),
        complete=complete,
        dropped=dropped,
        capture_evictions=evicted,
        unmatched_coverage=unmatched,
        sample_to_receive_or_cycle={
            stage: percentiles(values) for stage, values in latencies.items() if values
        },
        process_peaks=process_peaks,
        phase_process_peaks=dict(phase_peaks),
        phase_cgroup_observed_current_max=dict(phase_cgroup_current),
        resource_counter_deltas={
            key: {name: last[name] - first.get(name, last[name]) for name in last}
            for key, (first, last) in counters.items()
        },
        cgroup_last=cgroup_latest,
        reasons=dict(reasons),
        expired_observations=expired,
        coverage_before_observer_q=coverage_before_observer_q,
        recorder=footer,
        sampler_overhead=percentiles([value / 1e6 for value in sampler_ns])
        if sampler_ns
        else "N/A",
        pose_error_max=(
            dict(
                translation_m=max(value[0] for value in pose_errors),
                yaw_rad=max(value[1] for value in pose_errors),
            )
            if pose_errors
            else "N/A"
        ),
        match_quality="N/A: not exported",
        loop_quality="N/A: no observed native loop event",
        lock_wait="N/A",
        queue_depth="N/A",
        dds_loss="N/A: sequence gaps include KEEP_LAST/coalescing",
        control_causality="N/A: Twist has no capture key",
        native_publish_time="N/A",
        percentile_method="lower order statistic; first observation per capture/stage",
    )
