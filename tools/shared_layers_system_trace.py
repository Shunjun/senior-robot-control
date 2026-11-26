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
        # //Compatible: original callers expanded the complete coverage dataclass.
        # Its schema belongs to the contract, independently of this trace envelope.
        if stage in ("global", "local"):
            if "schema" in fields:
                fields = dict(record=fields)
            record = fields.get("record")
            if record is not None and (
                type(record.get("schema")) is not int or record["schema"] != 1
            ):
                raise ValueError("unsupported coverage contract schema")
        with self._lock:
            if self._closed:
                raise RuntimeError("trace already closed")
            if self.error is not None:
                raise RuntimeError("trace writer failed") from self.error
            # Phase/resource order follows the same monotonic queue-commit order.
            row = dict(schema=SCHEMA, stage=stage, steady_ns=time.monotonic_ns(), **fields)
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


def _process_stat(pid: int) -> dict:
    text = Path(f"/proc/{pid}/stat").read_text()
    fields = text[text.rindex(")") + 2 :].split()
    return dict(
        pid=pid,
        starttime=int(fields[19]),
        ppid=int(fields[1]),
        pgid=int(fields[2]),
        state=fields[0],
        cpu_ticks=int(fields[11]) + int(fields[12]),
    )


def process_identity(pid: int) -> dict | None:
    try:
        before = _process_stat(pid)
        group = Path(f"/proc/{pid}/cgroup").read_text().strip()
        after = _process_stat(pid)
        if before["starttime"] != after["starttime"]:
            raise RuntimeError(f"process identity unconfirmed: {pid} changed during read")
        # These are bracketed reads, not an atomic kernel snapshot. Reparenting is
        # checked separately when establishing descendant ownership.
        return dict(after, cgroup=group)
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

    def register(self, pid: int, *, expected=None, parent=None):
        row = process_identity(pid)
        if expected is not None:
            parent_now = process_identity(parent["pid"]) if parent else None
            if (
                row is None
                or row["starttime"] != expected["starttime"]
                or parent_now is None
                or parent_now["starttime"] != parent["starttime"]
                or row["ppid"] != parent["pid"]
                or expected["ppid"] != parent["pid"]
            ):
                self.writer.emit(
                    "process_unconfirmed",
                    pid=pid,
                    expected=expected,
                    parent=parent,
                    reason="descendant identity changed at registration",
                )
                return False
            # Parent checking also takes time: bind the final read to the candidate.
            current = process_identity(pid)
            if current is None or any(current[k] != row[k] for k in ("starttime", "ppid")):
                self.writer.emit(
                    "process_unconfirmed",
                    pid=pid,
                    expected=expected,
                    reason="descendant changed while checking parent",
                )
                return False
            row = current
        if row is None:
            raise RuntimeError("launched PID disappeared before identity capture")
        with self._lock:
            if len(self.owned) >= 512:
                raise RuntimeError("owned process observation capacity exceeded")
            self.owned[(pid, row["starttime"])] = row
        self.writer.emit("process_start", process=row)
        return True

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
                    if self.register(row["pid"], expected=row, parent=parent):
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
                current = process_identity(row["pid"])
                if current is None or any(current[k] != row[k] for k in ("starttime", "cgroup")):
                    self.writer.emit(
                        "process_unconfirmed",
                        pid=row["pid"],
                        expected=expected,
                        reason="process changed during full resource read",
                    )
                    continue
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


def _coverage_event(row):
    """Decode contract payloads without confusing their schema with the envelope.

    Older recorders also observed the exact same payload as a file event.
    Keep supporting both, and the original flattened helper trace shape.
    """
    stage = row["stage"]
    if stage == "file":
        stage = {"global-coverage.json": "global", "local-coverage.json": "local"}.get(
            Path(row["path"]).name
        )
    if stage not in ("global", "local"):
        return row
    record = row.get("record")
    if record is not None:
        if type(record.get("schema")) is not int or record["schema"] != 1:
            raise ValueError("unsupported coverage contract schema")
        if record.get("role", stage) != stage:
            raise ValueError("coverage file/role mismatch")
        row = dict(record, stage=stage, steady_ns=row["steady_ns"])
        row["deadline_ns"] = record["scan_deadline_steady_ns"]
    if row.get("status", "ready") != "ready":
        # Preserve/count the observation, but invalid/pending coverage has no latency.
        row = dict(row, stage=stage + "_status")
    return row


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
    cgroup_times = defaultdict(dict)
    phase_serial = 0
    phase_windows = []
    straddling = defaultdict(Counter)
    cpu_totals = {}
    control_windows = defaultdict(lambda: dict(status_samples=Counter(), output_samples=Counter()))
    route_status = "missing"
    navigation_ready = False
    route_observed_ns = state_observed_ns = -1_000_000_000
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
            row = _coverage_event(row)
            stage = row["stage"]
            if row.get("reason"):
                reasons[str(row["reason"])[:256]] += 1
            if stage == "phase":
                phase = row["name"]
                phase_serial += 1
                phase_windows.append((row["steady_ns"], phase))
                if len(phase_windows) > 128:
                    raise ValueError("phase observation capacity exceeded")
            window = control_windows[phase]
            if stage == "route_state":
                route_status = row.get("status") or (row.get("record") or {}).get(
                    "status", "missing"
                )
                route_observed_ns = row["steady_ns"]
                window["status_samples"][route_status] += 1
                if (
                    route_status == "running"
                    and navigation_ready
                    and 0 <= row["steady_ns"] - state_observed_ns <= 200_000_000
                ):
                    window.setdefault(
                        "first_running_qualified_status_observed_ns", row["steady_ns"]
                    )
                    window["last_running_qualified_status_observed_ns"] = row["steady_ns"]
                if route_status in ("failed", "succeeded", "canceled"):
                    window.setdefault("first_terminal_observed_ns", row["steady_ns"])
                    window.setdefault("terminal_status", route_status)
                    window["mppi_after_terminal"] = "N/A"
            if stage == "state":
                navigation_ready = row["record"].get("navigation_ready") is True
                state_observed_ns = row["steady_ns"]
                if not navigation_ready:
                    window.setdefault("first_disqualified_observed_ns", row["steady_ns"])
            if stage in ("control", "controller_output"):
                label = "final_cmd_vel" if stage == "control" else "private_controller"
                window["output_samples"][label] += 1
                if (
                    route_status == "running"
                    and navigation_ready
                    and 0 <= row["steady_ns"] - route_observed_ns <= 200_000_000
                    and 0 <= row["steady_ns"] - state_observed_ns <= 200_000_000
                ):
                    window["output_samples"][label + "_running_qualified"] += 1
                    if row.get("vx", 0) != 0 or row.get("wz", 0) != 0:
                        window["output_samples"][label + "_nonzero_running_qualified"] += 1
                        bounds = window.setdefault("nonzero_running_qualified_output_bounds", {})
                        bounds.setdefault(label, dict(first_observed_ns=row["steady_ns"]))
                        bounds[label]["last_observed_ns"] = row["steady_ns"]
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
                group = row["cgroup"]
                latest = cgroup_latest.setdefault(group, {})
                for name, value in row["values"].items():
                    if value != "N/A" and row["steady_ns"] >= cgroup_times[group].get(name, -1):
                        latest[name] = value
                        cgroup_times[group][name] = row["steady_ns"]
                for label in ("memory.current", "memory.swap.current"):
                    value = row["values"].get(label, "N/A").strip()
                    if value.isdigit():
                        peak = phase_cgroup_current[phase].setdefault(group, {})
                        peak[label] = max(peak.get(label, 0), int(value))
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
                    if old[2] == phase_serial:
                        sample_metrics["cpu_percent_one_core"] = percent
                    else:
                        straddling[phase][key] += 1
                    peaks["cpu_percent_one_core"] = max(
                        peaks.get("cpu_percent_one_core", 0), percent
                    )
                previous_cpu[key] = (row["steady_ns"], proc["cpu_ticks"], phase_serial)
                total = cpu_totals.setdefault(key, [row["steady_ns"], proc["cpu_ticks"], 0, 0])
                total[2:] = [row["steady_ns"], proc["cpu_ticks"]]
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
            if stage in ("qualified", "q", "tf", "slot", "global", "local") and row.get("key"):
                key = identity(row)
                if stage in ("qualified", "q", "tf", "slot", "global", "local"):
                    if row["deadline_ns"] != row["sample_steady_ns"] + 500_000_000:
                        raise ValueError("capture renewed scan deadline")
                observations = captures.setdefault(key, {})
                for other in observations.values():
                    for name in ("sample_steady_ns", "deadline_ns", "stamp_ns"):
                        if name in row and name in other and row[name] != other[name]:
                            raise ValueError("same capture changed original " + name)
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
    phase_latencies = defaultdict(lambda: defaultdict(list))
    phase_missing = defaultdict(Counter)
    phase_paired = defaultdict(Counter)

    def phase_at(stamp):
        return next(
            (name for start, name in reversed(phase_windows) if start <= stamp), "bootstrap"
        )

    for observations in captures.values():
        q = observations.get("q")
        if ("global" in observations or "local" in observations) and q is None:
            unmatched += 1
        for stage in ("qualified", "q", "tf", "slot", "global", "local"):
            row = observations.get(stage)
            if row is None:
                continue
            sample = row["sample_steady_ns"]
            if row.get("deadline_ns") != sample + 500_000_000:
                raise ValueError("capture renewed scan deadline")
            endpoint = row.get("cycle_completed_steady_ns", row["steady_ns"])
            observed_phase = phase_at(endpoint)
            expired += endpoint >= row["deadline_ns"]
            if stage in ("global", "local"):
                if q is None:
                    phase_missing[observed_phase][stage] += 1
                    continue
                if not all(name in observations for name in ("tf", "slot")):
                    phase_missing[observed_phase][stage] += 1
                else:
                    phase_paired[observed_phase][stage] += 1
                if "map_id" in q and (
                    row["map_id"],
                    row["reference_revision"],
                    row["reference_generation"],
                ) != (q["map_id"], q["reference_revision"], q["map_generation"]):
                    raise ValueError("coverage does not match original Q reference")
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
                    phase_latencies[observed_phase]["q_observed_to_" + stage].append(elapsed / 1e6)
                else:
                    coverage_before_observer_q += 1
            if endpoint >= sample:
                latencies[stage].append((endpoint - sample) / 1e6)
                phase_latencies[observed_phase][stage].append((endpoint - sample) / 1e6)

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
        latency_scope="all observed phases; use phase_capture_metrics for reference/save/restore",
        phase_windows=[
            dict(
                start_steady_ns=t,
                name=name,
                end_steady_ns=phase_windows[i][0]
                if i < len(phase_windows)
                else footer.get("steady_ns", "N/A"),
            )
            for i, (t, name) in enumerate([(metadata["steady_ns"], "bootstrap"), *phase_windows])
        ],
        phase_capture_metrics={
            name: dict(
                samples={
                    stage: len(phase_latencies[name][stage])
                    for stage in ("qualified", "q", "tf", "slot", "global", "local")
                },
                latency={
                    stage: percentiles(phase_latencies[name][stage])
                    if phase_latencies[name][stage]
                    else "N/A"
                    for stage in (
                        "qualified",
                        "q",
                        "tf",
                        "slot",
                        "global",
                        "local",
                        "q_observed_to_global",
                        "q_observed_to_local",
                    )
                },
                coverage_paired_q_tf_slot={
                    role: phase_paired[name][role] for role in ("global", "local")
                },
                coverage_missing_q_tf_slot=dict(phase_missing[name]),
                missing_roles=[
                    role for role in ("global", "local") if not phase_latencies[name][role]
                ],
            )
            for name in dict.fromkeys(
                [
                    "bootstrap",
                    "stationary_reference",
                    "save",
                    "restore_export",
                    *(name for _, name in phase_windows),
                ]
            )
        },
        process_peaks=process_peaks,
        phase_process_peaks=dict(phase_peaks),
        straddling_cpu_intervals={name: dict(counts) for name, counts in straddling.items()},
        process_cpu_total_interval={
            key: dict(
                start_steady_ns=start,
                end_steady_ns=end,
                cpu_percent_one_core=(
                    (last - first) / os.sysconf("SC_CLK_TCK") / ((end - start) / 1e9) * 100
                )
                if end > start
                else "N/A",
            )
            for key, (start, first, end, last) in cpu_totals.items()
        },
        phase_cgroup_observed_current_max=dict(phase_cgroup_current),
        resource_counter_deltas={
            key: {name: last[name] - first.get(name, last[name]) for name in last}
            for key, (first, last) in counters.items()
        },
        cgroup_last=cgroup_latest,
        cgroup_last_observed_ns=dict(cgroup_times),
        resource_scope=(
            "IO/context deltas are whole-run first-to-last per incarnation; "
            "phase CPU excludes straddling intervals; RSS peaks are not additive; "
            "PSS is not summed across different observation times; sampled current "
            "max differs from kernel memory.peak; cgroup includes recorder/producer"
        ),
        control_window_observations={
            name: dict(values) for name, values in control_windows.items()
        },
        control_window_scope=(
            "sampled running+qualified windows only, accepted is not active; "
            "status/state observations older than 200ms are unknown; "
            "terminal/disqualified times are observation upper bounds; "
            "output counts do not prove sustained MPPI or capture causality"
        ),
        sustained_mppi="N/A: no sustained MPPI claim; stationary reference cannot supply it",
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
