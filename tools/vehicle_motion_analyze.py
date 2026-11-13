"""Windowed diagnostics over raw capture records, never physical acceptance.

All windows use receive-steady time. ROS source times are used only for
interval alignment; their clocks are never subtracted from receive-steady.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TypeGuard

# Diagnostic discriminators only; these are NOT frozen controller/safety limits.
WINDOW_S = 0.5
MAX_SAMPLE_GAP_S = 0.21
MAX_SOURCE_LAG_S = 0.25
LINEAR_DIFFERENCE_MPS = 0.15
ALIGNMENT_WAIT_S = 0.25
REQUIRED = (
    "clock",
    "scan",
    "scan_motion",
    "motion",
    "wheel",
    "imu",
    "command",
    "safety",
    "truth",
    "ipc",
)


@dataclass(frozen=True)
class Sample:
    index: int
    stream: str
    steady: float
    stamp: float | None
    data: dict[str, Any]


def number(value: object) -> TypeGuard[int | float]:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _read(path: Path) -> tuple[list[dict[str, Any]], str]:
    if path.stat().st_size > 128 * 1024 * 1024:
        raise ValueError("trace exceeds 128 MiB; split capture windows")
    raw = path.read_bytes()
    if path.suffix == ".json":
        document = json.loads(raw)
        records = document["samples"]
    else:
        records = [json.loads(line) for line in raw.splitlines() if line.strip()]
    if not isinstance(records, list) or any(not isinstance(row, dict) for row in records):
        raise ValueError("trace must contain object records")
    return records, hashlib.sha256(raw).hexdigest()


def _parse(records: list[dict[str, Any]]) -> tuple[dict[str, list[Sample]], list[int]]:
    streams: dict[str, list[Sample]] = defaultdict(list)
    invalid = []
    last: dict[str, tuple[float, float | None]] = {}
    for index, row in enumerate(records):
        if row.get("kind") != "sample":
            continue
        stream, steady, stamp = (row.get(k) for k in ("stream", "received_steady", "stamp_ros"))
        data = row.get("data")
        if (
            not isinstance(stream, str)
            or not number(steady)
            or (stamp is not None and not number(stamp))
            or not isinstance(data, dict)
        ):
            invalid.append(index)
            if isinstance(stream, str):
                barrier_time = float(steady) if number(steady) else last.get(stream, (0.0, None))[0]
                streams[stream].append(Sample(index, stream, barrier_time, None, {"break": True}))
            continue
        previous = last.get(stream)
        if previous and (
            steady <= previous[0]
            or (stamp is not None and previous[1] is not None and stamp < previous[1])
        ):
            invalid.append(index)
            # A discontinuity terminates evidence continuity; do not sort it away.
            streams[stream].append(Sample(index, stream, float(steady), None, {"break": True}))
            last[stream] = (float(steady), stamp)
            continue
        last[stream] = (float(steady), stamp)
        streams[stream].append(Sample(index, stream, float(steady), stamp, data))
    return streams, invalid


def continuity_key(sample: Sample) -> tuple[object, object]:
    # //Compatible: older traces/fixtures carried only the producer source_epoch.
    return (
        sample.data.get("publisher_gid"),
        sample.data.get("producer_epoch", sample.data.get("source_epoch")),
    )


def _windows(samples: list[Sample], qualifies: Callable[[Sample], bool]) -> list[list[Sample]]:
    """Return sustained runs; a negative sample or missing interval breaks a run."""
    result: list[list[Sample]] = []
    run: list[Sample] = []
    for sample in samples:
        if not qualifies(sample):
            if len(run) >= 3 and run[-1].steady - run[0].steady >= WINDOW_S - 1e-9:
                result.append(run)
            run = []
            continue
        if run and (
            sample.steady - run[-1].steady > MAX_SAMPLE_GAP_S
            or continuity_key(sample) != continuity_key(run[-1])
        ):
            if len(run) >= 3 and run[-1].steady - run[0].steady >= WINDOW_S - 1e-9:
                result.append(run)
            run = []
        run.append(sample)
    if len(run) >= 3 and run[-1].steady - run[0].steady >= WINDOW_S - 1e-9:
        result.append(run)
    return result


def _interval_average(
    samples: list[Sample], start: float, end: float, *, at_steady: float | None = None
) -> float | None:
    """Integrate endpoint-interpolated velocities on the scan source interval."""
    if end <= start:
        return None
    # Never collect pieces from different receipt runs, even if ROS stamps repeat.
    runs: list[list[Sample]] = []
    run: list[Sample] = []
    for sample in samples:
        if at_steady is not None and sample.steady > at_steady + 0.11:
            continue
        if sample.stamp is None or not number(sample.data.get("vx")) or sample.data.get("break"):
            if run:
                runs.append(run)
            run = []
            continue
        if run:
            previous = run[-1]
            previous_stamp = previous.stamp
            assert previous_stamp is not None
            if (
                sample.steady <= previous.steady
                or sample.steady - previous.steady > MAX_SAMPLE_GAP_S
                or sample.stamp <= previous_stamp
                or sample.stamp - previous_stamp > MAX_SAMPLE_GAP_S
                or continuity_key(sample) != continuity_key(previous)
            ):
                runs.append(run)
                run = []
        run.append(sample)
    if run:
        runs.append(run)
    if at_steady is not None:
        # The latest receipt run is authoritative after a clock reset. Never
        # fall back to an older complete run with matching numerical ROS stamps.
        runs = (
            runs[-1:] if runs and abs(runs[-1][-1].steady - at_steady) <= MAX_SAMPLE_GAP_S else []
        )
    candidates: list[float] = []
    for run in runs:
        segments = []
        for left, right in zip(run, run[1:], strict=False):
            assert left.stamp is not None and right.stamp is not None
            lo, hi = max(start, left.stamp), min(end, right.stamp)
            if hi <= lo:
                continue
            v0, v1 = float(left.data["vx"]), float(right.data["vx"])
            dt = right.stamp - left.stamp
            a = v0 + (v1 - v0) * (lo - left.stamp) / dt
            b = v0 + (v1 - v0) * (hi - left.stamp) / dt
            segments.append((lo, hi, a, b))
        cursor, area = start, 0.0
        for lo, hi, a, b in segments:
            if abs(lo - cursor) > 1e-8:
                break
            area += (a + b) / 2 * (hi - lo)
            cursor = hi
        else:
            if abs(cursor - end) <= 1e-8:
                candidates.append(area / (end - start))
    # Ambiguous repeated clock passes cannot identify the interval being measured.
    return candidates[0] if len(candidates) == 1 else None


def _disagreements(streams: dict[str, list[Sample]]) -> list[list[Sample]]:
    def mismatched(scan: Sample) -> bool:
        start, vx = scan.data.get("interval_start_ros"), scan.data.get("vx")
        if (
            scan.stamp is None
            or scan.data.get("valid") is not True
            or not number(start)
            or not number(vx)
        ):
            return False
        # Require a nearby advancing ROS clock or independently timed truth.
        clock = streams.get("clock", []) or streams.get("truth", [])
        nearby = [
            row for row in clock if abs(row.steady - scan.steady) <= 0.11 and row.stamp is not None
        ]
        if not nearby:
            return False
        reference = min(nearby, key=lambda row: abs(row.steady - scan.steady))
        if (
            reference.stamp is None
            or not 0 <= reference.stamp - scan.stamp <= MAX_SOURCE_LAG_S + 1e-9
        ):
            return False
        wheel = _interval_average(
            streams.get("wheel", []), start, scan.stamp, at_steady=scan.steady
        )
        truth = _interval_average(
            streams.get("truth", []), start, scan.stamp, at_steady=scan.steady
        )
        return (
            wheel is not None
            and truth is not None
            and abs(wheel - truth) > LINEAR_DIFFERENCE_MPS
            and abs(float(vx) - truth) <= LINEAR_DIFFERENCE_MPS
        )

    return _windows(streams.get("scan_motion", []), mismatched)


def analyze_trace(path: Path) -> dict[str, object]:
    records, digest = _read(path)
    streams, invalid = _parse(records)
    evidence: list[dict[str, object]] = []

    def add(name: str, runs: list[list[Sample]], fields: list[str]) -> None:
        for run in runs:
            evidence.append(
                {
                    "finding": name,
                    "sample_indices": [s.index for s in run],
                    "start_steady": run[0].steady,
                    "end_steady": run[-1].steady,
                    "fields": fields,
                }
            )

    # Compare source-clock advancement across each receive-steady window.
    constant: list[Sample] = []
    pause_runs = []
    for row in streams.get("clock", []):
        if row.stamp is None or (constant and row.stamp != constant[-1].stamp):
            pause_runs.extend(_windows(constant, lambda s: s.stamp is not None))
            constant = []
        if row.stamp is not None:
            constant.append(row)
    pause_runs.extend(_windows(constant, lambda s: s.stamp is not None))
    add("clock_pause", pause_runs, ["stamp_ros", "received_steady"])
    add(
        "alignment_wait",
        _windows(
            streams.get("fusion_diagnostic", []),
            lambda s: (
                s.data.get("pending") is True
                and number(s.data.get("alignment_wait_s"))
                and s.data["alignment_wait_s"] >= ALIGNMENT_WAIT_S
            ),
        ),
        ["alignment_wait_s", "pending"],
    )
    add(
        "wheel_body_disagreement",
        _disagreements(streams),
        ["wheel.vx", "truth.vx", "scan_motion.vx", "interval_start_ros", "stamp_ros"],
    )

    # Starvation needs counter deltas across a sustained window, not reason strings.
    worker_runs = _windows(
        streams.get("worker_diagnostic", []),
        lambda s: (
            s.data.get("pending") is True
            and all(
                number(s.data.get(k))
                for k in ("submitted", "completed", "rejected_expired", "admitted")
            )
        ),
    )
    starving = []
    for run in worker_runs:
        first, last = run[0].data, run[-1].data
        keys = ("submitted", "completed", "rejected_expired", "admitted")
        if any(
            right.data[k] < left.data[k]
            for left, right in zip(run, run[1:], strict=False)
            for k in keys
        ):
            continue
        if (
            last["submitted"] - first["submitted"] >= 3
            and last["completed"] - first["completed"] >= 3
            and last["rejected_expired"] - first["rejected_expired"] >= 3
            and last["admitted"] == first["admitted"]
        ):
            starving.append(run)
    add("permit_starvation", starving, ["submitted", "completed", "rejected_expired", "admitted"])
    return {
        "schema": 1,
        "source": str(path),
        "source_sha256": digest,
        "analyzer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "raw_record_count": len(records),
        "counts": {k: len(v) for k, v in streams.items()},
        "findings": sorted({str(row["finding"]) for row in evidence}),
        "evidence": evidence,
        "invalid_samples": invalid,
        "missing_streams": [k for k in REQUIRED if not streams[k]],
        "physical_stop": "not_evaluated",
        "acceptance_frozen": False,
        "diagnostic_policy": {
            "window_s": WINDOW_S,
            "max_gap_s": MAX_SAMPLE_GAP_S,
            "max_source_lag_s": MAX_SOURCE_LAG_S,
            "difference_mps": LINEAR_DIFFERENCE_MPS,
            "alignment_wait_s": ALIGNMENT_WAIT_S,
        },
        "limitations": [
            "No physical-stop or acceptance verdict is computed.",
            "Absent numeric fusion/worker diagnostics mean unobservable, not healthy.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    result = analyze_trace(args.input)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
