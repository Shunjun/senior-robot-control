"""Bounded offline consumer of T3b; inputs are native bag JSON, never truth feedback.

Use PYTHONPATH=.:src .venv/bin/python -m tools.motion_fusion_replay
with --input FILE --output NEW_DIR.
Replay steady time is synthesized from bag receipt deltas: this is not an RTF/TTL qualification.
All policy values here are reproduction settings, not frozen physical limits.
"""

import argparse
import gzip
import hashlib
import json
import math
import time
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

from senior_control.motion.fusion import FusionConfig, FusionSnapshot, NumericalFusion
from senior_control.motion.observations import AdmissionConfig, Heading, Source, SourceRule
from senior_control.motion.sample_types import MotionSample
from senior_control.motion.source_admission import AdmissionDecision, AdmissionOwner
from senior_control.motion.temporal import ResolvedWindow, TemporalConfig, TemporalResolver
from tools.motion_replay_records import NativeAdapter, TruthIndex


@dataclass(frozen=True)
class ReplayPolicy:
    admission: AdmissionConfig
    temporal: TemporalConfig
    fusion: FusionConfig
    event_capacity: int


def replay_policy() -> ReplayPolicy:
    return ReplayPolicy(
        AdmissionConfig(
            tuple(SourceRule(s, 0.5, 0.3, 0.1, 2.0) for s in Source),
            2,
            0.1,
            1.0,
            0.2,
            0.2,
            0.1,
            0.05,
            0.3,
            8,
        ),
        TemporalConfig(1.0, 128, 0.08, 0.08, 0.08, True),
        FusionConfig((), 1.0, 0.2, 0.4, 0.15),
        8192,
    )


class ReplayChain:
    """Shallow composition. AdmissionOwner alone owns health and recovery."""

    def __init__(self, policy: ReplayPolicy):
        self.owner = AdmissionOwner(policy.admission, event_capacity=policy.event_capacity)
        self.resolver = TemporalResolver(policy.temporal)
        self.fusion = NumericalFusion(policy.fusion)
        self.last_window: ResolvedWindow | None = None

    def ingest(
        self,
        sample: MotionSample,
        now_ros: float,
        now_steady: float,
        heading: Heading | None = None,
    ) -> AdmissionDecision:
        decision = self.owner.ingest(
            sample, now_ros=now_ros, now_steady=now_steady, heading=heading
        )
        snapshot = self.owner.snapshot(
            after_cursor=self.resolver.cursor, now_ros=now_ros, now_steady=now_steady
        )
        self.resolver.consume(snapshot)
        return decision

    def window(self, a: float, b: float, now_ros: float, now_steady: float) -> FusionSnapshot:
        snapshot = self.owner.snapshot(
            after_cursor=self.resolver.cursor, now_ros=now_ros, now_steady=now_steady
        )
        self.resolver.consume(snapshot)
        self.last_window = self.resolver.resolve(a, b, snapshot)
        return self.fusion.update(self.last_window, snapshot, self.resolver)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run_native(path: Path, output: Path, max_records: int = 500000) -> dict[str, Any]:
    """Preserve input ordering, every admission result, every scan-triggered query and failure."""
    output.mkdir(parents=True, exist_ok=False)
    policy = replay_policy()
    adapter = NativeAdapter(path.parent.name)
    chain = ReplayChain(policy)
    truth = TruthIndex(0.08, 100000)
    topics: Counter[str] = Counter()
    decisions: Counter[str] = Counter()
    errors: list[dict[str, Any]] = []
    windows: list[dict[str, Any]] = []
    durations: dict[str, float] = defaultdict(float)
    coverage: Counter[str] = Counter()
    failures: Counter[str] = Counter()
    costs: list[float] = []
    start = time.perf_counter()
    previous: float | None = None
    steady, segment = 10.0, 0
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt") as data, (output / "admissions.jsonl").open("w") as admissions:
        for ordinal, line in enumerate(data, 1):
            if ordinal > max_records:
                raise ValueError("bounded record count exceeded")
            record = json.loads(line)
            topic = str(record["topic"])
            topics[topic] += 1
            now = int(record["bag_ns"]) / 1e9
            if not math.isfinite(now):
                raise ValueError("nonfinite bag time")
            if previous is not None:
                if now < previous:
                    segment += 1
                steady += max(0.0, now - previous)
            previous = now
            if topic == "/ground_truth/odom":
                truth.add_message(segment, record["message"])
                continue  # scoring-only object never reaches chain
            try:
                sample = adapter.convert(record, steady)
            except (KeyError, TypeError, ValueError) as error:
                errors.append({"ordinal": ordinal, "topic": topic, "error": str(error)})
                continue
            if sample is None:
                continue
            tick = time.perf_counter()
            decision = chain.ingest(sample, now, steady)
            decisions[f"{sample.source.value}:{decision.reason}"] += 1
            admissions.write(
                json.dumps(
                    {
                        "ordinal": ordinal,
                        "bag_ros": now,
                        "virtual_steady": steady,
                        "sample": asdict(sample),
                        "decision": asdict(decision),
                    },
                    allow_nan=False,
                )
                + "\n"
            )
            if topic == "/motion/scan":
                b = sample.end_ros
                a = sample.start_ros if record["message"]["valid"] else max(0.0, b - 0.1)
                if a < b <= now:
                    result = chain.window(a, b, now, steady)
                    resolved = chain.last_window
                    assert resolved is not None
                    for source in resolved.sources:
                        if source.reason == "covered":
                            coverage[source.source.value] += 1
                        else:
                            failures[f"{source.source.value}:{source.reason}"] += 1
                    durations[result.mode.value] += b - a
                    windows.append(
                        {
                            "ordinal": ordinal,
                            "clock_segment": segment,
                            "raw_scan_valid": record["message"]["valid"],
                            "raw_scan_reason": record["message"]["reason"],
                            "fusion": asdict(result),
                            "resolved": asdict(resolved),
                        }
                    )
                else:
                    errors.append(
                        {
                            "ordinal": ordinal,
                            "topic": topic,
                            "error": "noncausal_query_window",
                            "a": a,
                            "b": b,
                            "now": now,
                        }
                    )
            costs.append(time.perf_counter() - tick)
    error_values: dict[str, list[float]] = defaultdict(list)
    truth_count = 0
    with (output / "windows.jsonl").open("w") as out:
        for window in windows:
            f = window["fusion"]
            target = truth.average(window["clock_segment"], f["start_ros"], f["end_ros"])
            window["truth_average_scoring_only"] = target
            if target is not None:
                truth_count += 1
                for source in window["resolved"]["sources"]:
                    for axis, (actual, expected) in enumerate(
                        zip(source["values"], target, strict=True)
                    ):
                        if actual is not None:
                            error_values[f"{source['source'].value}:{axis}"].append(
                                actual - expected
                            )
                for axis, (component, expected) in enumerate(
                    zip(f["components"], target, strict=True)
                ):
                    if component["value"] is not None:
                        error_values[f"output:{axis}"].append(component["value"] - expected)
            out.write(json.dumps(window, allow_nan=False) + "\n")
    n = len(windows)
    sorted_cost = sorted(costs)
    summary: dict[str, Any] = {
        "input": str(path),
        "input_sha256": sha256(path),
        "policy": asdict(policy),
        "records": sum(topics.values()),
        "topics": dict(topics),
        "admission_results": dict(decisions),
        "windows": n,
        "truth_scored_windows": truth_count,
        "coverage_windows": dict(coverage),
        "coverage_fraction": {s: coverage[s] / n if n else None for s in (v.value for v in Source)},
        "window_failures": dict(failures),
        "query_support_seconds_by_mode": dict(durations),
        "numeric_ci_windows": sum(
            any(c["numerically_fused"] for c in w["fusion"]["components"]) for w in windows
        ),
        "same_window_errors": {
            key: {
                "n": len(xs),
                "bias": sum(xs) / len(xs),
                "mae": sum(abs(x) for x in xs) / len(xs),
                "max_abs": max(abs(x) for x in xs),
            }
            for key, xs in error_values.items()
        },
        "processing_s": time.perf_counter() - start,
        "chain_step_ms": {
            "max": max(costs) * 1000 if costs else None,
            "p95": sorted_cost[int(0.95 * (len(costs) - 1))] * 1000 if costs else None,
        },
        "adapter_errors": errors,
        "clock_receipt_rollbacks": segment,
        "limitations": [
            "virtual steady = nonnegative bag receipt deltas; "
            "not measured wall/RTF/TTL qualification",
            "no GPS in real records; no trusted real statistical models, so CI must remain absent",
            "wheel/IMU producer identity unavailable: recording-scoped synthetic epochs",
            "mode support durations sum query intervals, not total recording wall time",
            "raw scan validity unchanged; laser faults remain negative evidence",
            "caps are injected demonstration values; upper always unsupported; no vehicle output",
        ],
    }
    sources = [
        Path(__file__),
        Path(__file__).with_name("motion_replay_records.py"),
        *sorted(Path("src/senior_control/motion").glob("*.py")),
    ]
    summary["source_sha256"] = {str(p): sha256(p) for p in sources}
    summary["outputs_sha256"] = {p.name: sha256(p) for p in output.iterdir() if p.is_file()}
    (output / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-records", type=int, default=500000)
    args = parser.parse_args()
    result = run_native(args.input, args.output, args.max_records)
    print(
        json.dumps(
            {
                k: result[k]
                for k in ("windows", "numeric_ci_windows", "coverage_windows", "adapter_errors")
            }
        )
    )


if __name__ == "__main__":
    main()
