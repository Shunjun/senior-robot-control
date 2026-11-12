"""Labeled synthetic GPS/statistical CI and failure demo; never hardware evidence."""

import argparse
import json
from dataclasses import asdict, replace
from pathlib import Path
from typing import Any

from senior_control.models import Pose2D
from senior_control.motion.observations import (
    Covariance,
    CovarianceKind,
    Frame,
    Heading,
    Observation,
    Source,
    SourceStatus,
)
from senior_control.motion.sample_types import PointObservation
from tools.motion_fusion_replay import ReplayChain, replay_policy


def _point(
    source: Source,
    t: float,
    seq: int,
    values: tuple[float, ...],
    status: SourceStatus = SourceStatus.VALID,
) -> PointObservation:
    covariance = Covariance(
        (0.04 if source is Source.GPS else 0.01,) * len(values),
        CovarianceKind.DIAGONAL_ONLY,
        "SYNTHETIC injected statistical model, not measured",
        None,
    )
    return PointObservation(
        source,
        Frame.ENU if source is Source.GPS else Frame.BODY,
        t,
        100 + t,
        "synthetic",
        seq,
        1,
        status,
        values,
        covariance,
        "synthetic-known",
        Pose2D(0, 0, 0) if source is Source.GPS else None,
    )


def synthetic_demo() -> list[dict[str, Any]]:
    policy = replay_policy()
    policy = replace(
        policy, fusion=replace(policy.fusion, trusted_statistical_models=("synthetic-known",))
    )
    chain = ReplayChain(policy)
    sequences = {s: 0 for s in Source}
    results: list[dict[str, Any]] = []
    events = (
        [(i / 50, "imu") for i in range(41)]
        + [(i / 30, "wheel_gps") for i in range(25)]
        + [(i / 10, "scan_query") for i in range(1, 9)]
    )
    for t, kind in sorted(events, key=lambda x: (x[0], x[1] == "scan_query")):
        if kind == "imu":
            sequences[Source.IMU] += 1
            status = SourceStatus.UNAVAILABLE if 0.38 <= t < 0.45 else SourceStatus.VALID
            chain.ingest(_point(Source.IMU, t, sequences[Source.IMU], (0.0,), status), t, 100 + t)
        elif kind == "wheel_gps":
            for source, values in ((Source.WHEEL, (0.3, 0.0)), (Source.GPS, (0.5, 0.0))):
                sequences[source] += 1
                status = (
                    SourceStatus.DRIFT
                    if source is Source.GPS and 0.24 <= t < 0.45
                    else SourceStatus.VALID
                )
                h = Heading(0, 0, t, 100 + t, "synthetic-h", "synthetic known yaw")
                chain.ingest(
                    _point(source, t, sequences[source], values, status),
                    t,
                    100 + t,
                    h if source is Source.GPS else None,
                )
        else:
            sequences[Source.LASER] += 1
            status = SourceStatus.UNAVAILABLE if 0.24 <= t < 0.45 else SourceStatus.VALID
            sample = Observation(
                Source.LASER,
                Frame.BODY,
                t - 0.1,
                t,
                100 + t,
                "synthetic",
                sequences[Source.LASER],
                1,
                status,
                (0.3, 0.0, 0.0),
                Covariance(
                    (0.01,) * 3, CovarianceKind.DIAGONAL_ONLY, "SYNTHETIC known rate variance", None
                ),
                "synthetic-known",
            )
            chain.ingest(sample, t, 100 + t)
            results.append(
                {
                    "label": "synthetic inputs only",
                    "result": asdict(chain.window(t - 0.1, t, t, 100 + t)),
                }
            )
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = synthetic_demo()
    with args.output.open("x") as f:
        f.writelines(json.dumps(row, allow_nan=False) + "\n" for row in rows)
    print(f"{len(rows)} synthetic windows; not physical qualification")


if __name__ == "__main__":
    main()
