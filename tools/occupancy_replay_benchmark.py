"""Compare occupancy output, wall time and peak temporary memory on one fixed trace.

Run with a saved baseline module, e.g. git show <revision>:ros2/src/senior_mapping/
senior_mapping/occupancy.py > /tmp/occupancy-baseline.py. This is a local pure
algorithm replay, not a four-core vehicle/control-latency acceptance test.
"""

import argparse
import importlib.util
import json
import math
import sys
import time
import tracemalloc
from dataclasses import asdict
from pathlib import Path

from senior_mapping.evidence import MappingObservation, OccupancyThresholds, RangeRay, SceneHit
from senior_mapping.occupancy import OccupancyMaintainer

from vehicle_core.values import MapRef, Pose2D


def trace() -> tuple[MappingObservation, ...]:
    rays = tuple(RangeRay(i * math.tau / 360, 12.0, True, False, None) for i in range(360))
    return tuple(
        MappingObservation(
            MapRef("replay", 1),
            "anchor",
            Pose2D(0.025, 0.025, 0),
            float(index),
            rays,
            (SceneHit(2.025, 0.025, "static"),) if index < 3 else (),
            (False,) * len(rays),
            (False,) * len(rays),
            1.0,
            True,
            False,
            0.0,
        )
        for index in range(12)
    )


def replay(factory, inputs):
    limits = OccupancyThresholds(0.05, 3, 0.4, 3, 0.4, 0.1, 1.0, 1.0, 2.5, 0.5, 0.0, 6.0, 0.5, 0.5)
    owner = factory(limits)
    deltas = tuple(owner.update(frame) for frame in inputs)
    return deltas, owner.snapshot()


def measure(factory, inputs):
    started = time.perf_counter()
    result = replay(factory, inputs)
    elapsed = time.perf_counter() - started
    tracemalloc.start()
    replay(factory, inputs)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    return result, {"wall_s": elapsed, "peak_temporary_bytes": peak}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location("occupancy_baseline", args.baseline)
    if spec is None or spec.loader is None:
        raise ValueError("baseline is not a Python module")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    inputs = trace()
    previous, old = measure(module.OccupancyMaintainer, inputs)
    current, new = measure(OccupancyMaintainer, inputs)
    if current != previous:
        raise AssertionError("occupancy deltas or final evidence changed; timing is invalid")
    print(
        json.dumps(
            {
                "scope": "local_pure_replay",
                "frames": len(inputs),
                "rays_per_frame": 360,
                "same_output": True,
                "baseline": old,
                "current": new,
                "final_cells": [asdict(row) for row in current[1].cells],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
