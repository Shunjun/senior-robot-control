"""Bounded replay timing for the candidate-7 captured Fortress scan."""

import json
import statistics
import time
from pathlib import Path

from senior_control.config import DeviceProfile
from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.safety.guard import evaluate_motion
from senior_control.safety.self_filter import ScanObservation, filter_self


def main():
    path = Path(__file__).parents[1] / "fixtures/fortress_scan_seam.json"
    fixture = json.loads(path.read_text())
    raw = fixture["scan"]
    stamp = raw["stamp"]
    scan = ScanObservation(
        tuple(raw["ranges"]),
        raw["angle_min"],
        raw["angle_increment"],
        raw["range_min"],
        raw["range_max"],
        stamp,
        "ros",
        Pose2D(*map(float, fixture["sensor_origin"])),
        0.0,
        "base_link",
        float(fixture["reference_stamp"]),
        True,
        0.01,
        False,
        True,
    )
    profile = DeviceProfile(
        mode="simulation",
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)],
        self_mask_calibrated=True,
        braking_deceleration=0.3,
        yaw_braking_deceleration=0.5,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.8,
    )
    filters, guards = [], []
    for _ in range(8):
        start = time.perf_counter()
        observation = filter_self(scan, profile)
        filtered = time.perf_counter()
        decision = evaluate_motion(
            Velocity(0.05, 0.0),
            BodyTwist(-1.3947176746853806e-19, 0.0, -6.938893903907376e-17),
            [observation],
            profile,
            stamp,
            uncertainty_margin=0.1778164193973488,
        )
        end = time.perf_counter()
        assert decision.allowed, decision.reason
        filters.append(filtered - start)
        guards.append(end - filtered)
    print(
        json.dumps(
            {
                "fixture_capture_sha256": fixture["source_capture_sha256"],
                "scan_stamp": stamp,
                "filter_seconds": filters,
                "guard_seconds": guards,
                "filter_median": statistics.median(filters),
                "guard_median": statistics.median(guards),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
