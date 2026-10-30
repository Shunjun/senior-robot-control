"""Acceptance assertion for bounded reverse through a rear blind zone."""

from __future__ import annotations

import math
from pathlib import Path

from conftest import SimulationProfile

from senior_control.config import DeviceProfile
from senior_control.models import MapRef, Pose2D, Velocity
from senior_control.safety.motion_estimator import MotionEstimate
from senior_control.safety.recovery import (
    MapCorridorEvidence,
    RecoveryBounds,
    RecoveryContext,
    RecoveryController,
)
from senior_control.safety.self_filter import FilteredObservation, ScanObservation, filter_self

MAP = MapRef("acceptance", 1)


class AcceptanceMap:
    def corridor(
        self,
        map_ref: MapRef,
        pose: Pose2D,
        distance: float,
        now: float,
    ) -> MapCorridorEvidence:
        del pose, distance, now
        return MapCorridorEvidence(map_ref, True, False, False, 0.1, False)


def rear_occluded_scan() -> tuple[FilteredObservation, FilteredObservation]:
    scans: list[FilteredObservation] = []
    for angle_min in (-math.pi / 2, math.pi / 2):
        increment = math.pi / 90
        ranges = tuple(
            float("nan")
            if abs(
                abs((angle_min + index * increment + math.pi) % (2 * math.pi) - math.pi) - math.pi
            )
            < 0.4
            else 5.0
            for index in range(181)
        )
        raw = ScanObservation(
            ranges,
            angle_min,
            increment,
            0.05,
            8.0,
            10.0,
            "sim",
            Pose2D(0.0, 0.0, 0.0),
            10.0,
            "base_link",
            10.0,
            True,
            increment,
            False,
            False,
        )
        scans.append(filter_self(raw, DeviceProfile(self_mask_calibrated=True)))
    return scans[0], scans[1]


def test_rear_occlusion_requires_bounded_map_assisted_reverse(
    simulation_profile: SimulationProfile,
) -> None:
    worlds = Path(__file__).resolve().parents[2] / "ros2" / "src" / "senior_sim" / "worlds"
    assert (worlds / "wall_escape.sdf").is_file(), "wall_escape acceptance scene is missing"

    limits = DeviceProfile(
        mode="simulation",
        footprint=[
            (-0.337, -0.27),
            (0.337, -0.27),
            (0.337, 0.27),
            (-0.337, 0.27),
        ],
        self_mask_calibrated=True,
        braking_deceleration=1.0,
        yaw_braking_deceleration=1.0,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.5,
    )
    bounds = RecoveryBounds(0.08, 0.3, 0.5, 2.0, 1, 0.5, 0.01, 0.4, 0.6)
    controller = RecoveryController(limits, bounds, AcceptanceMap(), clock_domain="sim")
    context = RecoveryContext(
        candidates=(Velocity(0.8, 0.0),),
        measured=Velocity(0.0, 0.0),
        observations=rear_occluded_scan(),
        motion=MotionEstimate(
            Pose2D(0.0, 0.0, 0.0),
            Velocity(0.0, 0.0),
            (0.01, 0.01, 0.01),
            0.9,
            0.05,
        ),
        pose=Pose2D(1.0, 2.0, 0.0),
        map_ref=MAP,
        localization_quality=0.95,
        tf_stamp=10.0,
        planning_failed=False,
        slip=False,
        blocked=False,
        wheel_displacement=9.0,
    )

    decision = controller.step(context, 10.2)

    assert decision.reason == "map_assisted_reverse"
    assert decision.velocity == Velocity(-0.08, 0.0)
    assert abs(decision.velocity.vx) <= 0.08
    assert controller.distance_traveled <= 0.3
    assert simulation_profile.scene_seed == 210_202_610
