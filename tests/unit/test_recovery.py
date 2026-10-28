"""Bounded recovery. Speeds and distances here are explicit test configuration.

They are not a calibrated braking distance or a contact guarantee.
"""

import importlib.util
import math
from pathlib import Path

import pytest

from senior_control.config import DeviceProfile
from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D, Velocity
from senior_control.safety.guard import evaluate_motion, stopping_distance
from senior_control.safety.motion_estimator import MotionEstimate, body_displacement
from senior_control.safety.recovery import (
    MapCorridorEvidence,
    RecoveryBounds,
    RecoveryContext,
    RecoveryController,
)
from senior_control.safety.self_filter import ScanObservation, filter_self

ROOT = Path(__file__).resolve().parents[2]
RECT = ((-0.337, -0.27), (0.337, -0.27), (0.337, 0.27), (-0.337, 0.27))
NOW = 10.2
MAP = MapRef("suite", 2)


def limits(**overrides: object) -> DeviceProfile:
    values: dict[str, object] = {
        "mode": "simulation",
        "footprint": list(RECT),
        "self_mask_calibrated": True,
        "braking_deceleration": 1.0,
        "yaw_braking_deceleration": 1.0,
        "reaction_time": 0.2,
        "hard_margin": 0.02,
        "observation_max_age": 0.5,
    }
    values.update(overrides)
    return DeviceProfile(**values)  # type: ignore[arg-type]


def bounds(**overrides: object) -> RecoveryBounds:
    values: dict[str, object] = {
        "speed": 0.8,
        "single_distance": 0.5,
        "cumulative_distance": 1.0,
        "duration": 2.0,
        "attempts": 1,
        "observation_max_age": 0.5,
        "min_progress": 0.01,
        "progress_window": 0.4,
        "min_localization_quality": 0.6,
    }
    values.update(overrides)
    return RecoveryBounds(**values)  # type: ignore[arg-type]


def observation(
    angle_min: float,
    *,
    hit: float = 0.6,
    nan_rear: bool = False,
    no_return: bool = False,
):
    steps = 180
    increment = 2 * math.pi / steps
    ranges: list[float] = []
    best = 1e9
    forward_index = 0
    for index in range(steps + 1):
        angle = (angle_min + index * increment + math.pi) % (2 * math.pi) - math.pi
        distance = 5.0
        if nan_rear and abs(abs(angle) - math.pi) < 0.4:
            distance = float("nan")
        if no_return and abs(abs(angle) - math.pi) < 0.5:
            distance = float("inf")
        if abs(angle) < best and distance == distance:
            best = abs(angle)
            forward_index = index
        ranges.append(distance)
    if ranges[forward_index] == ranges[forward_index]:
        ranges[forward_index] = hit
    scan = ScanObservation(
        tuple(ranges),
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
        no_return,
        False,
    )
    return filter_self(scan, DeviceProfile(self_mask_calibrated=True))


def scan_pair(**kwargs: object):
    return (observation(-math.pi / 2, **kwargs), observation(math.pi / 2, **kwargs))  # type: ignore[arg-type]


def point_hit(x: float, y: float):
    angle = math.atan2(y, x)
    scan = ScanObservation(
        (math.hypot(x, y), 5.0),
        angle,
        0.01,
        0.05,
        8.0,
        10.0,
        "sim",
        Pose2D(0.0, 0.0, 0.0),
        10.0,
        "base_link",
        10.0,
        True,
        0.01,
        False,
        False,
    )
    return filter_self(scan, DeviceProfile(self_mask_calibrated=True))


def estimate(**overrides: object) -> MotionEstimate:
    values: dict[str, object] = {
        "pose_delta": Pose2D(0.0, 0.0, 0.0),
        "velocity": Velocity(0.0, 0.0),
        "covariance": (0.01, 0.01, 0.01),
        "quality": 0.9,
        "dt": 0.05,
    }
    values.update(overrides)
    return MotionEstimate(**values)  # type: ignore[arg-type]


class FixtureMap:
    """Read-only corridor fixture. It has no method that writes map state."""

    def __init__(self, evidence: MapCorridorEvidence | None) -> None:
        self.evidence = evidence
        self.queries: list[tuple[MapRef, Pose2D, float, float]] = []

    def corridor(
        self, map_ref: MapRef, pose: Pose2D, distance: float, now: float
    ) -> MapCorridorEvidence | None:
        self.queries.append((map_ref, pose, distance, now))
        return self.evidence


def corridor(**overrides: object) -> MapCorridorEvidence:
    values: dict[str, object] = {
        "map_ref": MAP,
        "known_free": True,
        "occupied": False,
        "unknown": False,
        "observation_age": 0.1,
        "no_return": False,
    }
    values.update(overrides)
    return MapCorridorEvidence(**values)  # type: ignore[arg-type]


def context(**overrides: object) -> RecoveryContext:
    values: dict[str, object] = {
        "candidates": (Velocity(0.8, 0.0), Velocity(-0.8, 0.0)),
        "measured": Velocity(0.0, 0.0),
        "observations": scan_pair(),
        "motion": estimate(),
        "pose": Pose2D(1.0, 2.0, 0.0),
        "map_ref": MAP,
        "localization_quality": 0.95,
        "tf_stamp": 10.0,
        "planning_failed": False,
        "slip": False,
        "blocked": False,
        "wheel_displacement": 0.0,
    }
    values.update(overrides)
    return RecoveryContext(**values)  # type: ignore[arg-type]


def controller(maps: FixtureMap, **bound_overrides: object) -> RecoveryController:
    return RecoveryController(limits(), bounds(**bound_overrides), maps, clock_domain="sim")


def navigation_module():
    path = ROOT / "ros2" / "src" / "senior_robot" / "senior_robot" / "navigation_adapter.py"
    spec = importlib.util.spec_from_file_location("navigation_adapter_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_front_blocked_selects_safe_reverse():
    # 0.8 m/s stops in 0.48 m. The 0.60 m hit is inside the forward hard sweep
    # and outside the reverse hard sweep. These numbers are test configuration.
    maps = FixtureMap(None)
    machine = controller(maps)
    sample = context()
    decision = machine.step(sample, NOW)
    gate = evaluate_motion(
        decision.velocity, sample.measured, list(sample.observations), limits(), NOW
    )
    assert decision.velocity == Velocity(-0.8, 0.0)
    assert decision.reason == "safe_reverse"
    assert not decision.finished
    assert not decision.cancelled
    assert gate.allowed
    assert maps.queries == []


def test_occluded_rear_uses_bounded_map_assisted_reverse_when_eligible():
    speed = 0.08
    travel = stopping_distance(speed, 0.2, 1.0)
    maps = FixtureMap(corridor())
    machine = controller(maps, speed=speed, single_distance=0.3, cumulative_distance=0.5)
    sample = context(
        candidates=(Velocity(0.8, 0.0),),
        observations=scan_pair(nan_rear=True),
        wheel_displacement=10.0,
    )
    decision = machine.step(sample, NOW)
    assert decision.reason == "map_assisted_reverse"
    assert decision.velocity == Velocity(-speed, 0.0)
    assert abs(decision.velocity.vx) <= speed
    assert decision.velocity.wz == 0.0
    assert not decision.finished
    assert not decision.cancelled
    assert machine.distance_traveled == pytest.approx(body_displacement(sample.motion))
    assert machine.distance_traveled < 10.0
    assert maps.queries
    queried_map, queried_pose, queried_distance, _queried_now = maps.queries[0]
    assert queried_map == MAP
    assert queried_pose == sample.pose
    assert queried_distance == pytest.approx(travel)
    assert queried_distance <= 0.3


def test_all_blocked_finishes_bounded():
    maps = FixtureMap(corridor())
    machine = controller(maps)
    blocked = context(
        observations=scan_pair() + (point_hit(-0.36, 0.0),),
        wheel_displacement=4.0,
    )
    decision = machine.step(blocked, NOW)
    assert decision.finished
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.reason == "protection_refused"
    assert not decision.cancelled
    assert machine.attempts_used <= 1
    assert machine.distance_traveled == pytest.approx(0.0)
    assert machine.distance_traveled <= 1.0
    opened = context(observations=scan_pair(hit=5.0), wheel_displacement=4.0)
    again = machine.step(opened, NOW + 0.1)
    assert again.finished
    assert again.velocity == Velocity(0.0, 0.0)
    assert again.reason == "protection_refused"
    assert machine.attempts_used == 0


async def test_cancel_rejected_does_not_start_new_goal():
    module = navigation_module()
    started: list[object] = []

    class RejectingExecutor:
        async def start(self, goal: object) -> str:
            started.append(goal)
            return "nav-1"

        async def cancel(self, execution_id: str) -> OperationResult:
            return OperationResult(execution_id, OperationStatus.FAILED)

    adapter = module.NavigationAdapter(RejectingExecutor())
    goal = module.MapGoal(MAP, Pose2D(1.0, 2.0, 0.0))
    execution_id = await adapter.navigate(goal)
    result = await adapter.cancel(execution_id)
    replacement = module.MapGoal(MAP, Pose2D(4.0, 5.0, 0.1))
    with pytest.raises(module.NavigationRejected):
        await adapter.navigate(replacement)
    assert result.status == OperationStatus.FAILED
    assert started == [goal]


def test_expired_tf_blocks_recovery():
    machine = controller(FixtureMap(corridor()))
    decision = machine.step(context(tf_stamp=NOW - 0.5 - 0.01), NOW)
    assert decision.reason == "expired_tf"
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.finished
    assert not decision.cancelled


def test_missing_bound_refuses_the_maneuver():
    for field in (
        "speed",
        "single_distance",
        "cumulative_distance",
        "duration",
        "attempts",
        "observation_max_age",
        "min_progress",
        "progress_window",
        "min_localization_quality",
    ):
        machine = controller(FixtureMap(corridor()), **{field: None})
        decision = machine.step(context(observations=scan_pair(hit=5.0)), NOW)
        assert decision.reason == "bounds_missing"
        assert decision.velocity == Velocity(0.0, 0.0)
        assert decision.finished


def test_live_obstacle_vetoes_historical_free_space():
    maps = FixtureMap(corridor())
    machine = controller(maps, speed=0.08, single_distance=0.3)
    sample = context(
        candidates=(Velocity(0.8, 0.0),),
        observations=scan_pair(nan_rear=True) + (point_hit(-0.36, 0.0),),
    )
    decision = machine.step(sample, NOW)
    assert decision.reason == "protection_refused"
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.finished
    assert decision.reason != "map_assisted_reverse"


def test_empty_observations_do_not_map_reverse():
    maps = FixtureMap(corridor())
    machine = controller(maps, speed=0.08, single_distance=0.3)
    decision = machine.step(
        context(candidates=(Velocity(0.8, 0.0),), observations=()),
        NOW,
    )
    assert decision.reason == "protection_refused"
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.finished


def test_unset_hard_margin_does_not_map_reverse_over_live_hit():
    maps = FixtureMap(corridor())
    machine = RecoveryController(
        limits(hard_margin=None),
        bounds(speed=0.08, single_distance=0.3),
        maps,
        clock_domain="sim",
    )
    decision = machine.step(
        context(
            candidates=(Velocity(0.8, 0.0),),
            observations=scan_pair(nan_rear=True) + (point_hit(-0.36, 0.0),),
        ),
        NOW,
    )
    assert decision.reason == "protection_refused"
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.finished


def test_no_return_ray_is_not_free_space():
    maps = FixtureMap(corridor(no_return=True))
    machine = controller(maps, speed=0.15, single_distance=0.3)
    sample = context(
        candidates=(Velocity(-0.15, 0.0),),
        observations=scan_pair(hit=5.0, no_return=True),
    )
    sensor = evaluate_motion(
        Velocity(-0.15, 0.0), sample.measured, list(sample.observations), limits(), NOW
    )
    decision = machine.step(sample, NOW)
    assert sensor.allowed
    assert decision.velocity == Velocity(0.0, 0.0)
    assert decision.reason == "blind_zone_blocked"
    assert decision.finished


def test_unknown_occupied_stale_and_over_budget_space_is_not_entered():
    refused = (
        corridor(known_free=False, unknown=True),
        corridor(known_free=False, occupied=True),
        corridor(observation_age=2.0),
        corridor(map_ref=MapRef("other", 1)),
    )
    for evidence in refused:
        machine = controller(
            FixtureMap(evidence), speed=0.08, single_distance=0.3, cumulative_distance=0.5
        )
        decision = machine.step(
            context(candidates=(Velocity(0.8, 0.0),), observations=scan_pair(nan_rear=True)),
            NOW,
        )
        assert decision.velocity == Velocity(0.0, 0.0)
        assert decision.reason == "blind_zone_blocked"
        assert decision.finished
    short = controller(FixtureMap(corridor()), speed=0.08, single_distance=0.001)
    over_budget = short.step(
        context(candidates=(Velocity(0.8, 0.0),), observations=scan_pair(nan_rear=True)),
        NOW,
    )
    assert over_budget.velocity == Velocity(0.0, 0.0)
    assert over_budget.reason == "distance_exceeded"
    assert over_budget.finished


def test_slip_blockage_or_degenerate_motion_cancels_without_retry():
    cases = (
        {"slip": True},
        {"blocked": True},
        {"motion": estimate(quality=0.0, covariance=(math.inf, math.inf, math.inf))},
    )
    reasons = ("slip", "blockage", "degenerate_motion")
    for overrides, reason in zip(cases, reasons, strict=True):
        machine = controller(FixtureMap(corridor()))
        decision = machine.step(context(**overrides), NOW)
        assert decision.reason == reason
        assert decision.velocity == Velocity(0.0, 0.0)
        assert decision.cancelled
        assert decision.finished
        retry = machine.step(
            context(observations=scan_pair(hit=5.0), slip=False, blocked=False),
            NOW + 0.2,
        )
        assert retry.velocity == Velocity(0.0, 0.0)
        assert retry.cancelled
        assert retry.reason == reason
        assert machine.attempts_used == 0


def test_planning_protection_and_blind_zone_reasons_differ():
    planning = controller(FixtureMap(None)).step(
        context(candidates=(), observations=scan_pair(hit=5.0), planning_failed=True),
        NOW,
    )
    protection = controller(FixtureMap(corridor())).step(
        context(observations=scan_pair() + (point_hit(-0.36, 0.0),)),
        NOW,
    )
    blind = controller(FixtureMap(None), speed=0.08, single_distance=0.3).step(
        context(candidates=(Velocity(0.8, 0.0),), observations=scan_pair(nan_rear=True)),
        NOW,
    )
    assert planning.reason == "planning_failure"
    assert protection.reason == "protection_refused"
    assert blind.reason == "blind_zone_blocked"
    assert len({planning.reason, protection.reason, blind.reason}) == 3
    assert planning.velocity == protection.velocity == blind.velocity == Velocity(0.0, 0.0)


def test_distance_budget_uses_body_displacement():
    maps = FixtureMap(None)
    machine = controller(maps, cumulative_distance=1.0, single_distance=0.5)
    started = machine.step(context(wheel_displacement=9.0), NOW)
    assert started.velocity == Velocity(-0.8, 0.0)
    moved = estimate(pose_delta=Pose2D(-0.2, 0.0, 0.0))
    stopped = machine.step(
        context(motion=moved, wheel_displacement=9.0, tf_stamp=NOW + 0.1),
        NOW + 0.1,
    )
    assert machine.distance_traveled == pytest.approx(0.2)
    assert machine.distance_traveled != pytest.approx(9.0)
    assert stopped.velocity == Velocity(0.0, 0.0)
    assert stopped.reason == "distance_exceeded"
    assert stopped.finished
    timed = controller(FixtureMap(None), duration=0.3)
    assert timed.step(context(), NOW).velocity == Velocity(-0.8, 0.0)
    expired = timed.step(context(tf_stamp=NOW + 0.31), NOW + 0.31)
    assert expired.velocity == Velocity(0.0, 0.0)
    assert expired.reason == "duration_exceeded"
    assert expired.finished


def test_progress_loss_cancels_without_another_attempt():
    machine = controller(FixtureMap(None), attempts=3, progress_window=0.4, min_progress=0.01)
    assert machine.step(context(), NOW).reason == "safe_reverse"
    stalled = machine.step(context(tf_stamp=NOW + 0.5), NOW + 0.5)
    assert stalled.reason == "blockage"
    assert stalled.cancelled
    assert stalled.velocity == Velocity(0.0, 0.0)
    retry = machine.step(
        context(observations=scan_pair(hit=5.0), tf_stamp=NOW + 0.6),
        NOW + 0.6,
    )
    assert retry.velocity == Velocity(0.0, 0.0)
    assert retry.cancelled
    assert machine.attempts_used == 1


def test_nav2_is_differential_without_forced_rotation():
    robot = ROOT / "ros2" / "src" / "senior_robot"
    nav = robot / "config" / "nav2.yaml"
    tree = robot / "behavior_trees" / "navigate.xml"
    launch = robot / "launch" / "navigation.launch.py"
    lattice = robot / "config" / "lattice_diff.json"
    assert nav.is_file()
    assert tree.is_file()
    assert launch.is_file()
    assert lattice.is_file()
    nav_text = nav.read_text()
    tree_text = tree.read_text()
    launch_text = launch.read_text()
    assert "nav2_smac_planner/SmacPlannerLattice" in nav_text
    assert "nav2_mppi_controller::MPPIController" in nav_text
    assert 'motion_model: "DiffDrive"' in nav_text
    assert "vy_max: 0.0" in nav_text
    assert "vy_std: 0.0" in nav_text
    assert "allow_unknown: false" in nav_text
    assert "[-0.337, -0.27]" in nav_text
    assert "[0.337, 0.27]" in nav_text
    assert "lattice_diff.json" in nav_text or "lattice_diff.json" in launch_text
    for forbidden in ("Spin", "BackUp", "DriveOnHeading", "spin_dist"):
        assert forbidden not in tree_text
    assert "/safety/command_in" in launch_text
    assert "gz_sim" not in launch_text
    import json

    control_set = json.loads(lattice.read_text())
    metadata = control_set["lattice_metadata"]
    assert metadata["motion_model"] == "diff"
    primitives = control_set["primitives"]
    assert primitives
    for primitive in primitives:
        assert primitive["trajectory_length"] >= 0.05
        net_x = primitive["poses"][-1][0] - primitive["poses"][0][0]
        net_y = primitive["poses"][-1][1] - primitive["poses"][0][1]
        assert math.hypot(net_x, net_y) >= 0.05
