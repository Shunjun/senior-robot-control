"""Angular startup-only occlusion policy and configuration wiring."""

import math
from dataclasses import replace

import pytest
from shapely.geometry import Point, box
from test_startup_learning_feasibility import BOUNDARY

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D
from senior_control.safety.self_filter import ScanObservation, filter_self
from senior_control.safety.startup_visibility import startup_visibility


def observation(front=0, rear=0, *, yaw=0.0, angle_min=-math.pi, count=360):
    step = math.tau / count
    # Half-cell centers put sector boundaries exactly on cell boundaries.
    scan = ScanObservation(
        tuple([5.0] * count),
        angle_min + step / 2,
        step,
        0.05,
        10.0,
        10.0,
        "ros",
        Pose2D(0, 0, yaw),
        10.0,
        "base",
        10.0,
    )
    raw = filter_self(scan, DeviceProfile(footprint=BOUNDARY))
    rays = list(raw.rays)
    for center, percent in ((0.0, front), (math.pi, rear)):
        selected = sorted(
            (
                r
                for r in rays
                if abs(math.remainder(yaw + scan.angle_min + r.index * step - center, math.tau))
                < math.pi / 6
            ),
            key=lambda r: math.remainder(yaw + scan.angle_min + r.index * step - center, math.tau),
        )
        n = round(len(selected) * percent / 100)
        for r in selected[:n]:
            rays[r.index] = replace(r, kind="self")
    return replace(raw, rays=tuple(rays))


@pytest.mark.parametrize("direction", ["front", "rear"])
@pytest.mark.parametrize("percent,blocked", [(69, False), (70, True)])
def test_60_degree_sector_threshold_uses_angular_coverage(direction, percent, blocked):
    obs = observation(count=1200, **{direction: percent})
    value = startup_visibility(obs, front_stop=True, rear_stop=True)
    ratio = value.front_occluded if direction == "front" else value.rear_occluded
    assert ratio == pytest.approx(percent / 100)
    assert (value.blocked_reason is not None) == blocked


@pytest.mark.parametrize(
    "front_stop,rear_stop", [(True, True), (False, True), (True, False), (False, False)]
)
def test_front_rear_boolean_switches_are_independent(front_stop, rear_stop):
    for front, rear in [(100, 0), (0, 100), (100, 100)]:
        policy = startup_visibility(
            observation(front, rear), front_stop=front_stop, rear_stop=rear_stop
        )
        assert (policy.blocked_reason is not None) == (
            (front_stop and front == 100) or (rear_stop and rear == 100)
        )


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi, -math.pi / 2])
@pytest.mark.parametrize("angle_min", [-math.pi, 0.0])
def test_installation_yaw_and_array_origin_do_not_change_body_sectors(yaw, angle_min):
    policy = startup_visibility(
        observation(front=0, rear=50, yaw=yaw, angle_min=angle_min), front_stop=True, rear_stop=True
    )
    assert policy.front_occluded == pytest.approx(0)
    assert policy.rear_occluded == pytest.approx(0.5)


def test_sector_edges_are_included_by_actual_cell_overlap_and_no_side_exemption():
    obs = observation()
    rays = list(obs.rays)
    # Make a cell whose CENTER lies outside 30deg but angular support overlaps it.
    step = obs.scan.angle_increment
    i = round((math.pi / 6 + step / 2 - obs.scan.angle_min) / step)
    rays[i] = replace(rays[i], kind="self")
    shifted = replace(
        obs, scan=replace(obs.scan, angle_min=obs.scan.angle_min - step / 4), rays=tuple(rays)
    )
    value = startup_visibility(shifted, front_stop=True, rear_stop=True)
    assert value.front_occluded == pytest.approx(0.25 / 60)
    assert value.rear_occluded == pytest.approx(0)
    side = box(-0.01, 1.0, 0.01, 2.0)
    assert value.remaining_unknown(side, Pose2D(0, 0, 0)).equals(side)


def test_policy_never_changes_measured_free_space_or_obstacles():
    obs = observation(rear=20)
    original = obs.rays
    policy = startup_visibility(obs, front_stop=True, rear_stop=True)
    rear = box(-2.0, -0.05, -0.5, 0.05)
    assert policy.remaining_unknown(rear, Pose2D(0, 0, 0)).is_empty
    assert obs.rays == original
    assert rear.covers(Point(-1.0, 0.0))


def test_same_scan_without_occlusion_does_not_exempt_other_uncertainty():
    policy = startup_visibility(observation(), front_stop=True, rear_stop=True)
    assert policy.front_occluded == policy.rear_occluded == 0
    rear = box(-2.0, -0.05, -0.5, 0.05)
    assert policy.remaining_unknown(rear, Pose2D(0, 0, 0)).equals(rear)


def test_policy_does_not_change_measured_free_space_or_hide_obstacles():
    from dataclasses import replace as dc_replace

    from shapely.geometry import box
    from test_directional_constraints import engine, snapshot

    from senior_control.models import Velocity

    ctx = snapshot(target=Velocity(0.05, 0), current=0)
    ctx = dc_replace(
        ctx,
        envelope=dc_replace(ctx.envelope, linear_interval=(-0.1, 0.1), yaw_interval=(-0.1, 0.1)),
    )
    policy = startup_visibility(observation(rear=45), front_stop=True, rear_stop=True)
    # The rear unknown coverage is accepted only in the policy-aware snapshot.
    free = ctx.perception.free_space.difference(box(-20, -0.03, -0.3, 0.03))
    ctx = dc_replace(ctx, perception=dc_replace(ctx.perception, free_space=free))
    guard = engine()
    cert = guard._sweep(ctx, Velocity(0, 0), stop_only=True, coarse=True)
    accepted = dc_replace(
        ctx, target=dc_replace(ctx.target, source="startup_learning"), startup_visibility=policy
    )
    assert guard._clear(cert, ctx, 0) == "sweep_unknown"
    assert guard._clear(cert, accepted, 0) is None
    assert accepted.perception.free_space.equals(free)
    manual = dc_replace(accepted, target=dc_replace(accepted.target, source="manual"))
    assert guard._clear(cert, manual, 0) == "sweep_unknown"
    obstacle = box(-0.4, -0.01, -0.3, 0.01)
    obstructed = dc_replace(accepted, perception=dc_replace(accepted.perception, occupied=obstacle))
    assert guard._clear(cert, obstructed, 0) == "sweep_obstructed"
    side_gap = free.difference(box(-0.03, 0.2, 0.03, 20))
    lateral = dc_replace(accepted, perception=dc_replace(accepted.perception, free_space=side_gap))
    assert guard._clear(cert, lateral, 0) == "sweep_unknown"


@pytest.mark.parametrize(
    "front_stop,rear_stop", [(True, True), (False, True), (True, False), (False, False)]
)
def test_file_and_real_node_override_loading_preserve_boolean_settings(
    tmp_path, front_stop, rear_stop
):
    import ast
    import json
    from pathlib import Path
    from types import SimpleNamespace as N

    from control_process_test_support import generic_motion_profile
    from senior_robot.motion_profile import select_motion_profile

    from senior_control.motion.hardware_profile import HardwareMotionProfile

    raw = generic_motion_profile().model_dump(mode="json")
    raw.update(
        startup_learning_front_occlusion_stop=front_stop,
        startup_learning_rear_occlusion_stop=rear_stop,
    )
    path = tmp_path / "hardware.json"
    path.write_text(json.dumps(raw))
    profiles = [
        select_motion_profile(hardware_path=str(path)),
        generic_motion_profile(),
    ]
    assert profiles[0].startup_learning_front_occlusion_stop is front_stop
    assert profiles[0].startup_learning_rear_occlusion_stop is rear_stop
    assert profiles[1].startup_learning_front_occlusion_stop is True
    assert profiles[1].startup_learning_rear_occlusion_stop is True
    module = ast.parse(Path("ros2/src/senior_robot/senior_robot/safety_node.py").read_text())
    initialize = next(
        n for n in ast.walk(module) if isinstance(n, ast.FunctionDef) and n.name == "_initialize"
    )
    start = next(
        i
        for i, n in enumerate(initialize.body)
        if isinstance(n, ast.ImportFrom)
        and any(a.name == "configure_startup_learning" for a in n.names)
    )
    block = ast.Module(body=initialize.body[start : start + 4], type_ignores=[])

    class Node:
        def __init__(self, override):
            self.values = {}
            self.override = override

        def declare_parameter(self, name, default):
            self.values[name] = default

        def get_parameter(self, name):
            value = (
                (front_stop if "front" in name else rear_stop)
                if self.override
                else self.values[name]
            )
            return N(value=value)

    for p in profiles:
        for override in (False, True):
            scope = {"candidate_profile": p, "self": Node(override)}
            exec(compile(block, "safety_node.py", "exec"), scope)
            loaded = scope["candidate_profile"]
            expected_front = front_stop if override else p.startup_learning_front_occlusion_stop
            expected_rear = rear_stop if override else p.startup_learning_rear_occlusion_stop
            assert loaded.startup_learning_front_occlusion_stop is expected_front
            assert loaded.startup_learning_rear_occlusion_stop is expected_rear
            result = startup_visibility(
                observation(front=100, rear=100),
                front_stop=loaded.startup_learning_front_occlusion_stop,
                rear_stop=loaded.startup_learning_rear_occlusion_stop,
            )
            assert (result.blocked_reason is not None) == (expected_front or expected_rear)
    schema = json.loads(Path("config/hardware-motion.schema.json").read_text())
    assert schema == HardwareMotionProfile.model_json_schema()
    for name in ("startup_learning_front_occlusion_stop", "startup_learning_rear_occlusion_stop"):
        with pytest.raises(ValueError):
            HardwareMotionProfile.model_validate({**raw, name: "false"})


@pytest.mark.parametrize(
    "center,expected", [(29.5, 1 / 60), (30.0, 0.5 / 60), (30.5, 0.0), (31.0, 0.0)]
)
@pytest.mark.parametrize("yaw", [0.0, math.pi / 2])
def test_sector_edges_clip_ray_cells_by_actual_angle(center, expected, yaw):
    # Full 1-degree scan with one unavailable cell near the +30-degree edge.
    base = observation(yaw=yaw)
    scan = replace(base.scan, angle_min=math.radians(center) - yaw, angle_increment=math.pi / 180)
    rays = tuple(replace(r, kind="self" if r.index == 0 else "hit") for r in base.rays)
    result = startup_visibility(
        replace(base, scan=scan, rays=rays), front_stop=True, rear_stop=True
    )
    assert result.front_occluded == pytest.approx(expected, abs=1e-14)


def test_validated_no_return_keeps_existing_available_coverage_contract():
    base = observation()
    ray = next(
        r
        for r in base.rays
        if abs(math.remainder(base.scan.angle_min + r.index * base.scan.angle_increment, math.tau))
        < 0.01
    )
    rays = tuple(replace(r, kind="invalid") if r.index == ray.index else r for r in base.rays)
    ranges = list(base.scan.ranges)
    ranges[ray.index] = math.inf
    obs = replace(base, rays=rays, scan=replace(base.scan, ranges=tuple(ranges)))
    unknown = startup_visibility(obs, front_stop=True, rear_stop=True)
    validated = startup_visibility(obs, front_stop=True, rear_stop=True, no_return_clear=True)
    assert unknown.front_occluded == pytest.approx(1 / 60)
    assert validated.front_occluded == 0
