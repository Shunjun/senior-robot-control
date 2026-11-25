"""Environment profile uses the common numeric schema, without control special cases."""

import json
from pathlib import Path

from senior_motion_control.hardware_profile import HardwareMotionProfile


def test_hardware_uses_requested_linear_rates():
    values = json.loads(Path("config/hardware-motion.json").read_text())
    profile = HardwareMotionProfile.model_validate(values)
    assert profile.accel == 0.5
    assert profile.decel == 0.8
    assert profile.reaction_linear_accel == 0.5


def test_standard_simulation_uses_shared_core_profile():
    path = Path("ros2/src/senior_sim/config/motion_profile.json")
    values = json.loads(path.read_text())
    profile = HardwareMotionProfile.model_validate(values)
    assert profile.wheel_topic == "/wheel/odom"
    assert profile.imu_topic == "/imu/data"
    assert profile.cap == 25 / 18
    assert profile.accel == 0.5
    assert profile.decel == 0.8
    assert profile.reaction_linear_accel == 0.5
    assert profile.yaw_accel == 0.5
    assert profile.lag_ms == 200
    assert profile.max_age_ms == 1000


def test_one_navigation_overlay_keeps_static_and_binds_both_live_roles(tmp_path):
    import yaml
    from senior_navigation.parameters import prepare_navigation_parameters, read_navigation_contract

    base = Path("ros2/src/senior_navigation/config/nav2.yaml")
    kwargs = dict(
        output=tmp_path / "nav.yaml",
        contract_directory=tmp_path,
        run_nonce="run",
        host_boot_id="host",
        global_instance="global",
        local_instance="local",
    )
    static = prepare_navigation_parameters(base, mode="static", **kwargs)
    data = yaml.safe_load(static.read_text())
    assert data["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"] == [
        "obstacle_layer",
        "inflation_layer",
    ]
    assert read_navigation_contract(static)["mode"] == "static"
    live = prepare_navigation_parameters(base, mode="live", **kwargs)
    data = yaml.safe_load(live.read_text())
    for role in ("global", "local"):
        params = data[role + "_costmap"][role + "_costmap"]["ros__parameters"]
        assert params["plugins"] == ["static_layer", "current_scan_layer", "inflation_layer"]
        assert params["current_scan_layer"]["role"] == role
        assert params["current_scan_layer"]["contract_directory"] == str(tmp_path)
        assert params["current_scan_layer"]["role_instance_permit"] == role
        assert params["update_frequency"] >= 5
        assert "obstacle_layer" not in params
    assert read_navigation_contract(live)["run_nonce"] == "run"
    from senior_navigation.parameters import rotate_role_permit

    old_receipt = tmp_path / "local-coverage.json"
    old_receipt.write_text("old instance")
    permit = rotate_role_permit(live, "local")
    assert permit != "local" and not old_receipt.exists()
    rotated = yaml.safe_load(live.read_text())
    assert read_navigation_contract(live)["local_instance_permit"] == permit
    assert (
        rotated["local_costmap"]["local_costmap"]["ros__parameters"]["current_scan_layer"][
            "role_instance_permit"
        ]
        == permit
    )
    assert read_navigation_contract(live)["global_instance_permit"] == "global"


async def test_hardware_supervision_holds_one_overlay_until_children_settle(tmp_path, monkeypatch):
    from senior_robot import service_runtime
    from vehicle_manager.profile import VehicleProfile

    profile = VehicleProfile(
        kind="real",
        credentials=tmp_path / "device.json",
        navigation_parameters=Path("ros2/src/senior_navigation/config/nav2.yaml"),
    )
    observed = []
    monkeypatch.setattr(
        service_runtime._platform, "_adapter_command", lambda p, _: [str(p.navigation_parameters)]
    )
    monkeypatch.setattr(
        service_runtime._platform, "_motion_command", lambda p, _: [str(p.navigation_parameters)]
    )

    async def supervise(root, command, environment, *, profile, ports):
        adapter, planner = ports.adapter_command(profile, "r"), ports.motion_command(profile, "r")
        assert adapter == planner
        overlay = Path(adapter[0])
        assert overlay.is_file() and overlay.parent.stat().st_mode & 0o777 == 0o700
        observed.append(overlay)
        return 7

    monkeypatch.setattr(service_runtime._canonical, "supervise", supervise)
    assert await service_runtime.supervise(tmp_path, [], {}, profile=profile) == 7
    assert not observed[0].exists()
    assert profile.navigation_parameters == Path("ros2/src/senior_navigation/config/nav2.yaml")
