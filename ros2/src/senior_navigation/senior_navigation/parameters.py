"""Startup-owned, shared planner/controller parameter overlay and run binding."""

import json
import os
import secrets
from pathlib import Path

import yaml


def prepare_navigation_parameters(
    base: Path,
    *,
    output: Path,
    mode: str,
    contract_directory: Path,
    run_nonce: str,
    host_boot_id: str,
    global_instance: str,
    local_instance: str,
) -> Path:
    if mode not in ("static", "live") or not contract_directory.is_absolute():
        raise ValueError("navigation mode/private contract directory required")
    data = yaml.safe_load(base.read_text())
    contract = dict(
        mode=mode,
        contract_directory=str(contract_directory),
        run_nonce=run_nonce,
        host_boot_id=host_boot_id,
        global_instance_permit=global_instance,
        local_instance_permit=local_instance,
    )
    data.setdefault("/**", {}).setdefault("ros__parameters", {})["navigation_contract"] = contract
    for role in ("global", "local"):
        params = data[role + "_costmap"][role + "_costmap"]["ros__parameters"]
        if mode == "live":
            params["plugins"] = ["static_layer", "current_scan_layer", "inflation_layer"]
            params.pop("obstacle_layer", None)
            # //Compatible: Humble StaticLayer reads use_maximum on the costmap node.
            params["use_maximum"] = False
            static = params.setdefault("static_layer", {})
            static.setdefault("plugin", "nav2_costmap_2d::StaticLayer")
            static.setdefault("map_subscribe_transient_local", True)
            static.pop("use_maximum", None)
            params["current_scan_layer"] = dict(
                plugin="senior_navigation::CurrentScanLayer",
                enabled=True,
                mode=mode,
                contract_directory=str(contract_directory),
                run_nonce=run_nonce,
                host_boot_id=host_boot_id,
                role=role,
                role_instance_permit=global_instance if role == "global" else local_instance,
            )
            params["update_frequency"] = max(5.0, params.get("update_frequency", 0.0))
        else:
            if "current_scan_layer" not in params.get("plugins", []):
                continue
            # Restore the baseline static inputs when the installed default is live.
            # An operator's existing static layer stack is otherwise preserved above.
            params["plugins"] = (["static_layer"] if role == "global" else []) + [
                "obstacle_layer",
                "inflation_layer",
            ]
            params.pop("current_scan_layer", None)
            if role == "local":
                params.pop("static_layer", None)
            params.setdefault(
                "obstacle_layer",
                dict(
                    plugin="nav2_costmap_2d::ObstacleLayer",
                    enabled=True,
                    observation_sources="scan",
                    scan=dict(
                        topic="/scan",
                        max_obstacle_height=2.0,
                        clearing=True,
                        marking=True,
                        data_type="LaserScan",
                        raytrace_max_range=3.0,
                        raytrace_min_range=0.0,
                        obstacle_max_range=2.5,
                        obstacle_min_range=0.0,
                        inf_is_valid=False,
                    ),
                ),
            )
    temporary = output.with_suffix(".tmp")
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
    os.replace(temporary, output)
    return output


def read_navigation_contract(path: Path) -> dict | None:
    data = yaml.safe_load(path.read_text()) or {}
    return data.get("/**", {}).get("ros__parameters", {}).get("navigation_contract")


def rotate_role_permit(path: Path, role: str) -> str:
    """Called by the actual spawning owner, never by the costmap plugin."""
    contract = read_navigation_contract(path)
    if contract is None:
        return ""
    permit = secrets.token_hex(16)
    # Receipt deletion is performed before the process is spawned with the new permit.
    receipt = Path(contract["contract_directory"]) / (role + "-coverage.json")
    receipt.unlink(missing_ok=True)
    data = yaml.safe_load(path.read_text())
    data["/**"]["ros__parameters"]["navigation_contract"][role + "_instance_permit"] = permit
    params = data[role + "_costmap"][role + "_costmap"]["ros__parameters"]
    if "current_scan_layer" in params:
        params["current_scan_layer"]["role_instance_permit"] = permit
    temporary = path.with_suffix(".tmp")
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "w") as stream:
        yaml.safe_dump(data, stream, sort_keys=False)
    os.replace(temporary, path)
    return permit


async def request_live_handoff(contract: dict):
    """Only the original simulator launch owns this private lifecycle endpoint."""
    import asyncio

    endpoint = contract.get("handoff_socket")
    if not endpoint:
        raise RuntimeError(
            "live navigation handoff incomplete: "
            "external static localization owner has no control handle"
        )
    reader, writer = await asyncio.open_unix_connection(endpoint, limit=8192)
    try:
        writer.write(
            (
                json.dumps(dict(command="prepare_live", run_nonce=contract["run_nonce"])) + "\n"
            ).encode()
        )
        await writer.drain()
        row = json.loads(await asyncio.wait_for(reader.readline(), 25))
        if row.get("ok") is not True:
            raise RuntimeError(row.get("error", "live navigation handoff incomplete"))
        contract.update(row["contract"])
    finally:
        writer.close()
        await writer.wait_closed()
