"""Require the reviewed same-version Nav2 producer overlay before starting it."""

import json
from pathlib import Path
from xml.etree import ElementTree

NAV2_COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
NAV2_VERSION = "1.1.20"
CONTRACT = "control-monotonic-v2"
# Controller/plugin interfaces must come from the same reviewed release.
ABI_PACKAGES = (
    "nav2_controller",
    "nav2_core",
    "nav2_util",
    "nav2_costmap_2d",
    "nav2_msgs",
    "nav_2d_utils",
    "nav_2d_msgs",
    "nav2_mppi_controller",
    "nav2_bt_navigator",
    "nav2_behavior_tree",
)


def require_nav2_abi() -> None:
    from ament_index_python.packages import get_package_share_directory

    for name in ABI_PACKAGES:
        share = Path(get_package_share_directory(name))
        version = ElementTree.parse(share / "package.xml").findtext("version")
        if version != NAV2_VERSION:
            raise RuntimeError(f"Nav2 ABI mismatch: {name}={version}, required {NAV2_VERSION}")


def require_navigation_overlay() -> None:
    require_nav2_abi()
    for name in ("nav2_controller", "nav2_mppi_controller"):
        require_overlay_package(name)


def require_overlay_package(name: str) -> None:
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory(name))
    try:
        marker = json.loads((share / "navigation-contract.json").read_text())
    except (OSError, ValueError) as exc:
        raise RuntimeError(f"{name} monotonic producer overlay is not installed") from exc
    if marker != {"commit": NAV2_COMMIT, "version": NAV2_VERSION, "contract": CONTRACT}:
        raise RuntimeError(f"{name} monotonic producer overlay version mismatch")
