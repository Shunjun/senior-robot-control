"""Select an opt-out transport profile for the RGB-D simulation launch."""

from collections.abc import Callable, Mapping
from pathlib import Path
from xml.etree import ElementTree


def transport_environment(
    environment: Mapping[str, str],
    enabled: bool,
    profile: str | Path,
    cwd: str | Path,
    detect_rmw: Callable[[], str],
) -> dict[str, str]:
    """Return child-only environment additions without replacing user settings."""
    if not enabled:
        return {}
    rmw = environment.get("RMW_IMPLEMENTATION", "")
    fast_rmws = {"rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp"}
    if rmw and rmw not in fast_rmws:
        return {}
    # //Compatible: Preserve user settings for Fast DDS 2.x and newer variable names.
    if any(
        value
        and (
            key.startswith(("FASTRTPS_", "FASTDDS_", "RMW_FASTRTPS_")) or key == "SKIP_DEFAULT_XML"
        )
        for key, value in environment.items()
    ):
        return {}
    if any(
        (Path(cwd) / name).is_file()
        for name in ("DEFAULT_FASTRTPS_PROFILES.xml", "DEFAULT_FASTDDS_PROFILES.xml")
    ):
        return {}
    if not rmw:
        try:
            rmw = detect_rmw()
        except Exception as exc:
            raise RuntimeError(
                "Unable to determine RMW implementation for RGB-D transport"
            ) from exc
    if rmw not in fast_rmws:
        return {}
    try:
        ElementTree.parse(profile)
    except (OSError, ElementTree.ParseError) as exc:
        raise ValueError(f"Invalid RGB-D transport profile {profile!r}: {exc}") from exc
    return {"FASTRTPS_DEFAULT_PROFILES_FILE": str(profile)}
