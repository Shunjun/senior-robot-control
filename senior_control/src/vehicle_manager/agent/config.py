"""Agent configuration with the original concrete diagnostic settings binding."""

import re
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import TypeVar
from urllib.parse import urlsplit

from vehicle_core.diagnostics import DiagnosticsSettings, diagnostics_from_environment

ConfigT = TypeVar("ConfigT", bound="AgentConfig")


@dataclass(frozen=True)
class AgentConfig:
    robot_id: str
    cloud_url: str
    device_token: str = field(repr=False)
    database_path: Path
    socket_path: Path
    workspace_map_bundle: Path | None = None
    workspace_map_sha256: str | None = None
    simulation_video_frame: Path | None = None
    ros_video_topic: str | None = None
    # //Compatible: accepted from old simulation environments; no longer gates settings.
    simulation_settings: bool = False
    diagnostics: DiagnosticsSettings = field(default_factory=DiagnosticsSettings)

    def __post_init__(self) -> None:
        url = urlsplit(self.cloud_url)
        if not self.robot_id.strip() or not self.device_token.strip():
            raise ValueError("robot id and credential are required")
        if url.scheme != "wss" or not url.hostname or url.username or url.password:
            raise ValueError("cloud URL must use authenticated TLS")
        if not self.database_path.is_absolute() or not self.socket_path.is_absolute():
            raise ValueError("database and socket paths must be absolute")
        if (self.workspace_map_bundle is None) != (self.workspace_map_sha256 is None):
            raise ValueError("workspace map bundle and SHA-256 must be configured together")
        if self.workspace_map_bundle is not None and not self.workspace_map_bundle.is_absolute():
            raise ValueError("workspace map bundle path must be absolute")
        if self.workspace_map_sha256 is not None and not re.fullmatch(
            r"[0-9a-f]{64}", self.workspace_map_sha256
        ):
            raise ValueError("workspace map SHA-256 must be 64 lowercase hexadecimal digits")
        if (
            self.simulation_video_frame is not None
            and not self.simulation_video_frame.is_absolute()
        ):
            raise ValueError("simulation video slot must have an absolute path")

    @classmethod
    def from_environment(cls: type[ConfigT], environment: Mapping[str, str]) -> ConfigT:
        try:
            bundle = environment.get("SENIOR_WORKSPACE_MAP_BUNDLE")
            video = environment.get("SENIOR_SIMULATION_VIDEO_FRAME")
            return cls(
                robot_id=environment["SENIOR_ROBOT_ID"],
                cloud_url=environment["SENIOR_CLOUD_URL"],
                device_token=environment["SENIOR_DEVICE_TOKEN"],
                database_path=Path(environment["SENIOR_DATABASE_PATH"]),
                socket_path=Path(environment["SENIOR_AGENT_SOCKET"]),
                workspace_map_bundle=Path(bundle) if bundle is not None else None,
                workspace_map_sha256=environment.get("SENIOR_WORKSPACE_MAP_SHA256"),
                simulation_video_frame=Path(video) if video is not None else None,
                ros_video_topic=environment.get("SENIOR_ROS_VIDEO_TOPIC"),
                simulation_settings=environment.get("SENIOR_SIMULATION_SETTINGS") == "1",
                diagnostics=diagnostics_from_environment(environment),
            )
        except KeyError as exc:
            raise ValueError(f"missing environment variable {exc.args[0]}") from exc
