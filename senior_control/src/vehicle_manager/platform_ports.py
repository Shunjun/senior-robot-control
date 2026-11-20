"""Named per-invocation effects of the existing installed runtime.

These records contain actual owners and functions, never a module container or
an alternate IPC implementation. ROS, learning, media and diagnostics stay at
those owners; the manager only orchestrates their lifecycle.
"""

from collections.abc import Awaitable, Callable, Coroutine, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from vehicle_manager.agent.pairing import Credentials
from vehicle_manager.profile import VehicleProfile


class Client(Protocol):
    async def request(self, kind: str, **fields: Any) -> dict[str, Any]: ...
    async def close(self) -> None: ...


class Server(Protocol):
    async def start(self) -> None: ...
    async def close(self) -> None: ...


class Learning(Protocol):
    async def start(self) -> None: ...
    async def close(self) -> None: ...


class Events(Protocol):
    reader: int
    writer: int

    def child_environment(self, environment: Mapping[str, str]) -> dict[str, str]: ...
    def agent_environment(self, environment: Mapping[str, str]) -> dict[str, str]: ...
    def report(self, module: str, phase: str, detail: str = "", **facts: Any) -> None: ...
    def close_reader(self) -> None: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class OperatorPorts:
    private_directory: Callable[[Path], None]
    vehicle_doctor: Callable[[VehicleProfile], int]
    vehicle_deploy: Callable[..., int]
    simulation_validate: Callable[[VehicleProfile], None]
    lifecycle: Callable[..., int]
    service_status: Callable[..., int]
    doctor: Callable[[VehicleProfile], int]
    deploy: Callable[..., int]
    pair: Callable[..., None]
    pair_device: Callable[..., Coroutine[Any, Any, str]]
    getpass: Callable[[str], str]
    install_once: Callable[[Path, str, int], None]
    service_root: Path
    env_root: Path
    unit_root: Path


@dataclass(frozen=True)
class DeploymentPorts:
    unit_root: Path
    install_once: Callable[[Path, str, int], None]
    max_path: int
    validate_runtime: Callable[..., object]
    validate_navigation: Callable[..., None]


@dataclass(frozen=True)
class ServicePorts:
    client: Callable[..., Client]
    server: Callable[..., Server]
    error: type[Exception]
    read_credentials: Callable[[Path], Credentials]
    identity: Callable[[Path], list[int]]
    record: Callable[..., None]
    prepare_socket: Callable[..., Awaitable[None]]
    verify_adapter_peer: Callable[..., Awaitable[list[int]]]
    runtime_profile: Callable[[Path], VehicleProfile]
    adapter_command: Callable[..., list[str]]
    motion_command: Callable[..., list[str]]
    camera_command: Callable[..., list[str]]
    video_environment: Callable[..., Awaitable[dict[str, str]]]
    wait_for_adapter: Callable[..., Awaitable[list[int] | None]]
    terminate: Callable[..., Awaitable[None]]
    terminate_motion_stack: Callable[..., Awaitable[None]]
    start_environment: Callable[..., dict[str, str]]
    learning: Callable[[Path], Learning]
    cleanup_learning: Callable[..., None]
    events: Callable[[], Events]
    supervise: Callable[..., Coroutine[Any, Any, int]]
    stop: Callable[[Path], Coroutine[Any, Any, int]]


@dataclass(frozen=True)
class SimulationPorts:
    client: Callable[..., Client]
    error: type[Exception]
    max_path: int
    read_credentials: Callable[[Path], Credentials]
    start_environment: Callable[..., dict[str, str]]
    learning: Callable[[Path], Learning]
    cleanup_learning: Callable[..., None]
    events: Callable[[], Events]
    terminate_motion_stack: Callable[..., Awaitable[None]]
    wait_for_adapter: Callable[..., Awaitable[None]]
    stop: Callable[..., Awaitable[None]]
    notify: Callable[[str], None]
    validate_profile: Callable[[VehicleProfile], None]
    remove_stale_socket: Callable[[Path], None]
    supervise: Callable[..., Coroutine[Any, Any, int]]
