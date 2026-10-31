"""Bounded IPC adapters for ROS runtime consumers."""

import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Protocol

from pydantic import BaseModel, ConfigDict, ValidationError

from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef
from senior_control.runtime.ipc import IpcError
from senior_control.tasks.models import TaskDefinition
from senior_control.waypoints.store import WaypointStore


class Requester(Protocol):
    async def request(self, kind: str, **fields: Any) -> dict[str, Any]: ...


class IpcMapPort:
    def __init__(self, client: Requester) -> None:
        self._client = client

    async def _call(self, kind: str, **fields: Any) -> None:
        try:
            reply = await self._client.request(kind, **fields)
        except (IpcError, OSError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS map service unavailable") from exc
        if reply.get("ok") is not True:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS map service unavailable")

    async def export(self, operation_id: str, map_ref: MapRef, destination: Path) -> None:
        await self._call(
            "map.export",
            operation_id=operation_id,
            map_id=map_ref.map_id,
            reference_revision=map_ref.reference_revision,
            path=str(destination),
        )

    async def import_(self, operation_id: str, path: Path) -> None:
        await self._call("map.import", operation_id=operation_id, path=str(path))

    async def clear(self, operation_id: str, expected: MapRef) -> None:
        await self._call(
            "map.clear",
            operation_id=operation_id,
            map_id=expected.map_id,
            reference_revision=expected.reference_revision,
        )


class IpcNavigation:
    """Only a matching explicit acknowledgement counts as an accepted action."""

    def __init__(self, client: Requester) -> None:
        self._client = client

    async def _call(self, kind: str, execution_id: str, **fields: Any) -> None:
        try:
            reply = await self._client.request(kind, execution_id=execution_id, **fields)
        except (IpcError, OSError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable") from exc
        if reply.get("ok") is not True or reply.get("execution_id") != execution_id:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable")

    async def start(self, execution_id: str, task: TaskDefinition) -> None:
        # //Compatible: legacy task IPC cannot fence an accepted reference generation.
        del execution_id, task
        raise DomainError(ErrorCode.UNAVAILABLE, "qualified ROS navigation unavailable")

    async def cancel(self, execution_id: str) -> None:
        await self._call("navigation.task_cancel", execution_id)


@dataclass(frozen=True)
class RuntimeState:
    localized: bool
    safety_clear: bool
    manual_driving: bool
    map_ref: MapRef | None
    execution_status: str


class StateReply(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    v: Literal[1]
    ok: Literal[True]
    sampled_at: float
    localized: bool
    safety_clear: bool
    manual_driving: bool
    map_id: str | None
    reference_revision: int | None
    execution_status: str


class IpcState:
    def __init__(self, client: Requester, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._client = client
        self._clock = clock

    async def read(self) -> RuntimeState:
        try:
            raw = await self._client.request("state.get")
            state = StateReply.model_validate(raw)
            age = self._clock() - state.sampled_at
            if not 0 <= age <= 0.5:
                raise ValueError("state is stale")
            if (state.map_id is None) != (state.reference_revision is None):
                raise ValueError("incomplete map reference")
            map_ref = (
                None
                if state.map_id is None or state.reference_revision is None
                else MapRef(state.map_id, state.reference_revision)
            )
            return RuntimeState(
                state.localized,
                state.safety_clear,
                state.manual_driving,
                map_ref,
                state.execution_status,
            )
        except (IpcError, OSError, ValueError, ValidationError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS state unavailable") from exc


class PoseReply(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    v: Literal[1]
    ok: Literal[True]
    sampled_at: float
    healthy: Literal[True]
    map_id: str
    reference_revision: int
    anchor_id: str
    x: float
    y: float
    yaw: float
    anchor_x: float
    anchor_y: float
    anchor_yaw: float


class IpcHomePort:
    def __init__(self, client: Requester, store: WaypointStore, *, robot_id: str) -> None:
        self._client = client
        self._store = store
        self._robot_id = robot_id

    async def home_revision(self) -> int:
        return await self._store.home_revision()

    async def set_current(self, expected_revision: int) -> int:
        # //Compatible: legacy pose IPC lacks a reference nonce, so it cannot qualify Home.
        del expected_revision
        raise DomainError(ErrorCode.UNAVAILABLE, "qualified ROS localization unavailable")
