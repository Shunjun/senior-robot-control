"""Qualified state/freshness projection over the original caller-owned IPC values."""

import time
from collections.abc import Callable
from typing import Any, Generic, TypeVar

from pydantic import ValidationError
from senior_contracts.errors import ErrorCode

from vehicle_manager.agent.request_ports import Requester

StateT = TypeVar("StateT")


class IpcState(Generic[StateT]):
    def __init__(
        self,
        client: Requester,
        *,
        parse_state: Callable[[Any], Any],
        state_factory: Callable[..., StateT],
        map_reference: Callable[[str, int], Any],
        error: type[Any],
        ipc_error: type[Exception],
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._parse_state = parse_state
        self._state_factory = state_factory
        self._map_reference = map_reference
        self._error = error
        self._ipc_error = ipc_error
        self._client = client
        self._clock = clock

    async def read(self) -> StateT:
        try:
            raw = await self._client.request("state.get")
            state = self._parse_state(raw)
            age = self._clock() - state.sampled_at
            if not 0 <= age <= 0.5:
                raise ValueError("state is stale")
            if (state.map_id is None) != (state.reference_revision is None):
                raise ValueError("incomplete map reference")
            map_ref = (
                None
                if state.map_id is None or state.reference_revision is None
                else self._map_reference(state.map_id, state.reference_revision)
            )
            geometry_live = (
                state.localized
                and map_ref is not None
                and state.map_generation is not None
                and state.map_generation == state.localization_generation
            )
            return self._state_factory(
                state.localized,
                state.safety_clear,
                state.manual_driving,
                map_ref,
                state.execution_status,
                state.map_generation,
                state.localization_generation,
                state.navigation_ready,
                state.route_progress,
                robot_pose=state.robot_pose if geometry_live else None,
                scan_points=tuple(state.scan_points) if geometry_live else (),
                navigation_block_reason=state.navigation_block_reason,
                runtime_observation=state.runtime_observation,
                motion_runtime=state.motion_runtime,
                motion_runtime_version=state.motion_runtime_version,
                manual_protocol_version=state.manual_protocol_version,
                manual_authority=state.manual_authority,
                task_lifecycle_version=state.task_lifecycle_version,
                task_evidence_ready=state.task_evidence_ready,
                valid_until=state.sampled_at + 0.5,
            )
        except (self._ipc_error, OSError, ValueError, ValidationError) as exc:
            raise self._error(ErrorCode.UNAVAILABLE, "ROS state unavailable") from exc
