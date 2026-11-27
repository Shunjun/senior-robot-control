"""Single owner of existing agent display sequence and safety frontiers."""

import time
from collections.abc import Callable
from typing import Any

from senior_contracts.errors import ErrorCode

from vehicle_manager.agent.snapshots import LiveState, TaskSummary
from vehicle_manager.agent.state_ports import (
    RuntimeReading,
    SnapshotNavigation,
    SnapshotPorts,
    StatePort,
)
from vehicle_manager.agent.workspace import WorkspaceReader
from vehicle_manager.tasks.store import TaskStore
from vehicle_manager.waypoints.store import WaypointStore


class SnapshotState:
    def __init__(
        self,
        state: StatePort,
        tasks: TaskStore,
        waypoints: WaypointStore[Any, Any, Any],
        workspace: WorkspaceReader | None = None,
        *,
        ports: SnapshotPorts,
        navigation: SnapshotNavigation | None = None,
        connected: Callable[[], bool] = lambda: True,
        epoch: Callable[[], int] = lambda: 0,
        manual_enabled: bool = False,
        clock: Callable[[], float] = time.monotonic,
        settings_enabled: bool = False,
        logs_enabled: Callable[[], bool] = lambda: False,
    ) -> None:
        self._ports = ports
        self._state = state
        self._tasks = tasks
        self._waypoints = waypoints
        self._workspace = workspace
        self._navigation = navigation
        self._connected, self._epoch = connected, epoch
        self._manual_enabled = manual_enabled
        self._settings_enabled, self._logs_enabled = settings_enabled, logs_enabled
        self._clock = clock
        self._sequence = 0
        self._last_sampled_steady: float | None = None
        self._safety_frontiers: dict[str, tuple[str, int, int]] = {}
        self._recovery_tokens: dict[str, tuple[int, int]] = {}

    def _safety_token(self, kind: str, boot: str, revision: int) -> int:
        previous = self._safety_frontiers.get(kind)
        if previous is None:
            counter = revision
        elif previous[0] != boot:
            counter = previous[2] + 1
        else:
            counter = previous[2] + max(0, revision - previous[1])
        self._safety_frontiers[kind] = boot, revision, counter
        token = self._epoch() * 1_000_000_000 + counter
        self._recovery_tokens[kind] = token, revision
        return token

    async def recovery_revision(self, kind: str, token: int) -> int:
        # Read the current T5 boot/version before translating an opaque UI token;
        # a delayed command cannot reset another boot's equal numeric revision.
        await self.collect()
        current = self._recovery_tokens.get(kind)
        if current is None or current[0] != token:
            raise self._ports.error(ErrorCode.CONFLICT, "motion recovery version changed")
        return current[1]

    def _runtime_revisions(self, runtime: RuntimeReading | None) -> tuple[int, int]:
        if runtime is not None:
            return (
                self._safety_token("estop_reset", runtime.boot_id, runtime.estop_revision),
                self._safety_token("fault_recover", runtime.boot_id, runtime.fault_revision),
            )
        fallback = (self._epoch() * 1_000_000_000, 0)
        return (
            self._recovery_tokens.get("estop_reset", fallback)[0],
            self._recovery_tokens.get("fault_recover", fallback)[0],
        )

    def _unavailable_frame(self) -> dict[str, object]:
        self._sequence += 1
        ui = self._ports.frame(
            None,
            sequence=self._epoch() * 1_000_000_000 + self._sequence,
            connected=self._connected(),
            task=None,
            manual=None,
            manual_enabled=False,
            tasks_enabled=False,
            runtime_enabled=False,
            safety_revisions=self._runtime_revisions(None),
            now=self._clock(),
            observation=self._ports.observation(
                status="ipc_unavailable", last_sampled_steady=self._last_sampled_steady
            ),
        )
        capabilities = ui["capabilities"]
        assert isinstance(capabilities, list)
        capabilities.append("task.names.v1")
        if self._settings_enabled:
            capabilities.append("settings.v1")
        if self._logs_enabled():
            capabilities.append("logs.v1")
        return ui

    async def collect(self) -> LiveState:
        tasks, deleted_task_ids = await self._tasks.snapshot_tasks()
        home = await self._waypoints.home()
        summaries = tuple(
            TaskSummary(task.task_id, task.revision, task.enabled, task.name) for task in tasks
        )
        task_revision = sum(task.revision for task in tasks)
        try:
            state = await self._state.read()
        except self._ports.error:
            return LiveState(
                False,
                await self._workspace.active_map() if self._workspace else None,
                home.waypoint if home else None,
                task_revision,
                "unavailable",
                tasks=summaries,
                deleted_task_ids=deleted_task_ids,
                navigation_ready=False,
                vehicle_ui=self._unavailable_frame(),
            )
        # //Compatible: legacy IPC has no reference nonce, so runnable status is unproven.
        runnable: tuple[str, ...] = ()
        localized = (
            state.localized
            and state.map_generation is not None
            and state.map_generation == state.localization_generation
        )
        if state.runtime_observation is not None:
            self._last_sampled_steady = state.runtime_observation.last_sampled_steady
        elif state.motion_runtime is not None:
            self._last_sampled_steady = state.motion_runtime.sampled_steady
        self._sequence += 1
        task_view = None
        if self._navigation is not None:
            self._navigation.executor.context = state.for_tasks(self._connected())
            current = await self._tasks.current_execution()
            if current is not None:
                try:
                    task_view = await self._navigation.execution_view(current.execution_id)
                    definition = next(
                        (
                            task
                            for task in tasks
                            if task.task_id == current.task_id
                            and task.task_id not in deleted_task_ids
                        ),
                        None,
                    )
                    if definition is not None:
                        task_view = {
                            **task_view,
                            "name": definition.name,
                            "definition_task_id": definition.task_id,
                        }
                    # //Compatible: hidden direct-navigation tasks have no catalog metadata.
                except KeyError:
                    pass  # //Compatible: unmigrated execution has no lifecycle projection.
        ui = self._ports.frame(
            state.motion_runtime,
            sequence=self._epoch() * 1_000_000_000 + self._sequence,
            connected=self._connected(),
            task=task_view,
            manual=state.manual_authority,
            manual_enabled=self._manual_enabled and state.manual_protocol_version == 1,
            tasks_enabled=self._navigation is not None,
            runtime_enabled=state.manual_protocol_version == 1,
            safety_revisions=self._runtime_revisions(state.motion_runtime),
            observation=state.runtime_observation,
            now=self._clock(),
        )
        capabilities = ui["capabilities"]
        assert isinstance(capabilities, list)
        capabilities.append("task.names.v1")
        if self._settings_enabled:
            capabilities.append("settings.v1")
        if self._logs_enabled():
            capabilities.append("logs.v1")
        return LiveState(
            localized,
            state.map_ref,
            home.waypoint if home else None,
            task_revision,
            state.execution_status,
            runnable,
            summaries,
            deleted_task_ids,
            map_generation=state.map_generation,
            map_grid_id=state.map_grid_id,
            route_progress=state.route_progress,
            navigation_ready=localized and state.map_ref is not None and state.navigation_ready,
            robot_pose=state.robot_pose,
            navigation_block_reason=state.navigation_block_reason,
            scan_points=state.scan_points,
            scan_layer=state.scan_layer,
            vehicle_ui=ui,
            manual_controller_generation=(state.manual_authority or {}).get(
                "controller_generation"
            ),
            manual_agent_epoch=(state.manual_authority or {}).get("connection_epoch"),
        )
