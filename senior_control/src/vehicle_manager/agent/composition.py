"""Original management bootstrap orchestration over explicit production effects."""

import asyncio
import logging
import os
import time
import uuid
from collections.abc import AsyncIterator, Callable
from contextlib import asynccontextmanager
from dataclasses import replace
from datetime import datetime, timezone
from typing import Any, TypeVar

from vehicle_manager.agent.bootstrap_ports import (
    AgentBootstrapPorts,
    AgentConfiguration,
    LocalClient,
)
from vehicle_manager.agent.logs_wire import AgentLogHandler, CloudLogGrants
from vehicle_manager.agent.main import Agent
from vehicle_manager.agent.settings_wire import SettingsRuntime
from vehicle_manager.settings.models import vehicle_settings_profile
from vehicle_manager.settings.service import RuntimeGate, SettingsService
from vehicle_manager.settings.store import SettingsStore
from vehicle_manager.storage.database import Database
from vehicle_manager.tasks.executor import TaskExecutor
from vehicle_manager.tasks.lifecycle import Context
from vehicle_manager.tasks.lifecycle_service import LifecycleService
from vehicle_manager.tasks.lifecycle_store import Admission, ContextProvider, LifecycleStore
from vehicle_manager.tasks.models import ExecutionContext
from vehicle_manager.tasks.navigation_port import NavigationEffects, TaskNavigation
from vehicle_manager.tasks.runtime import TaskRuntime
from vehicle_manager.tasks.store import TaskStore
from vehicle_manager.vehicle_logs.store import VehicleLogStore

logger = logging.getLogger("senior_control.agent")
ExecutorT = TypeVar("ExecutorT", bound=TaskExecutor)


def build_task_runtime(
    database: Database,
    tasks: TaskStore,
    navigation: TaskNavigation,
    *,
    boot_id: str,
    connected: Callable[[], bool] | None = None,
    admission: Admission | None = None,
    executor_factory: Callable[..., ExecutorT],
    navigation_effects: Callable[..., NavigationEffects],
) -> tuple[TaskRuntime, ExecutorT]:
    """Assembly only; all transitions and effects stay in the task driver/ports."""
    durable = LifecycleStore(tasks, boot_id=boot_id, admission=admission)
    effects = navigation_effects(durable, navigation)

    def current() -> Context:
        value = executor.current_context()
        return replace(value, connected=connected()) if connected is not None else value

    service = LifecycleService(durable, effects, context=current)
    executor = executor_factory(database, tasks, None, lifecycle=service)
    return TaskRuntime(tasks, executor), executor


async def pause_failed_tasks(executor: TaskExecutor) -> bool:
    """Use the existing durable owner; absence of a revoke proof remains pending."""
    retired = await executor.control_current("pause", "runtime-tasks-failed")
    if retired is None:
        return True
    assert executor.lifecycle is not None
    current = await executor.lifecycle.store.load(retired[0])
    return current.phase in {"paused", "succeeded", "failed", "canceled"}


async def run_agent(config: AgentConfiguration, *, ports: AgentBootstrapPorts) -> int:
    ports.configure_diagnostics()
    async with ports.database(config.database_path) as database:
        tasks = ports.tasks(database, robot_id=config.robot_id)
        waypoints = ports.waypoints(database, robot_id=config.robot_id)
        journal = ports.journal(database, robot_id=config.robot_id)
        await tasks.prepare()
        await waypoints.prepare()
        try:
            await journal.prepare()
        except Exception:
            logger.exception("could not initialize agent operation journal")
            return 78
        domain_value = os.environ.get("SENIOR_EXPECTED_ROS_DOMAIN")
        domain = int(domain_value) if domain_value is not None else None

        def ipc(*, timeout_ms: float = 1000.0) -> LocalClient:
            return ports.ipc(
                config.socket_path,
                timeout_ms=timeout_ms,
                robot_id=config.robot_id,
                ros_domain=domain,
                owner_record=os.environ.get("SENIOR_VERIFIED_ADAPTER_RUN"),
            )

        motion = ports.motion(
            ipc(),
            intent_client=ipc(),
            disable_client=ipc(),
            safety_client=ipc(),
        )
        state_client = ipc()
        map_client = ipc()
        navigation_client = ipc(timeout_ms=3000.0)
        workspace_client = ipc()
        display_client = ipc()
        state = ports.state(state_client)
        exported_grid = None
        if config.workspace_map_bundle is not None and config.workspace_map_sha256 is not None:
            try:
                exported_grid = await asyncio.to_thread(
                    ports.load_exported_grid,
                    config.workspace_map_bundle,
                    config.workspace_map_sha256,
                    config.robot_id,
                )
            except (ports.error, OSError):
                logger.warning("configured workspace display map is unavailable", exc_info=True)
        workspace = ports.workspace(
            config.robot_id, tasks, waypoints, workspace_client, exported_grid=exported_grid
        )
        display = ports.workspace(
            config.robot_id, tasks, waypoints, display_client, exported_grid=exported_grid
        )
        home = ports.home(state_client, waypoints, robot_id=config.robot_id)
        agent: Agent[Any]
        navigation = ports.navigation(
            navigation_client,
            waypoints,
            tasks=tasks,
        )
        context = ExecutionContext(False, False, False, False, None)

        settings_runtime: SettingsRuntime
        gate = RuntimeGate(
            lambda: settings_runtime.check(), refresh=lambda: settings_runtime.refresh()
        )

        @asynccontextmanager
        async def task_commit() -> AsyncIterator[ContextProvider]:
            while True:
                generation = gate.generation
                # Sample outside the gate; failed reads never reuse an earlier context.
                try:
                    sampled = await state.read()
                except ports.error:
                    # Withdrawal/recovery still commits; no new motion may use an old sample.
                    local = ExecutionContext(False, False, False, False, None)
                else:
                    local = sampled.for_tasks(agent.connected)
                    if "tasks" in agent.failed_modules:
                        local = replace(local, admission_ready=False)
                    authority = sampled.manual_authority
                    local = replace(
                        local,
                        manual_driving=local.manual_driving
                        or bool(authority and authority.get("state") in {"prepared", "active"}),
                    )
                async with gate.lock:
                    if gate.generation != generation:
                        continue
                    gate.invalidate()
                    admitted_generation = gate.generation

                    # This provider rechecks the original sample's expiry inside
                    # the SQLite transaction, even if it waited for a DB writer.
                    def current(
                        local: ExecutionContext = local, generation: int = admitted_generation
                    ) -> Context:
                        if gate.generation != generation:
                            return Context(agent.connected, False)
                        return replace(executor.current_context(local), connected=agent.connected)

                    yield current
                    return

        runtime, executor = ports.build_task_runtime(
            database,
            tasks,
            navigation,
            boot_id=uuid.uuid4().hex,
            connected=lambda: agent.connected,
            admission=task_commit,
        )
        motion.bind_tasks(executor)
        settings = None
        log_store = None
        logs = None
        log_connection = None
        log_handler = None
        previous_log_level = logger.level
        assert executor.lifecycle is not None
        settings_runtime = ports.settings_runtime(state, executor.lifecycle.store)
        settings = SettingsService(SettingsStore(database), vehicle_settings_profile(), gate)
        await settings.prepare()
        settings.start_debug_restore()
        candidate = VehicleLogStore(config.database_path.parent / "logs")
        try:
            await candidate.__aenter__()
        except (OSError, ValueError):
            logger.warning("vehicle log storage unavailable; control remains independent")
        else:
            log_store = candidate
            grants = CloudLogGrants()
            logs = ports.log_service(
                log_store, config.robot_id, grants, lambda: settings.published_snapshot
            )
            await logs.start()
            log_connection = ports.log_connection(logs, grants)
            log_handler = AgentLogHandler(logs)
            logging.getLogger("senior_control").addHandler(log_handler)
            logger.setLevel(logging.DEBUG)
            logger.info("agent_log_source_started", extra={"event": "agent_log_source_started"})
        motion.settings = settings
        navigation_actions = ports.navigation_actions(executor, state, workspace, waypoints)

        async def task_loop() -> None:
            nonlocal context
            while True:
                try:
                    live = await state.read()
                except ports.error:
                    context = ExecutionContext(False, False, False, False, None)
                else:
                    context = live.for_tasks(agent.connected)
                await runtime.tick(datetime.now(timezone.utc), time.monotonic(), context)
                await asyncio.sleep(0.2)

        media = ports.media()
        video_session = media.session

        async def cleanup() -> None:
            await asyncio.gather(
                state_client.close(),
                map_client.close(),
                navigation_client.close(),
                workspace_client.close(),
                display_client.close(),
            )
            if log_handler is not None:
                logging.getLogger("senior_control").removeHandler(log_handler)
                logger.setLevel(previous_log_level)
            try:
                if logs is not None:
                    await logs.close()
            finally:
                try:
                    if log_store is not None:
                        await log_store.__aexit__(None, None, None)
                finally:
                    if settings is not None:
                        await settings.close()
                    await media.close()

        async def background_fault(name: str) -> None:
            # Stop the failed task driver, persist pause/revoke through its sole owner.
            gate.invalidate()
            executor.context = replace(context, admission_ready=False)
            if not await pause_failed_tasks(executor):
                agent.report_runtime_event(
                    {
                        "module": "tasks",
                        "phase": "unconfirmed",
                        "detail": "task pause/revocation is pending",
                    }
                )

        event_fd = os.environ.get(ports.read_fd)

        async def lifecycle_loop() -> None:
            assert event_fd is not None
            await ports.observe(int(event_fd), agent.report_runtime_event)
            # EOF is already recorded by observe; no global shutdown or restart.

        async def logs_loop() -> None:
            if log_connection is not None:
                await log_connection.run(config.cloud_url, config.device_token)

        agent = ports.agent(
            config.robot_id,
            connector=ports.connector(
                config.cloud_url,
                config.device_token,
                config.robot_id,
            ),
            journal=journal,
            tasks=ports.store_task_port(
                tasks, robot_id=config.robot_id, verify_workspace=workspace.verify_task_edit
            ),
            home=home,
            maps=ports.maps(map_client),
            state=ports.snapshot_state(
                state,
                tasks,
                waypoints,
                display,
                navigation=navigation_actions,
                connected=lambda: agent.connected,
                epoch=lambda: agent.connection_epoch,
                manual_enabled=settings is not None,
                settings_enabled=settings is not None,
                logs_enabled=lambda: log_connection is not None and log_connection.connected,
            ),
            settings=settings,
            runtime_gate=gate,
            workspace=workspace,
            navigation=navigation_actions,
            motion=motion,
            video_session=video_session,
            map_root=config.database_path.parent / "map-transfer",
            background={
                "tasks": task_loop,
                **({"logs": logs_loop} if logs is not None else {}),
                **({"observer": lifecycle_loop} if event_fd is not None else {}),
            },
            background_fault=background_fault,
            cleanup=cleanup,
            # Accepted tasks are vehicle-owned; only manual authority follows eligibility.
        )
        agent.install_signal_handlers()
        await agent.run()
        if agent.auth_rejected or agent.runtime_failed:
            return 78
        return 1 if agent.runtime_failed else 0
