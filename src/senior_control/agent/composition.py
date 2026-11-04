"""Configuration and composition for the robot agent."""

import asyncio
import logging
import time
from collections.abc import Mapping
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urlsplit

from senior_control.agent.connection import IpcMotion
from senior_control.agent.dispatcher import StoreTaskPort
from senior_control.agent.journal import AgentJournal
from senior_control.agent.main import Agent
from senior_control.agent.ports import IpcHomePort, IpcMapPort, IpcNavigation, IpcState
from senior_control.agent.snapshots import LiveState, TaskSummary
from senior_control.agent.websocket import WebSocketConnector
from senior_control.errors import DomainError
from senior_control.runtime.ipc import IpcClient
from senior_control.storage.database import Database
from senior_control.tasks import ExecutionContext, TaskExecutor, TaskStore
from senior_control.tasks.runtime import TaskRuntime
from senior_control.waypoints.store import WaypointStore

logger = logging.getLogger("senior_control.agent")


@dataclass(frozen=True)
class AgentConfig:
    robot_id: str
    cloud_url: str
    device_token: str = field(repr=False)
    database_path: Path
    socket_path: Path

    def __post_init__(self) -> None:
        url = urlsplit(self.cloud_url)
        if not self.robot_id.strip() or not self.device_token.strip():
            raise ValueError("robot id and credential are required")
        if url.scheme != "wss" or not url.hostname or url.username or url.password:
            raise ValueError("cloud URL must use authenticated TLS")
        if not self.database_path.is_absolute() or not self.socket_path.is_absolute():
            raise ValueError("database and socket paths must be absolute")

    @classmethod
    def from_environment(cls, environment: Mapping[str, str]) -> "AgentConfig":
        try:
            return cls(
                robot_id=environment["SENIOR_ROBOT_ID"],
                cloud_url=environment["SENIOR_CLOUD_URL"],
                device_token=environment["SENIOR_DEVICE_TOKEN"],
                database_path=Path(environment["SENIOR_DATABASE_PATH"]),
                socket_path=Path(environment["SENIOR_AGENT_SOCKET"]),
            )
        except KeyError as exc:
            raise ValueError(f"missing environment variable {exc.args[0]}") from exc


class SnapshotState:
    def __init__(self, state: IpcState, tasks: TaskStore, waypoints: WaypointStore) -> None:
        self._state = state
        self._tasks = tasks
        self._waypoints = waypoints

    async def collect(self) -> LiveState:
        tasks = await self._tasks.list_tasks()
        home = await self._waypoints.home()
        summaries = tuple(TaskSummary(task.task_id, task.revision, task.enabled) for task in tasks)
        try:
            state = await self._state.read()
        except DomainError:
            return LiveState(
                False,
                None,
                home.waypoint if home else None,
                sum(task.revision for task in tasks),
                "unavailable",
                tasks=summaries,
            )
        # //Compatible: legacy IPC has no reference nonce, so runnable status is unproven.
        runnable: tuple[str, ...] = ()
        return LiveState(
            False,
            state.map_ref,
            home.waypoint if home else None,
            sum(task.revision for task in tasks),
            state.execution_status,
            runnable,
            summaries,
        )


async def run_agent(config: AgentConfig) -> int:
    async with Database(config.database_path) as database:
        tasks = TaskStore(database, robot_id=config.robot_id)
        waypoints = WaypointStore(database, robot_id=config.robot_id)
        journal = AgentJournal(database, robot_id=config.robot_id)
        await tasks.prepare()
        await waypoints.prepare()
        try:
            await journal.prepare()
        except Exception:
            logger.exception("could not initialize persistent agent safety state")
            return 78
        motion = IpcMotion(
            IpcClient(config.socket_path),
            heartbeat_client=IpcClient(config.socket_path),
            disable_client=IpcClient(config.socket_path),
        )
        try:
            await motion.verify_robot(config.robot_id)
        except Exception:
            logger.exception("ROS adapter identity does not match the paired robot")
            await motion.close()
            return 78
        state_client = IpcClient(config.socket_path)
        map_client = IpcClient(config.socket_path)
        navigation_client = IpcClient(config.socket_path)
        state = IpcState(state_client)
        home = IpcHomePort(state_client, waypoints, robot_id=config.robot_id)
        navigation = IpcNavigation(navigation_client)
        agent: Agent
        executor = TaskExecutor(
            database,
            tasks,
            navigation,
            link_up=lambda: agent.eligible_for_motion,
            generation=lambda: agent.continuity_generation,
        )
        runtime = TaskRuntime(tasks, executor, generation=lambda: agent.continuity_generation)

        async def task_loop() -> None:
            while True:
                try:
                    live = await state.read()
                except DomainError:
                    context = ExecutionContext(False, False, False, False, None)
                else:
                    context = ExecutionContext(
                        agent.eligible_for_motion,
                        live.localized,
                        live.safety_clear,
                        live.manual_driving,
                        live.map_ref,
                    )
                await runtime.tick(datetime.now(timezone.utc), time.monotonic(), context)
                await asyncio.sleep(0.2)

        async def cleanup() -> None:
            await asyncio.gather(
                state_client.close(), map_client.close(), navigation_client.close()
            )

        agent = Agent(
            config.robot_id,
            connector=WebSocketConnector(config.cloud_url, config.device_token, config.robot_id),
            journal=journal,
            tasks=StoreTaskPort(tasks, robot_id=config.robot_id, journal=journal),
            home=home,
            maps=IpcMapPort(map_client),
            state=SnapshotState(state, tasks, waypoints),
            motion=motion,
            source_clock_verified=False,
            map_root=config.database_path.parent / "map-transfer",
            background={"tasks": task_loop},
            cleanup=cleanup,
            on_eligibility_lost=tasks.mark_active_cancel_pending,
        )
        agent.install_signal_handlers()
        await agent.run()
        if agent.auth_rejected or agent.recovery_required:
            return 78
        return 1 if agent.runtime_failed else 0
