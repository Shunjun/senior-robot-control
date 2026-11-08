"""ROS-side end of the agent IPC.

The agent process owns the cloud link and the robot database. It owns no rclpy executor and
publishes no ``/cmd_vel``. This process takes its requests over a local Unix socket and
hands every motion request to the arbiter through ``CommandNode``: drive, stop, and estop
arrive as validated envelopes and go through ``submit``; disable and exit go through
``disconnect``. Nothing here writes a velocity of its own, so the arbiter's timer is the
only publisher and cannot replay an old lease behind a stop. ``rclpy`` is imported inside
``main`` so the request handling below can be tested without ROS.
"""

import asyncio
import os
from collections.abc import Callable
from pathlib import Path
from typing import TYPE_CHECKING, Any

from senior_control.mapping.workspace import WorkspaceMap
from senior_control.navigation import RoutePlan
from senior_control.protocol import (
    SimpleEnvelope,
    VelocityEnvelope,
    validate_envelope,
)
from senior_control.safety.arbiter import CommandResult

if TYPE_CHECKING:
    from senior_robot.navigation_adapter import RouteService

SOCKET_ENV = "SENIOR_AGENT_SOCKET"

_ENVELOPE_KINDS = {
    "drive": "drive.velocity",
    "stop": "drive.stop",
    "estop": "safety.estop",
}


class AdapterCore:
    """Maps IPC requests to the arbiter. After ``disable_motion`` nothing drives."""

    def __init__(
        self,
        *,
        robot_id: str,
        submit: Callable[..., CommandResult],
        disconnect: Callable[[int], None],
        request_exit: Callable[[], None],
        server_heartbeat: Callable[[int, float], bool] | None = None,
        require_source_deadline: bool = False,
        workspace_map: WorkspaceMap | None = None,
        state_provider: Callable[[], dict[str, Any]] | None = None,
        pose_provider: Callable[[], dict[str, Any]] | None = None,
        take_drive_feedback: Callable[[], list[dict[str, Any]]] | None = None,
        routes: "RouteService | None" = None,
        navigation_ready: Callable[[], bool] | None = None,
    ) -> None:
        self._robot_id = robot_id
        self._submit = submit
        self._disconnect = disconnect
        self._request_exit = request_exit
        self._server_heartbeat = server_heartbeat
        self._require_source_deadline = require_source_deadline
        self._workspace_map = workspace_map
        self._state_provider = state_provider
        self._pose_provider = pose_provider
        self._take_drive_feedback = take_drive_feedback
        self._routes = routes
        self._navigation_ready = navigation_ready
        self._active_epoch: int | None = None
        self._heartbeat_epoch: int | None = None
        self._highest_epoch = -1
        self._disabled_through_epoch = -1
        # //Compatible: legacy isolated adapter tests do not have a heartbeat callback.
        self.motion_disabled = server_heartbeat is not None
        self._exiting = False

    def release(self) -> None:
        """End the active epoch only. Safe to repeat."""
        epoch = self._heartbeat_epoch if self._heartbeat_epoch is not None else self._active_epoch
        self._heartbeat_epoch = self._active_epoch = None
        if epoch is not None:
            self._disconnect(epoch)

    async def handle(self, message: dict[str, Any]) -> dict[str, Any]:
        kind = message["type"]
        if kind in {
            "navigation.task_start", "navigation.task_status", "navigation.task_cancel",
            "navigation.current",
        }:
            return await self._route_request(kind, message)
        if kind == "drive_feedback":
            if self._take_drive_feedback is None:
                return {"ok": False, "error": "unsupported"}
            return {"ok": True, "feedback": self._take_drive_feedback()}
        if kind == "identity":
            return {"ok": message.get("robot_id") == self._robot_id}
        if kind == "server_heartbeat":
            epoch, deadline = message.get("connection_epoch"), message.get("deadline")
            if (
                self._exiting
                or self._server_heartbeat is None
                or not isinstance(epoch, int)
                or isinstance(epoch, bool)
                or isinstance(deadline, bool)
                or not isinstance(deadline, (int, float))
                or epoch <= self._disabled_through_epoch
            ):
                return {"ok": False, "error": "bad_request"}
            accepted = self._server_heartbeat(epoch, float(deadline))
            if accepted:
                self._heartbeat_epoch = epoch
            return {"ok": accepted}
        if kind in {"state.get", "pose.get"}:
            provider = self._state_provider if kind == "state.get" else self._pose_provider
            if provider is None or self._exiting:
                return {"ok": False, "error": "unavailable"}
            return provider()
        if kind in {"map.active", "map.manifest", "map.tile"}:
            if self._workspace_map is None or self._exiting:
                return {"ok": False, "error": "unavailable"}
            result = self._workspace_map.read(kind, message)
            if (
                kind == "map.manifest" and result.get("ok") is True
                and self._navigation_ready is not None and self._navigation_ready()
            ):
                result["navigation_ready"] = True
            return result
        if kind == "ping":
            return {"ok": True}
        if kind == "disable_motion":
            epoch = message.get("connection_epoch")
            if not isinstance(epoch, int) or isinstance(epoch, bool):
                return {"ok": False, "error": "bad_request"}
            if epoch < max(
                self._highest_epoch, self._disabled_through_epoch, self._heartbeat_epoch or 0
            ):
                return {"ok": True}
            self._disabled_through_epoch = epoch
            self.motion_disabled = True
            self.release()
            return {"ok": True}
        if kind == "exit":
            self._exiting = True
            self.motion_disabled = True
            self.release()
            self._request_exit()
            return {"ok": True}
        if kind in {"stop", "estop"}:
            return self._send(message, kind, safety=True)
        if self.motion_disabled:
            if kind == "command" and not self._exiting:
                return self._reenable(message)
            return {"ok": False, "error": "motion_disabled"}
        if kind == "drive":
            return self._send(message, kind, safety=False)
        if kind == "command":
            return self._send(message, None, safety=False)
        if kind == "ros_call":
            return {"ok": False, "error": "unsupported"}
        return {"ok": False, "error": "unknown_type"}

    async def _route_request(self, kind: str, message: dict[str, Any]) -> dict[str, Any]:
        routes = self._routes
        if routes is None:
            return {"ok": False, "error": "unavailable"}
        if kind == "navigation.current":
            current = routes.current
            return {"ok": True, "progress": current.model_dump() if current is not None else None}
        execution_id = message.get("execution_id")
        if not isinstance(execution_id, str) or not 1 <= len(execution_id) <= 128:
            return {"ok": False, "error": "bad_request"}
        if kind == "navigation.task_start":
            if self._exiting:
                return {"ok": False, "error": "unavailable"}
            try:
                plan = RoutePlan.model_validate(message.get("plan"))
            except ValueError:
                return {"ok": False, "error": "bad_request"}
            if plan.execution_id != execution_id:
                return {"ok": False, "error": "bad_request"}
            if not await routes.start(plan, wait_for_goal=False):
                return {"ok": False, "error": "rejected", "execution_id": execution_id}
            return {
                "ok": True, "execution_id": execution_id, "map_generation": plan.map_generation,
            }
        if kind == "navigation.task_cancel":
            return {"ok": await routes.cancel(execution_id), "execution_id": execution_id}
        progress = routes.status(execution_id)
        return {
            "ok": True, "execution_id": execution_id,
            "progress": progress.model_dump() if progress is not None else None,
        }

    def _reenable(self, message: dict[str, Any]) -> dict[str, Any]:
        raw = message.get("envelope")
        if not isinstance(raw, str):
            return {"ok": False, "error": "motion_disabled"}
        try:
            envelope = validate_envelope(raw.encode(), self._robot_id)
        except ValueError:
            return {"ok": False, "error": "motion_disabled"}
        if envelope.type != "control.acquire" or envelope.connection_epoch <= max(
            self._highest_epoch, self._disabled_through_epoch
        ):
            return {"ok": False, "error": "motion_disabled"}
        result = self._send(message, None, safety=False)
        if result.get("ok"):
            self.motion_disabled = False
        return result

    def _send(self, message: dict[str, Any], kind: str | None, *, safety: bool) -> dict[str, Any]:
        del safety
        raw = message.get("envelope")
        if not isinstance(raw, str):
            return {"ok": False, "error": "bad_request"}
        try:
            envelope = validate_envelope(raw.encode(), self._robot_id)
        except ValueError:
            return {"ok": False, "error": "bad_request"}
        if kind is not None and envelope.type != _ENVELOPE_KINDS[kind]:
            return {"ok": False, "error": "bad_request"}
        if not isinstance(envelope, (VelocityEnvelope, SimpleEnvelope)):
            return {"ok": False, "error": "bad_request"}
        if self._server_heartbeat is not None and (
            envelope.type.startswith("navigation.") or envelope.type == "safety.reset"
        ):
            return {"ok": False, "error": "unavailable"}
        received_at = message.get("received_at")
        if isinstance(received_at, bool) or not isinstance(received_at, (int, float)):
            received_at = None
        local_deadline = message.get("local_deadline")
        if isinstance(local_deadline, bool) or not isinstance(local_deadline, (int, float)):
            local_deadline = None
        if isinstance(envelope, VelocityEnvelope) and self._require_source_deadline:
            if local_deadline is None:
                return {"ok": False, "error": "bad_request"}
        if local_deadline is not None:
            result = self._submit(envelope, received_at, float(local_deadline))
        else:
            # //Compatible: isolated legacy adapter callers do not carry an absolute deadline.
            result = self._submit(envelope, received_at)
        if not result.accepted:
            # The agent distinguishes stale frames from an unconfirmed safety request.
            return {"ok": False, "accepted": False, "error": result.error.value}
        if envelope.type not in {"safety.estop", "safety.reset"}:
            self._highest_epoch = max(self._highest_epoch, envelope.connection_epoch)
            self._active_epoch = (
                None if envelope.type == "control.release" else envelope.connection_epoch
            )
        return {"ok": True, "accepted": True}


def main() -> None:
    import rclpy

    from senior_control.runtime.ipc import IpcServer
    from senior_robot.command_node import CommandNode
    from senior_robot.motion_lifecycle import MotionExecutor, run
    from senior_robot.simulation_reference import SimulationReference

    def spin(node, executor) -> None:
        robot_id = str(node.get_parameter("robot_id").value)
        path = Path(os.environ[SOCKET_ENV])
        node.declare_parameter("simulation_qualified_reference", False)
        node.declare_parameter("simulation_baseline_world", "")
        reference = None
        if node.get_parameter("simulation_qualified_reference").value:
            world = Path(str(node.get_parameter("simulation_baseline_world").value))
            reference = SimulationReference(node, world)
            node.simulation_reference = reference
        exiting = False

        def state() -> dict[str, Any]:
            assert reference is not None
            current = routes.current if routes is not None else None
            result = reference.state(
                node.motion_mode(),
                current.status if current is not None else "idle",
                navigation_ready=navigation_ready is not None and navigation_ready(),
            )
            if current is not None and current.map_generation == result["map_generation"]:
                result["route_progress"] = current.model_dump()
            return result

        def request_exit() -> None:
            nonlocal exiting
            exiting = True

        routes = None
        navigation_ready: Callable[[], bool] | None = None
        node.declare_parameter("navigation_controller_params", "")
        nav_params = str(node.get_parameter("navigation_controller_params").value)
        if reference is not None and nav_params and Path(nav_params).is_file():
            from senior_robot.dedicated_controller import DedicatedController
            from senior_robot.navigation_adapter import (
                RouteService,
                create_nav2_adapter,
                create_planner_readiness,
            )

            planner_active = create_planner_readiness(node)

            previous_failure = None

            def qualified(ref, generation):
                nonlocal previous_failure
                failure = None
                if exiting:
                    failure = "adapter exiting"
                elif reference is None or not reference.qualified(ref, generation):
                    failure = "map or localization generation changed"
                elif not planner_active():
                    failure = "planner state unavailable"
                elif not reference.safety_ready():
                    failure = f"safety status unavailable ({reference.safety_reason})"
                elif not node.navigation_qualified():
                    failure = "heartbeat or control authority unavailable"
                if failure != previous_failure:
                    if failure is not None and routes is not None and routes.current is not None:
                        node.get_logger().warning(f"Route qualification lost: {failure}")
                    previous_failure = failure
                return failure is None

            def can_navigate() -> bool:
                grid = reference.grid() if reference is not None else None
                return grid is not None and qualified(grid.map_ref, grid.grid_id)

            navigation_ready = can_navigate
            routes = RouteService(
                create_nav2_adapter(node),
                qualified=qualified,
                prepare=DedicatedController(node, Path(nav_params)).open,
                bind_source=lambda execution_id, topic, guard: node.bind_navigation_source(
                    execution_id, topic, qualified=guard
                ),
                clear_source=node.clear_navigation_source,
                source_active=node.navigation_source_matches,
                connection_epoch=node.current_navigation_epoch,
            )

        core = AdapterCore(
            robot_id=robot_id,
            submit=lambda envelope, received_at=None, local_deadline=None: node.submit(
                envelope, received_at, local_deadline=local_deadline
            ),
            disconnect=node.disconnect,
            request_exit=request_exit,
            server_heartbeat=node.server_heartbeat,
            require_source_deadline=True,
            take_drive_feedback=node.take_drive_feedback,
            workspace_map=WorkspaceMap(reference.grid) if reference is not None else None,
            state_provider=state if reference is not None else None,
            pose_provider=reference.pose_reply if reference is not None else None,
            routes=routes,
            navigation_ready=navigation_ready,
        )

        async def serve() -> None:
            server = IpcServer(path, core.handle)
            started = False
            try:
                await server.start()
                started = True
                executor.add_node(node)
                while rclpy.ok() and not exiting:
                    executor.spin_once(timeout_sec=0.0)
                    await asyncio.sleep(0.01)
                if routes is not None and routes.current is not None:
                    cancellation = asyncio.create_task(routes.cancel(routes.current.execution_id))
                    deadline = asyncio.get_running_loop().time() + 3.0
                    while not cancellation.done() and asyncio.get_running_loop().time() < deadline:
                        if rclpy.ok():
                            executor.spin_once(timeout_sec=0.0)
                        await asyncio.sleep(0.01)
                    cancellation.cancel()
                    await asyncio.gather(cancellation, return_exceptions=True)
            finally:
                try:
                    if started:
                        await server.close()
                finally:
                    # End any active lease through the arbiter before node destruction.
                    core.release()

        asyncio.run(serve())

    # //Compatible: use the same Humble signal lifecycle as the synchronous CLIs.
    run(
        lambda: CommandNode(require_server_heartbeat=True),
        cli=True,
        spin=spin,
        executor_factory=MotionExecutor,
    )
