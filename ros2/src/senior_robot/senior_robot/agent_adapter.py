"""ROS-side end of the agent IPC.

The agent process owns the cloud link and the robot database. It owns no rclpy executor and
publishes no ``/cmd_vel``. This process takes its requests over a local Unix socket and
hands native manual requests to ``CommandNode.manual_request`` and emergency stops to
``submit``. Disconnect and exit retire the original connection's authority; accepted tasks
keep their local execution lifetime. This adapter never writes an actuator velocity.
``rclpy`` is imported inside
``main`` so the request handling below can be tested without ROS.
"""

import asyncio
import fcntl
import os
import re
import stat
import time
from collections.abc import AsyncIterator, Callable
from contextlib import AsyncExitStack, asynccontextmanager
from pathlib import Path
from typing import TYPE_CHECKING, Any

from senior_contracts.wire import SimpleEnvelope, validate_envelope
from senior_mapping.workspace import WorkspaceMap
from senior_motion_control.arbiter import CommandResult

from vehicle_core.route_plan import RoutePlan

if TYPE_CHECKING:
    from senior_navigation.navigation_adapter import RouteService

SOCKET_ENV = "SENIOR_AGENT_SOCKET"

_ENVELOPE_KINDS = {
    "stop": "drive.stop",
    "estop": "safety.estop",
}


def _spin_ready_callbacks(executor: Any, *, clock: Callable[[], float] = time.monotonic) -> None:
    """Drain a bounded callback batch before yielding to the IPC event loop."""
    deadline = clock() + 0.004
    for _ in range(16):
        executor.spin_once(timeout_sec=0.0)
        if clock() >= deadline:
            break


class AdapterCore:
    """Maps IPC requests to the arbiter. After ``disable_motion`` nothing drives."""

    def __init__(
        self,
        *,
        robot_id: str,
        submit: Callable[..., CommandResult],
        disconnect: Callable[[int], None],
        request_exit: Callable[[], None],
        workspace_map: WorkspaceMap | None = None,
        state_provider: Callable[[], dict[str, Any]] | None = None,
        pose_provider: Callable[[], dict[str, Any]] | None = None,
        routes: "RouteService | None" = None,
        navigation_ready: Callable[[], bool] | None = None,
        ros_domain: int | None = None,
        manual: Callable[[dict[str, Any]], dict[str, Any]] | None = None,
    ) -> None:
        self._robot_id = robot_id
        self._manual = manual
        self._ros_domain = ros_domain
        self._submit = submit
        self._disconnect = disconnect
        self._request_exit = request_exit
        self._workspace_map = workspace_map
        self._state_provider = state_provider
        self._pose_provider = pose_provider
        self._routes = routes
        self._settled_sequence = 0
        self._task_effect_sequences: dict[str, int] = {}
        self._task_effects: dict[str, tuple[str, str, int] | tuple[str, str, int, str]] = {}
        self._task_plans: dict[str, dict[str, Any]] = {}
        self._navigation_ready = navigation_ready
        self._connection_epoch: int | None = None
        self._disabled_through_epoch = -1
        self._exiting = False

    def release(self) -> None:
        """End the active epoch only. Safe to repeat."""
        epoch, self._connection_epoch = self._connection_epoch, None
        if epoch is not None:
            self._disconnect(epoch)

    async def handle(self, message: dict[str, Any]) -> dict[str, Any]:
        kind = message["type"]
        if kind in {
            "navigation.task_start",
            "navigation.task_status",
            "navigation.task_cancel",
            "navigation.task_terminal",
            "navigation.task_settled",
            "navigation.current",
        }:
            return await self._route_request(kind, message)
        if kind == "learning_capability":
            import os

            return {
                "ok": True,
                "learning_protocol_version": 1
                if (self._manual is not None and os.environ.get("SENIOR_STARTUP_LEARNING_CLAIM"))
                else 0,
            }
        if kind == "learning":
            import asyncio
            import hashlib
            import json
            import os
            from pathlib import Path

            from senior_contracts.wire import LearningStartEnvelope, LearningStatusEnvelope
            from senior_perception.learning_client import request_once
            from senior_perception.learning_wire import BeginAttempt, QueryAttempt

            if self._manual is None or self._exiting:
                return {"ok": False, "error": "unavailable"}
            try:
                envelope = validate_envelope(message["envelope"].encode(), self._robot_id)
                if not isinstance(envelope, (LearningStartEnvelope, LearningStatusEnvelope)):
                    raise ValueError("learning_envelope")
                if envelope.connection_epoch <= self._disabled_through_epoch:
                    return {"ok": False, "accepted": False, "error": "offline"}
                claim = os.environ.get("SENIOR_STARTUP_LEARNING_CLAIM")
                if not claim:
                    return {"ok": False, "error": "unavailable"}
                admission = self._manual(message)
                if not admission.get("ok"):
                    return admission
                request = (
                    QueryAttempt(payload=envelope.payload)
                    if isinstance(envelope, LearningStatusEnvelope)
                    else BeginAttempt(
                        start_request_id=envelope.request_id,
                        payload=envelope.payload,
                        fingerprint=hashlib.sha256(
                            json.dumps(
                                [
                                    envelope.robot_id,
                                    envelope.session_id,
                                    envelope.connection_epoch,
                                    envelope.controller_generation,
                                    envelope.payload.model_dump(),
                                ],
                                sort_keys=True,
                                separators=(",", ":"),
                            ).encode()
                        ).hexdigest(),
                        owner_boot_id=admission["owner_boot_id"],
                        output_boot_id=admission["output_boot_id"],
                        generation=admission["generation"],
                        admitted=admission["can_start"],
                        deadline_steady=admission["deadline_steady"],
                    )
                )
                reply = await asyncio.to_thread(
                    request_once, Path(claim).name.removeprefix(".startup-learning-"), request
                )
                if not reply.ok or reply.data is None:
                    return {"ok": False, "accepted": False, "error": "conflict"}
                data = dict(reply.data)
                if isinstance(envelope, LearningStatusEnvelope):
                    data["can_start"] = bool(data["can_start"] and admission["can_start"])
                    if not admission["can_start"]:
                        data["cannot_start_reason"] = "control_source_conflict"
                return {"ok": True, "data": data}
            except (KeyError, TypeError, ValueError, OSError):
                return {"ok": False, "error": "unavailable"}
        if kind in {"manual", "manual_intent"}:
            if self._manual is None or self._exiting:
                return {"ok": False, "error": "unsupported"}
            raw = message.get("envelope")
            if not isinstance(raw, str):
                return {"ok": False, "error": "bad_request"}
            try:
                envelope = validate_envelope(raw.encode(), self._robot_id)
            except ValueError:
                return {"ok": False, "error": "bad_request"}
            if kind == "manual_intent" and envelope.type != "drive.intent":
                return {"ok": False, "error": "bad_request"}
            if kind == "manual" and envelope.type == "drive.intent":
                return {"ok": False, "error": "bad_request"}
            if envelope.connection_epoch <= self._disabled_through_epoch:
                return {"ok": False, "error": "offline"}
            reply = self._manual(message)
            if reply.get("ok"):
                self._connection_epoch = max(self._connection_epoch or 0, envelope.connection_epoch)
            return reply
        if kind == "identity":
            reply = {"ok": message.get("robot_id") == self._robot_id}
            # //Compatible: legacy adapters omit their runtime ROS domain.
            if reply["ok"] and self._ros_domain is not None:
                return {**reply, "ros_domain": self._ros_domain}
            return reply
        if kind in {"state.get", "pose.get"}:
            provider = self._state_provider if kind == "state.get" else self._pose_provider
            if provider is None or self._exiting:
                return {"ok": False, "error": "unavailable"}
            result = provider()
            if kind == "state.get":
                result["task_lifecycle_version"] = 2 if self._routes is not None else 0
                result["manual_protocol_version"] = 1 if self._manual is not None else 0
            return result
        if kind in {"map.active", "map.manifest", "map.tile"}:
            if self._workspace_map is None or self._exiting:
                return {"ok": False, "error": "unavailable"}
            result = self._workspace_map.read(kind, message)
            if (
                kind == "map.manifest"
                and result.get("ok") is True
                and self._navigation_ready is not None
                and self._navigation_ready()
            ):
                result["navigation_ready"] = True
            return result
        if kind == "ping":
            return {"ok": True}
        if kind == "disable_motion":
            epoch = message.get("connection_epoch")
            if not isinstance(epoch, int) or isinstance(epoch, bool):
                return {"ok": False, "error": "bad_request"}
            if epoch < max(self._disabled_through_epoch, self._connection_epoch or 0):
                return {"ok": True}
            self._disabled_through_epoch = epoch
            self._connection_epoch = None
            self._disconnect(epoch)
            return {"ok": True}
        if kind == "exit":
            self._exiting = True
            self.release()
            self._request_exit()
            return {"ok": True}
        if kind in {"stop", "estop"}:
            return self._send(message, kind, safety=True)
        if kind == "ros_call":
            return {"ok": False, "error": "unsupported"}
        return {"ok": False, "error": "unknown_type"}

    def _compact_task_effects(self, frontier: int) -> None:
        self._settled_sequence = max(self._settled_sequence, frontier)
        for key, issued in list(self._task_effect_sequences.items()):
            if issued <= self._settled_sequence:
                self._task_effect_sequences.pop(key)
                self._task_effects.pop(key, None)
                self._task_plans.pop(key, None)

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
        sequence = message.get("effect_sequence", 0)
        if type(sequence) is not int or sequence < 0:
            return {"ok": False, "error": "bad_request"}
        if kind == "navigation.task_settled":
            epoch, effect_id = message.get("task_epoch"), message.get("effect_id")
            frontier = message.get("settled_through")
            if (
                type(epoch) is not int
                or epoch < 1
                or not isinstance(effect_id, str)
                or not 1 <= len(effect_id) <= 128
                or sequence < 1
                or type(frontier) is not int
                or frontier < 0
            ):
                return {"ok": False, "error": "bad_request"}
            if sequence <= self._settled_sequence:
                # An old cleanup retry is only an ACK. Never apply its frontier
                # or scope to newer in-flight records, even with altered payload.
                return dict(
                    ok=True,
                    lifecycle_version=2,
                    execution_id=execution_id,
                    effect_id=effect_id,
                    task_epoch=epoch,
                    accepted=True,
                    collected=True,
                )
            if frontier >= sequence:
                return {"ok": False, "error": "bad_request"}
            self._compact_task_effects(frontier)
            identity = ("terminal_settled", execution_id, epoch)
            known = self._task_effects.get(effect_id)
            if known is not None and (
                known != identity or self._task_effect_sequences.get(effect_id) != sequence
            ):
                return {"ok": False, "error": "conflict"}
            covered = [
                key
                for key, old in self._task_effects.items()
                if old[1] == execution_id and old[2] <= epoch
            ]
            if known is None and len(self._task_effects) >= 256 and not covered:
                # //Compatible: R2 could issue GC without reserving it here.
                # At capacity only query an existing downstream obligation;
                # never create a new scope or discard another tombstone.
                found, collected = routes.query_terminal_collection(
                    execution_id, epoch, effect_id, sequence, frontier
                )
                if not found:
                    return {"ok": False, "error": "unavailable"}
            else:
                # The business identity is durably settled. Replace its old live
                # records with one immutable GC reservation BEFORE issuing I/O.
                for key in covered:
                    self._task_effects.pop(key, None)
                    self._task_effect_sequences.pop(key, None)
                    self._task_plans.pop(key, None)
                self._task_effects[effect_id] = identity
                self._task_effect_sequences[effect_id] = sequence
                collected = routes.collect_terminal(
                    execution_id, epoch, effect_id, sequence, frontier
                )
            return dict(
                ok=True,
                lifecycle_version=2,
                execution_id=execution_id,
                effect_id=effect_id,
                task_epoch=epoch,
                accepted=collected,
                collected=collected is True,
            )
        plan_value = message.get("plan")
        has_effect = message.get("effect_id") is not None or (
            kind == "navigation.task_start"
            and isinstance(plan_value, dict)
            and plan_value.get("effect_id") is not None
        )
        if has_effect:
            if self._settled_sequence > 0 and sequence <= self._settled_sequence:
                return {"ok": False, "error": "retired_scope", "execution_id": execution_id}
            frontier = message.get("settled_through", 0)
            if type(frontier) is not int or frontier < 0 or (sequence > 0 and frontier >= sequence):
                return {"ok": False, "error": "bad_request"}
            self._compact_task_effects(frontier)
        requested_epoch = (
            message.get("plan", {}).get("task_epoch")
            if kind == "navigation.task_start" and isinstance(message.get("plan"), dict)
            else message.get("task_epoch")
        )
        if type(requested_epoch) is int and any(
            old[0] == "terminal_settled" and old[1] == execution_id and requested_epoch <= old[2]
            for old in self._task_effects.values()
        ):
            return {"ok": False, "error": "retired_scope", "execution_id": execution_id}
        if kind == "navigation.task_start":
            if self._exiting:
                return {"ok": False, "error": "unavailable"}
            try:
                plan = RoutePlan.model_validate(message.get("plan"))
            except ValueError:
                return {"ok": False, "error": "bad_request"}
            if sequence < 1:
                return {"ok": False, "error": "bad_request"}
            if plan.execution_id != execution_id:
                return {"ok": False, "error": "bad_request"}
            retained = sum(
                not (old[1] == execution_id and old[2] < plan.task_epoch)
                for old in self._task_effects.values()
            )
            if plan.effect_id not in self._task_effects and retained >= 256:
                return {"ok": False, "error": "unavailable"}
            if plan.effect_id in self._task_effects:
                matched = (
                    self._task_effect_sequences.get(plan.effect_id) == sequence
                    and self._task_plans.get(plan.effect_id) == plan.model_dump()
                    and self._task_effects[plan.effect_id]
                    == (
                        "plan",
                        execution_id,
                        plan.task_epoch,
                    )
                )
                return {
                    "ok": matched,
                    "execution_id": execution_id,
                    "task_epoch": plan.task_epoch,
                    "effect_id": plan.effect_id,
                    "lifecycle_version": 2,
                    "accepted": matched,
                    "map_generation": plan.map_generation,
                }
            if not await routes.start(plan, wait_for_goal=False):
                return {"ok": False, "error": "rejected", "execution_id": execution_id}
            for key, old in list(self._task_effects.items()):
                if old[1] == execution_id and old[2] < plan.task_epoch:
                    self._task_effects.pop(key)
                    self._task_effect_sequences.pop(key, None)
                    self._task_plans.pop(key, None)
            self._task_effects[plan.effect_id] = ("plan", execution_id, plan.task_epoch)
            self._task_plans[plan.effect_id] = plan.model_dump()
            self._task_effect_sequences[plan.effect_id] = sequence
            return {
                "ok": True,
                "accepted": True,
                "lifecycle_version": 2,
                "effect_id": plan.effect_id,
                "task_epoch": plan.task_epoch,
                "execution_id": execution_id,
                "map_generation": plan.map_generation,
            }
        effect_id, epoch = message.get("effect_id"), message.get("task_epoch")
        if effect_id is not None:
            if (
                sequence < 1
                or len(self._task_effects) >= 256
                and effect_id not in self._task_effects
            ):
                return {"ok": False, "error": "unavailable"}
            if (
                not isinstance(effect_id, str)
                or not 1 <= len(effect_id) <= 128
                or type(epoch) is not int
                or epoch < 1
            ):
                return {"ok": False, "error": "bad_request"}
            effect_kind = (
                "revoke"
                if kind == "navigation.task_cancel"
                else "terminal_release"
                if kind == "navigation.task_terminal"
                else message.get("effect_kind")
            )
            outcome = message.get("terminal_outcome")
            if effect_kind == "terminal_release" and (
                not isinstance(outcome, str) or outcome not in {"succeeded", "canceled", "failed"}
            ):
                return {"ok": False, "error": "bad_request"}
            identity = (
                (effect_kind, execution_id, epoch, outcome)
                if effect_kind == "terminal_release"
                else (effect_kind, execution_id, epoch)
            )
            known = self._task_effects.get(effect_id)
            if known is not None and self._task_effect_sequences.get(effect_id, 0) != sequence:
                return {"ok": False, "error": "conflict"}
            if known is not None and known != identity:
                return {"ok": False, "error": "conflict"}
            if effect_kind == "terminal_release":
                if kind == "navigation.task_terminal":
                    self._task_effects[effect_id] = identity
                    self._task_effect_sequences[effect_id] = sequence
                released = None
                if effect_id in self._task_effects:
                    options = {"dispatch_sequence": sequence} if sequence else {}
                    released = await routes.release_terminal(
                        execution_id, epoch, effect_id, outcome, **options
                    )
                return dict(
                    ok=True,
                    lifecycle_version=2,
                    execution_id=execution_id,
                    effect_id=effect_id,
                    task_epoch=epoch,
                    terminal_outcome=outcome,
                    identity_released=released is True,
                    accepted=released,
                )
            if kind == "navigation.task_cancel":
                self._task_effects[effect_id] = identity
                self._task_effect_sequences[effect_id] = sequence
                await routes.cancel(execution_id, task_epoch=epoch)
            retired = (
                routes.output_retired(execution_id, epoch) if effect_kind == "revoke" else False
            )
            return {
                "ok": True,
                "lifecycle_version": 2,
                "execution_id": execution_id,
                "effect_id": effect_id,
                "task_epoch": epoch,
                "accepted": (retired if effect_kind == "revoke" else True)
                if effect_id in self._task_effects
                else None,
                "sources_fenced_through_epoch": epoch if retired else None,
                "controller_output_exited": retired,
            }
        if kind in {"navigation.task_terminal", "navigation.task_cancel"}:
            return {"ok": False, "error": "bad_request"}
        progress = routes.status(execution_id)
        return {
            "ok": True,
            "execution_id": execution_id,
            "progress": progress.model_dump() if progress is not None else None,
        }

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
        if not isinstance(envelope, SimpleEnvelope):
            return {"ok": False, "error": "bad_request"}
        received_at = message.get("received_at")
        if isinstance(received_at, bool) or not isinstance(received_at, (int, float)):
            received_at = None
        result = self._submit(envelope, received_at)
        if not result.accepted:
            # The agent distinguishes stale frames from an unconfirmed safety request.
            return {"ok": False, "accepted": False, "error": result.error.value}
        return {"ok": True, "accepted": True}


@asynccontextmanager
async def _launch_endpoint(path: Path) -> AsyncIterator[str | None]:
    """Only this launch/run may reclaim its own definitely refused stale socket.

    The existing helper's direct real adapter never sets this context and keeps
    its PID/peer credential ownership unchanged. Hold the endpoint lock through
    bind, recording and server close, including during recovered startup.
    """
    owner = os.environ.get("SENIOR_LAUNCH_SOCKET_OWNER")
    if owner is None:
        yield None
        return
    if re.fullmatch(r"[0-9a-f]{32}", owner) is None:
        raise ValueError("invalid launch endpoint ownership context")
    from vehicle_core.ipc import _prepare_socket

    descriptor = os.open(
        path.with_suffix(".launch.lock"),
        os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW,
        0o600,
    )
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_mode & 0o077:
            raise ValueError("foreign launch endpoint lock")
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if os.environ.get("SENIOR_LAUNCH_RECOVERED") == "1":
            await _prepare_socket(path, expected_run_id=owner)
        yield owner
    finally:
        os.close(descriptor)


def main() -> None:
    import rclpy
    from senior_motion_control.command_node import CommandNode
    from senior_sensor_ingress.ros_lifecycle import MotionExecutor, run

    from vehicle_core.ipc import IpcServer

    def spin(node, executor, should_stop: Callable[[], bool]) -> None:
        robot_id = str(node.get_parameter("robot_id").value)
        path = Path(os.environ[SOCKET_ENV])
        node.declare_parameter("ipc_replace_existing_socket", True)
        node.declare_parameter("hardware_navigation_reference", "")
        hardware_reference = str(node.get_parameter("hardware_navigation_reference").value)
        reference = None
        if hardware_reference:
            from senior_localization.hardware_reference import HardwareReference

            reference = HardwareReference(node, Path(hardware_reference))
        exiting = False

        def state() -> dict[str, Any]:
            current = routes.current if routes is not None else None
            manual = node.manual_view()
            manual_active = manual is not None and manual["state"] == "active"
            result = (
                reference.state(
                    "manual" if manual_active else "idle",
                    current.status if current is not None else "idle",
                    navigation_ready=navigation_ready is not None and navigation_ready(),
                )
                if reference is not None
                else {
                    "v": 1,
                    "ok": True,
                    "sampled_at": time.monotonic(),
                    "localized": False,
                    "map_id": None,
                    "reference_revision": None,
                    "execution_status": current.status if current is not None else "idle",
                    "manual_driving": False,
                    "safety_clear": False,
                    "navigation_ready": False,
                    "map_generation": None,
                    "localization_generation": None,
                }
            )
            result["task_evidence_ready"] = node.task_motion_allowed(continuing=True)
            runtime = node.motion_runtime()
            result["runtime_observation"] = node.runtime_observation()
            result["motion_runtime"] = runtime
            result["motion_runtime_version"] = 1
            result["manual_authority"] = manual
            result["manual_driving"] = manual_active
            result["navigation_block_reason"] = (
                None if result["navigation_ready"] else previous_failure or "navigation unavailable"
            )
            if current is not None and current.map_generation == result["map_generation"]:
                result["route_progress"] = current.model_dump()
            return result

        def request_exit() -> None:
            nonlocal exiting
            node.shutdown_motion()
            exiting = True

        routes = None
        controller = None
        navigation_ready: Callable[[], bool] | None = None
        previous_failure = None
        previous_route_failure = None
        node.declare_parameter("navigation_controller_params", "")
        nav_params = str(node.get_parameter("navigation_controller_params").value)
        node.declare_parameter("navigation_odometry_topic", "/wheel/odom")
        if reference is not None and nav_params and Path(nav_params).is_file():
            from senior_navigation.dedicated_controller import DedicatedController
            from senior_navigation.navigation_adapter import (
                RouteService,
                create_nav2_adapter,
                create_planner_readiness,
            )

            planner_active = create_planner_readiness(node)
            controller = DedicatedController(
                node,
                Path(nav_params),
                odometry_topic=str(node.get_parameter("navigation_odometry_topic").value),
            )

            def qualified(ref, generation, *, continuing=True):
                nonlocal previous_failure, previous_route_failure
                failure = None
                if exiting:
                    failure = "adapter exiting"
                elif reference is None or not reference.qualified(ref, generation):
                    failure = "map or localization generation changed"
                elif not planner_active() or not controller.available():
                    failure = "planner state unavailable"
                elif not node.task_motion_allowed(continuing=continuing):
                    failure = (
                        f"safety status unavailable ({reference.safety_reason} "
                        f"{reference.safety_ages})"
                    )
                elif not node.navigation_qualified():
                    failure = "local motion authority unavailable"
                if continuing:
                    if failure != previous_route_failure:
                        current = routes.current if routes is not None else None
                        if (
                            failure is not None
                            and current is not None
                            and current.status == "running"
                        ):
                            node.get_logger().warning(f"Route qualification lost: {failure}")
                        previous_route_failure = failure
                else:
                    previous_failure = failure
                return failure is None

            def can_navigate() -> bool:
                nonlocal previous_failure
                grid = reference.grid() if reference is not None else None
                if grid is None:
                    previous_failure = "map or localization unavailable"
                    return False
                return qualified(grid.map_ref, grid.grid_id, continuing=False)

            navigation_ready = can_navigate
            routes = RouteService(
                create_nav2_adapter(node),
                qualified=qualified,
                prepare=controller.open,
                bind_source=lambda execution_id, topic, guard: node.bind_navigation_source(
                    execution_id,
                    topic,
                    qualified=guard,
                    task_epoch=routes.current.task_epoch if routes.current else 0,
                ),
                clear_source=node.clear_navigation_source,
                complete_source=node.complete_navigation_source,
                source_active=node.navigation_source_matches,
                source_retired=node.navigation_source_retired,
                retire_source=node.retire_navigation_source,
                release_identity=node.request_terminal_release,
                collect_identity=node.collect_terminal,
                query_collection=node.query_terminal_collection,
            )

        core = AdapterCore(
            robot_id=robot_id,
            submit=node.submit,
            disconnect=node.disconnect,
            request_exit=request_exit,
            workspace_map=WorkspaceMap(reference.grid) if reference is not None else None,
            state_provider=state,
            pose_provider=reference.pose_reply if reference is not None else None,
            routes=routes,
            navigation_ready=navigation_ready,
            manual=node.manual_request,
            ros_domain=node.context.get_domain_id(),
        )

        async def serve() -> None:
            server = IpcServer(
                path,
                core.handle,
                replace_existing=node.get_parameter("ipc_replace_existing_socket").value,
            )
            started = False
            warmer = None
            ownership = AsyncExitStack()

            async def warm_controller() -> None:
                assert controller is not None
                while not exiting:
                    try:
                        if reference is not None and reference.pose() is not None:
                            await controller.warm()
                    except (RuntimeError, OSError) as exc:
                        node.get_logger().warning(f"Scoped Nav2 warm-up failed: {exc}")
                    await asyncio.sleep(2)

            try:
                owner = await ownership.enter_async_context(_launch_endpoint(path))
                if owner is not None and node.get_parameter("ipc_replace_existing_socket").value:
                    raise ValueError("launch endpoint requires replace_existing=false")
                await server.start()
                started = True
                if owner is not None:
                    from vehicle_core.ipc import _record

                    _record(path, run_id=owner)
                executor.add_node(node)
                if controller is not None:
                    warmer = asyncio.create_task(warm_controller())
                yielded_at = time.monotonic()
                last_stall_log = float("-inf")
                while rclpy.ok() and not exiting and not should_stop():
                    resumed_at = time.monotonic()
                    cpu_at = time.thread_time()
                    _spin_ready_callbacks(executor)
                    finished_at = time.monotonic()
                    callback_wall = finished_at - resumed_at
                    callback_cpu = time.thread_time() - cpu_at
                    resume_delay = resumed_at - yielded_at
                    if (
                        max(callback_wall, resume_delay) > 0.05
                        and finished_at - last_stall_log >= 1.0
                    ):
                        last_stall_log = finished_at
                        node.get_logger().warning(
                            "adapter_loop_stall "
                            f"callback_wall={callback_wall:.6f} "
                            f"callback_cpu={callback_cpu:.6f} "
                            f"resume_delay={resume_delay:.6f}"
                        )
                    yielded_at = finished_at
                    await asyncio.sleep(0.005)
                if routes is not None and routes.current is not None:
                    cancellation = asyncio.create_task(routes.cancel(routes.current.execution_id))
                    deadline = asyncio.get_running_loop().time() + 3.0
                    while not cancellation.done() and asyncio.get_running_loop().time() < deadline:
                        if rclpy.ok():
                            _spin_ready_callbacks(executor)
                        await asyncio.sleep(0.01)
                    cancellation.cancel()
                    await asyncio.gather(cancellation, return_exceptions=True)
            finally:
                if warmer is not None:
                    warmer.cancel()
                    await asyncio.gather(warmer, return_exceptions=True)
                if controller is not None:
                    await controller.close()
                try:
                    if started:
                        await server.close()
                finally:
                    try:
                        await ownership.aclose()
                    finally:
                        # End any active lease before node destruction.
                        core.release()

        asyncio.run(serve())

    # //Compatible: use the same Humble signal lifecycle as the synchronous CLIs.
    run(
        lambda: CommandNode(),
        cli=True,
        spin=spin,
        executor_factory=MotionExecutor,
    )
