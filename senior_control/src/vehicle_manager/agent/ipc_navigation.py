"""Existing map/navigation request and acknowledgement rules, without execution authority."""

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from pydantic import ValidationError
from senior_contracts.errors import ErrorCode
from senior_contracts.navigation import RouteProgress

from vehicle_manager.agent.local_ports import NavigationValues
from vehicle_manager.agent.request_ports import Requester
from vehicle_manager.tasks.lifecycle import Effect
from vehicle_manager.tasks.models import TaskDefinition
from vehicle_manager.tasks.ports import MapReference
from vehicle_manager.tasks.store import TaskStore
from vehicle_manager.waypoints.store import WaypointStore


class IpcMapPort:
    def __init__(self, client: Requester, *, error: type[Any], ipc_error: type[Exception]) -> None:
        self._error = error
        self._ipc_error = ipc_error
        self._client = client

    async def _call(self, kind: str, **fields: Any) -> None:
        try:
            reply = await self._client.request(kind, **fields)
        except (self._ipc_error, OSError) as exc:
            raise self._error(ErrorCode.UNAVAILABLE, "ROS map service unavailable") from exc
        if reply.get("ok") is not True:
            raise self._error(ErrorCode.UNAVAILABLE, "ROS map service unavailable")

    async def export(self, operation_id: str, map_ref: MapReference, destination: Path) -> None:
        await self._call(
            "map.export",
            operation_id=operation_id,
            map_id=map_ref.map_id,
            reference_revision=map_ref.reference_revision,
            path=str(destination),
        )

    async def import_(self, operation_id: str, path: Path) -> None:
        await self._call("map.import", operation_id=operation_id, path=str(path))

    async def clear(self, operation_id: str, expected: MapReference) -> None:
        await self._call(
            "map.clear",
            operation_id=operation_id,
            map_id=expected.map_id,
            reference_revision=expected.reference_revision,
        )

    async def manage(self, kind: str, operation_id: str, body: Mapping[str, Any]) -> None:
        try:
            reply = await self._client.request(kind, operation_id=operation_id, **body)
        except (self._ipc_error, OSError) as exc:
            raise self._error(
                ErrorCode.OPERATION_UNKNOWN, "map operation acknowledgement lost"
            ) from exc
        if reply.get("ok") is not True:
            try:
                code = ErrorCode(reply.get("error"))
            except (ValueError, TypeError):
                code = ErrorCode.UNAVAILABLE
            raise self._error(code, "map operation rejected")
        if reply.get("operation_id") != operation_id:
            raise self._error(ErrorCode.OPERATION_UNKNOWN, "map acknowledgement identity changed")


class IpcNavigation:
    """Only a matching explicit acknowledgement counts as an accepted action."""

    def __init__(
        self,
        client: Requester,
        waypoints: WaypointStore[Any, Any, Any] | None = None,
        *,
        tasks: TaskStore | None = None,
        values: NavigationValues,
    ) -> None:
        self._values = values
        self._client = client
        self._waypoints = waypoints
        self._tasks = tasks

    async def _call(self, kind: str, execution_id: str, **fields: Any) -> dict[str, Any]:
        try:
            reply = await self._client.request(kind, execution_id=execution_id, **fields)
        except (self._values.ipc_error, OSError) as exc:
            raise self._values.error(ErrorCode.UNAVAILABLE, "ROS navigation unavailable") from exc
        if (
            kind == "navigation.task_start"
            and reply.get("ok") is False
            and reply.get("error") == "rejected"
            and reply.get("execution_id") == execution_id
        ):
            raise self._values.route_rejected()
        if reply.get("ok") is not True or reply.get("execution_id") != execution_id:
            raise self._values.error(ErrorCode.UNAVAILABLE, "ROS navigation unavailable")
        return reply

    async def task_poses(self, task: TaskDefinition) -> tuple[Any, ...]:
        if self._waypoints is None:
            raise self._values.route_rejected()
        return await self._waypoints.route_poses(task.map_ref, task.waypoint_ids)

    async def task_effect(self, effect: Effect, task: TaskDefinition) -> bool | None:
        if effect.kind == "plan":
            if effect.map_ref is None:
                raise self._values.route_rejected()
            direct = await self._tasks.execution_route(effect.task_id) if self._tasks else None
            if direct is None:
                raise self._values.route_rejected()
            poses = tuple(
                self._values.pose(*direct[int(action.split(":", 1)[0])])
                for action in effect.remaining
            )
            plan = self._values.route_plan(
                execution_id=effect.task_id,
                task_id=task.task_id,
                map_id=task.map_ref.map_id,
                reference_revision=task.map_ref.reference_revision,
                map_generation=effect.map_ref,
                poses=[[p.x, p.y, p.yaw] for p in poses],
                task_epoch=effect.epoch,
                effect_id=effect.effect_id,
                action_ids=list(effect.remaining),
                pose_ref=effect.pose_ref,
            )
            reply = await self._call(
                "navigation.task_start",
                effect.task_id,
                plan=plan.model_dump(),
                effect_sequence=effect.dispatch_sequence,
                settled_through=effect.settled_through,
            )
        elif effect.kind in {"terminal_release", "terminal_settled"}:
            reply = await self._call(
                "navigation.task_settled"
                if effect.kind == "terminal_settled"
                else "navigation.task_terminal",
                effect.task_id,
                effect_id=effect.effect_id,
                task_epoch=effect.epoch,
                terminal_outcome=effect.terminal_outcome,
                effect_sequence=effect.dispatch_sequence,
                settled_through=effect.settled_through,
            )
        else:
            reply = await self._call(
                "navigation.task_cancel",
                effect.task_id,
                effect_sequence=effect.dispatch_sequence,
                settled_through=effect.settled_through,
                effect_id=effect.effect_id,
                task_epoch=effect.epoch,
            )
        return self._effect_result(reply, effect)

    async def task_effect_status(self, effect: Effect) -> bool | None:
        reply = await self._call(
            "navigation.task_settled"
            if effect.kind == "terminal_settled"
            else "navigation.task_status",
            effect.task_id,
            effect_sequence=effect.dispatch_sequence,
            settled_through=effect.settled_through,
            effect_id=effect.effect_id,
            task_epoch=effect.epoch,
            effect_kind=effect.kind,
            terminal_outcome=effect.terminal_outcome,
        )
        return self._effect_result(reply, effect)

    @staticmethod
    def _effect_result(reply: dict[str, Any], effect: Effect) -> bool | None:
        if (
            reply.get("lifecycle_version") != 2
            or reply.get("effect_id") != effect.effect_id
            or reply.get("task_epoch") != effect.epoch
        ):
            return None
        if reply.get("execution_id") != effect.task_id:
            return None
        if effect.kind == "terminal_settled":
            return (
                True if reply.get("collected") is True and reply.get("accepted") is True else None
            )
        if effect.kind == "terminal_release":
            return (
                True
                if (
                    reply.get("terminal_outcome") == effect.terminal_outcome
                    and reply.get("identity_released") is True
                    and reply.get("accepted") is True
                )
                else None
            )
        if effect.kind == "revoke":
            return (
                True
                if (
                    (reply.get("sources_fenced_through_epoch") or -1) >= effect.epoch
                    and reply.get("controller_output_exited") is True
                )
                else None
            )
        result = reply.get("accepted")
        return result if isinstance(result, bool) else None

    async def status(self, execution_id: str) -> RouteProgress | None:
        reply = await self._call("navigation.task_status", execution_id)
        return self._progress(reply)

    async def current(self) -> RouteProgress | None:
        try:
            reply = await self._client.request("navigation.current")
        except (self._values.ipc_error, OSError) as exc:
            raise self._values.error(ErrorCode.UNAVAILABLE, "ROS navigation unavailable") from exc
        if reply.get("ok") is not True:
            raise self._values.error(ErrorCode.UNAVAILABLE, "ROS navigation unavailable")
        return self._progress(reply)

    def _progress(self, reply: dict[str, Any]) -> RouteProgress | None:
        data = reply.get("progress")
        if data is None:
            return None
        try:
            return RouteProgress.model_validate(data)
        except ValidationError as exc:
            raise self._values.error(
                ErrorCode.UNAVAILABLE, "invalid ROS navigation status"
            ) from exc
