"""Concrete management IPC bindings over the original injectable transport classes."""

import time
from collections.abc import Callable

from pydantic import ValidationError

from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.ipc import IpcClient, IpcError
from vehicle_core.values import MapRef, Pose2D
from vehicle_manager.agent.ipc_motion import IpcMotion as _IpcMotion
from vehicle_manager.agent.ipc_motion import TaskAwareMotion as _TaskAwareMotion
from vehicle_manager.agent.ipc_navigation import IpcMapPort as _IpcMapPort
from vehicle_manager.agent.ipc_state import IpcState as _IpcState
from vehicle_manager.agent.local_ports import MotionErrors
from vehicle_manager.agent.local_state import RuntimeState, StateReply
from vehicle_manager.agent.pose_reply import PoseReply
from vehicle_manager.agent.request_ports import Requester
from vehicle_manager.waypoints.home_capture import HomeCapture as _HomeCapture
from vehicle_manager.waypoints.ports import HomeWriter, WaypointValuePorts


class MotionRejected(IpcError):
    """The adapter explicitly rejected a command without executing it."""

    def __init__(self, kind: str, code: ErrorCode | None) -> None:
        super().__init__(f"ros side rejected {kind}")
        self.code = code


class IpcMotion(_IpcMotion):
    """//Compatible: original signature and exception classes, canonical inherited methods."""

    def __init__(
        self,
        client: IpcClient,
        *,
        intent_client: IpcClient,
        disable_client: IpcClient | None = None,
        safety_client: IpcClient | None = None,
    ) -> None:
        super().__init__(
            client,
            intent_client=intent_client,
            disable_client=disable_client,
            safety_client=safety_client,
            errors=MotionErrors(DomainError, IpcError, MotionRejected),
        )


class IpcMapPort(_IpcMapPort):
    """//Compatible: original request signature and concrete transport/domain errors."""

    def __init__(self, client: Requester) -> None:
        super().__init__(client, error=DomainError, ipc_error=IpcError)


class IpcState(_IpcState[RuntimeState]):
    """//Compatible: original DTO factories and read return type; no copied state."""

    def __init__(self, client: Requester, *, clock: Callable[[], float] = time.monotonic) -> None:
        super().__init__(
            client,
            parse_state=StateReply.model_validate,
            state_factory=RuntimeState,
            map_reference=MapRef,
            error=DomainError,
            ipc_error=IpcError,
            clock=clock,
        )


class IpcHomePort(_HomeCapture[MapRef, Pose2D]):
    """//Compatible: original constructor, state/parser and concrete local values."""

    def __init__(self, client: Requester, store: HomeWriter[Pose2D], *, robot_id: str) -> None:
        self._client = client
        super().__init__(
            lambda: client.request("pose.get"),
            store,
            robot_id=robot_id,
            state=IpcState(client),
            parse_pose=PoseReply.model_validate,
            values=WaypointValuePorts(map_reference=MapRef, pose=Pose2D, error=DomainError),
            transport_errors=(IpcError, OSError, ValueError, ValidationError),
        )


class TaskAwareMotion(_TaskAwareMotion, IpcMotion):
    """//Compatible: original constructor/MRO; task/manual methods have one manager owner."""
