"""//Compatible: original motion constructors over the sole management request implementation."""

from vehicle_manager.agent.connection import HeartbeatFrame as HeartbeatFrame
from vehicle_manager.agent.connection import Link as Link
from vehicle_manager.agent.connection import MotionPort as MotionPort
from vehicle_manager.agent.ipc_motion import IpcMotion as _IpcMotion
from vehicle_manager.agent.local_ports import MotionErrors

from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.ipc import IpcClient, IpcError


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
