"""//Compatible: historical imports forward to the sole cloud implementation."""

import asyncio as asyncio
import time as time
from collections.abc import Callable

from cloud.limits import CloudLimits
from cloud.relay import AUTHORITY_TYPES as AUTHORITY_TYPES
from cloud.relay import CLOSE_BAD_FRAME as CLOSE_BAD_FRAME
from cloud.relay import CLOSE_NORMAL as CLOSE_NORMAL
from cloud.relay import CLOSE_OVERLOAD as CLOSE_OVERLOAD
from cloud.relay import CLOSE_POLICY as CLOSE_POLICY
from cloud.relay import CLOSE_SUPERSEDED as CLOSE_SUPERSEDED
from cloud.relay import CLOSE_TOO_BIG as CLOSE_TOO_BIG
from cloud.relay import CONNECTION as CONNECTION
from cloud.relay import PROTECTED_TYPES as PROTECTED_TYPES
from cloud.relay import RECONNECT_GRACE_SECONDS as RECONNECT_GRACE_SECONDS
from cloud.relay import SAFETY_TYPES as SAFETY_TYPES
from cloud.relay import SNAPSHOT as SNAPSHOT
from cloud.relay import SNAPSHOT_TTL_SECONDS as SNAPSHOT_TTL_SECONDS
from cloud.relay import VIDEO as VIDEO
from cloud.relay import VIDEO_CLOSE as VIDEO_CLOSE
from cloud.relay import VIDEO_CLOSE_TTL_SECONDS as VIDEO_CLOSE_TTL_SECONDS
from cloud.relay import WORKSPACE as WORKSPACE
from cloud.relay import WORKSPACE_REPLY_SECONDS as WORKSPACE_REPLY_SECONDS
from cloud.relay import Connection as Connection
from cloud.relay import Hub as _Hub
from cloud.relay import Outbox as _Outbox
from cloud.relay import _Link as _Link
from cloud.relay import _pump_in as _pump_in
from cloud.relay import _pump_out as _pump_out
from cloud.relay import _settings_frame as _settings_frame
from cloud.relay import _watch_heartbeat as _watch_heartbeat
from cloud.relay import _WorkspaceRequest as _WorkspaceRequest
from cloud.relay import logger as logger
from cloud.relay import serve_connection as serve_connection
from cloud.snapshots import SnapshotCache

from senior_control.runtime.diagnostics import DiagnosticDeque


class Outbox(_Outbox):
    """//Compatible: original constructor; business implementation is cloud.relay.Outbox."""

    def __init__(
        self,
        limit: int,
        *,
        on_discard: Callable[[str, bytes], None] | None = None,
        diagnostic_name: str = "cloud.transport_outbox",
    ) -> None:
        super().__init__(
            limit,
            on_discard=on_discard,
            diagnostic_name=diagnostic_name,
            deque_factory=DiagnosticDeque,
        )


class Hub(_Hub):
    """//Compatible: original constructor with the existing process diagnostic owner."""

    def __init__(self, limits: CloudLimits, snapshots: SnapshotCache | None = None) -> None:
        super().__init__(limits, snapshots, deque_factory=DiagnosticDeque)
