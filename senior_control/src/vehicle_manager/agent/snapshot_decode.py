"""//Compatible: original local values and snapshot entry points."""

from senior_contracts.wire import SnapshotEnvelope

from vehicle_core.values import MapRef
from vehicle_manager.agent.snapshots import (
    HomeSummary as HomeSummary,
)
from vehicle_manager.agent.snapshots import (
    LiveState as LiveState,
)
from vehicle_manager.agent.snapshots import (
    RobotSnapshot as RobotSnapshot,
)
from vehicle_manager.agent.snapshots import (
    StatePort as StatePort,
)
from vehicle_manager.agent.snapshots import (
    TaskSummary as TaskSummary,
)
from vehicle_manager.agent.snapshots import (
    _map_payload as _map_payload,
)
from vehicle_manager.agent.snapshots import (
    snapshot_envelope as snapshot_envelope,
)
from vehicle_manager.agent.snapshots import (
    snapshot_from_envelope as _snapshot_from_envelope,
)


def snapshot_from_envelope(envelope: SnapshotEnvelope) -> RobotSnapshot:
    """//Compatible: decode with the original local MapRef constructor."""
    return _snapshot_from_envelope(envelope, map_reference=MapRef)
