"""//Compatible: original display values and motion enum at the projection boundary."""

from senior_motion_control.contracts import MotionMode

from vehicle_manager.agent.vehicle_state import (
    ConnectionState as ConnectionState,
)
from vehicle_manager.agent.vehicle_state import (
    RuntimeSnapshot as RuntimeSnapshot,
)
from vehicle_manager.agent.vehicle_state import (
    VehicleState as VehicleState,
)
from vehicle_manager.agent.vehicle_state import (
    _estop_overlay as _estop_overlay,
)
from vehicle_manager.agent.vehicle_state import (
    project_vehicle_state as _project_vehicle_state,
)


def project_vehicle_state(
    runtime: RuntimeSnapshot,
    connection: ConnectionState,
    *,
    now_steady: float | None = None,
    software_estop_pending: bool = False,
    previous: VehicleState | None = None,
) -> VehicleState:
    return _project_vehicle_state(
        runtime,
        connection,
        estop_mode=MotionMode.ESTOP,
        now_steady=now_steady,
        software_estop_pending=software_estop_pending,
        previous=previous,
    )
