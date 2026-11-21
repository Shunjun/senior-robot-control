"""//Compatible: original concrete values/stores/helpers are the same manager objects."""

from vehicle_manager.storage.bindings import Database as Database
from vehicle_manager.storage.domain_bindings import StoredWaypoint as StoredWaypoint
from vehicle_manager.storage.domain_bindings import Waypoint as Waypoint
from vehicle_manager.storage.domain_bindings import WaypointStore as WaypointStore
from vehicle_manager.storage.domain_bindings import (
    adopted_home_blocks_foreign as adopted_home_blocks_foreign,
)
from vehicle_manager.waypoints.store import WAYPOINT_STATEMENTS as WAYPOINT_STATEMENTS
from vehicle_manager.waypoints.store import replace_imported_home as replace_imported_home

from vehicle_core.errors import DomainError as DomainError
from vehicle_core.values import MapRef as MapRef
from vehicle_core.values import Pose2D as Pose2D
