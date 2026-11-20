"""//Compatible: original local route imports refer to the same canonical objects."""

from senior_navigation.route import MAX_ROUTE_WAYPOINTS as MAX_ROUTE_WAYPOINTS
from senior_navigation.route import QualifiedRoute as QualifiedRoute
from senior_navigation.route import RouteProgress as RouteProgress
from senior_navigation.route import RouteStatus as RouteStatus

from vehicle_core.errors import DomainError as DomainError
from vehicle_core.errors import ErrorCode as ErrorCode
from vehicle_core.route_plan import RoutePlan as RoutePlan
from vehicle_core.route_plan import RouteRejected as RouteRejected
from vehicle_core.values import MapRef as MapRef
from vehicle_core.values import Pose2D as Pose2D
