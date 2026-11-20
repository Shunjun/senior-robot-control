"""//Compatible: original local route imports refer to the same canonical objects."""

from senior_navigation.route import MAX_ROUTE_WAYPOINTS as MAX_ROUTE_WAYPOINTS
from senior_navigation.route import QualifiedRoute as QualifiedRoute
from senior_navigation.route import RoutePlan as RoutePlan
from senior_navigation.route import RouteProgress as RouteProgress
from senior_navigation.route import RouteRejected as RouteRejected
from senior_navigation.route import RouteStatus as RouteStatus

from senior_control.errors import DomainError as DomainError
from senior_control.errors import ErrorCode as ErrorCode
from senior_control.models import MapRef as MapRef
from senior_control.models import Pose2D as Pose2D
