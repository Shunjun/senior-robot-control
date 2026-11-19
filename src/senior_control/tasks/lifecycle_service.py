"""//Compatible: explicit aliases of the sole manager lifecycle implementation."""

from vehicle_manager.tasks.lifecycle_service import LifecyclePort as LifecyclePort
from vehicle_manager.tasks.lifecycle_service import LifecycleService as LifecycleService
from vehicle_manager.tasks.lifecycle_service import ReconcileResult as ReconcileResult
from vehicle_manager.tasks.lifecycle_store import MAX_EFFECT_BATCH as MAX_EFFECT_BATCH
from vehicle_manager.tasks.lifecycle_store import ContextProvider as ContextProvider
from vehicle_manager.tasks.lifecycle_store import LifecycleStore as LifecycleStore
from vehicle_manager.tasks.lifecycle_store import StaleEffect as StaleEffect
from vehicle_manager.tasks.lifecycle_store import VersionConflict as VersionConflict
from vehicle_manager.tasks.models import TaskDefinition as TaskDefinition

from senior_control.models import Pose2D as Pose2D
