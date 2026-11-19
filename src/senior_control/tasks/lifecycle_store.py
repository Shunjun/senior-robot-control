"""//Compatible: explicit aliases of the sole manager lifecycle implementation."""

from vehicle_manager.storage.agent_schema import (
    AGENT_JOURNAL_STATEMENTS as AGENT_JOURNAL_STATEMENTS,
)
from vehicle_manager.tasks.lifecycle_store import MAX_EFFECT_BATCH as MAX_EFFECT_BATCH
from vehicle_manager.tasks.lifecycle_store import SCHEMA as SCHEMA
from vehicle_manager.tasks.lifecycle_store import Admission as Admission
from vehicle_manager.tasks.lifecycle_store import ContextProvider as ContextProvider
from vehicle_manager.tasks.lifecycle_store import EffectPage as EffectPage
from vehicle_manager.tasks.lifecycle_store import EffectStatus as EffectStatus
from vehicle_manager.tasks.lifecycle_store import LifecycleStore as LifecycleStore
from vehicle_manager.tasks.lifecycle_store import RequestConflict as RequestConflict
from vehicle_manager.tasks.lifecycle_store import StaleEffect as StaleEffect
from vehicle_manager.tasks.lifecycle_store import StoredEffect as StoredEffect
from vehicle_manager.tasks.lifecycle_store import T as T
from vehicle_manager.tasks.lifecycle_store import VersionConflict as VersionConflict
from vehicle_manager.tasks.lifecycle_store import _decode as _decode
from vehicle_manager.tasks.lifecycle_store import _effect as _effect
from vehicle_manager.tasks.lifecycle_store import _json as _json
from vehicle_manager.tasks.lifecycle_store import _json_default as _json_default
from vehicle_manager.tasks.models import ExecutionRecord as ExecutionRecord
from vehicle_manager.tasks.models import TaskDefinition as TaskDefinition

from senior_control.models import Pose2D as Pose2D
from senior_control.tasks.store import TaskStore as TaskStore
