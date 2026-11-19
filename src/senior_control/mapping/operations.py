"""//Compatible: historical imports forward the canonical map implementation."""

from pathlib import Path

from senior_mapping.archive import MapArchive
from senior_mapping.operations import _GRAPH_WAIT_SECONDS as _GRAPH_WAIT_SECONDS
from senior_mapping.operations import OPERATION_JOURNAL_STATEMENTS as OPERATION_JOURNAL_STATEMENTS
from senior_mapping.operations import Checkpoint as Checkpoint
from senior_mapping.operations import HomeLink as HomeLink
from senior_mapping.operations import MapOperations as _CanonicalMapOperations
from senior_mapping.operations import MotionGate as MotionGate
from senior_mapping.operations import OperationRecord as OperationRecord
from senior_mapping.operations import PoseGraphLoader as PoseGraphLoader
from senior_mapping.operations import PoseGraphVerifier as PoseGraphVerifier
from senior_mapping.operations import ProcessCrash as ProcessCrash
from senior_mapping.operations import QuiescenceFence as QuiescenceFence
from senior_mapping.operations import TaskLink as TaskLink
from senior_mapping.operations import _check_owned_path as _check_owned_path
from senior_mapping.operations import _closed as _closed
from senior_mapping.operations import _consume_task_exception as _consume_task_exception
from senior_mapping.operations import _copy_tree as _copy_tree
from senior_mapping.operations import _flag as _flag
from senior_mapping.operations import _history_dir as _history_dir
from senior_mapping.operations import _operation_id as _operation_id
from senior_mapping.operations import _owned_path as _owned_path
from senior_mapping.operations import _package_digest as _package_digest
from senior_mapping.operations import _payload as _payload
from senior_mapping.operations import _StoredOperation as _StoredOperation
from senior_mapping.operations import _text as _text
from senior_mapping.operations import _whole as _whole
from senior_mapping.store import MapReferenceStore

from senior_control.storage.database import Database
from senior_control.storage.map_persistence import map_persistence


class MapOperations(_CanonicalMapOperations):
    """//Compatible: old constructor injects management hooks; all business methods are inherited.

    This owner class differs from the canonical class; shared value classes do not.
    """

    def __init__(
        self,
        database: Database,
        store: MapReferenceStore,
        archive: MapArchive,
        motion: MotionGate,
        *,
        robot_id: str,
        map_root: Path,
        config_dir: Path,
        loader: PoseGraphLoader | None = None,
        checkpoint: Checkpoint | None = None,
        fault_after: str | None = None,
    ) -> None:
        super().__init__(
            database,
            store,
            archive,
            motion,
            persistence=map_persistence(database),
            robot_id=robot_id,
            map_root=map_root,
            config_dir=config_dir,
            loader=loader,
            checkpoint=checkpoint,
            fault_after=fault_after,
        )
