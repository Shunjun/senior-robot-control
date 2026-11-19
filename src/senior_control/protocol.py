"""Local runtime interfaces and explicit shared WebSocket contract imports."""

from typing import Any, Literal

from pydantic import Field, model_serializer, model_validator

# //Compatible: local imports retain the same shared WebSocket contract objects.
from senior_contracts.wire import _MAX_SAFE_WEB_INTEGER as _MAX_SAFE_WEB_INTEGER
from senior_contracts.wire import DEVICE_REVOKED_CLOSE_CODE as DEVICE_REVOKED_CLOSE_CODE
from senior_contracts.wire import HEARTBEAT_INTERVAL_MS as HEARTBEAT_INTERVAL_MS
from senior_contracts.wire import HEARTBEAT_WINDOW_SECONDS as HEARTBEAT_WINDOW_SECONDS
from senior_contracts.wire import MAX_CANDIDATE_CHARS as MAX_CANDIDATE_CHARS
from senior_contracts.wire import MAX_MESSAGE_BYTES as MAX_MESSAGE_BYTES
from senior_contracts.wire import MAX_SDP_CHARS as MAX_SDP_CHARS
from senior_contracts.wire import OPERATION_TYPES as OPERATION_TYPES
from senior_contracts.wire import WORKSPACE_REQUEST_TYPES as WORKSPACE_REQUEST_TYPES
from senior_contracts.wire import CandidatePayload as CandidatePayload
from senior_contracts.wire import EmptyPayload as EmptyPayload
from senior_contracts.wire import Envelope as Envelope
from senior_contracts.wire import EnvelopeBase as EnvelopeBase
from senior_contracts.wire import Frame as Frame
from senior_contracts.wire import HeartbeatRequest as HeartbeatRequest
from senior_contracts.wire import HomePayload as HomePayload
from senior_contracts.wire import LearningActiveModel as LearningActiveModel
from senior_contracts.wire import LearningStartEnvelope as LearningStartEnvelope
from senior_contracts.wire import LearningStartPayload as LearningStartPayload
from senior_contracts.wire import LearningStartReceipt as LearningStartReceipt
from senior_contracts.wire import LearningStatus as LearningStatus
from senior_contracts.wire import LearningStatusEnvelope as LearningStatusEnvelope
from senior_contracts.wire import LearningStatusPayload as LearningStatusPayload
from senior_contracts.wire import ManualIntentEnvelope as ManualIntentEnvelope
from senior_contracts.wire import ManualIntentPayload as ManualIntentPayload
from senior_contracts.wire import MapRefPayload as MapRefPayload
from senior_contracts.wire import OperationEnvelope as OperationEnvelope
from senior_contracts.wire import OperationPayload as OperationPayload
from senior_contracts.wire import PreparedAuthorityPayload as PreparedAuthorityPayload
from senior_contracts.wire import RecoveryEnvelope as RecoveryEnvelope
from senior_contracts.wire import RecoveryPayload as RecoveryPayload
from senior_contracts.wire import ResponseEnvelope as ResponseEnvelope
from senior_contracts.wire import ResponsePayload as ResponsePayload
from senior_contracts.wire import RobotPosePayload as RobotPosePayload
from senior_contracts.wire import ScanPointPayload as ScanPointPayload
from senior_contracts.wire import SdpPayload as SdpPayload
from senior_contracts.wire import SettingsApplyEnvelope as SettingsApplyEnvelope
from senior_contracts.wire import SettingsApplyPayload as SettingsApplyPayload
from senior_contracts.wire import SettingsEnvelope as SettingsEnvelope
from senior_contracts.wire import SettingsEnvelopeBase as SettingsEnvelopeBase
from senior_contracts.wire import SettingsGetEnvelope as SettingsGetEnvelope
from senior_contracts.wire import SettingsGetPayload as SettingsGetPayload
from senior_contracts.wire import SettingsResultEnvelope as SettingsResultEnvelope
from senior_contracts.wire import SettingsResultPayload as SettingsResultPayload
from senior_contracts.wire import ShortText as ShortText
from senior_contracts.wire import SimpleEnvelope as SimpleEnvelope
from senior_contracts.wire import SnapshotEnvelope as SnapshotEnvelope
from senior_contracts.wire import SnapshotPayload as SnapshotPayload
from senior_contracts.wire import StrictModel as StrictModel
from senior_contracts.wire import TaskCommandReceiptPayload as TaskCommandReceiptPayload
from senior_contracts.wire import TaskExecutionCommandPayload as TaskExecutionCommandPayload
from senior_contracts.wire import TaskExecutionEnvelope as TaskExecutionEnvelope
from senior_contracts.wire import TaskNavigationState as TaskNavigationState
from senior_contracts.wire import TaskSummaryPayload as TaskSummaryPayload
from senior_contracts.wire import VideoCandidateEnvelope as VideoCandidateEnvelope
from senior_contracts.wire import VideoCloseEnvelope as VideoCloseEnvelope
from senior_contracts.wire import VideoEnvelope as VideoEnvelope
from senior_contracts.wire import VideoSdpEnvelope as VideoSdpEnvelope
from senior_contracts.wire import WorkspaceEnvelope as WorkspaceEnvelope
from senior_contracts.wire import WorkspacePayload as WorkspacePayload
from senior_contracts.wire import WorkspaceRequest as WorkspaceRequest
from senior_contracts.wire import _adapter as _adapter
from senior_contracts.wire import _frame_adapter as _frame_adapter
from senior_contracts.wire import serialize_envelope as serialize_envelope
from senior_contracts.wire import validate_envelope as validate_envelope
from senior_contracts.wire import validate_frame as validate_frame


class BodyReferenceState(StrictModel):
    average: dict[Literal["vx", "vy", "wz"], float]
    source: str = Field(min_length=1, max_length=128)
    computed_steady: float
    valid_until_steady: float

    @model_validator(mode="after")
    def complete(self) -> "BodyReferenceState":
        if set(self.average) != {"vx", "vy", "wz"}:
            raise ValueError("independent body average needs all components")
        if self.valid_until_steady <= self.computed_steady:
            raise ValueError("body reference interval is invalid")
        return self


class RuntimeObservation(StrictModel):
    """Reporting metadata only: never an input to motion admission or recovery."""

    status: Literal["current", "missing", "revoked", "ipc_unavailable"]
    last_sampled_steady: float | None = None


class MotionRuntimeState(StrictModel):
    """Trusted local /motion/runtime JSON; never accepted from a control socket."""

    schema_version: Literal[1] = Field(alias="schema")
    robot_id: str = Field(min_length=1, max_length=128)
    boot_id: str = Field(min_length=1, max_length=128)
    sequence: int = Field(ge=0, le=_MAX_SAFE_WEB_INTEGER)
    sampled_steady: float
    mode: Literal[
        "idle",
        "manual",
        "task",
        "braking",
        "obstacle_hold",
        "feedback_stop",
        "execution_fault",
        "estop",
    ]
    reason: str = Field(max_length=512)
    stationary: bool
    healthy: bool
    evidence_stamp_steady: float | None
    evidence_deadline_steady: float | None
    estop_revision: int = Field(ge=0)
    fault_revision: int = Field(ge=0)
    confirmed_estop_status: Literal["clear", "software_latched"]
    allowed_actions: list[
        Literal["acquire", "estop_reset", "fault_recover", "replan_task", "resume_task"]
    ]
    task_id: str | None = Field(default=None, max_length=128)
    output_generation: int = Field(ge=0)
    body_reference: BodyReferenceState | None
    wheel_linear_mps: float | None

    @model_serializer(mode="wrap")
    def serialize_schema(self, handler: Any) -> dict[str, Any]:
        value: dict[str, Any] = handler(self)
        if "schema_version" in value:
            value["schema"] = value.pop("schema_version")
        return value

    @model_validator(mode="after")
    def evidence_window(self) -> "MotionRuntimeState":
        if (self.evidence_stamp_steady is None) != (self.evidence_deadline_steady is None):
            raise ValueError("runtime evidence interval must be paired")
        if (
            self.evidence_stamp_steady is not None
            and self.evidence_deadline_steady is not None
            and self.evidence_deadline_steady <= self.evidence_stamp_steady
        ):
            raise ValueError("runtime evidence interval is invalid")
        return self
