"""Versioned, bounded remote control wire format."""

from typing import Annotated, Any, Literal, get_args

from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    PrivateAttr,
    TypeAdapter,
    ValidationError,
    model_serializer,
    model_validator,
)

from senior_contracts.errors import ErrorCode
from senior_contracts.navigation import RouteProgress

HEARTBEAT_WINDOW_SECONDS = 5.0
HEARTBEAT_INTERVAL_MS = 1000.0
DEVICE_REVOKED_CLOSE_CODE = 4403
_MAX_SAFE_WEB_INTEGER = 2**53 - 1


class StrictModel(BaseModel):
    model_config = ConfigDict(
        extra="forbid",
        allow_inf_nan=False,
        strict=True,
        revalidate_instances="always",
        hide_input_in_errors=True,
    )


class HeartbeatRequest(StrictModel):
    v: Literal[1]
    type: Literal["heartbeat.request"]
    robot_id: str = Field(min_length=1, max_length=128)
    connection_epoch: int = Field(ge=1)
    nonce: str = Field(min_length=1, max_length=128)
    sent_at: float


class EmptyPayload(StrictModel):
    authority_id: str | None = Field(default=None, min_length=1, max_length=128)
    # //Compatible: this optional shape is part of persisted emergency-intent fingerprints.
    gesture: int | None = Field(default=None, ge=1, le=_MAX_SAFE_WEB_INTEGER)

    @model_serializer(mode="wrap")
    def serialize_authority(self, handler: Any) -> dict[str, Any]:
        value: dict[str, Any] = handler(self)
        if self.authority_id is None:
            value.pop("authority_id", None)
        return value


class EnvelopeBase(StrictModel):
    _received_steady: float | None = PrivateAttr(default=None)
    v: Literal[1]
    robot_id: str = Field(min_length=1, max_length=128)
    session_id: str = Field(min_length=1, max_length=128)
    connection_epoch: int = Field(ge=0)
    request_id: str = Field(min_length=1, max_length=128)
    seq: int = Field(ge=0, le=2**63 - 1)
    expires_at: float


class SimpleEnvelope(EnvelopeBase):
    type: Literal[
        "control.acquire",
        "control.release",
        "drive.stop",
        "safety.estop",
    ]
    payload: EmptyPayload
    controller_generation: int | None = Field(default=None, ge=1, le=_MAX_SAFE_WEB_INTEGER)

    @model_serializer(mode="wrap")
    def serialize_context(self, handler: Any) -> dict[str, Any]:
        value: dict[str, Any] = handler(self)
        if self.controller_generation is None:
            value.pop("controller_generation", None)
        return value

    @model_validator(mode="after")
    def validate_gesture(self) -> "SimpleEnvelope":
        if self.payload.gesture is not None and self.type != "drive.stop":
            raise ValueError("gesture is only supported for drive.stop")
        if self.payload.authority_id is not None and self.type not in {
            "control.release",
            "drive.stop",
        }:
            raise ValueError("authority id is only supported for control.release and drive.stop")
        if (
            self.type == "drive.stop"
            and self.payload.authority_id is not None
            and self.payload.gesture is None
        ):
            raise ValueError("scoped stop requires a gesture")
        if self.controller_generation is not None and self.type not in {
            "control.acquire",
            "control.release",
            "drive.stop",
        }:
            raise ValueError("trusted controller context belongs only to manual requests")
        return self


class ManualIntentPayload(StrictModel):
    authority_id: str = Field(min_length=1, max_length=128)
    gesture: int = Field(ge=1, le=_MAX_SAFE_WEB_INTEGER)
    sequence: int = Field(ge=1, le=_MAX_SAFE_WEB_INTEGER)
    linear: float = Field(ge=-1, le=1)
    turn: float = Field(ge=-1, le=1)
    gear: Literal["low", "high"]


class ManualIntentEnvelope(EnvelopeBase):
    type: Literal["drive.intent"]
    payload: ManualIntentPayload
    controller_generation: int | None = Field(default=None, ge=1, le=_MAX_SAFE_WEB_INTEGER)

    @model_serializer(mode="wrap")
    def serialize_context(self, handler: Any) -> dict[str, Any]:
        value: dict[str, Any] = handler(self)
        if self.controller_generation is None:
            value.pop("controller_generation", None)
        return value


class RecoveryPayload(StrictModel):
    revision: int = Field(ge=0, le=_MAX_SAFE_WEB_INTEGER)


class RecoveryEnvelope(EnvelopeBase):
    type: Literal["safety.estop.reset", "safety.fault.recover"]
    payload: RecoveryPayload


class SettingsApplyPayload(StrictModel):
    request_id: str = Field(min_length=1, max_length=128)
    expected_version: int = Field(ge=0, le=_MAX_SAFE_WEB_INTEGER)
    group: Literal["driving", "logging"]
    values: dict[str, Any]
    restore_defaults: bool = False


class SettingsResultPayload(StrictModel):
    request_id: str = Field(min_length=1, max_length=128)


class SettingsGetPayload(StrictModel):
    pass


class SettingsEnvelopeBase(EnvelopeBase):
    controller_generation: int | None = Field(default=None, ge=1, le=_MAX_SAFE_WEB_INTEGER)

    @model_serializer(mode="wrap")
    def serialize_context(self, handler: Any) -> dict[str, Any]:
        value: dict[str, Any] = handler(self)
        if self.controller_generation is None:
            value.pop("controller_generation", None)
        return value


class SettingsGetEnvelope(SettingsEnvelopeBase):
    type: Literal["settings.get"]
    payload: SettingsGetPayload


class SettingsApplyEnvelope(SettingsEnvelopeBase):
    type: Literal["settings.apply"]
    payload: SettingsApplyPayload


class SettingsResultEnvelope(SettingsEnvelopeBase):
    type: Literal["settings.result"]
    payload: SettingsResultPayload


SettingsEnvelope = SettingsGetEnvelope | SettingsApplyEnvelope | SettingsResultEnvelope


class LearningStartPayload(StrictModel):
    expected_active_revision: int = Field(ge=0, le=_MAX_SAFE_WEB_INTEGER)
    expected_authority_id: str | None = Field(min_length=1, max_length=128)


class LearningStatusPayload(StrictModel):
    attempt_id: str | None = Field(default=None, pattern=r"^[0-9a-f]{32}$")
    start_request_id: str | None = Field(default=None, min_length=1, max_length=128)

    @model_validator(mode="after")
    def one_identity(self) -> "LearningStatusPayload":
        if self.attempt_id is not None and self.start_request_id is not None:
            raise ValueError("learning_query_identity")
        return self


class LearningStartEnvelope(SettingsEnvelopeBase):
    type: Literal["learning.start"]
    payload: LearningStartPayload


class LearningStatusEnvelope(SettingsEnvelopeBase):
    type: Literal["learning.status"]
    payload: LearningStatusPayload


class LearningStartReceipt(StrictModel):
    start_request_id: str = Field(min_length=1, max_length=128)
    attempt_id: str = Field(pattern=r"^[0-9a-f]{32}$")
    state: Literal["accepted", "busy"]


class LearningActiveModel(StrictModel):
    attempt_id: str = Field(pattern=r"^[0-9a-f]{32}$")
    model_id: str = Field(pattern=r"^[0-9a-f]{32}$")
    point_count: int = Field(ge=0, le=2048)


class LearningStatus(StrictModel):
    run_id: str = Field(pattern=r"^[0-9a-f]{32}$")
    active_revision: int = Field(ge=0, le=_MAX_SAFE_WEB_INTEGER)
    active_model: LearningActiveModel | None
    attempt_id: str = Field(pattern=r"^[0-9a-f]{32}$")
    phase: str = Field(min_length=1, max_length=64)
    reason: str = Field(max_length=256)
    motion_complete: bool
    capture_complete: bool
    compute_complete: bool
    durable: bool
    can_start: bool
    cannot_start_reason: str | None = Field(max_length=256)


class TaskExecutionCommandPayload(StrictModel):
    request_id: str = Field(min_length=1, max_length=128)
    expected_version: int = Field(ge=0, le=_MAX_SAFE_WEB_INTEGER)
    task_id: str = Field(min_length=1, max_length=128)
    action: Literal["pause", "resume", "cancel", "takeover"]


class TaskExecutionEnvelope(EnvelopeBase):
    type: Literal["task.execution.command"]
    payload: TaskExecutionCommandPayload


OPERATION_TYPES = (
    "task.create",
    "task.update",
    "task.delete",
    "task.enable",
    "home.set",
    "navigation.goal",
    "navigation.return_home",
    "waypoint.create",
    "map.export",
    "map.import",
    "map.clear",
    "operation.query",
)


class OperationPayload(StrictModel):
    """Device-owned edit. The robot dedupes by ``operation_id`` and guards by revision."""

    operation_id: str = Field(min_length=1, max_length=128)
    expected_revision: int | None = Field(default=None, ge=0)
    body: dict[str, Any] = Field(default_factory=dict)


class OperationEnvelope(EnvelopeBase):
    type: Literal[
        "task.create",
        "task.update",
        "task.delete",
        "task.enable",
        "home.set",
        "navigation.goal",
        "navigation.return_home",
        "waypoint.create",
        "map.export",
        "map.import",
        "map.clear",
        "operation.query",
    ]
    payload: OperationPayload


ShortText = Annotated[str, Field(min_length=1, max_length=128)]


class MapRefPayload(StrictModel):
    map_id: ShortText
    reference_revision: int = Field(ge=0)


WorkspaceRequest = Literal["task.get", "waypoint.list", "map.manifest", "map.tile"]
WORKSPACE_REQUEST_TYPES = frozenset(get_args(WorkspaceRequest))


class WorkspacePayload(StrictModel):
    map_ref: MapRefPayload
    task_id: ShortText | None = None
    cursor: ShortText | None = None
    grid_id: ShortText | None = None
    tile_x: int | None = Field(default=None, ge=0, le=2047)
    tile_y: int | None = Field(default=None, ge=0, le=2047)

    @model_serializer(mode="wrap")
    def serialize(self, handler: Any) -> dict[str, Any]:
        result: dict[str, Any] = handler(self)
        return {field: value for field, value in result.items() if value is not None}


class WorkspaceEnvelope(EnvelopeBase):
    type: WorkspaceRequest
    payload: WorkspacePayload

    @model_validator(mode="after")
    def matching_fields(self) -> "WorkspaceEnvelope":
        fields = self.payload.model_fields_set - {"map_ref"}
        match self.type:
            case "task.get":
                valid = fields == {"task_id"} and self.payload.task_id is not None
            case "waypoint.list":
                valid = fields <= {"cursor"}
            case "map.manifest":
                valid = not fields
            case "map.tile":
                valid = fields == {"grid_id", "tile_x", "tile_y"} and all(
                    getattr(self.payload, field) is not None for field in fields
                )
        if not valid:
            raise ValueError("invalid workspace read parameters")
        return self


class HomePayload(StrictModel):
    waypoint_id: ShortText
    map_ref: MapRefPayload
    status: ShortText


class TaskSummaryPayload(StrictModel):
    task_id: ShortText
    revision: int = Field(ge=0)
    enabled: bool


class ScanPointPayload(StrictModel):
    x: float
    y: float


class RobotPosePayload(ScanPointPayload):
    yaw: float


class SnapshotPayload(StrictModel):
    """The robot's own view of itself, pushed on connect and after each change."""

    connected: bool
    localized: bool
    map_ref: MapRefPayload | None
    home: HomePayload | None
    task_revision: int = Field(ge=0)
    runnable_task_ids: list[ShortText] = Field(max_length=200)
    execution_status: str = Field(min_length=1, max_length=64)
    collected_at: float
    # Set only by the cloud when it sends its cached copy to a browser; an agent may omit both.
    stale: bool = False
    last_synced_at: float | None = None
    tasks: list[TaskSummaryPayload] = Field(default_factory=list, max_length=200)
    # home_edit.revision. 0 is the store's answer when that row does not exist yet.
    home_revision: int = Field(default=0, ge=0)
    map_generation: ShortText | None = None
    route_progress: RouteProgress | None = None
    navigation_ready: bool = False
    # //Compatible: legacy snapshots have no live geometry.
    robot_pose: RobotPosePayload | None = None
    navigation_block_reason: str | None = Field(default=None, max_length=256)
    scan_points: list[ScanPointPayload] = Field(default_factory=list, max_length=96)
    # //Compatible: absent on unmigrated vehicles; never fabricate a new capability.
    vehicle_ui: dict[str, Any] | None = None
    # Relay-only owner metadata; stripped before control-socket delivery.
    manual_controller_generation: int | None = None
    manual_agent_epoch: int | None = None

    @model_serializer(mode="wrap")
    def serialize(self, handler: Any) -> dict[str, Any]:
        result: dict[str, Any] = handler(self)
        for optional in ("vehicle_ui", "manual_controller_generation", "manual_agent_epoch"):
            if getattr(self, optional) is None:
                result.pop(optional, None)
        # //Compatible: omit generation in snapshots without a qualified source.
        if self.map_generation is None:
            result.pop("map_generation", None)
        if self.route_progress is None:
            result.pop("route_progress", None)
        # //Compatible: omit optional geometry when the source has no live observation.
        if self.navigation_block_reason is None:
            result.pop("navigation_block_reason", None)
        if self.robot_pose is None:
            result.pop("robot_pose", None)
        if not self.scan_points:
            result.pop("scan_points", None)
        # //Compatible: a missing readiness bit keeps the legacy manifest fallback.
        if "navigation_ready" not in self.model_fields_set:
            result.pop("navigation_ready", None)
        return result


class SnapshotEnvelope(EnvelopeBase):
    type: Literal["snapshot"]
    payload: SnapshotPayload


class TaskNavigationState(StrictModel):
    """Optional lifecycle details on navigation acceptance and operation query."""

    execution_id: str = Field(min_length=1, max_length=128)
    phase: Literal["starting", "running", "revoking", "paused", "succeeded", "canceled", "failed"]
    epoch: int = Field(ge=0, le=_MAX_SAFE_WEB_INTEGER)
    completed: list[Annotated[str, Field(min_length=1, max_length=256)]]
    remaining: list[Annotated[str, Field(min_length=1, max_length=256)]]

    @model_validator(mode="after")
    def unique_actions(self) -> "TaskNavigationState":
        actions = self.completed + self.remaining
        if len(set(actions)) != len(actions):
            raise ValueError("task action identities must be unique")
        return self


class TaskCommandReceiptPayload(StrictModel):
    request_id: str = Field(min_length=1, max_length=128)
    task_id: str = Field(min_length=1, max_length=128)
    accepted: bool
    reason: str = Field(min_length=1, max_length=512)
    version: int = Field(ge=0, le=_MAX_SAFE_WEB_INTEGER)


class PreparedAuthorityPayload(StrictModel):
    authority_id: str = Field(min_length=1, max_length=128)
    state: Literal["prepared", "active", "closed"]


class ResponsePayload(StrictModel):
    request_type: Literal[
        "learning.start",
        "learning.status",
        "safety.estop.reset",
        "safety.fault.recover",
        "task.execution.command",
        "settings.get",
        "settings.apply",
        "settings.result",
        "task.create",
        "task.update",
        "task.delete",
        "task.enable",
        "home.set",
        "navigation.goal",
        "navigation.return_home",
        "waypoint.create",
        "map.export",
        "map.import",
        "map.clear",
        "operation.query",
        "control.acquire",
        "control.release",
        "drive.stop",
        "safety.estop",
        "video.offer",
        "video.answer",
        "video.candidate",
        "video.close",
        "task.get",
        "waypoint.list",
        "map.manifest",
        "map.tile",
    ]
    ok: bool
    error: ErrorCode | None = None
    operation_id: str | None = None
    data: dict[str, Any] | None = None
    status: (
        Literal["accepted", "dispatching", "running", "succeeded", "failed", "needs_review"] | None
    ) = None

    @model_validator(mode="after")
    def consistent_result(self) -> "ResponsePayload":
        if self.ok == (self.error is not None):
            raise ValueError("response success and error conflict")
        settings = self.request_type.startswith("settings.")
        learning = self.request_type.startswith("learning.")
        if learning and (self.data is not None) != self.ok:
            raise ValueError("learning_response_data")
        workspace = self.request_type in WORKSPACE_REQUEST_TYPES
        task = self.request_type == "task.execution.command"
        if (workspace or task) and (self.data is not None) != self.ok:
            raise ValueError("response data does not match its request")
        # //Compatible: older navigation/query replies omit task details.
        optional_manual = self.request_type == "control.acquire"
        optional_task = self.request_type in {
            "navigation.goal",
            "navigation.return_home",
            "operation.query",
        }
        if self.data is not None:
            if not self.ok or not (
                workspace or task or optional_task or optional_manual or settings or learning
            ):
                raise ValueError("response data does not match its request")
            if learning:
                dto = (
                    LearningStartReceipt
                    if self.request_type == "learning.start"
                    else LearningStatus
                )
                dto.model_validate(self.data)
            elif optional_task:
                TaskNavigationState.model_validate(self.data)
            elif task:
                TaskCommandReceiptPayload.model_validate(self.data)
            elif optional_manual:
                PreparedAuthorityPayload.model_validate(self.data)
        return self

    @model_serializer(mode="wrap")
    def serialize(self, handler: Any) -> dict[str, Any]:
        result: dict[str, Any] = handler(self)
        # //Compatible: strict v1 peers accept the original response fields.
        if self.data is None and self.request_type != "settings.result":
            result.pop("data", None)
        return result


class ResponseEnvelope(EnvelopeBase):
    type: Literal["response"]
    payload: ResponsePayload
    controller_generation: int | None = Field(default=None, ge=1, le=_MAX_SAFE_WEB_INTEGER)

    @model_serializer(mode="wrap")
    def serialize_context(self, handler: Any) -> dict[str, Any]:
        value: dict[str, Any] = handler(self)
        if self.controller_generation is None:
            value.pop("controller_generation", None)
        return value


MAX_SDP_CHARS = 16384
MAX_CANDIDATE_CHARS = 512


class SdpPayload(StrictModel):
    sdp: str = Field(min_length=1, max_length=MAX_SDP_CHARS)


class CandidatePayload(StrictModel):
    candidate: str = Field(min_length=1, max_length=MAX_CANDIDATE_CHARS, pattern=r"^candidate:")


class VideoSdpEnvelope(EnvelopeBase):
    """Video signaling. The controller sends the offer; the vehicle sends the answer."""

    type: Literal["video.offer", "video.answer"]
    payload: SdpPayload


class VideoCandidateEnvelope(EnvelopeBase):
    """An ICE candidate, in either direction."""

    type: Literal["video.candidate"]
    payload: CandidatePayload


class VideoCloseEnvelope(EnvelopeBase):
    """Ends the video of one signaling id. The relay sends it when the viewer goes away."""

    type: Literal["video.close"]
    payload: EmptyPayload


VideoEnvelope = VideoSdpEnvelope | VideoCandidateEnvelope | VideoCloseEnvelope

Envelope = Annotated[
    SimpleEnvelope
    | TaskExecutionEnvelope
    | LearningStartEnvelope
    | LearningStatusEnvelope
    | SettingsGetEnvelope
    | SettingsApplyEnvelope
    | SettingsResultEnvelope
    | ManualIntentEnvelope
    | RecoveryEnvelope
    | OperationEnvelope
    | WorkspaceEnvelope
    | SnapshotEnvelope
    | ResponseEnvelope,
    Field(discriminator="type"),
]
Frame = Annotated[
    SimpleEnvelope
    | TaskExecutionEnvelope
    | LearningStartEnvelope
    | LearningStatusEnvelope
    | SettingsGetEnvelope
    | SettingsApplyEnvelope
    | SettingsResultEnvelope
    | ManualIntentEnvelope
    | RecoveryEnvelope
    | OperationEnvelope
    | WorkspaceEnvelope
    | SnapshotEnvelope
    | ResponseEnvelope
    | VideoSdpEnvelope
    | VideoCandidateEnvelope
    | VideoCloseEnvelope,
    Field(discriminator="type"),
]
_adapter = TypeAdapter(Envelope)
_frame_adapter = TypeAdapter(Frame)
MAX_MESSAGE_BYTES = 65536


def validate_envelope(raw: bytes, expected_robot_id: str) -> Envelope:
    """Validate a bounded message for the authenticated robot."""
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    try:
        envelope = _adapter.validate_json(raw)
    except (ValidationError, ValueError) as exc:
        raise ValueError("invalid envelope") from exc
    if envelope.robot_id != expected_robot_id:
        raise ValueError("wrong robot")
    return envelope


def validate_frame(raw: bytes, expected_robot_id: str) -> Envelope | VideoEnvelope:
    """Like `validate_envelope`, and also accepts video signaling for the relay to forward."""
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    try:
        frame = _frame_adapter.validate_json(raw)
    except (ValidationError, ValueError) as exc:
        raise ValueError("invalid envelope") from exc
    if frame.robot_id != expected_robot_id:
        raise ValueError("wrong robot")
    return frame


def serialize_envelope(envelope: Envelope | VideoEnvelope) -> bytes:
    """Serialize a validated envelope with the same size bound as input."""
    try:
        validated = _frame_adapter.validate_python(envelope)
    except ValidationError as exc:
        raise ValueError("invalid envelope") from exc
    excluded: dict[str, set[str]] = {}
    if isinstance(validated, SimpleEnvelope):
        excluded["payload"] = {
            name for name in ("gesture", "authority_id") if getattr(validated.payload, name) is None
        }
    raw = _frame_adapter.dump_json(validated, exclude=excluded)
    if len(raw) > MAX_MESSAGE_BYTES:
        raise ValueError("message too large")
    return raw
