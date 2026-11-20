"""Vehicle-local runtime state, independent of management and ROS implementations."""

from typing import Any, Literal

from pydantic import Field, model_serializer, model_validator
from senior_contracts.wire import _MAX_SAFE_WEB_INTEGER, StrictModel


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
