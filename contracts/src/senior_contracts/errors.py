"""Stable domain error codes."""

from enum import Enum


class ErrorCode(str, Enum):
    VALIDATION = "validation"
    CONFLICT = "conflict"
    OFFLINE = "offline"
    UNAVAILABLE = "unavailable"
    LOCALIZATION_UNAVAILABLE = "localization_unavailable"
    SAFETY_REJECTED = "safety_rejected"
    TIMEOUT = "timeout"
    OPERATION_UNKNOWN = "operation_unknown"
