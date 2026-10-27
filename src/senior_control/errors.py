"""Stable domain error codes."""

from enum import Enum


class ErrorCode(str, Enum):
    VALIDATION = "validation"
    CONFLICT = "conflict"
    OFFLINE = "offline"
    LOCALIZATION_UNAVAILABLE = "localization_unavailable"
    SAFETY_REJECTED = "safety_rejected"
    TIMEOUT = "timeout"
    OPERATION_UNKNOWN = "operation_unknown"


class DomainError(Exception):
    def __init__(self, code: ErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code
