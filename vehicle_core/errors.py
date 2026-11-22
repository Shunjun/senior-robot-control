"""Vehicle-local exceptions and the shared wire error codes."""

# //Compatible: local callers keep the same wire error enum.
from senior_contracts.errors import ErrorCode as ErrorCode


class DomainError(Exception):
    def __init__(self, code: ErrorCode, message: str) -> None:
        super().__init__(message)
        self.code = code
