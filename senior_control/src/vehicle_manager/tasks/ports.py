"""Read-only map-reference shape consumed by task definitions and scan context."""

from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from typing import Protocol

from senior_contracts.errors import ErrorCode


class MapReference(Protocol):
    @property
    def map_id(self) -> str: ...

    @property
    def reference_revision(self) -> int: ...


@dataclass(frozen=True)
class TaskStorePorts:
    """Original schema preparation and value/error constructors, scoped to one store."""

    prepare_schema: Callable[[], Awaitable[None]]
    map_reference: Callable[[str, int], MapReference]
    error: Callable[[ErrorCode, str], Exception]
