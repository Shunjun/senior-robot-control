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


class PoseReading(Protocol):
    """Only the original pose coordinates serialized by lifecycle acceptance."""

    @property
    def x(self) -> float: ...

    @property
    def y(self) -> float: ...

    @property
    def yaw(self) -> float: ...


@dataclass(frozen=True)
class TaskStorePorts:
    """Original schema preparation and value/error constructors, scoped to one store."""

    prepare_schema: Callable[[], Awaitable[None]]
    map_reference: Callable[[str, int], MapReference]
    error: Callable[[ErrorCode, str], Exception]


class LiveMapReference(Protocol):
    """The original live admission owner; no copied or cached qualification."""

    @property
    def map_ref(self) -> MapReference: ...

    @property
    def map_generation(self) -> str: ...

    @property
    def review_required(self) -> bool: ...


@dataclass(frozen=True)
class TaskExecutionErrors:
    """Concrete local exception identities, supplied once for each executor."""

    domain: type[Exception]
    qualification_unavailable: type[Exception]
    route_rejected: type[Exception]
    code: Callable[[Exception], ErrorCode]
