"""Read-only map-reference shape consumed by task definitions and scan context."""

from typing import Protocol


class MapReference(Protocol):
    @property
    def map_id(self) -> str: ...

    @property
    def reference_revision(self) -> int: ...
