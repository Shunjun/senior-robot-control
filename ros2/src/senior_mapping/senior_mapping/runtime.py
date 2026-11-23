"""One serial mapping lifecycle. Loading is a barrier, never a second live session."""

import asyncio
from typing import Any, Protocol, cast

from senior_contracts.wire import MapCreateBody, MapPhase, MapTargetBody

from senior_mapping.catalog import CatalogEntry, MapCatalog
from senior_mapping.catalog_api import MapCommand
from senior_mapping.workspace import WorkspaceGrid
from vehicle_core.errors import DomainError, ErrorCode


class MappingSession(Protocol):
    entry: CatalogEntry
    localized: bool

    async def activate(self) -> None: ...
    async def flush(self) -> None: ...
    async def close(self) -> None: ...
    async def ingest(self, observation: Any) -> None: ...
    def grid(self) -> WorkspaceGrid | None: ...


class MappingBackend(Protocol):
    async def stopped(self) -> bool: ...
    async def delete(self, entry: CatalogEntry) -> None: ...
    async def open(self, entry: CatalogEntry, *, restore: bool) -> MappingSession: ...


class MappingRuntime:
    def __init__(
        self, catalog: MapCatalog, backend: MappingBackend, *, restore_required: bool = False
    ) -> None:
        self.catalog, self.backend = catalog, backend
        self.phase: MapPhase = "idle"
        self._session: MappingSession | None = None
        self._lifecycle = asyncio.Lock()
        self._updates = asyncio.Lock()
        self._closing = False
        self._owns_reference = restore_required
        self.last_error: Exception | None = None

    @property
    def owns_reference(self) -> bool:
        return self._owns_reference

    def grid(self) -> WorkspaceGrid | None:
        return (
            self._session.grid()
            if self.phase in {"mapping", "saving", "pending_calibration"} and self._session
            else None
        )

    async def ingest(self, observation: Any) -> bool:
        # Do not queue loading-window data behind the switch barrier.
        if self.phase != "mapping" or self._closing:
            return False
        async with self._updates:
            if self.phase != "mapping" or self._session is None:
                return False
            error = getattr(self._session, "error", None)
            if error is not None:
                self.phase = "failed"
                self.last_error = error
                raise DomainError(ErrorCode.UNAVAILABLE, "automatic checkpoint failed") from error
            await self._session.ingest(observation)
            return True

    async def execute(
        self, kind: MapCommand, operation_id: str, body: MapCreateBody | MapTargetBody
    ) -> None:
        async with self._lifecycle:
            if self._closing:
                raise DomainError(ErrorCode.UNAVAILABLE, "mapping is closing")
            selected = await self.catalog.selection()
            if selected.generation != body.expected_generation:
                raise DomainError(ErrorCode.CONFLICT, "map selection changed")
            target_body = cast(MapTargetBody, body)
            target = (
                None
                if kind == "map.create"
                else next(
                    (
                        row
                        for row in await self.catalog.list()
                        if row.map_ref.map_id == target_body.map_id
                        and row.map_ref.reference_revision == target_body.reference_revision
                    ),
                    None,
                )
            )
            if kind != "map.create" and target is None:
                raise DomainError(ErrorCode.CONFLICT, "map no longer exists")
            if kind == "map.activate" and target is not None and target.checkpoint_id is None:
                raise DomainError(ErrorCode.CONFLICT, "target map has no complete checkpoint")
            if kind == "map.delete" and target is not None and target.map_ref != selected.map_ref:
                await self.catalog.delete_inactive(target.map_ref)
                await self.backend.delete(target)
                return
            if not await self.backend.stopped():
                raise DomainError(ErrorCode.CONFLICT, "stop the vehicle before changing maps")
            self._owns_reference = True
            self.phase = "loading"
            try:
                # Finish accepted updates and the old save before touching another graph.
                async with self._updates:
                    await self._retire()
                    if kind == "map.delete":
                        await self.catalog.withdraw(expected_generation=selected.generation)
                        assert target is not None
                        await self.catalog.delete_inactive(target.map_ref)
                        await self.backend.delete(target)
                        self.phase = "idle"
                        return
                    if kind == "map.create":
                        target = await self.catalog.create(
                            operation_id, cast(MapCreateBody, body).name
                        )
                    assert target is not None
                    session = await self.backend.open(target, restore=kind != "map.create")
                    self._session = session
                    # New sessions publish their first complete graph/Home cut before selection.
                    target = session.entry
                    activation = await self.catalog.begin_activation(
                        target.map_ref, expected_generation=selected.generation
                    )
                    assert activation.token is not None
                    await self.catalog.finish_activation(
                        activation.token, localized=session.localized
                    )
                    await session.activate()
                    self.phase = "mapping" if session.localized else "pending_calibration"
                    self.last_error = None
            except BaseException as exc:
                self.phase = "failed"
                if isinstance(exc, Exception):
                    self.last_error = exc
                raise

    async def restore(self) -> None:
        selected = await self.catalog.selection()
        if selected.map_ref is not None:
            await self.execute(
                "map.activate",
                "startup",
                MapTargetBody(
                    map_id=selected.map_ref.map_id,
                    reference_revision=selected.map_ref.reference_revision,
                    expected_generation=selected.generation,
                ),
            )

    async def _retire(self) -> None:
        if self._session is not None:
            await self._session.flush()
            await self._session.close()
            self._session = None

    async def close(self) -> None:
        self._closing = True
        async with self._lifecycle:
            self.phase = "loading"
            async with self._updates:
                try:
                    if self._session is not None:
                        try:
                            await self._session.flush()
                        finally:
                            await self._session.close()
                            self._session = None
                finally:
                    self.phase = "idle"
