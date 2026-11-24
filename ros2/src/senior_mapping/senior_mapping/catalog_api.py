"""Catalog metadata endpoint plus an explicitly supplied production lifecycle owner."""

import asyncio
from dataclasses import asdict
from typing import Any, Literal, Protocol, cast

from pydantic import ValidationError
from senior_contracts.wire import (
    MapCatalogEntryPayload,
    MapCatalogReply,
    MapCreateBody,
    MapPhase,
    MapRuntimeStatus,
    MapSelectionPayload,
    MapTargetBody,
    parse_mapping_body,
)

from senior_mapping.catalog import MapCatalog
from vehicle_core.errors import DomainError, ErrorCode

MapCommand = Literal["map.create", "map.activate", "map.delete", "map.calibrate", "map.home"]


class MapRuntime(Protocol):
    @property
    def phase(self) -> MapPhase: ...

    async def execute(
        self, kind: MapCommand, operation_id: str, body: MapCreateBody | MapTargetBody
    ) -> None:
        """Return only after the owner's durable operation; uncertain outcomes must raise."""
        ...


class CatalogEndpoint:
    def __init__(
        self, catalog: MapCatalog, *, robot_id: str, runtime: MapRuntime | None = None
    ) -> None:
        self.catalog, self.robot_id, self.runtime = catalog, robot_id, runtime
        self._commands = asyncio.Lock()

    async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
        try:
            phase = "unavailable" if self.runtime is None else self.runtime.phase
            available = self.runtime is not None and phase not in {"unavailable", "failed"}
            if kind == "map.status":
                return MapRuntimeStatus(phase=phase).model_dump()
            if kind == "map.catalog":
                if fields:
                    raise DomainError(ErrorCode.VALIDATION, "catalog read takes no map reference")
                entries, selection = await self.catalog.view()
                return MapCatalogReply(
                    robot_id=self.robot_id,
                    entries=[
                        MapCatalogEntryPayload.model_validate(asdict(entry)) for entry in entries
                    ],
                    selection=MapSelectionPayload.model_validate(
                        {
                            "map_ref": None
                            if selection.map_ref is None
                            else asdict(selection.map_ref),
                            "generation": selection.generation,
                            "state": selection.state,
                        }
                    ),
                    runtime_available=available,
                    phase=phase,
                ).model_dump()
            if kind not in {
                "map.create",
                "map.activate",
                "map.delete",
                "map.calibrate",
                "map.home",
            }:
                raise DomainError(ErrorCode.VALIDATION, "unsupported map command")
            operation_id = fields.pop("operation_id", None)
            if not isinstance(operation_id, str) or not 1 <= len(operation_id) <= 128:
                raise DomainError(ErrorCode.VALIDATION, "map operation identity is required")

            body = parse_mapping_body(kind, fields)
            if not available or self.runtime is None:
                raise DomainError(ErrorCode.UNAVAILABLE, "production mapping runtime unavailable")
            async with self._commands:
                selected = await self.catalog.selection()
                if selected.generation != body.expected_generation:
                    raise DomainError(ErrorCode.CONFLICT, "map selection changed")
                await self.runtime.execute(cast(MapCommand, kind), operation_id, body)
                if kind == "map.calibrate" and self.runtime.phase != "mapping":
                    raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "local match failed")
            return {"ok": True, "operation_id": operation_id}
        except ValidationError:
            return {"ok": False, "error": ErrorCode.VALIDATION.value}
        except DomainError as exc:
            return {"ok": False, "error": exc.code.value}
