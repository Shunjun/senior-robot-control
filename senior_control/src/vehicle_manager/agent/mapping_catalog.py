"""Management catalog reads and lifecycle RPCs, over the existing local connections.

Metadata is read from its functional owner's tables in the manager database;
physical graph/localization effects stay with the registered mapping runtime.
No directory-only edit is acknowledged as starting a SLAM session.
"""

from typing import Any

from pydantic import ValidationError
from senior_contracts.wire import MapCreateBody, MapPhase, MapRuntimeStatus, MapTargetBody

from vehicle_core.ipc import IpcError
from vehicle_manager.agent.local_adapters import IpcMapPort
from vehicle_manager.agent.request_ports import Requester
from vehicle_manager.storage.database import Database


class RemoteMapRuntime:
    def __init__(self, reads: Requester, writes: Requester) -> None:
        self._reads, self._writes = reads, IpcMapPort(writes)
        self.phase: MapPhase = "unavailable"

    async def refresh(self) -> None:
        # A missing producer never turns old persisted metadata into live capability.
        self.phase = "unavailable"
        try:
            reply = MapRuntimeStatus.model_validate(await self._reads.request("map.status"))
        except (IpcError, OSError, ValidationError):
            return
        self.phase = reply.phase

    async def execute(
        self, kind: str, operation_id: str, body: MapCreateBody | MapTargetBody
    ) -> None:
        await self._writes.manage(kind, operation_id, body.model_dump())


class MappingRequests:
    def __init__(
        self, endpoint: Requester, runtime: RemoteMapRuntime, reads: Requester, writes: Requester
    ) -> None:
        self._endpoint, self._runtime = endpoint, runtime
        self._reads, self._writes = reads, writes

    async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
        if kind in {"map.catalog", "map.create", "map.activate", "map.delete"}:
            await self._runtime.refresh()
            return await self._endpoint.request(kind, **fields)
        owner = self._writes if kind in {"map.import", "map.export", "map.clear"} else self._reads
        return await owner.request(kind, **fields)


async def mapping_requests(
    database: Database, reads: Requester, writes: Requester, *, robot_id: str
) -> Requester | None:
    try:
        from senior_mapping.catalog import MapCatalog
        from senior_mapping.catalog_api import CatalogEndpoint
    except ModuleNotFoundError as exc:
        if exc.name not in {
            "senior_mapping",
            "senior_mapping.catalog",
            "senior_mapping.catalog_api",
        }:
            raise
        # //Compatible: installing management without optional mapping keeps original ports.
        return None
    catalog = MapCatalog(database, robot_id=robot_id)
    await catalog.prepare_reader()
    runtime = RemoteMapRuntime(reads, writes)
    endpoint = CatalogEndpoint(catalog, robot_id=robot_id, runtime=runtime)
    return MappingRequests(endpoint, runtime, reads, writes)
