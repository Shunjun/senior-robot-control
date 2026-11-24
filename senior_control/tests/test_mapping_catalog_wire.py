"""Catalog requests cross cloud correlation, management journal and actual local IPC.

The graph fixture is synthetic, not native SLAM/localization acceptance evidence.
"""

import json
import tempfile
import time
from pathlib import Path

import pytest
from agent_fakes import FakeHome, FakeTasks
from cloud.limits import CloudLimits
from cloud.runtime_relay import Hub
from senior_contracts.wire import (
    MapCatalogEnvelope,
    ResponseEnvelope,
    ResponsePayload,
    serialize_envelope,
    validate_envelope,
)
from senior_mapping.catalog import MapCatalog
from senior_mapping.catalog_api import CatalogEndpoint
from senior_mapping.checkpoints import MapCheckpoints
from senior_robot.agent_adapter import AdapterCore
from test_map_archive import archive_for, filled_store
from vehicle_manager.agent.exported_workspace import WorkspaceReader
from vehicle_manager.agent.local_adapters import IpcMapPort
from vehicle_manager.agent.mapping_catalog import mapping_requests
from vehicle_manager.agent.production_bindings import AgentJournal
from vehicle_manager.agent.request_bindings import Deadlines, Dispatcher
from vehicle_manager.observability import Metrics
from vehicle_manager.storage.bindings import Database
from vehicle_manager.storage.domain_bindings import TaskStore, WaypointStore

from vehicle_core.ipc import IpcClient, IpcServer
from vehicle_core.values import MapRef, OperationStatus


@pytest.fixture
def ipc_path():
    with tempfile.TemporaryDirectory(prefix="mc-") as directory:
        yield Path(directory) / "m.sock"


async def test_catalog_cloud_journal_ipc_create_switch_delete_and_replay(
    tmp_path: Path, ipc_path: Path
):
    async with Database(tmp_path / "robot.db") as database:
        catalog = MapCatalog(database, robot_id="r1")
        await catalog.prepare()

        class Runtime:
            phase = "mapping"
            calls = 0

            async def execute(self, kind, operation_id, body):
                self.calls += 1
                if kind == "map.create":
                    entry = await catalog.create(operation_id, body.name)
                    store = filled_store(entry.map_ref)
                    repository = MapCheckpoints(tmp_path / "cuts", archive_for(store), catalog)
                    await repository.save(store, home_id="home", expected=None)
                    target = entry.map_ref
                else:
                    target = MapRef(body.map_id, body.reference_revision)
                if kind == "map.delete":
                    await catalog.delete_inactive(target)
                else:
                    selected = await catalog.begin_activation(
                        target, expected_generation=body.expected_generation
                    )
                    await catalog.finish_activation(selected.token, localized=True)

        runtime = Runtime()
        endpoint = CatalogEndpoint(catalog, robot_id="r1", runtime=runtime)
        core = AdapterCore(
            robot_id="r1",
            submit=lambda **_: None,
            disconnect=lambda _: None,
            request_exit=lambda: None,
            mapping=endpoint,
        )
        server = IpcServer(ipc_path, core.handle)
        client = IpcClient(ipc_path)
        tasks, points = TaskStore(database, robot_id="r1"), WaypointStore(database, robot_id="r1")
        await tasks.prepare()
        await points.prepare()
        journal = AgentJournal(database, robot_id="r1")
        await journal.prepare()
        managed = await mapping_requests(database, client, client, robot_id="r1")
        assert managed is not None
        reader = WorkspaceReader("r1", tasks, points, managed)
        dispatcher = Dispatcher(
            "r1",
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=IpcMapPort(managed),
            deadlines=Deadlines(),
            metrics=Metrics(),
            map_root=tmp_path / "transfer",
            admit=lambda _: None,
        )
        hub = Hub(CloudLimits())
        agent, viewer = hub.attach_agent("r1", epoch=2), hub.attach_controller("r1", "owner")
        assert viewer is not None
        while viewer.outbox.pending:
            await viewer.outbox.get()
        sequence = 0

        async def request(kind, body=None, operation_id=""):
            nonlocal sequence
            sequence += 1
            frame = validate_envelope(
                json.dumps(
                    dict(
                        v=1,
                        robot_id="r1",
                        session_id="web",
                        request_id=str(sequence),
                        seq=sequence,
                        connection_epoch=2,
                        expires_at=time.time() + 10,
                        type=kind,
                        payload={}
                        if kind == "map.catalog"
                        else {"operation_id": operation_id, "body": body},
                    )
                ).encode(),
                "r1",
            )
            hub.handle_control_frame(viewer, serialize_envelope(frame))
            received = validate_envelope(await agent.outbox.get(), "r1")
            if isinstance(received, MapCatalogEnvelope):
                payload = ResponsePayload(
                    request_type=kind, ok=True, data=await reader.read(received)
                )
            else:
                result = await dispatcher.run(received)
                payload = ResponsePayload(
                    request_type=kind,
                    ok=True,
                    status=result.status.value,
                    operation_id=operation_id,
                )
            reply = ResponseEnvelope(
                **received.model_dump(exclude={"type", "payload"}), type="response", payload=payload
            )
            hub.handle_agent_frame(agent, serialize_envelope(reply))
            return validate_envelope(await viewer.outbox.get(), "r1").payload

        await server.start()
        try:
            assert (await request("map.catalog")).data["entries"] == []
            create = {"name": "仓库", "expected_generation": 0}
            assert (await request("map.create", create, "new-a")).status == "succeeded"
            await request("map.create", create, "new-a")
            assert runtime.calls == 1
            first = (await request("map.catalog")).data
            a = first["entries"][0]["map_ref"]
            await request("map.create", {"name": "院子", "expected_generation": 1}, "new-b")
            second = (await request("map.catalog")).data
            b = second["entries"][1]["map_ref"]
            await request("map.activate", {**a, "expected_generation": 2}, "activate-a")
            await request("map.delete", {**b, "expected_generation": 3}, "delete-b")
            final = (await request("map.catalog")).data
            assert [row["name"] for row in final["entries"]] == ["仓库"]
            assert final["selection"]["map_ref"] == a
            assert (await journal.get("delete-b")).status is OperationStatus.SUCCEEDED
        finally:
            await client.close()
            await server.close()


async def test_lost_map_ack_keeps_the_durable_operation_unknown(tmp_path):
    from agent_fakes import operation

    from vehicle_core.errors import DomainError, ErrorCode
    from vehicle_core.ipc import IpcError

    async with Database(tmp_path / "robot.db") as database:
        catalog = MapCatalog(database, robot_id="r1")
        await catalog.prepare()
        journal = AgentJournal(database, robot_id="r1")
        await journal.prepare()

        class LostReply:
            async def request(self, kind, **fields):
                await catalog.create(fields["operation_id"], fields["name"])
                raise IpcError("acknowledgement lost after the request was applied")

        dispatcher = Dispatcher(
            "r1",
            journal=journal,
            tasks=FakeTasks(),
            home=FakeHome(),
            maps=IpcMapPort(LostReply()),
            deadlines=Deadlines(),
            metrics=Metrics(),
            map_root=tmp_path / "transfer",
            admit=lambda _: None,
        )
        issued = operation(
            "map.create", "lost", {"name": "仓库", "expected_generation": 0}, robot_id="r1"
        )
        with pytest.raises(DomainError) as failed:
            await dispatcher.run(issued)
        assert failed.value.code is ErrorCode.OPERATION_UNKNOWN
        assert len(await catalog.list()) == 1
        assert (await journal.get("lost")).status is OperationStatus.NEEDS_REVIEW
