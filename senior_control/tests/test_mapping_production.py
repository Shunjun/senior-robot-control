"""Real manager/catalog/waypoint/checkpoint assembly; native ROS bytes are synthetic here."""

import asyncio
import hashlib
import json
import math
import tempfile
import uuid
from pathlib import Path

import pytest
from senior_robot.agent_adapter import AdapterCore
from test_agent import rig as rig
from vehicle_manager.agent.mapping_catalog import mapping_requests
from vehicle_manager.storage.bindings import Database
from vehicle_manager.storage.domain_bindings import WaypointStore

from vehicle_core.ipc import IpcClient, IpcServer


class NativeFixture:
    def __init__(self, root):
        self.root = root
        self.root.mkdir(mode=0o700)
        self.boot = ""
        self.sequence = 0
        self.anchors = []
        self.started = 0
        self.misaligned = False
        self.corrected = False
        self.cuts = {}

    def transfer(self, cut):
        path = self.root / "transfer.json"
        payload = json.dumps(cut).encode()
        path.write_bytes(payload)
        path.chmod(0o600)
        return dict(ok=True, path=str(path), sha256=hashlib.sha256(payload).hexdigest())

    async def request(self, kind, **fields):
        action = kind.removeprefix("map.backend.")
        if action == "status":
            return dict(ok=True, stopped=True)
        if action == "start":
            self.started += 1
            self.boot, self.sequence = str(uuid.uuid4()), 0
            self.cuts.clear()
            self.anchors = (
                json.loads(Path(fields["prefix"] + ".posegraph").read_text())
                if fields.get("prefix")
                else []
            )
            return dict(ok=True, boot_id=self.boot, loaded=self.transfer(self.cut()))
        if action == "stop":
            return dict(ok=True)
        assert fields["boot_id"] == self.boot
        if action == "gate":
            return dict(ok=True, sequence=self.sequence)
        if action == "next":
            sequence = fields["sequence"]
            if sequence in self.cuts:
                return self.transfer(self.cuts[sequence])
            assert sequence == self.sequence + 1
            if self.sequence >= 4:
                return dict(ok=True, pending=True)
            self.sequence += 1
            self.anchors.append(dict(id=len(self.anchors), x=0.0, y=0.0, yaw=0.0))
            if self.corrected and (self.sequence >= 4 or self.started > 1):
                for row in self.anchors:
                    row["yaw"] = 0.15
            self.cuts[sequence] = json.loads(json.dumps(self.cut()))
            return self.transfer(self.cuts[sequence])
        if action == "capture":
            prefix = self.root / "capture"
            data = [json.dumps(self.anchors).encode(), b"synthetic Karto dataset"]
            for suffix, payload in zip((".posegraph", ".data"), data, strict=True):
                Path(str(prefix) + suffix).write_bytes(payload)
            cut = self.cut()
            cut.update(
                prefix=str(prefix),
                pair_digest=hashlib.sha256(
                    b"".join(hashlib.sha256(p).digest() for p in data)
                ).hexdigest(),
            )
            return self.transfer(cut)
        raise AssertionError(kind)

    def cut(self):
        return dict(
            boot_id=self.boot,
            sequence=self.sequence,
            odom_frame="odom",
            map_frame="map",
            anchors=self.anchors,
            keyframe_id=len(self.anchors) - 1,
            stamp=self.sequence * 0.6,
            sensor=[
                0.0,
                0.0,
                0.15 if self.corrected and (self.sequence >= 4 or self.started > 1) else 0.0,
            ],
            angle_min=-3.14,
            angle_increment=6.28 / 60,
            range_min=0.1,
            range_max=10.0,
            ranges=[2.0 if self.misaligned else 0.5] * 60,
        )


@pytest.fixture
async def native_pair(tmp_path):
    native = NativeFixture(tmp_path / "native")
    core = AdapterCore(
        robot_id="r1",
        submit=lambda **_: None,
        disconnect=lambda _: None,
        request_exit=lambda: None,
        mapping=native,
    )
    with tempfile.TemporaryDirectory(prefix="mapping-", dir="/tmp") as directory:
        socket = Path(directory) / "m.sock"
        server, client = IpcServer(socket, core.handle), IpcClient(socket, timeout_ms=5000)
        await server.start()
        try:
            yield native, client
        finally:
            await client.close()
            await server.close()


@pytest.mark.parametrize("scenario", ["matched", "pending", "corrected"])
async def test_production_assembly_continuously_saves_and_reopens_canonical_home(
    tmp_path, native_pair, scenario
):
    native, client = native_pair
    matched = scenario != "pending"
    native.corrected = scenario == "corrected"
    database_path = tmp_path / "robot.db"
    async with Database(database_path) as database:
        points = WaypointStore(database, robot_id="r1")
        await points.prepare()
        managed = await mapping_requests(
            database,
            client,
            client,
            robot_id="r1",
            root=tmp_path / "maps",
            waypoints=points,
            checkpoint_delay_s=0.05,
        )
        running = asyncio.create_task(managed.run())
        for _ in range(100):
            if (await managed.request("map.catalog"))["runtime_available"]:
                break
            await asyncio.sleep(0.01)
        try:
            reply = await managed.request(
                "map.create", operation_id="first", name="仓库", expected_generation=0
            )
            assert reply["ok"]
            for _ in range(200):
                catalog = await managed.request("map.catalog")
                if catalog["entries"][0]["version"] >= 2 and native.sequence == 4:
                    break
                await asyncio.sleep(0.01)
            else:
                raise AssertionError("automatic checkpoint did not progress")
            first = catalog["entries"][0]
            manifest = await managed.request("map.manifest", **first["map_ref"])
            assert manifest["display_only"] is True
            assert (await points.home()).waypoint.map_ref.map_id == first["map_ref"]["map_id"]
        finally:
            running.cancel()
            await asyncio.gather(running, return_exceptions=True)
            await managed.close()
        latest = (await managed.request("map.catalog"))["entries"][0]
    if native.corrected:
        from senior_mapping.references import SlamSnapshot

        directory = (
            tmp_path
            / "maps/checkpoints"
            / hashlib.sha256(first["map_ref"]["map_id"].encode()).hexdigest()
            / latest["checkpoint_id"]
        )
        saved = SlamSnapshot.read(directory)
        assert saved.bindings
        assert all(
            math.isclose(math.hypot(item.world_x, item.world_y), 0.5, abs_tol=1e-8)
            for item in saved.bindings
            if not item.removed
        )
    native.misaligned = not matched
    async with Database(database_path) as database:
        points = WaypointStore(database, robot_id="r1")
        await points.prepare()
        managed = await mapping_requests(
            database,
            client,
            client,
            robot_id="r1",
            root=tmp_path / "maps",
            waypoints=points,
            checkpoint_delay_s=0.05,
        )
        running = asyncio.create_task(managed.run())
        try:
            for _ in range(200):
                catalog = await managed.request("map.catalog")
                if catalog["phase"] == ("mapping" if matched else "pending_calibration"):
                    break
                await asyncio.sleep(0.01)
            else:
                raise AssertionError(catalog)
            assert catalog["selection"]["map_ref"] == first["map_ref"]
            assert (await points.home()).waypoint.id == first["home_id"]
            assert native.started == 2
            assert (await points.home()).waypoint.status == (
                "confirmed" if matched else "needs_review"
            )
        finally:
            running.cancel()
            await asyncio.gather(running, return_exceptions=True)
            await managed.close()


async def test_shutdown_does_not_cancel_map_flush_at_the_motion_safety_deadline(rig):
    from agent_fakes import FakeLink
    from test_agent import finish, start
    from vehicle_manager.agent.request_bindings import Deadlines

    events = rig.motion.events
    rig.agent._deadlines = Deadlines(edit=1.0, map=0.5, safety_ms=20.0)
    # Production grants persistence its own bounded budget after motion revocation.
    rig.agent._cleanup_timeout_ms = 500.0

    async def save():
        events.append("map-save-start")
        await asyncio.sleep(0.08)
        events.append("map-save-durable")

    rig.agent._cleanup = save
    task = await start(rig, FakeLink(1))
    await finish(task, rig)
    assert "map-save-durable" in events
    assert next(i for i, event in enumerate(events) if event.startswith("disable:")) < events.index(
        "map-save-start"
    )
    assert events.index("map-save-durable") < events.index("exit")


async def test_deleted_map_reclaims_its_persisted_graph_files(tmp_path):
    native = NativeFixture(tmp_path / "native")
    async with Database(tmp_path / "robot.db") as database:
        points = WaypointStore(database, robot_id="r1")
        await points.prepare()
        managed = await mapping_requests(
            database, native, native, robot_id="r1", root=tmp_path / "maps", waypoints=points
        )
        running = asyncio.create_task(managed.run())
        try:
            for _ in range(100):
                if (await managed.request("map.catalog"))["runtime_available"]:
                    break
                await asyncio.sleep(0.01)
            assert (
                await managed.request(
                    "map.create", operation_id="create", name="temporary", expected_generation=0
                )
            )["ok"]
            entry = (await managed.request("map.catalog"))["entries"][0]
            directory = (
                tmp_path
                / "maps/checkpoints"
                / hashlib.sha256(entry["map_ref"]["map_id"].encode()).hexdigest()
            )
            assert list(directory.rglob("graph.posegraph"))
            assert (
                await managed.request(
                    "map.delete", operation_id="delete", **entry["map_ref"], expected_generation=1
                )
            )["ok"]
            assert (await managed.request("map.catalog"))["entries"] == []
            assert not directory.exists()
        finally:
            running.cancel()
            await asyncio.gather(running, return_exceptions=True)
            await managed.close()


async def test_native_internal_error_is_reported_as_unavailable():
    from senior_mapping.native_session import NativeGraph

    from vehicle_core.errors import DomainError, ErrorCode

    class FailedClient:
        async def request(self, *args, **kwargs):
            return dict(ok=False, error="internal")

    graph = NativeGraph(FailedClient(), None)
    with pytest.raises(DomainError) as result:
        await graph.request("gate")
    assert result.value.code is ErrorCode.UNAVAILABLE
