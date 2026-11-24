"""Real manager/catalog/waypoint/checkpoint assembly; native ROS bytes are synthetic here."""

import asyncio
import base64
import hashlib
import json
import math
import tempfile
import time
import uuid
from pathlib import Path
from types import SimpleNamespace

import pytest
from senior_robot.agent_adapter import AdapterCore
from test_agent import rig as rig
from vehicle_manager.agent.mapping_catalog import mapping_requests
from vehicle_manager.storage.bindings import Database
from vehicle_manager.storage.domain_bindings import WaypointStore

from vehicle_core.ipc import IpcClient, IpcServer
from vehicle_core.values import Pose2D


class NativeFixture:
    def __init__(self, root):
        self.root = root
        self.root.mkdir(mode=0o700)
        self.boot = ""
        self.sequence = 0
        self.anchors = []
        self.started = 0
        self.seeds = []
        self.misaligned = False
        self.corrected = False
        self.cuts = {}
        from senior_localization.live_mapping_reference import LiveMappingReference

        self.reference = LiveMappingReference(SimpleNamespace(task_motion_allowed=lambda **_: True))
        self.pose_sequence = 0
        self.raster = None

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
            self.seeds.append(fields.get("seed"))
            self.boot, self.sequence = str(uuid.uuid4()), 0
            self.cuts.clear()
            self.pose_sequence = 0
            self.reference.begin(self.boot)
            self.anchors = (
                json.loads(Path(fields["prefix"] + ".posegraph").read_text())
                if fields.get("prefix")
                else []
            )
            return dict(ok=True, boot_id=self.boot, loaded=self.transfer(self.cut()))
        if action == "stop":
            self.reference.withdraw()
            return dict(ok=True)
        assert fields["boot_id"] == self.boot
        if action == "grid":
            self.raster = dict(
                boot_id=self.boot,
                sequence=self.sequence,
                grid_id="native-fixture-raster",
                width=20,
                height=20,
                resolution=0.1,
                origin=[-1.0, -1.0, 0.0],
                cells_b64=base64.b64encode(bytes(400)).decode(),
            )
            return self.transfer(self.raster)
        if action == "reference":
            if not fields["active"]:
                self.reference.withdraw()
                return dict(ok=True)
            from senior_mapping.workspace import WorkspaceGrid

            from vehicle_core.values import MapRef

            anchor = self.anchors[-1]
            self.reference.install(
                self.boot,
                WorkspaceGrid(
                    MapRef(fields["map_id"], fields["reference_revision"]),
                    self.raster["grid_id"],
                    20,
                    20,
                    0.1,
                    Pose2D(-1.0, -1.0, 0.0),
                    bytes(400),
                    waypoint_writes=True,
                ),
                fields["generation"],
                str(anchor["id"]),
                Pose2D(anchor["x"], anchor["y"], anchor["yaw"]),
                Pose2D(0.0, 0.0, 0.0),
            )
            self.pose_sequence += 1
            clock = SimpleNamespace(sec=1, nanosec=0)
            now = time.monotonic_ns()
            header = SimpleNamespace(
                valid=True,
                stamp=clock,
                received_ros=clock,
                received_steady_ns=now,
                sample_steady_ns=now,
                clock_generation=1,
                bridge_boot_id=self.boot,
                source_epoch=self.boot,
                capture_generation=1,
                raw_gid="synthetic-raw",
                bridge_gid="synthetic-bridge",
                sequence=self.pose_sequence,
            )
            self.reference.observe(self.boot, header, Pose2D(0.0, 0.0, 0.0))
            return dict(ok=True)
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
            odom=[0.0, 0.0, 0.0],
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
        state_provider=lambda: native.reference.state("idle", navigation_ready=True),
        pose_provider=native.reference.pose_reply,
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
            assert manifest.get("display_only", False) is False
            assert manifest["waypoint_writes"] is True
            state = await managed.state_requests(client).request("state.get")
            assert state["navigation_ready"] is True
            assert state["localized"] is True
            assert state["map_generation"] == state["localization_generation"]
            assert state["map_grid_id"] == manifest["grid_id"]
            assert (await points.home()).waypoint.map_ref.map_id == first["map_ref"]["map_id"]
            if scenario == "matched":
                selected = (await managed.request("map.catalog"))["selection"]
                revision = await points.home_revision()
                reply = await managed.request(
                    "map.home",
                    operation_id="home-coordinates",
                    **first["map_ref"],
                    expected_generation=selected["generation"],
                    expected_home_revision=revision,
                    x=0.12,
                    y=0.08,
                    yaw=0.4,
                )
                assert reply["ok"], reply
                assert (await points.home()).waypoint.pose == Pose2D(0.12, 0.08, 0.4)
                assert native.anchors[-1]["x"] == 0.0
                pose = await managed.state_requests(client).request("pose.get")
                assert (pose["x"], pose["y"]) == (0.0, 0.0)

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
            if scenario == "matched":
                assert (await points.home()).waypoint.pose == Pose2D(0.12, 0.08, 0.4)
            assert (await points.home()).waypoint.status == (
                "confirmed" if matched else "needs_review"
            )
            if not matched:
                original_home = (await points.home()).waypoint.pose
                native.misaligned = False
                reply = await managed.request(
                    "map.calibrate",
                    operation_id="manual-match",
                    **first["map_ref"],
                    expected_generation=catalog["selection"]["generation"],
                    x=0.08,
                    y=-0.03,
                    yaw=0.1,
                )
                assert reply["ok"], reply
                assert native.started == 3
                assert native.seeds[-1] == [0.08, -0.03, 0.1]
                assert (await managed.request("map.catalog"))["phase"] == "mapping"
                assert (await points.home()).waypoint.pose == original_home
                assert (await points.home()).waypoint.status == "confirmed"

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


async def test_checkpoint_home_sync_does_not_overwrite_a_later_canonical_edit(tmp_path):
    from senior_mapping.references import AnchoredWaypoint, WaypointBinding
    from vehicle_manager.agent.mapping_catalog import MappingHome

    from vehicle_core.errors import DomainError, ErrorCode
    from vehicle_core.values import MapRef, Pose2D

    ref, origin = MapRef("map-home", 1), Pose2D(0.0, 0.0, 0.0)
    binding = WaypointBinding(AnchoredWaypoint("home", origin, "0", False), origin)

    class Active:
        def waypoint_bindings(self, source):
            return (binding,)

    async with Database(tmp_path / "home.db") as database:
        points = WaypointStore(database, robot_id="r1")
        await points.prepare()

        async def save(pose):
            return await points.save_home(
                waypoint_id="home",
                pose=pose,
                map_ref=ref,
                anchor_id="0",
                anchor_pose=origin,
                source_robot_id="r1",
                status="confirmed",
                adopted=True,
                latitude=None,
                longitude=None,
                expected_revision=await points.home_revision(),
            )

        await save(origin)
        coordinator = MappingHome(points, "r1")
        await coordinator(ref, "home", Active(), object(), {"0": origin}, before=True)
        edited = Pose2D(2.0, 1.0, 0.3)
        await save(edited)
        with pytest.raises(DomainError) as result:
            await coordinator(ref, "home", Active(), object(), {"0": origin}, before=False)
        assert result.value.code is ErrorCode.CONFLICT
        assert (await points.home()).waypoint.pose == edited


async def test_edited_home_is_rebased_before_binding_to_a_corrected_anchor(tmp_path):
    from senior_mapping.references import AnchoredWaypoint, WaypointBinding
    from vehicle_manager.agent.mapping_catalog import MappingHome

    from vehicle_core.values import MapRef

    ref = MapRef("corrected-home", 1)
    old_anchor = Pose2D(1.0, 1.0, 0.0)
    new_anchor = Pose2D(2.0, 1.0, math.pi / 2)
    initial = Pose2D(2.0, 1.0, 0.0)

    class Active:
        observation_sequence = 1
        changes = ()

        def waypoint_bindings(self, source):
            return (WaypointBinding(AnchoredWaypoint("home", initial, "0", False), old_anchor),)

        def update_waypoints(self, source, **fields):
            self.changes = fields["changes"]

    async with Database(tmp_path / "rebased-home.db") as database:
        points = WaypointStore(database, robot_id="r1")
        await points.prepare()

        async def save(pose):
            await points.save_home(
                waypoint_id="home",
                pose=pose,
                map_ref=ref,
                anchor_id="0",
                anchor_pose=old_anchor,
                source_robot_id="r1",
                status="confirmed",
                adopted=True,
                latitude=None,
                longitude=None,
                expected_revision=await points.home_revision(),
            )

        await save(initial)
        coordinator, active, source = MappingHome(points, "r1"), Active(), object()
        await coordinator(ref, "home", active, source, {"0": old_anchor}, before=True)
        await coordinator(ref, "home", active, source, {"0": old_anchor}, before=False)
        await save(Pose2D(3.0, 1.0, 0.0))
        await coordinator(ref, "home", active, source, {"0": new_anchor}, before=True)
        (changed,) = active.changes
        assert math.isclose(changed.waypoint.pose.x, 2.0, abs_tol=1e-8)
        assert math.isclose(changed.waypoint.pose.y, 3.0, abs_tol=1e-8)
        assert math.isclose(changed.waypoint.pose.yaw, math.pi / 2, abs_tol=1e-8)


def test_native_reference_refresh_preserves_a_fresh_pose_within_the_same_generation():
    from senior_localization.live_mapping_reference import LiveMappingReference
    from senior_mapping.navigation_bridge import NativeNavigationBridge
    from senior_mapping.workspace import WorkspaceGrid

    from vehicle_core.values import MapRef

    node = SimpleNamespace(task_motion_allowed=lambda **_: True)
    reference = LiveMappingReference(node)
    bridge = NativeNavigationBridge(node, reference, None)
    bridge.begin("owned-boot")
    grid = WorkspaceGrid(
        MapRef("map", 1), "native-raster", 2, 2, 0.1, Pose2D(0.0, 0.0, 0.0), bytes(4)
    )
    bridge._grid = (1, grid, object())
    bridge.observe_cut(
        dict(
            boot_id="owned-boot",
            sequence=1,
            keyframe_id="0",
            anchors=[dict(id="0", x=0.0, y=0.0, yaw=0.0)],
            odom=[0.0, 0.0, 0.0],
        )
    )
    bridge._publisher = SimpleNamespace(publish=lambda message: None)
    fields = dict(
        active=True,
        sequence=1,
        grid_sequence=1,
        map_id="map",
        reference_revision=1,
        generation="reference-1",
    )
    bridge.install(fields)
    stamp = SimpleNamespace(sec=1, nanosec=0)
    now = time.monotonic_ns()
    header = SimpleNamespace(
        valid=True,
        stamp=stamp,
        received_ros=stamp,
        received_steady_ns=now,
        sample_steady_ns=now,
        clock_generation=1,
        bridge_boot_id="receipt-boot",
        source_epoch="lidar-source",
        capture_generation=1,
        raw_gid="raw",
        bridge_gid="relay",
        sequence=1,
    )
    reference.observe("owned-boot", header, Pose2D(0.02, 0.0, 0.0))
    before = reference.pose_reply()
    assert before["ok"]
    bridge.install(fields)
    assert reference.pose_reply() == before, "A normal map refresh revoked a still-fresh pose"
    bridge.install({**fields, "generation": "reference-2"})
    assert not reference.pose_reply()["ok"], "A corrected reference retained the old qualification"
