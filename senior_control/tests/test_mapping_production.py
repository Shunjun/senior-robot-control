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


def admitted_profile():
    from dataclasses import replace

    from test_scan_geometry import synthetic_profile

    return replace(synthetic_profile(), profile_id="synthetic-admitted")


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
        from senior_mapping.navigation_bridge import NativeNavigationBridge

        self.navigation = NativeNavigationBridge(SimpleNamespace(), self.reference, None)
        self.pose_sequence = 0
        self.raster = None
        self.coverage_roles = {"global", "local"}
        self.receipt_cycle = 0
        self.selection_observations = []
        self.navigation._publish_fixed_grid = lambda grid: None
        self.navigation._publish_current = lambda *args: None
        self.navigation.host_boot_id = "host"
        self.navigation.contract = dict(
            contract_directory=str(self.root),
            run_nonce="fixture",
            global_instance_permit="g",
            local_instance_permit="l",
        )

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
            self.navigation.begin(self.boot)
            from senior_mapping.shared_scan import LatestScanSlot

            if self.navigation.slot is not None:
                self.navigation.slot.close()
            self.navigation.slot = LatestScanSlot(
                self.root / "latest.json",
                nonce="fixture",
                backend_boot=self.boot,
                host_boot_id="host",
            )
            self.anchors = (
                json.loads(Path(fields["prefix"] + ".posegraph").read_text())
                if fields.get("prefix")
                else []
            )
            return dict(
                ok=True,
                boot_id=self.boot,
                loaded=self.transfer(self.cut()),
                qualified_scan_schema=1,
                qualified_scan_path=str(self.navigation.slot.path),
                qualified_scan_nonce="fixture",
                host_boot_id="host",
            )
        if action == "stop":
            if self.navigation.slot is not None:
                self.navigation.slot.close()
                self.navigation.slot = None
            self.reference.withdraw()
            return dict(ok=True)
        assert fields["boot_id"] == self.boot
        if action == "grid":
            if self.corrected and self.sequence == 4:
                # A real raster RPC yields after the corrected graph commit becomes visible.
                await asyncio.sleep(0.02)
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
                self.navigation.withdraw()
                return dict(ok=True)
            if fields.get("navigation_selected") is True:
                before = (self.reference.grid() is not None, self.reference._navigation_covered())
                self.navigation.authorize_reference(fields)
                self.selection_observations.append((before, self.reference._authorization))
            else:
                self.navigation.authorize_reference(fields)
            return dict(ok=True)
        if action == "install_fixed":
            from senior_mapping.ros_runtime import _read_fixed_transfer

            ack = self.navigation.install_fixed(
                _read_fixed_transfer(fields["path"], fields["sha256"]),
                graph_cut=self.navigation._cut,
            )
            if fields.get("coverage_reason") == "coverage_limit":
                self.navigation.authorize_reference(
                    {**fields, "sequence": self.navigation._cut["sequence"]}
                )
            return ack
        if action == "gate":
            return dict(ok=True, sequence=self.sequence)
        if action == "next":
            self.emit_scan()
            if getattr(self, "pending_graph", False):
                return dict(ok=True, pending=True)
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
            self.navigation.observe_cut(self.cut())
            self.cuts[sequence] = json.loads(json.dumps(self.cut()))
            return self.transfer(self.cuts[sequence])
        if action == "capture":
            if getattr(self, "fail_capture", False):
                return dict(ok=False, error="operation_unknown")
            slot = self.root / "capture"
            slot.mkdir(mode=0o700)  # Native CaptureDirectory reserves one directory per pair.
            prefix = slot / "graph"
            data = [json.dumps(self.anchors).encode(), b"synthetic Karto dataset"]
            for suffix, payload in zip((".posegraph", ".data"), data, strict=True):
                Path(str(prefix) + suffix).write_bytes(payload)
            if getattr(self, "extra_capture_file", False):
                (slot / "unexpected").write_text("must not be removed")
            cut = self.cut()
            cut.update(
                prefix=str(prefix),
                pair_digest=hashlib.sha256(
                    b"".join(hashlib.sha256(p).digest() for p in data)
                ).hexdigest(),
            )
            return self.transfer(cut)
        raise AssertionError(kind)

    def scan_capture(self):
        from senior_perception.vision.raw_capture import capture_from_message

        now = time.monotonic_ns()
        ros = time.time_ns()
        stamp = SimpleNamespace(sec=ros // 10**9, nanosec=ros % 10**9)
        header = SimpleNamespace(
            valid=True,
            stamp=stamp,
            received_ros=stamp,
            received_steady_ns=now,
            sample_steady_ns=now,
            clock_generation=1,
            bridge_boot_id=self.boot,
            source_epoch=self.boot,
            capture_generation=1,
            raw_gid="synthetic-raw",
            bridge_gid="synthetic-bridge",
            sequence=self.pose_sequence,
            frame_id="laser",
            reason="",
        )
        return capture_from_message(header), header

    def emit_scan(self):
        if self.navigation.slot is None or self.navigation._authorized is None:
            return
        from senior_mapping.shared_scan import Beam, QualifiedScan, ScanKey

        self.pose_sequence += 1
        capture, header = self.scan_capture()
        beams = tuple(
            Beam(
                i,
                (0.0, 0.0),
                (0.5 * math.cos(-3.14 + i * 6.28 / 60), 0.5 * math.sin(-3.14 + i * 6.28 / 60)),
                "hit",
            )
            for i in range(60)
        )
        self.navigation.observe_scan(
            QualifiedScan(
                ScanKey.of(self.boot, capture),
                capture,
                "synthetic-admitted",
                Pose2D(0, 0, 0),
                beams,
                capture.time.sample_steady_ns + 500_000_000,
                "host",
            ),
            header,
        )
        self.navigation.slot.flush()
        self.complete_costmaps()

    def complete_costmaps(self):
        """Explicit synthetic Nav2 results; these never claim installed layer coverage."""
        from senior_mapping.shared_scan import CurrentLayerCoverage

        self.receipt_cycle += 1
        auth = self.reference._authorization
        scan = self.navigation._scan
        if auth is None or scan is None or not auth.lease_until_steady_ns:
            return
        now = time.monotonic_ns()
        for role in self.coverage_roles:
            self.reference.observe_costmap_coverage(
                role,
                CurrentLayerCoverage(
                    run_nonce=auth.run_nonce,
                    host_boot_id=auth.host_boot_id,
                    owner_instance=auth.owner_instance,
                    authority_epoch=auth.authority_epoch,
                    role=role,
                    role_instance_permit=getattr(auth, role + "_instance_permit"),
                    plugin_instance=role,
                    cycle=self.receipt_cycle,
                    status="ready",
                    map_id=auth.map_id,
                    reference_revision=auth.reference_revision,
                    reference_generation=auth.reference_generation,
                    backend_boot=auth.backend_boot,
                    key=scan.key,
                    sample_steady_ns=scan.capture.time.sample_steady_ns,
                    scan_deadline_steady_ns=scan.valid_until_steady_ns,
                    coverage_domain_epoch=auth.coverage_domain_epoch,
                    geometry=auth.geometry,
                    expected_fixed_revision=auth.fixed_revision,
                    expected_grid_id=auth.expected_grid_id,
                    cycle_started_steady_ns=now,
                    cycle_completed_steady_ns=now,
                    valid_until_steady_ns=auth.lease_until_steady_ns,
                    master_frame="map",
                    master_geometry=auth.geometry,
                    updated_bounds=(-1.0, -1.0, 1.0, 1.0),
                    global_all_hits_covered=True,
                    local_in_window_hits_covered=True,
                    transform_valid=True,
                    cleanup_complete=True,
                ),
            )

    def cut(self):
        from dataclasses import asdict

        return dict(
            capture=asdict(self.scan_capture()[0]),
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


async def test_native_graph_consumes_single_slot_before_the_next_capture(tmp_path):
    from senior_mapping.native_session import NativeGraph

    from vehicle_core.values import MapRef

    native = NativeFixture(tmp_path / "native")
    native.boot = "boot"
    graph = NativeGraph(native, MapRef("map", 1))
    graph.boot = native.boot
    snapshot = SimpleNamespace(anchors=(), fixed=None)
    await graph.capture_consistent(tmp_path / "first", lambda: snapshot)
    await graph.capture_consistent(tmp_path / "second", lambda: snapshot)
    assert not (native.root / "capture").exists()
    for name in ("graph.posegraph", "graph.data"):
        assert (tmp_path / "first" / name).read_bytes() == (tmp_path / "second" / name).read_bytes()
    assert (tmp_path / "first" / "native-proof.json").is_file()
    assert (tmp_path / "second" / "native-proof.json").is_file()


async def test_native_graph_does_not_remove_an_unknown_slot_file(tmp_path):
    from senior_mapping.native_session import NativeGraph

    from vehicle_core.values import MapRef

    native = NativeFixture(tmp_path / "native")
    native.boot = "boot"
    native.extra_capture_file = True
    graph = NativeGraph(native, MapRef("map", 1))
    graph.boot = native.boot
    with pytest.raises(OSError):
        await graph.capture_consistent(
            tmp_path / "copy", lambda: SimpleNamespace(anchors=(), fixed=None)
        )
    assert (native.root / "capture" / "unexpected").read_text() == "must not be removed"


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

        managed._runtime.backend.geometry_profile = admitted_profile()
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
                    ref = catalog["entries"][0]["map_ref"]
                    view = await managed.request("map.manifest", **ref)
                    live = await managed.state_requests(client).request("state.get")
                    # Receipt of cut 4 precedes publishing its qualified navigation view.
                    # During correction the production owner deliberately exposes read-only data.
                    published = (
                        not view.get("display_only", False)
                        and live.get("navigation_ready") is True
                        and live.get("map_grid_id") == view.get("grid_id")
                        and live.get("map_generation") == live.get("localization_generation")
                    )
                    # Explicit synthetic admission: wait for the C2 window so
                    # this resume regression has actual saved occupied geometry.
                    session = managed._runtime._session
                    persisted_surface = bool(session.active.fixed_snapshot(session.source).supports)
                    if (
                        published
                        and persisted_surface
                        and (
                            scenario != "corrected" or math.isclose(live["robot_pose"]["yaw"], 0.15)
                        )
                    ):
                        break
                await asyncio.sleep(0.01)
            else:
                raise AssertionError("automatic checkpoint/navigation publication did not settle")
            assert native.selection_observations
            before_selection, selected_auth = native.selection_observations[0]
            # Bootstrap basis exists without navigation permission.
            assert before_selection == (True, False)
            assert selected_auth.status == "selected" and selected_auth.lease_until_steady_ns == 0
            first = catalog["entries"][0]
            if scenario == "matched":
                destination = tmp_path / "map-transfer/authoritative.zip"
                exported = await managed.request(
                    "map.export",
                    operation_id="runtime-export",
                    **first["map_ref"],
                    path=str(destination),
                )
                assert exported["ok"], exported
                from senior_mapping.archive import MapArchive
                from senior_mapping.references import load_snapshot

                manifest, members = MapArchive.inspect_bundle(destination)
                exported_cut = load_snapshot(members["occupancy/occupancy.json"])
                assert manifest.format_version == 2
                assert exported_cut.fixed is not None
                assert exported_cut.fixed.watermark.backend_boot == native.boot
            manifest = await managed.request("map.manifest", **first["map_ref"])
            assert manifest.get("display_only", False) is False
            assert manifest["waypoint_writes"] is True
            state = await managed.state_requests(client).request("state.get")
            assert state["navigation_ready"] is True, {
                "state": state,
                "mapping_phase": managed._runtime.phase,
                "mapping_error": repr(managed._runtime.last_error),
            }
            assert state["localized"] is True
            assert state["map_generation"] == state["localization_generation"]
            assert state["map_grid_id"] == manifest["grid_id"]
            if scenario == "matched":
                from dataclasses import replace

                # Real manager/IPC state cannot be reopened by fresh scans or local-only success.
                native.coverage_roles = {"local"}
                previous_global = native.reference._costmaps["global"]
                native.receipt_cycle += 1
                native.reference.observe_costmap_coverage(
                    "global", replace(previous_global, cycle=native.receipt_cycle, status="invalid")
                )
                native.emit_scan()
                refused = await managed.state_requests(client).request("state.get")
                assert refused["localized"] and not refused["navigation_ready"]
                native.coverage_roles = {"global", "local"}
                native.complete_costmaps()
                restored = await managed.state_requests(client).request("state.get")
                assert restored["navigation_ready"]
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
        directory = (
            tmp_path
            / "maps/checkpoints"
            / hashlib.sha256(first["map_ref"]["map_id"].encode()).hexdigest()
            / latest["checkpoint_id"]
        )
        from senior_mapping.references import load_snapshot

        saved = load_snapshot((directory / "occupancy.json").read_bytes())
        assert saved.fixed is not None
        assert all(
            math.isclose(math.hypot(x, y), 0.5, abs_tol=1e-8)
            for support in saved.fixed.supports
            for x, y in support.local_endpoints
        )
    from senior_mapping.references import load_snapshot

    saved_directory = (
        tmp_path
        / "maps/checkpoints"
        / hashlib.sha256(first["map_ref"]["map_id"].encode()).hexdigest()
        / latest["checkpoint_id"]
    )
    saved_fixed = load_snapshot((saved_directory / "occupancy.json").read_bytes()).fixed
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

        managed._runtime.backend.geometry_profile = admitted_profile()
        running = asyncio.create_task(managed.run())
        try:
            for _ in range(200):
                catalog = await managed.request("map.catalog")
                if catalog["phase"] == ("mapping" if matched else "pending_calibration"):
                    break
                await asyncio.sleep(0.01)
            else:
                raise AssertionError((catalog, repr(managed._runtime.last_error)))
            assert catalog["selection"]["map_ref"] == first["map_ref"]
            assert (await points.home()).waypoint.id == first["home_id"]
            assert native.started == 2
            if matched:
                session = managed._runtime._session
                for _ in range(100):
                    resumed = session.active.fixed_snapshot(session.source)
                    if (
                        resumed.watermark.evidence_revision
                        >= saved_fixed.watermark.evidence_revision + 5
                    ):
                        break
                    await asyncio.sleep(0.01)
                assert {s.support_id for s in resumed.supports} == {
                    s.support_id for s in saved_fixed.supports
                }, "resume must bind current local persistent support"
                assert resumed.watermark.backend_boot != saved_fixed.watermark.backend_boot
                assert resumed.watermark.fixed_revision > saved_fixed.watermark.fixed_revision
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

        managed._runtime.backend.geometry_profile = admitted_profile()
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
    from test_shared_scan import install_synthetic_fixed

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
    bridge._publish_current = lambda *args: None
    fields = dict(
        active=True,
        sequence=1,
        grid_sequence=1,
        map_id="map",
        reference_revision=1,
        generation="reference-1",
    )
    install_synthetic_fixed(bridge, fields)
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
    install_synthetic_fixed(bridge, fields)
    assert reference.pose_reply() == before, "A normal map refresh revoked a still-fresh pose"
    install_synthetic_fixed(bridge, {**fields, "generation": "reference-2"})
    assert not reference.pose_reply()["ok"], "A corrected reference retained the old qualification"


def shared_owner(tmp_path, monkeypatch, *, profile=True):
    """Real session/store/tracker; synthetic-admitted C1 bytes, no ROS process."""
    from dataclasses import asdict

    from senior_mapping.native_session import NativeMappingBackend, NativeSession
    from test_scan_geometry import episode_frame

    from vehicle_core.values import MapRef

    native = NativeFixture(tmp_path / "wire")
    native.boot = "backend"
    native.navigation.begin(native.boot)
    native.sequence = 1
    native.anchors = [dict(id=0, x=0.0, y=0.0, yaw=0.0)]
    clock = {"now": episode_frame(1).capture.time.sample_steady_ns}
    backend = NativeMappingBackend(
        native, None, tmp_path, robot_id="test", home=None, home_id="home"
    )
    backend.geometry_profile = admitted_profile() if profile else None
    backend.clock_ns = lambda: clock["now"]
    entry = SimpleNamespace(map_ref=MapRef("test-map", 1), home_id="home", checkpoint_id=None)
    owner = NativeSession(backend, entry)
    owner.graph.boot = native.boot
    owner._configure_scan_channel(
        dict(
            qualified_scan_schema=1,
            qualified_scan_path=str(tmp_path / "slot"),
            qualified_scan_nonce="nonce",
            host_boot_id="host",
        )
    )
    cut = native.cut()
    cut["capture"] = asdict(episode_frame(1).capture)
    owner._apply(cut)
    native.navigation.observe_cut(cut)
    return owner, native, clock


async def offer_owner_scan(owner, clock, seq, *, offset=0.0, wall=True, beams=None):
    from dataclasses import asdict

    from senior_mapping.shared_scan import ReferenceBasis, ScanEnvelope
    from test_scan_geometry import episode_frame

    cut = owner.active.fixed_snapshot(owner.source)
    scan = episode_frame(seq, offset=offset, wall=wall)
    if beams is not None:
        from dataclasses import replace

        scan = replace(scan, beams=beams)
    clock["now"] = scan.capture.time.sample_steady_ns
    basis = ReferenceBasis(
        cut.watermark.map_ref.map_id,
        cut.watermark.map_ref.reference_revision,
        cut.watermark.reference_generation,
        cut.watermark.graph_sequence,
        cut.watermark.anchor_digest,
        str(owner._graph_cut["keyframe_id"]),
        Pose2D(0, 0, 0),
        Pose2D(0, 0, 0),
    )
    row = ScanEnvelope(
        1,
        seq,
        "valid",
        "",
        "nonce",
        "host",
        "backend",
        scan.key,
        None,
        owner._reference_floor_ns,
        basis,
        scan.valid_until_steady_ns,
        scan,
    )
    owner._scan_path.write_text(json.dumps(asdict(row)))
    owner._scan_path.chmod(0o600)
    await owner.ingest(None)
    return owner.active.fixed_snapshot(owner.source)


@pytest.mark.asyncio
async def test_shared_owner_pending_graph_consumes_independent_scan_and_motion(
    tmp_path, monkeypatch
):
    owner, native, clock = shared_owner(tmp_path, monkeypatch)
    native.pending_graph = True  # PENDING forever; no fabricated graph commits
    for seq in range(2, 5):
        cut = await offer_owner_scan(owner, clock, seq)
    assert owner.graph.sequence == 1
    assert len(cut.supports) == 2
    original = {s.support_id for s in cut.supports}
    wall = max(cut.supports, key=lambda s: s.local_endpoints[0][1])
    for seq, offset in ((5, 0.2), (6, 0.4), (7, 0.6), (8, 0.8)):
        cut = await offer_owner_scan(owner, clock, seq, offset=offset)
    assert wall.support_id in {s.support_id for s in cut.supports}
    assert len(original & {s.support_id for s in cut.supports}) == 1
    for seq in range(9, 13):
        cut = await offer_owner_scan(owner, clock, seq, offset=0.8)
    assert len(cut.supports) == 2
    assert cut.watermark.evidence_revision > 1


async def test_initial_unbound_slot_cannot_retire_the_new_fixed_reference(tmp_path, monkeypatch):
    from senior_mapping.shared_scan import LatestScanSlot

    owner, native, clock = shared_owner(tmp_path, monkeypatch)
    native.pending_graph = True
    monkeypatch.setattr(native, "emit_scan", lambda: None)
    slot = LatestScanSlot(
        owner._scan_path, nonce="nonce", backend_boot="backend", host_boot_id="host"
    )
    native.navigation.slot = slot
    try:
        slot.invalidate("qualified_receipt_unavailable")
        slot.flush()
        await owner.ingest(None)
        assert not owner._fixed_retired
        assert owner._last_scan_capture is None  # Startup status never authorizes a scan.
        assert owner._installed_watermark == owner.active.fixed_snapshot(owner.source).watermark
        slot.invalidate("current_reference_input_invalid")
        slot.flush()
        await owner.ingest(None)
        assert owner._fixed_retired  # Current-reference failure remains authoritative.
    finally:
        slot.close()


async def test_retired_unbound_slot_still_revokes_the_new_reference(tmp_path, monkeypatch):
    from senior_mapping.shared_scan import LatestScanSlot

    owner, native, clock = shared_owner(tmp_path, monkeypatch)
    native.pending_graph = True
    monkeypatch.setattr(native, "emit_scan", lambda: None)
    slot = LatestScanSlot(
        owner._scan_path, nonce="nonce", backend_boot="backend", host_boot_id="host"
    )
    native.navigation.slot = slot
    try:
        slot.retire()
        slot.flush()
        await owner.ingest(None)
        assert owner._fixed_retired
        assert owner._installed_watermark is None
    finally:
        slot.close()


@pytest.mark.asyncio
async def test_shared_owner_invalid_envelope_fences_old_file(tmp_path, monkeypatch):
    owner, native, clock = shared_owner(tmp_path, monkeypatch, profile=False)
    native.pending_graph = True
    cut = await offer_owner_scan(owner, clock, 2)
    assert not cut.supports  # no invented physical profile
    assert 0 not in cut.grid.cells and owner._installed_watermark == cut.watermark
    previous = owner._scan_path.read_bytes()
    row = json.loads(previous)
    row.update(revision=3, status="invalid", scan=None, key=None, valid_until_steady_ns=0)
    owner._scan_path.write_text(json.dumps(row))
    await owner.ingest(None)
    owner._scan_path.write_bytes(previous)
    await owner.ingest(None)
    assert owner._fixed_retired
    assert owner.active.fixed_snapshot(owner.source).watermark == cut.watermark


async def test_shared_owner_occlusion_then_other_anchor_free_removes_only_wall_fragment(
    tmp_path, monkeypatch
):
    from dataclasses import asdict

    from senior_mapping.references import decode_fixed_snapshot, encode_fixed_snapshot
    from senior_mapping.shared_scan import Beam
    from test_scan_geometry import episode_frame

    owner, native, clock = shared_owner(tmp_path, monkeypatch)
    native.pending_graph = True
    for sequence in range(2, 5):
        cut = await offer_owner_scan(owner, clock, sequence)
    object_support = min(cut.supports, key=lambda s: s.local_endpoints[0][1])
    wall_support = max(cut.supports, key=lambda s: s.local_endpoints[0][1])
    # Missing/occluded beams never accumulate free, regardless of elapsed span.
    for sequence in range(5, 17):
        cut = await offer_owner_scan(
            owner, clock, sequence, beams=(Beam(0, (0, 0.025), None, "invalid"),)
        )
    assert cut.supports == (
        tuple(sorted((object_support, wall_support), key=lambda s: s.support_id))
    )
    native.sequence = 2
    native.anchors.append(dict(id=1, x=0.0, y=0.0, yaw=0.0))
    graph = native.cut()
    graph["capture"] = asdict(episode_frame(16).capture)
    owner._apply(graph)
    native.navigation.observe_cut(graph)
    for sequence in range(17, 31):
        cut = await offer_owner_scan(
            owner, clock, sequence, beams=(Beam(0, (0, 0.025), (6, 0.025), "hit"),)
        )
    remaining = next(s for s in cut.supports if s.support_id == object_support.support_id)
    assert remaining.anchor_id == "0"
    assert len(remaining.local_cells) < len(object_support.local_cells)
    removed = set(object_support.local_cells) - set(remaining.local_cells)
    assert all(
        (math.floor(x / 0.05), math.floor(y / 0.05)) not in removed
        for x, y in remaining.local_endpoints
    )
    assert next(s for s in cut.supports if s.support_id == wall_support.support_id) == wall_support
    assert cut.basis.anchor_id == "1"
    assert any(tile.anchor_id == "1" for tile in cut.free_tiles)
    assert decode_fixed_snapshot(encode_fixed_snapshot(cut)) == cut


@pytest.mark.parametrize("retirement", ["source", "capture"])
async def test_production_save_keeps_independent_q_and_observed_retirement_fences_late_file(
    tmp_path, native_pair, monkeypatch, retirement
):
    import threading

    from senior_mapping import checkpoints as checkpoint_module
    from senior_mapping.shared_scan import read_latest_scan

    native, client = native_pair
    async with Database(tmp_path / "robot.db") as database:
        points = WaypointStore(database, robot_id="r1")
        await points.prepare()
        managed = await mapping_requests(
            database,
            client,
            client,
            robot_id="r1",
            root=tmp_path / "maps",
            waypoints=points,
            checkpoint_delay_s=30,
        )
        running = asyncio.create_task(managed.run())
        writer = None
        release = threading.Event()
        capture_failed = False
        try:
            for _ in range(100):
                if (await managed.request("map.catalog"))["runtime_available"]:
                    break
                await asyncio.sleep(0.01)
            assert (
                await managed.request(
                    "map.create",
                    operation_id="concurrent-save",
                    name="capture",
                    expected_generation=0,
                )
            )["ok"]
            session = managed._runtime._session
            for _ in range(100):
                if session.graph.sequence == 4:
                    break
                await asyncio.sleep(0.01)
            assert session.graph.sequence == 4
            entered = threading.Event()
            original = checkpoint_module._publish_files

            def held_disk(*args):
                entered.set()
                assert release.wait(4)
                return original(*args)

            monkeypatch.setattr(checkpoint_module, "_publish_files", held_disk)
            writer = asyncio.create_task(session._save(0))
            assert await asyncio.to_thread(entered.wait, 3)
            before = session.active.fixed_snapshot(session.source)
            key = native.navigation._last_key
            # Independent source callbacks continue while the graph/fixed cut
            # is frozen and the actual durable writer is held at its disk boundary.
            for _ in range(2):
                await asyncio.sleep(0.02)
                native.emit_scan()
            assert native.navigation._last_key.sequence > key.sequence
            row = read_latest_scan(
                session._scan_path,
                0,
                nonce=session._scan_nonce,
                backend_boot=session.graph.boot,
                host_boot_id=session._host_boot_id,
            ).envelope
            assert row.scan is not None
            assert row.valid_until_steady_ns == row.scan.capture.time.sample_steady_ns + 500_000_000
            assert session.active.fixed_snapshot(session.source).watermark == before.watermark
            previous_file = session._scan_path.read_bytes()
            release.set()
            await writer
            monkeypatch.setattr(checkpoint_module, "_publish_files", original)
            if retirement == "capture":
                from vehicle_core.errors import DomainError

                durable_id = session.entry.checkpoint_id
                native.fail_capture = True
                with pytest.raises(DomainError):
                    await session._save(0)
                capture_failed = True
                assert session.entry.checkpoint_id == durable_id
            else:
                native.navigation.invalidate_current("synthetic-source-retired", retired=True)
                native.navigation.slot.flush()
            for _ in range(30):
                await session.ingest(None)
                if session._fixed_retired:
                    break
                await asyncio.sleep(0.01)
            assert session._fixed_retired and not session.localized
            after = session.active.fixed_snapshot(session.source)
            session._scan_path.write_bytes(previous_file)
            await session.ingest(None)
            assert session.active.fixed_snapshot(session.source) == after
        finally:
            release.set()
            if writer is not None:
                await asyncio.gather(writer, return_exceptions=True)
            running.cancel()
            await asyncio.gather(running, return_exceptions=True)
            if capture_failed:
                from vehicle_core.errors import DomainError

                with pytest.raises((DomainError, RuntimeError)):
                    await managed.close()
            else:
                await managed.close()


async def test_shared_owner_capacity_rejection_keeps_old_bound_supports(tmp_path, monkeypatch):
    from senior_mapping.shared_scan import Beam

    owner, native, clock = shared_owner(tmp_path, monkeypatch)
    native.pending_graph = True
    for sequence in range(2, 5):
        before = await offer_owner_scan(owner, clock, sequence)
    tracker = owner._evidence.tracker
    after = await offer_owner_scan(owner, clock, 5, beams=(Beam(0, (0, 0), (500, 0), "hit"),))
    assert after == before
    # This existing 500m engineering fixture is outside the deployed 20m
    # range. Store atomicity is unchanged; C2 must observe the real rejected
    # frame rather than manufacturing a sequence gap on the next complete one.
    assert owner._evidence.tracker._previous.key.sequence == 5
    assert owner._evidence.reason == "coverage_limit"
    assert not owner._fixed_retired
    assert native.reference.coverage_reason == "coverage_limit"
    identities = tracker.bound_support_ids
    for sequence in range(6, 10):
        cut = await offer_owner_scan(owner, clock, sequence)
    assert {s.support_id for s in cut.supports} == set(identities)
    assert owner._evidence.tracker.bound_support_ids == identities


async def test_shared_owner_graph_correction_overlaps_independent_wall_before_object_motion(
    tmp_path, monkeypatch
):
    from dataclasses import asdict

    from test_scan_geometry import episode_frame

    owner, native, clock = shared_owner(tmp_path, monkeypatch)
    native.pending_graph = True
    for sequence in range(2, 5):
        cut = await offer_owner_scan(owner, clock, sequence, wall=False)
    object_support = cut.supports[0]

    def graph_commit(sequence, sample):
        native.sequence = sequence
        graph = native.cut()
        graph["capture"] = asdict(episode_frame(sample).capture)
        owner._apply(graph)
        native.navigation.observe_cut(graph)
        return owner.active.fixed_snapshot(owner.source)

    native.anchors.append(dict(id=1, x=0.0, y=0.0, yaw=0.0))
    graph_commit(2, 4)
    for sequence in range(5, 9):
        cut = await offer_owner_scan(owner, clock, sequence)
    wall = next(s for s in cut.supports if s.anchor_id == "1")
    native.anchors[1].update(x=-1.5, y=-1.5)
    native.anchors.append(dict(id=2, x=0.0, y=0.0, yaw=0.0))
    corrected = graph_commit(3, 8)
    raster = owner.active._store._fixed_raster
    poses = {a.anchor_id: a.pose for a in corrected.anchors}
    overlap = (
        raster.support_projection(object_support, poses["0"]).occupied
        & raster.support_projection(wall, poses["1"]).occupied
    )
    assert overlap, "independent contributions must really share cells after graph correction"
    old_x = math.floor((4.5 - corrected.grid.origin.x) / 0.05)
    old_y = math.floor((1.55 - corrected.grid.origin.y) / 0.05)
    assert corrected.grid.cells[old_y * corrected.grid.width + old_x] == 255
    for sequence in range(9, 12):
        await offer_owner_scan(owner, clock, sequence)
    for sequence, offset in ((12, 0.2), (13, 0.4), (14, 0.6), (15, 0.8)):
        cut = await offer_owner_scan(owner, clock, sequence, offset=offset)
    assert object_support.support_id not in {s.support_id for s in cut.supports}
    assert next(s for s in cut.supports if s.support_id == wall.support_id) == wall
    x0, y0, _, _ = raster.extent
    assert all(cut.grid.cells[(y - y0) * cut.grid.width + x - x0] == 100 for x, y in overlap)


def legacy_wall_consumer():
    """Admitted synthetic legacy O at (60, 0), independent of the T2 curve."""
    from senior_mapping.active import ActiveMap
    from senior_mapping.evidence import OccupancyThresholds, SourceIdentity
    from senior_mapping.fixed_evidence import FixedEvidenceConsumer
    from senior_mapping.references import AnchorPose, AnchorSnapshot, GridExtent, LegacyBase
    from senior_mapping.workspace import WorkspaceGrid, fixed_grid_id
    from test_scan_geometry import episode_frame

    from vehicle_core.values import MapRef

    ref, pose, origin = MapRef("legacy-independence", 1), Pose2D(0, 0, 0), Pose2D(0, -1, 0)
    cells = bytearray(b"\xff") * (120 * 80)
    index = 20 * 120 + 60
    cells[index] = 100
    grid = WorkspaceGrid(
        ref,
        fixed_grid_id(ref, 120, 80, 0.05, origin, bytes(cells)),
        120,
        80,
        0.05,
        origin,
        bytes(cells),
        True,
        False,
    )
    mask = bytearray((len(cells) + 7) // 8)
    mask[index // 8] |= 1 << (index % 8)
    base = LegacyBase(
        grid.grid_id,
        grid,
        "legacy-derived",
        "a" * 64,
        "synthetic-independent-legacy-wall",
        bytes(mask),
        (),
    )
    active, source, clock = ActiveMap(ref, "odom", 0.05), object(), {"now": 0}
    active.bind_verified_source(source, backend_boot="backend", source_epoch="source")
    active.commit(
        source,
        1,
        "0",
        AnchorSnapshot(
            ref,
            "odom",
            (AnchorPose("0", pose, True),),
            GridExtent(120, 80, 0, -1),
        ),
        (),
    )
    consumer = FixedEvidenceConsumer(
        OccupancyThresholds(0.05, 3, 1, 3, 1, 0.1, 1, 1, 2, -1, -5, 5, 2, 0.5),
        profile=admitted_profile(),
        clock=lambda: clock["now"],
    )
    active.bind_fixed_context(
        source,
        source_identity=SourceIdentity.from_key(episode_frame(1).key),
        host_boot_id="host",
        acceptance_floor=None,
        acceptance_floor_steady_ns=0,
        accepting=lambda: True,
        clock=lambda: clock["now"],
        profiles=consumer.profiles(),
        legacy_base=base,
    )
    return active, source, consumer, clock, index


def consume_legacy_scan(active, source, consumer, clock, scan):
    from senior_mapping.evidence import VerifiedBasis
    from senior_mapping.references import decode_fixed_snapshot, encode_fixed_snapshot

    clock["now"] = scan.capture.time.sample_steady_ns
    cut = active.fixed_snapshot(source)
    pose = Pose2D(0, 0, 0)
    basis = VerifiedBasis(
        "0",
        pose,
        pose,
        1,
        cut.watermark.anchor_digest,
        cut.watermark.map_ref,
        active.map_generation,
    )
    active.accept_scan(source, scan, basis)
    cut = consumer.consume(active, source, scan, basis)
    assert decode_fixed_snapshot(encode_fixed_snapshot(cut)) == cut
    return cut


def legacy_target(cut, index):
    grid = cut.grid
    x = int(round((3.025 - grid.origin.x) / 0.05 - 0.5))
    y = int(round((0.025 - grid.origin.y) / 0.05 - 0.5))
    return (
        bool(cut.legacy_base.live_mask[index // 8] & (1 << (index % 8))),
        grid.cells[y * grid.width + x],
    )


def test_legacy_wall_is_independent_of_overlapping_new_curve_and_its_motion():
    from test_scan_geometry import episode_frame

    active, source, consumer, clock, index = legacy_wall_consumer()
    states = []
    for seq, offset in ((2, 0), (3, 0), (4, 0), (5, 0.2), (6, 0.4), (7, 0.6), (8, 0.8)):
        cut = consume_legacy_scan(
            active, source, consumer, clock, episode_frame(seq, offset=offset, wall=False)
        )
        states.append(legacy_target(cut, index))
        assert not cut.free_tiles
        if seq == 4:
            assert len(cut.supports) == 1
        if seq == 6:
            assert not cut.supports
    # Codec round-trips above check the live bit as well as the rendered O.
    assert states == [(True, 100)] * 7


def test_restored_occupied_hit_keeps_prior_before_real_free_clears_wall():
    from dataclasses import replace

    from senior_mapping.shared_scan import Beam, ScanKey
    from test_scan_geometry import episode_frame

    from vehicle_core.sample_time import SampleTime

    active, source, consumer, clock, index = legacy_wall_consumer()
    consumer.profile = None
    beliefs, states = [], []
    for seq in range(2, 9):
        scan = episode_frame(seq, wall=False)
        ns = 10_000_000_000 + seq * 400_000_000
        capture = replace(
            scan.capture,
            time=SampleTime(
                1_700_000_000_000_000_000 + ns,
                1_700_000_000_000_000_000 + ns,
                ns,
                ns,
                1,
            ),
        )
        scan = replace(
            scan,
            key=ScanKey.of("backend", capture),
            capture=capture,
            valid_until_steady_ns=ns + 500_000_000,
            odom_base=Pose2D(0, 0, 0),
            beams=(Beam(0, (0, 0.025), (3.025 if seq == 2 else 4.025, 0.025), "hit"),),
        )
        cut = consume_legacy_scan(active, source, consumer, clock, scan)
        beliefs.append(consumer.free.cells[(60, 0)].belief)
        states.append(legacy_target(cut, index))
    assert beliefs == [5, 4, 3, 2, 1, 0, -1]
    assert states == [(True, 100)] * 6 + [(False, 0)]


async def test_shared_owner_capacity_keeps_motion_revocation_through_following_frames(
    tmp_path,
    monkeypatch,
):
    from dataclasses import replace

    from senior_mapping.references import decode_fixed_snapshot, encode_fixed_snapshot
    from senior_mapping.shared_scan import Beam
    from test_scan_geometry import episode_frame

    def stationary_frame(seq, **kw):
        scan = episode_frame(seq, **kw)
        return replace(
            scan,
            odom_base=Pose2D(0, 0, 0),
            beams=tuple(replace(b, origin_odom=(0, 0)) for b in scan.beams),
        )

    monkeypatch.setattr("test_scan_geometry.episode_frame", stationary_frame)
    owner, native, clock = shared_owner(tmp_path, monkeypatch)
    native.pending_graph = True
    rejected, committed = [], []
    apply = owner.active.apply_evidence

    def record_capacity(*args, **kwargs):
        from vehicle_core.errors import DomainError

        before = owner.active.fixed_snapshot(owner.source)
        delta = kwargs["delta"]
        assert delta.expected == before.watermark
        try:
            result = apply(*args, **kwargs)
        except DomainError as exc:
            if "fixed_capacity" in str(exc):
                assert owner.active.fixed_snapshot(owner.source) == before
                rejected.append(delta)
            raise
        committed.append(delta)
        return result

    monkeypatch.setattr(owner.active, "apply_evidence", record_capacity)
    object_id, wall = None, None
    states = []
    for seq in range(2, 19):
        offset = 0 if seq <= 10 else min(0.8, (seq - 10) * 0.2)
        points = [b.endpoint_odom for b in stationary_frame(seq, offset=offset, wall=True).beams]
        # 130 curve/wall hits + 39 sparse 14m background echoes; all within 20m.
        for i in range(48):
            angle = 2 * math.pi * i / 48
            if min(angle, 2 * math.pi - angle) >= 0.65:
                points.append((14 * math.cos(angle), 14 * math.sin(angle)))
        points.sort(key=lambda p: math.atan2(p[1], p[0]))
        beams = tuple(Beam(i, (0, 0), p, "hit") for i, p in enumerate(points))
        assert len(beams) == 169
        assert max(math.hypot(*p) for p in points) < 20
        cut = await offer_owner_scan(owner, clock, seq, offset=offset, beams=beams)
        if seq == 4:
            object_id = min(cut.supports, key=lambda s: s.local_endpoints[0][1]).support_id
            wall = max(cut.supports, key=lambda s: s.local_endpoints[0][1])
        if seq >= 12:
            states.append(
                (
                    seq,
                    object_id in {s.support_id for s in cut.supports},
                    object_id in owner._evidence.tracker.bound_support_ids,
                )
            )
            assert next(s for s in cut.supports if s.support_id == wall.support_id) == wall
            assert wall.support_id in owner._evidence.tracker.bound_support_ids
            if seq == 12:
                assert owner._evidence.reason == "coverage_limit"
                assert native.reference.coverage_reason == "coverage_limit"
            assert decode_fixed_snapshot(encode_fixed_snapshot(cut)) == cut
    assert len(rejected) == 1 and rejected[0].key.sequence == 12
    assert {r.support_id for r in rejected[0].revocations} == {object_id}
    fallback = next(d for d in committed if d.key.sequence == 12)
    assert fallback.expected == rejected[0].expected and fallback.key == rejected[0].key
    assert fallback.evidence_revision == rejected[0].evidence_revision
    assert fallback.revocations == rejected[0].revocations
    assert not (
        fallback.upsert_supports
        or fallback.free_tiles
        or fallback.support_cell_removals
        or fallback.legacy_edits
    )
    assert states == [(seq, False, False) for seq in range(12, 19)]
    assert owner.graph.sequence == 1 and not owner._fixed_retired
