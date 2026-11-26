"""Task7 real production episode; collection never launches ROS.

Explicit SENIOR_SHARED_LAYERS_PROCESS=1, ROS_LOCALHOST_ONLY=1 and a VM-reserved
ROS_DOMAIN_ID are required. Portable helper checks do not qualify the ROS chain.
"""

import asyncio
import hashlib
import json
import math
import os
import threading
import time
from pathlib import Path

import pytest

native_only = pytest.mark.skipif(
    os.environ.get("SENIOR_SHARED_LAYERS_PROCESS") != "1",
    reason="unverified: explicit isolated ROS/Launch/native execution required",
)


def support_points(cut, support):
    anchor = next(a.pose for a in cut.anchors if a.anchor_id == support.anchor_id)
    c, s = math.cos(anchor.yaw), math.sin(anchor.yaw)
    return [
        (anchor.x + c * x - s * y, anchor.y + s * x + c * y) for x, y in support.local_endpoints
    ]


def object_supports(cut, offset):
    return {
        support.support_id
        for support in cut.supports
        if any(
            abs(x - (3 + offset)) < 0.09 and abs(y) < 0.35 for x, y in support_points(cut, support)
        )
    }


def assert_same_fixed(actual, expected):
    from senior_mapping.references import validate_fixed_snapshot

    # validate renders through the original FixedRaster and compares every O/F/U cell.
    validate_fixed_snapshot(actual)
    validate_fixed_snapshot(expected)
    assert (
        actual.grid.width,
        actual.grid.height,
        actual.grid.resolution,
        actual.grid.origin,
        actual.grid.cells,
    ) == (
        expected.grid.width,
        expected.grid.height,
        expected.grid.resolution,
        expected.grid.origin,
        expected.grid.cells,
    )
    assert {s.support_id: s.local_cells for s in actual.supports} == {
        s.support_id: s.local_cells for s in expected.supports
    }
    assert actual.legacy_base == expected.legacy_base
    assert {tile.tile_key: tile.active_mask for tile in actual.free_tiles} == {
        tile.tile_key: tile.active_mask for tile in expected.free_tiles
    }
    # Source/counter metadata may advance on later genuine scans; masks and every
    # O/F/U cell must agree. The original decoder validates each provenance itself.


@native_only
async def test_owner_adapter_native_capture_restore_and_retirement(tmp_path, monkeypatch):
    pytest.importorskip("rclpy", reason="unverified: ROS/colcon unavailable")
    from senior_mapping import checkpoints as checkpoint_module
    from senior_mapping.archive import STABLE_LAYER, MapArchive
    from senior_mapping.references import encode_fixed_snapshot, load_snapshot
    from senior_mapping.shared_scan import read_latest_scan
    from shared_layers_native_fixture import NativeFixture, admitted_profile
    from vehicle_manager.agent.mapping_catalog import mapping_requests
    from vehicle_manager.storage.bindings import Database
    from vehicle_manager.storage.domain_bindings import WaypointStore

    from vehicle_core.ipc import IpcClient, IpcError

    release = threading.Event()
    async with NativeFixture(tmp_path / "native") as harness:
        wire = IpcClient(harness.socket)
        captures = []

        class ObservedClient:
            async def request(self, kind, **fields):
                if kind != "map.backend.capture":
                    return await wire.request(kind, **fields)
                started = time.monotonic_ns()
                harness.trace.emit("capture_rpc_begin", boot_id=fields.get("boot_id"))
                try:
                    return await wire.request(kind, **fields)
                finally:
                    completed = time.monotonic_ns()
                    captures.append((started, completed))
                    harness.trace.emit(
                        "capture_rpc_end",
                        started_ns=started,
                        completed_ns=completed,
                        timing_scope="RPC upper bound; native SaveToFile overlap unknown",
                    )

        client = ObservedClient()
        managed = running = writer = None
        try:
            # Real map_server/AMCL/planner + real warmed DedicatedController first.
            await harness.wait(lambda: harness.static_ready(client), timeout=60)
            await harness.consumer_parameters()
            old_actions = set(harness.startup.groups)
            overlay_path = harness.parameters
            async with Database(tmp_path / "catalog.db") as database:
                points = WaypointStore(database, robot_id="shared-process-test")
                await points.prepare()
                managed = await mapping_requests(
                    database,
                    client,
                    client,
                    robot_id="shared-process-test",
                    root=tmp_path / "maps",
                    waypoints=points,
                    checkpoint_delay_s=30,
                )
                assert managed is not None
                managed._runtime.backend.geometry_profile = admitted_profile()
                running = asyncio.create_task(managed.run())

                async def idle():
                    return (await managed.request("map.catalog"))["runtime_available"]

                await harness.wait(idle)
                reply = await managed.request(
                    "map.create",
                    operation_id="create-process-map",
                    name="isolated synthetic scan",
                    expected_generation=0,
                )
                assert reply["ok"], reply
                session = managed._runtime._session
                assert session is not None
                await harness.wait(lambda: harness.ready(client, session), timeout=60)
                assert harness.startup.phase == "live"
                assert harness.parameters == overlay_path
                assert harness.startup.contract["mode"] == "live"
                exits = [
                    event
                    for event in harness.events
                    if event[0] == "exit" and event[1] in old_actions
                ]
                assert {event[1] for event in exits} == old_actions
                assert all(event[2] == 0 for event in exits)
                live_starts = [
                    event
                    for event in harness.events
                    if event[0] == "start" and event[1] in harness.startup.groups
                ]
                assert live_starts and min(event[3] for event in live_starts) > max(
                    e[3] for e in exits
                )
                await harness.consumer_parameters()
                harness.trace.emit(
                    "live_parameters", path=str(overlay_path), contents=overlay_path.read_text()
                )

                def stable_A():
                    cut = session.active.fixed_snapshot(session.source)
                    return cut if object_supports(cut, 0) else None

                before = await harness.wait(stable_A)
                old_object = object_supports(before, 0)
                independent_wall = {
                    support.support_id
                    for support in before.supports
                    if support.support_id not in old_object
                }
                assert independent_wall, "original independent background wall must be stable"
                original_graph = session.graph.sequence
                native_before = await harness.native_watermark()
                assert native_before[0] == session.graph.boot and native_before[1] == original_graph
                original_key = before.watermark.evidence_key
                raw_cut = dict(session._graph_cut)
                sensor = raw_cut["sensor"]
                raw_A = [
                    (
                        sensor[0]
                        + distance
                        * math.cos(
                            sensor[2] + raw_cut["angle_min"] + i * raw_cut["angle_increment"]
                        ),
                        sensor[1]
                        + distance
                        * math.sin(
                            sensor[2] + raw_cut["angle_min"] + i * raw_cut["angle_increment"]
                        ),
                    )
                    for i, distance in enumerate(raw_cut["ranges"])
                    if distance is not None
                ]
                assert any(abs(x - 3) < 0.09 and abs(y) < 0.35 for x, y in raw_A)
                harness.trace.emit(
                    "raw_native_A",
                    graph_sequence=original_graph,
                    capture=raw_cut["capture"],
                    ranges_sha256=hashlib.sha256(
                        json.dumps(raw_cut["ranges"]).encode()
                    ).hexdigest(),
                )
                entered = threading.Event()
                write_files = checkpoint_module._publish_files

                def held_disk(*args):
                    entered.set()
                    assert release.wait(10), "test disk barrier was not released"
                    return write_files(*args)

                monkeypatch.setattr(checkpoint_module, "_publish_files", held_disk)
                writer = asyncio.create_task(session._save(0))
                assert await asyncio.to_thread(entered.wait, 10)
                harness.source.offset = 0.4

                def during_save():
                    row = harness.observe_slot(session)
                    if (
                        row
                        and row["key"]["sequence"] > original_key.sequence
                        and harness.index.pair(row, "q", "qualified", "tf", "slot")
                    ):
                        return row
                    return None

                newer = await harness.wait(during_save, timeout=5)
                assert session._capturing and session.graph.sequence == original_graph
                release.set()
                await writer
                writer = None
                monkeypatch.setattr(checkpoint_module, "_publish_files", write_files)

                def stable_B():
                    cut = session.active.fixed_snapshot(session.source)
                    if object_supports(cut, 0.4) and old_object.isdisjoint(
                        support.support_id for support in cut.supports
                    ):
                        return cut
                    return None

                moved = await harness.wait(stable_B)
                assert independent_wall <= {support.support_id for support in moved.supports}
                assert moved.watermark.graph_sequence == original_graph
                assert session.graph.sequence == original_graph
                assert await harness.native_watermark() == native_before
                await harness.wait(
                    lambda: any(
                        bytes(value % 256 for value in message.data) == moved.grid.cells
                        and message.info.width == moved.grid.width
                        and message.info.height == moved.grid.height
                        for message in harness.maps
                    )
                )
                assert moved.watermark.evidence_key.sequence > newer["key"]["sequence"]
                assert moved.watermark.evidence_revision > before.watermark.evidence_revision
                assert session._installed_watermark == moved.watermark
                assert raw_cut["ranges"] == session._graph_cut["ranges"], "native raw must retain A"
                harness.trace.emit(
                    "fixed_revocation",
                    removed_support_ids=sorted(old_object),
                    retained_wall_ids=sorted(independent_wall),
                    graph_sequence=session.graph.sequence,
                    fixed_revision=moved.watermark.fixed_revision,
                )
                # Observer overlap with RPC is only an upper bound. No invented native publish time.
                overlapping = sum(
                    start <= row["sample_steady_ns"] <= end
                    for start, end in captures
                    for stages in harness.index.rows.values()
                    for row in [stages.get("q")]
                    if row
                )
                harness.trace.emit(
                    "capture_overlap",
                    q_samples_inside_rpc=overlapping,
                    native_SaveToFile_overlap="unverified: no native substage timestamps",
                )

                # S07: true ProductionMappingRequests.export and original archive regrid.
                destination = (tmp_path / "map-transfer" / "shared-process.zip").resolve()
                reply = await managed.request(
                    "map.export",
                    operation_id="export-process-map",
                    map_id=session.entry.map_ref.map_id,
                    reference_revision=session.entry.map_ref.reference_revision,
                    path=str(destination),
                )
                assert reply["ok"], reply
                archive_bytes = destination.read_bytes()
                archive_hash = hashlib.sha256(archive_bytes).hexdigest()
                manifest, files = MapArchive.inspect_bundle(destination)
                exported = load_snapshot(files[STABLE_LAYER]).fixed
                assert manifest.format_version == 2 and exported is not None
                assert_same_fixed(exported, session.active.fixed_snapshot(session.source))
                assert old_object.isdisjoint(s.support_id for s in exported.supports)
                grid = exported.grid
                expected_pixels = bytes(
                    {100: 0, 0: 254, 255: 205}[grid.cells[y * grid.width + x]]
                    for y in reversed(range(grid.height))
                    for x in range(grid.width)
                )
                assert files["grid/map.pgm"].split(b"\n", 3)[-1] == expected_pixels
                proof = json.loads(files["pose_graph/native-proof.json"])
                assert proof["sequence"] == original_graph
                assert "pose_graph/graph.data" in files and "pose_graph/graph.posegraph" in files
                harness.trace.emit(
                    "export",
                    path=str(destination),
                    sha256=archive_hash,
                    pair_digest=proof["pair_digest"],
                    cells=len(grid.cells),
                )

                # Close through original session owner before new raw/backend.
                # The catalog remains the sole selector.
                async with managed._runtime._updates:
                    await managed._runtime._retire()
                old_boot = session.graph.boot
                old_floor = session._reference_floor_ns
                harness.new_raw_source()
                selection = await managed._runtime.catalog.selection()
                reply = await managed.request(
                    "map.activate",
                    operation_id="restore-process-map",
                    map_id=manifest.map_ref.map_id,
                    reference_revision=manifest.map_ref.reference_revision,
                    expected_generation=selection.generation,
                )
                assert reply["ok"], reply
                resumed = managed._runtime._session
                fresh = resumed.active.fixed_snapshot(resumed.source)
                assert fresh.watermark.backend_boot != exported.watermark.backend_boot
                assert (
                    fresh.watermark.reference_generation != exported.watermark.reference_generation
                )
                assert fresh.watermark.source.raw_gid != exported.watermark.source.raw_gid
                assert fresh.watermark.evidence_key is None
                assert_same_fixed(fresh, exported)
                assert hashlib.sha256(destination.read_bytes()).hexdigest() == archive_hash
                restored_destination = destination.with_name("restored-shared-process.zip")
                reply = await managed.request(
                    "map.export",
                    operation_id="export-restored-process-map",
                    map_id=manifest.map_ref.map_id,
                    reference_revision=manifest.map_ref.reference_revision,
                    path=str(restored_destination),
                )
                assert reply["ok"], reply
                _, restored_files = MapArchive.inspect_bundle(restored_destination)
                restored_export = load_snapshot(restored_files[STABLE_LAYER]).fixed
                assert_same_fixed(restored_export, exported)
                assert restored_files["grid/map.pgm"] == files["grid/map.pgm"]
                harness.trace.emit(
                    "restored_export",
                    path=str(restored_destination),
                    sha256=hashlib.sha256(restored_destination.read_bytes()).hexdigest(),
                )
                # R/source floors and actual fresh capture are required before readiness returns.
                assert resumed._reference_floor_ns > old_floor
                await harness.wait(lambda: harness.ready(client, resumed), timeout=60)
                latest = harness.observe_slot(resumed)
                assert latest["key"]["backend_boot"] != old_boot
                assert latest["sample_steady_ns"] > resumed._reference_floor_ns
                assert all(
                    row["key"]["backend_boot"] != old_boot for row in harness.coverage_rows.values()
                )

                # S09: retained real Q/slot, no new scan and an old fixed install input.
                old_slot = tmp_path / "captured-old-slot.json"
                old_slot.write_bytes(resumed._scan_path.read_bytes())
                old_slot.chmod(0o600)
                captured = read_latest_scan(
                    old_slot,
                    0,
                    nonce=resumed._scan_nonce,
                    backend_boot=resumed.graph.boot,
                    host_boot_id=resumed._host_boot_id,
                ).envelope
                assert captured and captured.scan
                from shared_layers_native_fixture import capture_record

                from tools.shared_layers_system_trace import identity

                old_row = capture_record(
                    captured.key.backend_boot,
                    captured.scan.host_boot_id,
                    captured.scan.capture,
                    deadline=captured.valid_until_steady_ns,
                )
                harness.source.enabled = False

                def captured_q():
                    return next(
                        (
                            message
                            for message in reversed(harness.current)
                            if message.valid
                            and identity(
                                capture_record(
                                    message.backend_boot,
                                    message.host_boot_id,
                                    message.capture,
                                    deadline=message.valid_until_steady_ns,
                                )
                            )
                            == identity(old_row)
                        ),
                        None,
                    )

                old_q = await harness.wait(captured_q)
                old_deadline = old_q.valid_until_steady_ns

                async def retired_without_scan():
                    state = await client.request("state.get")
                    return (
                        time.monotonic_ns() > old_deadline
                        and state.get("navigation_ready") is False
                        and harness.current
                        and not harness.current[-1].valid
                    )

                await harness.wait(retired_without_scan)
                captured = read_latest_scan(
                    old_slot,
                    0,
                    nonce=resumed._scan_nonce,
                    backend_boot=resumed.graph.boot,
                    host_boot_id=resumed._host_boot_id,
                ).envelope
                assert captured.valid_until_steady_ns == old_deadline
                assert captured.valid_until_steady_ns < time.monotonic_ns()
                assert old_q.valid_until_steady_ns == old_deadline
                late = tmp_path / "late-fixed.json"
                payload = encode_fixed_snapshot(moved, transfer=True)
                late.write_bytes(payload)
                late.chmod(0o600)
                denied = await client.request(
                    "map.backend.install_fixed",
                    boot_id=old_boot,
                    path=str(late),
                    sha256=hashlib.sha256(payload).hexdigest(),
                    acceptance_floor_steady_ns=old_floor,
                )
                assert denied.get("ok") is False
                harness.trace.emit(
                    "late_fixed_denied",
                    old_backend=old_boot,
                    reply=denied,
                    old_deadline_ns=old_deadline,
                )

                # Restore via the original catalog/source flow after retirement.
                harness.source.enabled = True
                selection = await managed._runtime.catalog.selection()
                reply = await managed.request(
                    "map.activate",
                    operation_id="fresh-after-retirement",
                    map_id=manifest.map_ref.map_id,
                    reference_revision=manifest.map_ref.reference_revision,
                    expected_generation=selection.generation,
                )
                assert reply["ok"], reply
                active = managed._runtime._session
                state = await harness.wait(lambda: harness.ready(client, active), timeout=60)
                assert active.graph.boot != resumed.graph.boot
                assert active.active.map_generation != resumed.active.map_generation
                assert active._reference_floor_ns > old_deadline
                # R4 normal active route through original agent/RouteService/DedicatedController.
                plan = dict(
                    execution_id="shared-process-route",
                    task_epoch=1,
                    effect_id="shared-process-start",
                    action_ids=["goal"],
                    map_id=state["map_id"],
                    reference_revision=state["reference_revision"],
                    map_generation=state["map_generation"],
                    poses=[[0.7, 0.0, 0.0]],
                )
                accepted = await client.request(
                    "navigation.task_start",
                    execution_id=plan["execution_id"],
                    effect_sequence=1,
                    plan=plan,
                )
                assert accepted.get("ok") and accepted.get("accepted"), accepted
                harness.source.following = True

                async def route_active():
                    progress = await harness.route_status(client, plan)
                    assert progress["status"] not in {"failed", "succeeded", "canceled", "missing"}
                    return progress.get("status") == "running" and progress

                await harness.wait(route_active)
                assert harness.run_directory.exists()
                running.cancel()
                await asyncio.gather(running, return_exceptions=True)
                running = None
                # Adapter receives real Launch shutdown while the original route is active.
                await harness.shutdown()
                # Local manager cleanup can encounter a closed remote socket after actual
                # adapter exit. Preserve that error; it is not a substitute for PID settlement.
                try:
                    await managed.close()
                except IpcError as exc:
                    assert harness.launch_task.result() == 0
                    harness.trace.emit("post_exit_manager_rpc", reason=str(exc))
                managed = None
        finally:
            release.set()
            if writer is not None:
                await asyncio.gather(writer, return_exceptions=True)
            if running is not None:
                running.cancel()
                await asyncio.gather(running, return_exceptions=True)
            if managed is not None:
                await managed.close()
            await wire.close()


@native_only
@pytest.mark.skipif(
    not os.environ.get("SENIOR_SHARED_LAYERS_REFERENCE_TRACE"),
    reason="unverified: separate VM reference measurement dispatch required",
)
async def test_frozen_stationary_system_reference(tmp_path):
    """Stationary sensor/SLAM reference only; not a sustained MPPI or S16 workload.

    Bootstrap/postlude are outside the frozen 300-frame window. No parameter sweep,
    native sleeps or repeated save attempts. UI/loop/match/lock metrics remain N/A.
    """
    pytest.importorskip("rclpy", reason="unverified: ROS/colcon unavailable")
    from senior_mapping.archive import STABLE_LAYER, MapArchive
    from senior_mapping.references import load_snapshot
    from shared_layers_native_fixture import NativeFixture, admitted_profile
    from vehicle_manager.agent.mapping_catalog import mapping_requests
    from vehicle_manager.storage.bindings import Database
    from vehicle_manager.storage.domain_bindings import WaypointStore

    from vehicle_core.ipc import IpcClient

    path = Path(os.environ["SENIOR_SHARED_LAYERS_REFERENCE_TRACE"]).resolve(strict=True)
    async with NativeFixture(tmp_path / "reference") as harness:
        client = IpcClient(harness.socket)
        managed = running = None
        try:
            await harness.wait(lambda: harness.static_ready(client), timeout=60)
            async with Database(tmp_path / "catalog.db") as database:
                waypoints = WaypointStore(database, robot_id="shared-process-test")
                await waypoints.prepare()
                managed = await mapping_requests(
                    database,
                    client,
                    client,
                    robot_id="shared-process-test",
                    root=tmp_path / "maps",
                    waypoints=waypoints,
                    checkpoint_delay_s=120,
                )
                managed._runtime.backend.geometry_profile = admitted_profile()
                running = asyncio.create_task(managed.run())

                async def idle():
                    return (await managed.request("map.catalog"))["runtime_available"]

                await harness.wait(idle)
                reply = await managed.request(
                    "map.create",
                    operation_id="reference-create",
                    name="frozen reference",
                    expected_generation=0,
                )
                assert reply["ok"], reply
                session = managed._runtime._session
                state = await harness.wait(lambda: harness.ready(client, session), timeout=60)
                await harness.consumer_parameters()
                plan = dict(
                    execution_id="shared-process-route",
                    task_epoch=1,
                    effect_id="shared-process-start",
                    action_ids=["goal"],
                    map_id=state["map_id"],
                    reference_revision=state["reference_revision"],
                    map_generation=state["map_generation"],
                    poses=[[0.7, 0.0, 0.0]],
                )
                reply = await client.request(
                    "navigation.task_start",
                    execution_id=plan["execution_id"],
                    effect_sequence=1,
                    plan=plan,
                )
                assert reply.get("ok") and reply.get("accepted"), reply
                harness.trace.emit("route_accepted", reply=reply, active=False)
                await harness.route_status(client, plan)
                # Zero wheel/TF cannot satisfy the original .2m/10s progress checker.
                # Observe the single execution's real terminal; never refill the window.
                harness.source.begin_frozen(path)
                harness.trace.emit("phase", name="stationary_reference")
                saved = False
                degraded = set()
                deadline = time.monotonic() + 70
                while harness.source.tick < 1200:
                    if harness.pump_task.done():
                        await harness.pump_task
                    assert time.monotonic() < deadline, "frozen input delivery did not finish"
                    if harness.source.operations:
                        operation = harness.source.operations.popleft()
                        if operation == "save":
                            assert not saved
                            harness.trace.emit("phase", name="save")
                            await session._save(0)
                            saved = True
                            harness.trace.emit("phase", name="stationary_reference")
                    state = await client.request("state.get")
                    harness.trace.emit("state", record=state)
                    if not state.get("navigation_ready"):
                        degraded.add(str(state.get("navigation_block_reason")))
                    progress = await harness.route_status(client, plan)
                    if progress["status"] in {"failed", "missing", "cancel_pending", "canceled"}:
                        degraded.add("original stationary route: " + progress["status"])
                    await asyncio.sleep(0.1)
                progress = await harness.route_status(client, plan)
                if progress["status"] in {"failed", "missing", "cancel_pending", "canceled"}:
                    degraded.add("original stationary route: " + progress["status"])
                assert saved and list(harness.source.operations) == ["restore_export"]
                harness.trace.emit(
                    "reference_end",
                    frames=300,
                    degradations=sorted(degraded),
                    workload="stationary-only sensor/SLAM",
                    full_load_success=False,
                    sustained_mppi="N/A; original progress requires motion",
                )
                # Same explicit B postlude on both versions, outside measured raw window.
                harness.source.frozen = None
                harness.source.offset = 0.4
                harness.source.operations.clear()
                harness.trace.emit("phase", name="restore_export")
                # Stop/cancel via original route owner before original catalog selection.
                await harness.cancel_route(client, plan)
                await harness.wait(lambda: managed._runtime.backend.stopped())
                selection = await managed._runtime.catalog.selection()
                reply = await managed.request(
                    "map.activate",
                    operation_id="reference-restore",
                    map_id=session.entry.map_ref.map_id,
                    reference_revision=session.entry.map_ref.reference_revision,
                    expected_generation=selection.generation,
                )
                assert reply["ok"], reply
                restored = managed._runtime._session
                await harness.wait(lambda: harness.ready(client, restored), timeout=60)
                destination = (tmp_path / "map-transfer" / "reference.zip").resolve()
                reply = await managed.request(
                    "map.export",
                    operation_id="reference-export",
                    map_id=restored.entry.map_ref.map_id,
                    reference_revision=restored.entry.map_ref.reference_revision,
                    path=str(destination),
                )
                assert reply["ok"], reply
                _, files = MapArchive.inspect_bundle(destination)
                exported = load_snapshot(files[STABLE_LAYER]).fixed
                assert exported is not None
                assert not object_supports(exported, 0)
                harness.trace.emit(
                    "reference_export",
                    sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
                    ui="unverified: Task6 integration required",
                )
                running.cancel()
                await asyncio.gather(running, return_exceptions=True)
                running = None
                await managed.close()
                managed = None
                assert not degraded, sorted(degraded)
        finally:
            if running is not None:
                running.cancel()
                await asyncio.gather(running, return_exceptions=True)
            if managed is not None:
                await managed.close()
            await client.close()


def test_capture_index_pairs_original_sample_not_latest_sequence():
    from shared_layers_native_fixture import CaptureIndex

    index = CaptureIndex(capacity=2)

    def row(sequence):
        return dict(
            host_boot_id="host",
            key=dict(
                backend_boot="backend",
                bridge_boot_id="bridge",
                source_epoch="source",
                capture_generation=1,
                clock_generation=1,
                raw_gid="aa",
                bridge_gid="bb",
                sequence=sequence,
            ),
            stamp_ns=sequence,
            sample_steady_ns=sequence * 200_000_000,
            deadline_ns=sequence * 200_000_000 + 500_000_000,
        )

    first, second = row(1), row(2)
    # Exact original trigger: current[-1] is frame 1, latest slot is frame 2.
    assert first["deadline_ns"] != second["deadline_ns"]
    index.add("q", first)
    index.add("slot", second)
    assert index.pair(second, "q", "slot") is None
    index.add("q", second)
    assert index.pair(second, "q", "slot") is not None
    changed = dict(second, deadline_ns=second["deadline_ns"] + 1)
    with pytest.raises(AssertionError, match="deadline"):
        index.add("slot", changed)
    for sequence in (3, 4, 5):
        index.add("q", row(sequence))
    assert len(index.rows) == 2
    assert index.evicted > 0


def test_frozen_raw_episode_is_bounded_and_keeps_unknown_unknown(tmp_path):
    from shared_layers_sensor_source import frozen_episode, raycast

    frames = frozen_episode()
    assert len(frames) == 300
    assert frames[-1]["offset_ns"] == 59_800_000_000
    assert {frame["beams"] for frame in frames} == {2048}
    assert [frame["operation"] for frame in frames if frame["operation"]] == [
        "save",
        "restore_export",
    ]
    # No intersection is unknown, never a finite max-range free return.
    import math

    assert math.isnan(raycast((0, 0), (1, 0), ()))


def test_system_trace_is_bounded_reports_loss_and_summarizes(tmp_path):
    from tools.shared_layers_system_trace import TraceWriter, summarize_system_trace

    path = tmp_path / "trace.jsonl"
    with TraceWriter(path, dict(run_id="portable", host_boot_id="host"), capacity=1) as writer:
        for i in range(10000):
            writer.emit("control", sequence=i)
    summary = summarize_system_trace(path)
    assert summary["scope"] == "observed_system_trace"
    assert summary["dropped"] > 0
    assert summary["complete"] is True
    assert summary["lock_wait"] == "N/A"
    path.write_text(path.read_text() + "{broken\n")
    with pytest.raises(ValueError, match="line"):
        summarize_system_trace(path)


def test_coverage_matches_each_original_q_with_different_valid_sequences(tmp_path):
    from shared_layers_native_fixture import CaptureIndex

    from tools.shared_layers_system_trace import TraceWriter, summarize_system_trace

    index = CaptureIndex(capacity=4)
    rows = []
    for sequence in (1, 2):
        row = dict(
            host_boot_id="host",
            key=dict(
                backend_boot="backend",
                bridge_boot_id="bridge",
                source_epoch="source",
                capture_generation=1,
                clock_generation=1,
                raw_gid="aa",
                bridge_gid="bb",
                sequence=sequence,
            ),
            stamp_ns=sequence,
            sample_steady_ns=sequence * 100_000_000,
            deadline_ns=sequence * 100_000_000 + 500_000_000,
        )
        index.add("q", row)
        index.add("tf", row)
        index.add("slot", row)
        rows.append(row)
    records = []
    for role, original in zip(("global", "local"), rows, strict=True):
        coverage = dict(
            original,
            scan_deadline_steady_ns=original["deadline_ns"],
            cycle_started_steady_ns=300_000_000,
            cycle_completed_steady_ns=400_000_000,
            valid_until_steady_ns=original["deadline_ns"],
            cleanup_complete=True,
            transform_valid=True,
        )
        assert index.coverage(coverage) == original
        records.append((role, coverage))
    foreign = dict(records[0][1], host_boot_id="other-host")
    assert index.coverage(foreign) is None
    path = tmp_path / "paired.jsonl"
    with TraceWriter(path, dict(run_id="portable", host_boot_id="host")) as trace:
        for row in rows:
            trace.emit("q", **row)
            trace.emit("tf", **row)
            trace.emit("slot", **row)
        for role, row in records:
            trace.emit(role, **row)
    summary = summarize_system_trace(path)
    assert summary["unmatched_coverage"] == 0
    assert summary["sample_to_receive_or_cycle"]["global"]["max_ms"] == 300
    assert summary["sample_to_receive_or_cycle"]["local"]["max_ms"] == 200


def test_trace_proc_identity_does_not_accept_reused_pid():
    from tools.shared_layers_system_trace import alive, process_identity

    original = process_identity(os.getpid())
    assert original is not None and alive(original)
    assert not alive(dict(original, starttime=original["starttime"] + 1))


def test_real_coverage_schema_stays_in_contract_payload(tmp_path):
    from dataclasses import asdict

    from senior_mapping.shared_scan import CurrentLayerCoverage

    from tools.shared_layers_system_trace import TraceWriter

    coverage = asdict(CurrentLayerCoverage())
    path = tmp_path / "coverage.jsonl"
    with TraceWriter(path, {}) as trace:
        # Exact original independent probe input and emission call.
        trace.emit("global", **dict(coverage, deadline_ns=0))
        with pytest.raises(ValueError, match="schema"):
            trace.emit("local", **dict(coverage, schema=2))
    row = json.loads(path.read_text().splitlines()[1])
    assert row["schema"] == 1
    assert row["record"] == dict(coverage, deadline_ns=0)


def coverage_input(sequence, sample):
    """Real production dataclass shape, no ROS or fabricated native result."""
    from dataclasses import asdict

    from senior_mapping.shared_scan import CostmapGeometry, CurrentLayerCoverage, ScanKey

    key = ScanKey("backend", "bridge", "source", 1, 1, "aa", "bb", sequence)
    geometry = CostmapGeometry(10, 10, 0.05, 0.0, 0.0)
    return asdict(
        CurrentLayerCoverage(
            run_nonce="run",
            host_boot_id="host",
            owner_instance="owner",
            role="global",
            plugin_instance="plugin",
            cycle=sequence,
            status="ready",
            key=key,
            backend_boot="backend",
            sample_steady_ns=sample,
            scan_deadline_steady_ns=sample + 500_000_000,
            cycle_started_steady_ns=sample + 100_000_000,
            cycle_completed_steady_ns=sample + 200_000_000,
            valid_until_steady_ns=sample + 500_000_000,
            geometry=geometry,
            master_geometry=geometry,
            updated_bounds=(0.0, 0.0, 0.5, 0.5),
            transform_valid=True,
            cleanup_complete=True,
        )
    )


def test_file_coverage_is_counted_in_its_reference_phase(tmp_path):
    from tools.shared_layers_system_trace import summarize_system_trace

    coverage = coverage_input(1, 1_000_000_000)
    original = {name: coverage[name] for name in ("host_boot_id", "key", "sample_steady_ns")}
    original.update(stamp_ns=1, deadline_ns=1_500_000_000)

    def row(stage, t, **fields):
        return dict(schema=1, stage=stage, steady_ns=t, **fields)

    rows = [
        row("metadata", 0),
        row("q", 1_050_000_000, **original),
        row("phase", 1_100_000_000, name="stationary_reference"),
        row("file", 1_250_000_000, path="/private/global-coverage.json", record=coverage),
        row("phase", 1_300_000_000, name="save"),
        row("trace_end", 2_000_000_000, dropped=0),
    ]
    path = tmp_path / "file.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    result = summarize_system_trace(path)
    assert result["sample_to_receive_or_cycle"]["global"]["max_ms"] == 200
    phase = result["phase_capture_metrics"]["stationary_reference"]
    assert phase["samples"]["global"] == 1 and phase["samples"]["local"] == 0
    assert phase["latency"]["local"] == "N/A"
    assert result["phase_capture_metrics"]["bootstrap"]["samples"]["global"] == 0
    assert result["phase_capture_metrics"]["save"]["latency"]["global"] == "N/A"


async def test_bounded_observe_reads_both_roles_after_bootstrap(tmp_path):
    from shared_layers_native_fixture import NativeFixture

    from tools.shared_layers_system_trace import TraceWriter

    fixture = NativeFixture(tmp_path)
    fixture.run_directory = tmp_path
    tmp_path.chmod(0o700)
    fixture._slot_record = lambda session: None  # Only the file/async-owner boundary is tested.
    with TraceWriter(tmp_path / "trace.jsonl", {}) as trace:
        fixture.trace = trace
        from dataclasses import asdict

        from senior_mapping.shared_scan import CurrentLayerCoverage

        # Native queueCoverageLocked can publish before it has an authorization owner.
        for role in ("global", "local"):
            file = tmp_path / (role + "-coverage.json")
            file.write_text(json.dumps(asdict(CurrentLayerCoverage(role=role))))
            file.chmod(0o600)
        await fixture.observe(None)
        assert not fixture.coverage_rows
        for cycle in (1, 3):
            for role, seq in (("global", cycle), ("local", cycle + 1)):
                record = coverage_input(seq, time.monotonic_ns() - 210_000_000)
                record["role"] = role
                q = dict(
                    record,
                    deadline_ns=record["scan_deadline_steady_ns"],
                    stamp_ns=seq,
                    map_generation=record["reference_generation"],
                )
                fixture.index.add("q", q)
                fixture.index.add("tf", q)
                fixture.index.add("slot", q)
                file = tmp_path / (role + "-coverage.json")
                file.write_text(json.dumps(record))
                file.chmod(0o600)
            await fixture.observe(None)
        assert fixture.coverage_rows["global"]["key"]["sequence"] == 3
        assert fixture.coverage_rows["local"]["key"]["sequence"] == 4
    rows = [json.loads(line) for line in (tmp_path / "trace.jsonl").read_text().splitlines()]
    assert [r["stage"] for r in rows].count("global") == 3
    assert [r["stage"] for r in rows].count("local") == 3


async def test_route_observer_keeps_real_terminal_after_current_disappears(tmp_path):
    from senior_navigation.navigation_adapter import RouteService, _Route
    from senior_robot.agent_adapter import AdapterCore
    from shared_layers_native_fixture import NativeFixture

    from tools.shared_layers_system_trace import TraceWriter
    from vehicle_core.route_plan import RoutePlan

    plan = RoutePlan(
        execution_id="probe",
        map_id="map",
        reference_revision=1,
        task_epoch=1,
        effect_id="start",
        action_ids=["goal"],
        map_generation="generation",
        poses=[[0.7, 0.0, 0.0]],
    )
    service = RouteService(None, qualified=lambda *args: False)
    route = _Route(plan, asyncio.get_running_loop().create_future())
    service._current = route
    service._finish(route, "failed")
    assert service.current is None
    core = AdapterCore(
        robot_id="probe",
        submit=lambda *a, **k: None,
        disconnect=lambda *a: None,
        request_exit=lambda: None,
        routes=service,
    )

    class Client:
        async def request(self, kind, **fields):
            assert kind == "navigation.task_status" and fields["execution_id"] == "probe"
            return await core.handle(dict(type=kind, **fields))

    fixture = NativeFixture(tmp_path)
    with TraceWriter(tmp_path / "route.jsonl", {}) as trace:
        fixture.trace = trace
        assert (await fixture.route_status(Client(), plan.model_dump()))["status"] == "failed"
        service._terminal.clear()
        assert (await fixture.route_status(Client(), plan.model_dump()))["status"] == "missing"


async def test_cancel_requires_original_effect_query_and_output_fence(tmp_path):
    from senior_navigation.navigation_adapter import RouteService, _Route
    from senior_robot.agent_adapter import AdapterCore
    from shared_layers_native_fixture import NativeFixture

    from tools.shared_layers_system_trace import TraceWriter
    from vehicle_core.route_plan import RoutePlan

    calls = []
    retired = False
    plan = RoutePlan(
        execution_id="route",
        map_id="map",
        reference_revision=1,
        task_epoch=1,
        effect_id="start",
        action_ids=["goal"],
        map_generation="generation",
        poses=[[0.7, 0.0, 0.0]],
    )
    service = RouteService(
        None, qualified=lambda *args: False, source_retired=lambda execution: retired
    )
    route = _Route(plan, asyncio.get_running_loop().create_future())
    service._finish(route, "failed")
    core = AdapterCore(
        robot_id="probe",
        submit=lambda *a, **k: None,
        disconnect=lambda *a: None,
        request_exit=lambda: None,
        routes=service,
    )

    class Client:
        async def request(self, kind, **fields):
            nonlocal retired
            calls.append((kind, fields))
            # The real wire owner must first report handled but unconfirmed.
            # Advance only its injected gate predicate, never invent a wire ACK.
            if len(calls) == 4:
                retired = True
            reply = await core.handle(dict(type=kind, **fields))
            if len(calls) == 1:
                assert reply["ok"] and reply["accepted"] is False
                assert reply["controller_output_exited"] is False
            return reply

    fixture = NativeFixture(tmp_path)
    with TraceWriter(tmp_path / "cancel.jsonl", {}) as trace:
        fixture.trace = trace
        await fixture.cancel_route(Client(), dict(execution_id="route", task_epoch=1))
    assert [kind for kind, _ in calls] == ["navigation.task_cancel"] + [
        "navigation.task_status"
    ] * 4
    assert calls[1][1]["effect_sequence"] == 2
    assert calls[1][1]["effect_kind"] == "revoke"


def test_resource_phase_cpu_and_cgroup_keep_original_observations(tmp_path):
    from tools.shared_layers_system_trace import summarize_system_trace

    ticks = os.sysconf("SC_CLK_TCK")
    proc = dict(
        pid=123, starttime=5, cpu_ticks=0, status="VmRSS:\t4 kB\n", smaps_rollup="Pss: 3 kB\n"
    )

    def row(stage, t, **fields):
        return dict(schema=1, stage=stage, steady_ns=t, **fields)

    rows = [
        row("metadata", 0),
        row("resource", 2_000_000_000, process=proc),
        row("phase", 2_900_000_000, name="save"),
        row("resource", 3_000_000_000, process=dict(proc, cpu_ticks=ticks)),
        row(
            "cgroup",
            3_010_000_000,
            cgroup="task",
            values={"memory.current": "4096", "memory.peak": "8192", "cpu.stat": "usage_usec 200"},
        ),
        row("cgroup", 3_210_000_000, cgroup="task", values={"cpu.stat": "usage_usec 300"}),
        row("trace_end", 4_000_000_000, dropped=0),
    ]
    path = tmp_path / "resources.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    result = summarize_system_trace(path)
    assert "cpu_percent_one_core" not in result["phase_process_peaks"]["save"]["123:5"]
    assert result["straddling_cpu_intervals"]["save"]["123:5"] == 1
    assert result["process_peaks"]["123:5"]["cpu_percent_one_core"] == 100
    assert result["cgroup_last"]["task"]["memory.peak"] == "8192"
    assert result["cgroup_last_observed_ns"]["task"]["memory.peak"] == 3_010_000_000
    assert result["cgroup_last_observed_ns"]["task"]["cpu.stat"] == 3_210_000_000


def test_control_window_requires_running_and_excludes_terminal_or_stale_status(tmp_path):
    from tools.shared_layers_system_trace import summarize_system_trace

    def row(stage, t, **fields):
        return dict(schema=1, stage=stage, steady_ns=t, **fields)

    rows = [
        row("metadata", 0),
        row("phase", 1, name="stationary_reference"),
        row("route_accepted", 2, active=False),
        row("state", 3, record=dict(navigation_ready=True)),
        row("control", 4, vx=0.1, wz=0),
        row("route_state", 5, status="starting"),
        row("controller_output", 6, vx=0.1, wz=0),
        row("route_state", 7, status="running"),
        row("controller_output", 8, vx=0.1, wz=0),
        row("control", 9, vx=0.1, wz=0),
        row("control", 1_000_000_000, vx=0.1, wz=0),
        row("route_state", 1_000_000_001, status="failed"),
        row("control", 1_000_000_002, vx=0.1, wz=0),
        row("trace_end", 2_000_000_000, dropped=0),
    ]
    path = tmp_path / "control.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    result = summarize_system_trace(path)["control_window_observations"]["stationary_reference"]
    assert result["output_samples"]["final_cmd_vel"] == 4
    assert result["output_samples"]["final_cmd_vel_nonzero_running_qualified"] == 1
    assert result["nonzero_running_qualified_output_bounds"]["final_cmd_vel"] == dict(
        first_observed_ns=9, last_observed_ns=9
    )
    assert result["first_terminal_observed_ns"] == 1_000_000_001
    assert result["mppi_after_terminal"] == "N/A"


def test_discovery_and_full_sample_reject_changed_incarnation(tmp_path, monkeypatch):
    import tools.shared_layers_system_trace as module

    real_identity = module.process_identity
    root = dict(pid=100, starttime=1, ppid=1, cgroup="", cpu_ticks=0)
    child = dict(pid=200, starttime=2, ppid=100, cgroup="", cpu_ticks=0)
    reused = dict(child, starttime=3)
    writer = module.TraceWriter(tmp_path / "unused", {})
    sampler = module.ResourceSampler(writer)
    sampler.owned[(100, 1)] = root
    monkeypatch.setattr(Path, "iterdir", lambda path: iter((Path("/proc/100"), Path("/proc/200"))))
    counts = {100: 0, 200: 0}

    def identity(pid):
        counts[pid] += 1
        return root if pid == 100 else child if counts[pid] == 1 else reused

    monkeypatch.setattr(module, "process_identity", identity)
    sampler._discover()
    assert set(sampler.owned) == {(100, 1)}
    assert any(row["stage"] == "process_unconfirmed" for row in list(writer.queue.queue))
    sampler.owned = {(200, 2): child}
    reads = iter((child, reused))
    monkeypatch.setattr(module, "process_identity", lambda pid: next(reads))
    monkeypatch.setattr(Path, "read_text", lambda path: "VmRSS: 4 kB\n")
    monkeypatch.setattr(Path, "read_bytes", lambda path: b"observed\0")
    writer.queue.queue.clear()
    sampler._resources(True)
    assert not any(row["stage"] == "resource" for row in list(writer.queue.queue))
    reads = iter((child, reused))
    monkeypatch.setattr(module, "_process_stat", lambda pid: next(reads))
    with pytest.raises(RuntimeError, match="identity unconfirmed"):
        real_identity(200)
