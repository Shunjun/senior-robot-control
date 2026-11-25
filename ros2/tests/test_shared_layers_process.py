"""Opt-in S05/S07/S09 through the real manager owner, adapter IPC and native child.

Requires a separately prepared isolated ROS adapter with an authenticated bounded
scan/odom/static-TF producer. No service is started by collection. Run with
SENIOR_SHARED_LAYERS_PROCESS=1 and SENIOR_SHARED_LAYERS_ADAPTER_SOCKET pointing at
that adapter's private Unix socket. Missing ROS is an explicit unverified skip.
The profile below is synthetic-admitted, never physical deployment evidence.
"""

import asyncio
import os
import threading
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("SENIOR_SHARED_LAYERS_PROCESS") != "1",
    reason="unverified: explicit isolated ROS adapter/native process harness required",
)


async def until(predicate, *, timeout=20):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = await predicate()
        if result:
            return result
        await asyncio.sleep(0.02)
    raise AssertionError("real shared-layer process observation did not arrive")


async def test_owner_adapter_native_capture_restore_and_retirement(tmp_path, monkeypatch):
    rclpy = pytest.importorskip("rclpy", reason="unverified: ROS/colcon unavailable")
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from senior_interfaces.msg import CurrentObstacles
    from senior_mapping import checkpoints as checkpoint_module
    from senior_mapping.references import encode_fixed_snapshot
    from senior_mapping.shared_scan import read_latest_scan
    from test_mapping_production import admitted_profile
    from vehicle_manager.agent.mapping_catalog import mapping_requests
    from vehicle_manager.storage.bindings import Database
    from vehicle_manager.storage.domain_bindings import WaypointStore

    from vehicle_core.ipc import IpcClient

    socket = os.environ.get("SENIOR_SHARED_LAYERS_ADAPTER_SOCKET")
    assert socket, "supply the real isolated adapter socket; no mock fallback"
    rclpy.init()
    node = rclpy.create_node("shared_layers_process_observer")
    current, maps = [], []
    node.create_subscription(
        CurrentObstacles, "/senior/mapping/current_obstacles", lambda row: current.append(row), 1
    )
    node.create_subscription(
        OccupancyGrid,
        "/map",
        lambda row: maps.append(row),
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    client = IpcClient(Path(socket))
    managed = running = None
    release = threading.Event()
    try:
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
            managed._runtime.backend.geometry_profile = admitted_profile()
            running = asyncio.create_task(managed.run())

            async def idle():
                return (await managed.request("map.catalog"))["runtime_available"]

            await until(idle)
            reply = await managed.request(
                "map.create",
                operation_id="create-process-map",
                name="isolated synthetic scan",
                expected_generation=0,
            )
            assert reply["ok"], reply
            session = managed._runtime._session

            async def observed():
                rclpy.spin_once(node, timeout_sec=0)
                cut = session.active.fixed_snapshot(session.source)
                return cut if cut.supports and maps and current and current[-1].valid else None

            before = await until(observed)
            original_graph = before.watermark.graph_sequence
            original_key = before.watermark.evidence_key
            entered = threading.Event()
            write_files = checkpoint_module._publish_files

            def held_disk(*args):
                entered.set()
                assert release.wait(10), "test disk barrier was not released"
                return write_files(*args)

            monkeypatch.setattr(checkpoint_module, "_publish_files", held_disk)
            writer = asyncio.create_task(session._save(0))
            assert await asyncio.to_thread(entered.wait, 10)

            async def during_save():
                rclpy.spin_once(node, timeout_sec=0)
                row = read_latest_scan(
                    session._scan_path,
                    0,
                    nonce=session._scan_nonce,
                    backend_boot=session.graph.boot,
                    host_boot_id=session._host_boot_id,
                ).envelope
                if row and row.scan and row.key.sequence > original_key.sequence:
                    assert (
                        row.valid_until_steady_ns
                        == row.scan.capture.time.sample_steady_ns + 500_000_000
                    )
                    assert current[-1].valid_until_steady_ns == row.valid_until_steady_ns
                    return row

            newer = await until(during_save, timeout=5)
            assert session._capturing
            assert session.graph.sequence == original_graph
            release.set()
            await writer
            monkeypatch.setattr(checkpoint_module, "_publish_files", write_files)

            async def same_anchor_progress():
                rclpy.spin_once(node, timeout_sec=0)
                cut = session.active.fixed_snapshot(session.source)
                return (
                    cut
                    if (
                        cut.watermark.graph_sequence == original_graph
                        and cut.watermark.evidence_key
                        and cut.watermark.evidence_key.sequence > newer.key.sequence
                    )
                    else None
                )

            cut = await until(same_anchor_progress)
            assert cut.watermark.evidence_revision > before.watermark.evidence_revision
            assert session._installed_watermark == cut.watermark
            assert bytes(v % 256 for v in maps[-1].data) == cut.grid.cells
            await session.flush()
            entry = session.entry
            async with session.repository.open(entry) as loaded:
                durable = loaded.snapshot.fixed
                assert durable is not None
            await session.close()
            managed._runtime._session = None
            late = tmp_path / "late-fixed.json"
            payload = encode_fixed_snapshot(cut, transfer=True)
            late.write_bytes(payload)
            late.chmod(0o600)
            import hashlib

            denied = await client.request(
                "map.backend.install_fixed",
                boot_id=session.graph.boot,
                path=str(late),
                sha256=hashlib.sha256(payload).hexdigest(),
                acceptance_floor_steady_ns=session._reference_floor_ns,
            )
            assert denied.get("ok") is False
            resumed = await managed._runtime.backend.open(entry, restore=True)
            try:
                fresh = resumed.active.fixed_snapshot(resumed.source)
                assert fresh.watermark.backend_boot != durable.watermark.backend_boot
                assert fresh.watermark.evidence_key is None
                assert fresh.watermark.evidence_revision >= durable.watermark.evidence_revision
                assert fresh.grid.cells == durable.grid.cells
                assert fresh.supports == durable.supports
            finally:
                await resumed.close()
    finally:
        release.set()
        if running is not None:
            running.cancel()
            await asyncio.gather(running, return_exceptions=True)
        if managed is not None:
            await managed.close()
        await client.close()
        node.destroy_node()
        rclpy.shutdown()
