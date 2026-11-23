"""System resource measurements retain real counter and process identities."""

from pathlib import Path

from vehicle_manager.performance.proc import ProcSampler


def test_machine_and_module_counters_do_not_charge_reused_pid(tmp_path: Path):
    def write(total_busy, total_idle, process_ticks, birth):
        (tmp_path / "stat").write_text(
            f"cpu {total_busy} 0 0 {total_idle} 0 0 0 0 0 0\n"
            + "".join(f"cpu{i} 0\n" for i in range(4))
        )
        (tmp_path / "meminfo").write_text("MemTotal: 1000 kB\nMemAvailable: 250 kB\n")
        process = tmp_path / "42"
        process.mkdir(exist_ok=True)
        fields = ["0"] * 22
        fields[0], fields[11], fields[19], fields[21] = "S", str(process_ticks), str(birth), "3"
        (process / "stat").write_text("42 (video worker 编码) " + " ".join(fields))

    clock = [10.0]
    sampler = ProcSampler(tmp_path, page_size=4096, clock=lambda: clock[0])
    write(100, 300, 10, 1)
    first = sampler.sample({"video": (42,), "navigation": (), "mapping": ()})
    assert first.cpu_percent is None
    assert first.memory_used_bytes == 750 * 1024
    assert first.modules[0].rss_bytes == 3 * 4096
    clock[0] += 1
    write(200, 600, 30, 1)
    second = sampler.sample({"video": (42,), "navigation": (), "mapping": ()})
    assert second.cpu_count == 4 and second.cpu_percent == 25.0
    assert second.modules[0].cpu_percent == 5.0
    assert second.modules[1].status == "inactive"
    clock[0] += 1
    write(300, 900, 999, 2)
    replaced = sampler.sample({"video": (42,)}, expected_births={42: 1})
    assert replaced.modules[0].cpu_percent is None
    assert replaced.modules[0].rss_bytes is None
    assert replaced.modules[0].status == "unavailable"


async def test_cached_sample_survives_snapshot_cloud_roundtrip_without_resampling(tmp_path: Path):
    import asyncio
    import time

    from cloud.snapshots import SnapshotCache
    from cloud.snapshots import snapshot_envelope as cloud_envelope
    from senior_contracts.wire import SnapshotEnvelope
    from vehicle_manager.agent.snapshots import (
        RobotSnapshot,
        snapshot_envelope,
        snapshot_from_envelope,
    )
    from vehicle_manager.performance.monitor import PerformanceMonitor

    (tmp_path / "stat").write_text("cpu 100 0 0 300 0 0 0 0\ncpu0 0\n")
    (tmp_path / "meminfo").write_text("MemTotal: 1000 kB\nMemAvailable: 250 kB\n")
    monitor = PerformanceMonitor(ProcSampler(tmp_path, page_size=4096))
    runner = asyncio.create_task(monitor.run())
    try:

        async def ready():
            while monitor.read() is None:
                await asyncio.sleep(0.001)

        await asyncio.wait_for(ready(), 1)
        sample = monitor.read()
        assert sample is not None and monitor.read() is sample
        assert sample.modules[0].status == "unavailable"  # No invented PID attribution.
        snapshot = RobotSnapshot(
            "robot", True, False, None, None, 0, "idle", time.time(), performance=sample
        )
        wire = snapshot_envelope(
            snapshot,
            robot_id="robot",
            session_id="session",
            connection_epoch=1,
            request_id="sample",
            seq=1,
            expires_at=time.time() + 10,
        )
        received = SnapshotEnvelope.model_validate_json(wire.model_dump_json())
        cache = SnapshotCache()
        cache.store(received)
        cache.mark_offline("robot")
        offline = cache.get("robot")
        assert offline is not None and offline.stale and not offline.snapshot.connected
        forwarded = cloud_envelope(
            offline.snapshot,
            robot_id="robot",
            session_id="web",
            connection_epoch=1,
            request_id="cached",
            seq=1,
            expires_at=time.time() + 10,
            stale=True,
            last_synced_at=offline.last_synced_at,
        )
        restored = snapshot_from_envelope(forwarded, map_reference=lambda *args: None)
        assert restored.performance == sample
        assert monitor.read() is sample  # Serialization/cache reads never sample procfs.
    finally:
        runner.cancel()
        await asyncio.gather(runner, return_exceptions=True)


async def test_registered_subtree_is_not_transferred_to_a_recycled_root(tmp_path: Path):
    import asyncio

    from vehicle_manager.performance.monitor import PerformanceMonitor

    def process(pid, parent, birth):
        directory = tmp_path / str(pid)
        directory.mkdir(exist_ok=True)
        fields = ["0"] * 22
        fields[0], fields[1], fields[19], fields[21] = "S", str(parent), str(birth), "1"
        (directory / "stat").write_text(f"{pid} (owned) " + " ".join(fields))
        task = directory / "task" / str(pid)
        task.mkdir(parents=True, exist_ok=True)
        (task / "children").write_text("43" if pid == 42 else "")

    (tmp_path / "stat").write_text("cpu 100 0 0 300 0 0 0 0\ncpu0 0\n")
    (tmp_path / "meminfo").write_text("MemTotal: 1000 kB\nMemAvailable: 250 kB\n")
    process(42, 1, 1)
    process(43, 42, 1)
    monitor = PerformanceMonitor(ProcSampler(tmp_path, page_size=4096))
    monitor.event(
        {"resource_module": "video", "phase": "resource_started", "pid": 42, "process_start": 1}
    )
    runner = asyncio.create_task(monitor.run())

    async def ready(sequence):
        while monitor.read() is None or monitor.read().sequence < sequence:
            await asyncio.sleep(0.005)

    try:
        await asyncio.wait_for(ready(1), 1)
        first = monitor.read().modules[0]
        assert first.status == "running" and first.process_count == 2
        assert first.rss_bytes == 2 * 4096
        process(42, 1, 2)
        await asyncio.wait_for(ready(2), 2)
        reused = monitor.read().modules[0]
        assert reused.status == "unavailable"
        assert reused.cpu_percent is None and reused.rss_bytes is None
    finally:
        runner.cancel()
        await asyncio.gather(runner, return_exceptions=True)
