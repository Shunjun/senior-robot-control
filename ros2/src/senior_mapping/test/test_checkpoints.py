"""Real files and SQLite pointers restore one verified graph/stable/Home cut."""

from senior_mapping.catalog import MapCatalog
from senior_mapping.checkpoints import MapCheckpoints
from test_map_archive import archive_for, filled_store
from vehicle_manager.storage.bindings import Database


async def test_complete_checkpoint_reopens_with_its_own_graph_and_home(tmp_path):
    path = tmp_path / "catalog.db"
    root = tmp_path / "checkpoints"
    async with Database(path) as database:
        catalog = MapCatalog(database, robot_id="robot-a")
        await catalog.prepare()
        entry = await catalog.create("create", "仓库")
        store = filled_store(entry.map_ref)
        checkpoints = MapCheckpoints(root, archive_for(store), catalog)
        first = await checkpoints.save(store, home_id="home", expected=None)
        selected = await catalog.begin_activation(entry.map_ref, expected_generation=0)
        await catalog.finish_activation(selected.token, localized=True)
        second = await checkpoints.save(
            store, home_id="home", expected=first.checkpoint_id, session_token=selected.token
        )
        assert first.checkpoint_id != second.checkpoint_id
    async with Database(path) as database:
        catalog = MapCatalog(database, robot_id="robot-a")
        await catalog.prepare()
        (entry,) = await catalog.list()
        checkpoints = MapCheckpoints(root, archive_for(store), catalog)
        async with checkpoints.open(entry) as loaded:
            assert loaded.snapshot.map_ref == store.map_ref
            assert loaded.snapshot.occupancy == store.occupancy()
            assert loaded.home_id == "home"
            assert (loaded.graph_directory / "graph.posegraph").read_text() == "nodes"
            assert entry.checkpoint_id == second.checkpoint_id
        assert (await catalog.selection()).state == "pending_calibration"


async def test_cancelled_cleanup_cannot_grant_a_lease_to_a_disappearing_version(
    tmp_path, monkeypatch
):
    import asyncio
    import threading
    from pathlib import Path

    import pytest
    from senior_mapping import checkpoints as module

    from vehicle_core.errors import DomainError

    async with Database(tmp_path / "catalog.db") as database:
        catalog = MapCatalog(database, robot_id="robot-a")
        await catalog.prepare()
        entry = await catalog.create("create", "仓库")
        store = filled_store(entry.map_ref)
        checkpoints = MapCheckpoints(tmp_path / "cuts", archive_for(store), catalog)
        first = await checkpoints.save(store, home_id="home", expected=None)
        selected = await catalog.begin_activation(entry.map_ref, expected_generation=0)
        await catalog.finish_activation(selected.token, localized=True)
        second = await checkpoints.save(
            store, home_id="home", expected=first.checkpoint_id, session_token=selected.token
        )
        entered, release = threading.Event(), threading.Event()
        original = module.shutil.rmtree

        def remove(path, *args, **kwargs):
            if Path(path).name == first.checkpoint_id:
                entered.set()
                assert release.wait(3)
            return original(path, *args, **kwargs)

        monkeypatch.setattr(module.shutil, "rmtree", remove)
        writer = asyncio.create_task(
            checkpoints.save(
                store, home_id="home", expected=second.checkpoint_id, session_token=selected.token
            )
        )
        reader = None
        try:
            assert await asyncio.to_thread(entered.wait, 2)
            writer.cancel()
            await asyncio.gather(writer, return_exceptions=True)

            async def read_old():
                async with checkpoints.open(first) as loaded:
                    return loaded.graph_directory.exists()

            reader = asyncio.create_task(read_old())
            with pytest.raises(asyncio.TimeoutError):
                await asyncio.wait_for(asyncio.shield(reader), 0.1)
            release.set()
            with pytest.raises(DomainError):
                await reader
            (current,) = await catalog.list()
            async with checkpoints.open(current) as loaded:
                assert loaded.graph_directory.is_dir()
        finally:
            release.set()
            await asyncio.gather(writer, return_exceptions=True)
            if reader is not None:
                await asyncio.gather(reader, return_exceptions=True)
            await checkpoints.quiesce()


async def test_active_public_api_captures_home_and_resumes_without_private_store_access(tmp_path):
    from senior_mapping.active import ActiveMap
    from senior_mapping.archive import MapArchive
    from senior_mapping.references import (
        AnchoredWaypoint,
        AnchorPose,
        AnchorSnapshot,
        GridExtent,
        WaypointBinding,
    )
    from test_map_archive import Graph

    from vehicle_core.values import Pose2D

    async with Database(tmp_path / "catalog.db") as database:
        catalog = MapCatalog(database, robot_id="robot-a")
        await catalog.prepare()
        entry = await catalog.create("create", "仓库")
        pose = Pose2D(0, 0, 0)
        active, source = ActiveMap(entry.map_ref, "wheel_odom", 0.5), object()
        active.bind_verified_source(source, backend_boot="graph-a", source_epoch="scanner-a")
        anchors = AnchorSnapshot(
            entry.map_ref, "wheel_odom", (AnchorPose("kf1", pose, True),), GridExtent(2, 2, 0, 0)
        )
        active.commit(source, 1, "kf1", anchors, ())
        home = WaypointBinding(
            AnchoredWaypoint("home", Pose2D(0.25, 0.25, 0.2), "kf1", False), pose
        )
        active.update_waypoints(source, expected_sequence=1, changes=(home,))
        repository = MapCheckpoints(
            tmp_path / "cuts", MapArchive(Graph(entry.map_ref), source_robot_id="robot-a"), catalog
        )
        saved = await active.save_checkpoint(source, repository, home_id="home", expected=None)
        async with repository.open(saved) as loaded:
            assert loaded.snapshot.waypoint_bindings == (home,)
            new_source = object()
            restored = ActiveMap.resume_verified(
                loaded.snapshot, new_source, backend_boot="graph-b", source_epoch="scanner-b"
            )
        assert restored.grid() is not None
        assert restored.map_generation != active.map_generation
        restored.update_waypoints(new_source, expected_sequence=0, changes=(home,))
        active.revoke(source)
