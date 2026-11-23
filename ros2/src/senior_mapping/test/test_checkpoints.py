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
