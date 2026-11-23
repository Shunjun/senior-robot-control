"""Serial lifecycle barriers, using actual catalog transactions and held persistence."""

import asyncio

from senior_contracts.wire import MapTargetBody
from senior_mapping.catalog import MapCatalog
from senior_mapping.checkpoints import MapCheckpoints
from senior_mapping.runtime import MappingRuntime
from test_map_archive import archive_for, filled_store
from vehicle_manager.storage.bindings import Database


async def test_switch_drops_loading_input_and_waits_for_old_save_before_new_backend(tmp_path):
    async with Database(tmp_path / "robot.db") as database:
        catalog = MapCatalog(database, robot_id="robot-a")
        await catalog.prepare()
        entries = []
        for name in ("a", "b"):
            entry = await catalog.create(name, name)
            store = filled_store(entry.map_ref)
            saved = await MapCheckpoints(tmp_path / "cuts", archive_for(store), catalog).save(
                store, home_id="home", expected=None
            )
            entries.append(saved)
        events = []
        blocked, release = asyncio.Event(), asyncio.Event()

        class Session:
            localized = True

            def __init__(self, entry):
                self.entry = entry

            async def activate(self):
                pass

            async def flush(self):
                events.append(("flush", self.entry.name))
                if self.entry.name == "a":
                    blocked.set()
                    await release.wait()

            async def close(self):
                events.append(("close", self.entry.name))

            async def ingest(self, observation):
                events.append(("ingest", observation))

            def grid(self):
                return None

        class Backend:
            async def stopped(self):
                return True

            async def open(self, entry, *, restore):
                events.append(("open", entry.name))
                return Session(entry)

        runtime = MappingRuntime(catalog, Backend())
        await runtime.execute(
            "map.activate", "a", MapTargetBody(**entries[0].map_ref.__dict__, expected_generation=0)
        )
        assert await runtime.ingest("accepted")
        switching = asyncio.create_task(
            runtime.execute(
                "map.activate",
                "b",
                MapTargetBody(**entries[1].map_ref.__dict__, expected_generation=1),
            )
        )
        await blocked.wait()
        assert runtime.phase == "loading"
        assert not await runtime.ingest("discarded")
        assert ("open", "b") not in events
        release.set()
        await switching
        assert (
            events.index(("flush", "a"))
            < events.index(("close", "a"))
            < events.index(("open", "b"))
        )
        assert await runtime.ingest("new")
        assert ("ingest", "discarded") not in events
        assert (await catalog.selection()).map_ref == entries[1].map_ref
        await runtime.close()


async def test_failed_old_save_never_opens_the_target_map(tmp_path):
    import pytest

    async with Database(tmp_path / "robot.db") as database:
        catalog = MapCatalog(database, robot_id="robot-a")
        await catalog.prepare()
        entries = []
        for name in ("a", "b"):
            entry = await catalog.create(name, name)
            store = filled_store(entry.map_ref)
            entries.append(
                await MapCheckpoints(tmp_path / "cuts", archive_for(store), catalog).save(
                    store, home_id="home", expected=None
                )
            )
        opened = []

        class Session:
            localized = True

            def __init__(self, entry):
                self.entry = entry

            async def activate(self):
                pass

            async def flush(self):
                raise OSError("disk full")

            async def close(self):
                raise AssertionError("must not retire unsaved graph")

            async def ingest(self, observation):
                pass

            def grid(self):
                return None

        class Backend:
            async def stopped(self):
                return True

            async def open(self, entry, *, restore):
                opened.append(entry.name)
                return Session(entry)

        runtime = MappingRuntime(catalog, Backend())
        await runtime.execute(
            "map.activate", "a", MapTargetBody(**entries[0].map_ref.__dict__, expected_generation=0)
        )
        with pytest.raises(OSError, match="disk full"):
            await runtime.execute(
                "map.activate",
                "b",
                MapTargetBody(**entries[1].map_ref.__dict__, expected_generation=1),
            )
        assert opened == ["a"]
        assert runtime.phase == "failed"
        assert (await catalog.selection()).map_ref == entries[0].map_ref


async def test_incomplete_target_is_rejected_without_retiring_current_map(tmp_path):
    import pytest

    from vehicle_core.errors import DomainError

    async with Database(tmp_path / "robot.db") as database:
        catalog = MapCatalog(database, robot_id="robot-a")
        await catalog.prepare()
        entry = await catalog.create("a", "a")
        store = filled_store(entry.map_ref)
        entry = await MapCheckpoints(tmp_path / "cuts", archive_for(store), catalog).save(
            store, home_id="home", expected=None
        )
        incomplete = await catalog.create("incomplete", "interrupted-create")
        retired = []

        class Session:
            localized = True

            def __init__(self, entry):
                self.entry = entry

            async def activate(self):
                pass

            async def flush(self):
                retired.append("flush")

            async def close(self):
                retired.append("close")

            async def ingest(self, observation):
                pass

            def grid(self):
                return None

        class Backend:
            async def stopped(self):
                return True

            async def open(self, entry, *, restore):
                return Session(entry)

        runtime = MappingRuntime(catalog, Backend())
        await runtime.execute(
            "map.activate", "a", MapTargetBody(**entry.map_ref.__dict__, expected_generation=0)
        )
        with pytest.raises(DomainError):
            await runtime.execute(
                "map.activate",
                "incomplete",
                MapTargetBody(**incomplete.map_ref.__dict__, expected_generation=1),
            )
        assert retired == []
        assert runtime.phase == "mapping"
        assert (await catalog.selection()).map_ref == entry.map_ref
        await runtime.close()
