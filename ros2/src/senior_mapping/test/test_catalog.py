"""Catalog metadata survives restart without becoming localization authority."""

import pytest
from senior_mapping.catalog import MapCatalog
from vehicle_manager.storage.bindings import Database

from vehicle_core.errors import DomainError


async def test_two_map_catalog_reopens_and_rejects_old_activation_results(tmp_path):
    path = tmp_path / "maps.db"
    async with Database(path) as database:
        catalog = MapCatalog(database, robot_id="robot")
        await catalog.prepare()
        first = await catalog.create("create-a", "仓库")
        second = await catalog.create("create-b", "院子")
        assert await catalog.create("create-a", "仓库") == first
        await catalog.checkpoint(first.map_ref, "a" * 32, home_id="home-a", expected=None)
        old = await catalog.begin_activation(first.map_ref, expected_generation=0)
        await catalog.finish_activation(old.token, localized=True)
        first_ref = first.map_ref
        await catalog.checkpoint(
            first_ref, "b" * 32, home_id="home-a", expected="a" * 32, session_token=old.token
        )
        with pytest.raises(DomainError):
            await catalog.checkpoint(
                first_ref, "d" * 32, home_id="home-a", expected="a" * 32, session_token=old.token
            )
        saved = (await catalog.list())[0]
        assert saved.map_ref == first_ref and saved.checkpoint_id == "b" * 32
        await catalog.checkpoint(second.map_ref, "c" * 32, home_id="home-b", expected=None)
        current = await catalog.begin_activation(second.map_ref, expected_generation=old.generation)
        with pytest.raises(DomainError):
            await catalog.finish_activation(old.token, localized=True)
        with pytest.raises(DomainError):
            await catalog.checkpoint(
                first_ref, "d" * 32, home_id="home-a", expected="b" * 32, session_token=old.token
            )
        pending = await catalog.finish_activation(current.token, localized=False)
        assert pending.state == "pending_calibration"
        with pytest.raises(DomainError):
            await catalog.delete_inactive(second.map_ref)
        await catalog.delete_inactive(first_ref)
    async with Database(path) as database:
        restored = MapCatalog(database, robot_id="robot")
        await restored.prepare()
        rows = await restored.list()
        assert len(rows) == 1 and rows[0].map_ref == second.map_ref
        assert rows[0].home_id == "home-b" and rows[0].checkpoint_id == "c" * 32
        selected = await restored.selection()
        assert selected.map_ref == second.map_ref and selected.state == "pending_calibration"
        # A deleted create operation is a tombstone, never a second new map.
        with pytest.raises(DomainError):
            await restored.create("create-a", "仓库")
