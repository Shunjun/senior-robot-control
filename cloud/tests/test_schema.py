"""Existing historical SQL formats must reopen through the independent lifespan."""

import hashlib
import json
from importlib.resources import files

import pytest
from cloud.application import create_app
from cloud.auth import hash_password
from cloud.config import CloudSettings
from cloud.migrations import migrate
from cloud.schema import historical_migrations
from cloud.storage import Database


@pytest.mark.parametrize("version", range(6))
async def test_historical_versions_reopen_without_changing_existing_credentials(tmp_path, version):
    rows = json.loads(files("cloud").joinpath("historical_schema.json").read_text())
    assert [row["version"] for row in rows] == [1, 2, 3, 4, 5]
    for row in rows:
        assert [hashlib.sha256(sql.encode()).hexdigest() for sql in row["statements"]] == row[
            "sha256"
        ]
    database = Database(tmp_path / "cloud.db")
    async with database:
        await migrate(database, historical_migrations()[:version])
        if version >= 4:
            await database.connection.execute(
                "INSERT INTO cloud_robot(robot_id,name,device_hash,created_at) VALUES (?,?,?,?)",
                ("old-robot", "original", "original-digest", 123.0),
            )
            if version == 5:
                await database.connection.execute(
                    "UPDATE cloud_robot SET connection_epoch = 17 WHERE robot_id = 'old-robot'"
                )
            await database.connection.commit()
    app = create_app(
        CloudSettings(secret="s" * 32),
        database=database,
        admin_password_hash=hash_password("password"),
        allowed_origins=("https://cloud.test",),
    )
    assert not database.is_open
    for _ in range(2):
        async with app.router.lifespan_context(app):
            async with database.connection.execute("PRAGMA user_version") as cursor:
                assert await cursor.fetchone() == (5,)
            async with database.connection.execute(
                "SELECT name FROM sqlite_master WHERE type = 'table'"
            ) as cursor:
                tables = {row[0] for row in await cursor.fetchall()}
            assert {
                "map_operation",
                "waypoint",
                "task_definition",
                "cloud_session",
                "cloud_pairing_code",
                "cloud_robot",
            } <= tables
            if version >= 4:
                async with database.connection.execute(
                    "SELECT name,device_hash,created_at,connection_epoch FROM cloud_robot "
                    "WHERE robot_id = 'old-robot'"
                ) as cursor:
                    assert await cursor.fetchone() == (
                        "original",
                        "original-digest",
                        123.0,
                        17 if version == 5 else 0,
                    )
        assert not database.is_open
