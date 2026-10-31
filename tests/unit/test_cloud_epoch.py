from pathlib import Path

from senior_control.cloud.auth import TokenDigest
from senior_control.cloud.registry import RobotRegistry
from senior_control.storage.database import Database


async def test_epoch_survives_cloud_database_reopen(tmp_path: Path) -> None:
    path = tmp_path / "cloud.db"
    async with Database(path) as database:
        await database.connection.execute(
            "CREATE TABLE cloud_robot (robot_id TEXT PRIMARY KEY, revoked_at REAL, "
            "connection_epoch INTEGER NOT NULL DEFAULT 0)"
        )
        await database.connection.execute("INSERT INTO cloud_robot(robot_id) VALUES ('robot-a')")
        await database.connection.commit()
        registry = RobotRegistry(database, TokenDigest(b"x" * 32))
        assert await registry.allocate_epoch("robot-a") == 1
    async with Database(path) as database:
        registry = RobotRegistry(database, TokenDigest(b"x" * 32))
        assert await registry.allocate_epoch("robot-a") == 2
