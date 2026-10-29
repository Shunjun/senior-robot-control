"""Robot rows and device credentials. Only keyed digests of device tokens are stored."""

import time
from dataclasses import dataclass

from senior_control.cloud.auth import TokenDigest
from senior_control.cloud.db import fetch_all, write_transaction
from senior_control.storage.database import Database

ROBOT_STATEMENTS = (
    """
    CREATE TABLE cloud_robot (
        robot_id TEXT PRIMARY KEY,
        name TEXT NOT NULL,
        device_hash TEXT NOT NULL UNIQUE,
        created_at REAL NOT NULL,
        revoked_at REAL
    )
    """,
)

INSERT_ROBOT = (
    "INSERT INTO cloud_robot (robot_id, name, device_hash, created_at) VALUES (?, ?, ?, ?)"
)


@dataclass(frozen=True)
class RobotRecord:
    robot_id: str
    name: str


class RobotRegistry:
    def __init__(self, database: Database, digest: TokenDigest) -> None:
        self._database = database
        self._digest = digest

    async def authenticate(self, device_token: str | None) -> RobotRecord | None:
        if not device_token or len(device_token) > 256:
            return None
        rows = await fetch_all(
            self._database,
            "SELECT robot_id, name FROM cloud_robot WHERE device_hash = ? AND revoked_at IS NULL",
            (self._digest(device_token),),
        )
        return RobotRecord(str(rows[0][0]), str(rows[0][1])) if rows else None

    async def get_active(self, robot_id: str) -> RobotRecord | None:
        rows = await fetch_all(
            self._database,
            "SELECT robot_id, name FROM cloud_robot WHERE robot_id = ? AND revoked_at IS NULL",
            (robot_id,),
        )
        return RobotRecord(str(rows[0][0]), str(rows[0][1])) if rows else None

    async def list_active(self) -> list[RobotRecord]:
        rows = await fetch_all(
            self._database,
            "SELECT robot_id, name FROM cloud_robot WHERE revoked_at IS NULL"
            " ORDER BY created_at, robot_id",
        )
        return [RobotRecord(str(row[0]), str(row[1])) for row in rows]

    async def revoke(self, robot_id: str) -> bool:
        async with write_transaction(self._database) as connection:
            async with connection.execute(
                "UPDATE cloud_robot SET revoked_at = ? WHERE robot_id = ? AND revoked_at IS NULL",
                (time.time(), robot_id),
            ) as cursor:
                return cursor.rowcount == 1
