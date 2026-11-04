"""One-time pairing codes. Redemption is a single conditional UPDATE under the write lock."""

import secrets
import time
import uuid

from senior_control.cloud.auth import TokenDigest
from senior_control.cloud.db import write_transaction
from senior_control.cloud.registry import INSERT_ROBOT
from senior_control.storage.database import Database

PAIRING_STATEMENTS = (
    """
    CREATE TABLE cloud_pairing_code (
        code_hash TEXT PRIMARY KEY,
        created_at REAL NOT NULL,
        expires_at REAL NOT NULL,
        redeemed_at REAL
    )
    """,
)

_ALPHABET = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789"
CODE_LENGTH = 10


class PairingBusy(Exception):
    """Too many unredeemed codes are live."""


def normalize_code(code: str) -> str:
    return code.strip().upper()


class PairingService:
    def __init__(
        self, database: Database, digest: TokenDigest, *, ttl_seconds: float, max_live: int
    ) -> None:
        self._database = database
        self._digest = digest
        self._ttl = ttl_seconds
        self._max_live = max_live

    async def issue(self) -> str:
        code = "".join(secrets.choice(_ALPHABET) for _ in range(CODE_LENGTH))
        now = time.time()
        async with write_transaction(self._database) as connection:
            await connection.execute(
                "DELETE FROM cloud_pairing_code WHERE expires_at <= ? OR redeemed_at IS NOT NULL",
                (now,),
            )
            async with connection.execute("SELECT COUNT(*) FROM cloud_pairing_code") as cursor:
                row = await cursor.fetchone()
            if row is not None and int(row[0]) >= self._max_live:
                raise PairingBusy
            await connection.execute(
                "INSERT INTO cloud_pairing_code (code_hash, created_at, expires_at)"
                " VALUES (?, ?, ?)",
                (self._digest(code), now, now + self._ttl),
            )
        return code

    async def redeem(self, code: str, name: str) -> tuple[str, str] | None:
        """Return (robot_id, device_token), or None for an unknown, expired or used code."""
        now = time.time()
        robot_id = uuid.uuid4().hex
        device_token = secrets.token_urlsafe(32)
        async with write_transaction(self._database) as connection:
            async with connection.execute(
                "UPDATE cloud_pairing_code SET redeemed_at = ?"
                " WHERE code_hash = ? AND redeemed_at IS NULL AND expires_at > ?",
                (now, self._digest(normalize_code(code)), now),
            ) as cursor:
                if cursor.rowcount != 1:
                    return None
            await connection.execute(
                INSERT_ROBOT, (robot_id, name, self._digest(device_token), now)
            )
        return robot_id, device_token
