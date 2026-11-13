"""Settings and request receipts share one SQLite commit; no global migration bump."""

import json
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

import aiosqlite

from senior_control.settings.models import (
    DrivingSettings,
    LoggingSettings,
    SettingsResult,
    SettingsSnapshot,
)
from senior_control.storage.database import Database


class SettingsStore:
    """One instance/vehicle database; service is the only settings write owner.

    T6 may index these request IDs but must query this receipt, never execute or
    finalize another copy. Settings own atomic receipt+value, not task lifecycle.
    """

    def __init__(self, database: Database) -> None:
        self._database = database

    @asynccontextmanager
    async def transaction(self) -> AsyncIterator[aiosqlite.Connection]:
        async with self._database.write_lock:
            connection = self._database.connection
            await connection.rollback()
            await connection.execute("BEGIN IMMEDIATE")
            try:
                yield connection
                await connection.commit()
            except BaseException:
                await connection.rollback()
                raise

    async def prepare(self, defaults: SettingsSnapshot) -> SettingsSnapshot:
        async with self.transaction() as connection:
            async with connection.execute(
                "SELECT name FROM sqlite_master WHERE type = 'table' "
                "AND name IN ('vehicle_settings', 'vehicle_settings_request')"
            ) as cursor:
                existing = {str(row[0]) for row in await cursor.fetchall()}
            if existing and len(existing) != 2:
                raise ValueError("incomplete settings schema")
            await connection.execute(
                "CREATE TABLE IF NOT EXISTS vehicle_settings "
                "(id INTEGER PRIMARY KEY CHECK (id = 1), schema_version INTEGER NOT NULL, "
                "payload TEXT NOT NULL)"
            )
            await connection.execute(
                "CREATE TABLE IF NOT EXISTS vehicle_settings_request "
                "(request_id TEXT PRIMARY KEY, fingerprint TEXT NOT NULL, receipt TEXT NOT NULL)"
            )
            async with connection.execute(
                "SELECT schema_version, payload FROM vehicle_settings WHERE id = 1"
            ) as cursor:
                row = await cursor.fetchone()
            if row is None:
                if existing:
                    raise ValueError("settings row missing")
                await connection.execute(
                    "INSERT INTO vehicle_settings VALUES (1, 1, ?)", (defaults.model_dump_json(),)
                )
                return defaults
            if row[0] != 1:
                raise ValueError("unknown settings schema")
            payload = str(row[1])
            self._validate_snapshot_shape(json.loads(payload))
            return SettingsSnapshot.model_validate_json(payload)

    async def lookup(
        self, connection: aiosqlite.Connection, request_id: str
    ) -> tuple[str, SettingsResult] | None:
        async with connection.execute(
            "SELECT fingerprint, receipt FROM vehicle_settings_request WHERE request_id = ?",
            (request_id,),
        ) as cursor:
            row = await cursor.fetchone()
        if row is None:
            return None
        payload = str(row[1])
        document: object = json.loads(payload)
        if not isinstance(document, dict):
            raise ValueError("settings unavailable: malformed receipt")
        self._validate_snapshot_shape(document.get("snapshot"))
        return str(row[0]), SettingsResult.model_validate_json(payload)

    async def assert_current(
        self, connection: aiosqlite.Connection, expected: SettingsSnapshot
    ) -> None:
        async with connection.execute(
            "SELECT schema_version, payload FROM vehicle_settings WHERE id = 1"
        ) as cursor:
            row = await cursor.fetchone()
        if row is None or row[0] != 1:
            raise ValueError("settings unavailable: durable state disappeared")
        payload = str(row[1])
        self._validate_snapshot_shape(json.loads(payload))
        if SettingsSnapshot.model_validate_json(payload) != expected:
            raise ValueError("settings unavailable: another settings writer changed durable state")

    async def write_snapshot(
        self, connection: aiosqlite.Connection, snapshot: SettingsSnapshot
    ) -> None:
        await connection.execute(
            "UPDATE vehicle_settings SET payload = ? WHERE id = 1", (snapshot.model_dump_json(),)
        )

    async def write_receipt(
        self, connection: aiosqlite.Connection, fingerprint: str, result: SettingsResult
    ) -> None:
        await connection.execute(
            "INSERT INTO vehicle_settings_request VALUES (?, ?, ?)",
            (result.request_id, fingerprint, result.model_dump_json()),
        )

    @staticmethod
    def _validate_snapshot_shape(document: object) -> None:
        # Durable state is a complete image. Model construction defaults are NEVER
        # permitted to fill missing fields in a damaged image or request receipt.
        if not isinstance(document, dict) or set(document) != set(SettingsSnapshot.model_fields):
            raise ValueError("settings unavailable: incomplete snapshot")
        for name, expected in (
            ("driving", DrivingSettings.model_fields),
            ("logging", LoggingSettings.model_fields),
        ):
            values = document.get(name)
            if not isinstance(values, dict) or set(values) != set(expected):
                raise ValueError("settings unavailable: incomplete settings group")
