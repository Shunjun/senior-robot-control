"""Upgrade and recovery of the robot agent's persistent safety obligation."""

from pathlib import Path

import pytest

from senior_control.agent.journal import AGENT_JOURNAL_STATEMENTS, AgentJournal
from senior_control.storage.database import Database


async def test_existing_robot_database_upgrades_without_losing_operation_or_safety_intent(
    tmp_path: Path,
) -> None:
    path = tmp_path / "robot.db"
    async with Database(path) as database:
        for statement in AGENT_JOURNAL_STATEMENTS:
            await database.connection.execute(statement)
        await database.connection.execute(
            "INSERT INTO agent_operation VALUES (?, ?, ?, ?, ?, ?)",
            ("edit", "robot-1", "fingerprint", "succeeded", None, 3),
        )
        await database.connection.execute("PRAGMA user_version = 3")
        await database.connection.commit()
        journal = AgentJournal(database, robot_id="robot-1")
        await journal.prepare()
        assert not await journal.safety.pending()
        await journal.safety.mark()
        entry = await journal.get("edit")
        assert entry is not None and entry.revision == 3
        async with database.connection.execute("PRAGMA user_version") as cursor:
            row = await cursor.fetchone()
            assert row is not None and row[0] == 3

    async with Database(path) as database:
        journal = AgentJournal(database, robot_id="robot-1")
        await journal.prepare()
        assert await journal.safety.pending()


async def test_unknown_agent_schema_refuses_start_without_clearing_safety_intent(
    tmp_path: Path,
) -> None:
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id="robot-1")
        await journal.prepare()
        await journal.safety.mark()
        await database.connection.execute("UPDATE agent_schema_version SET version = 2")
        await database.connection.commit()
        with pytest.raises(ValueError, match="unknown agent schema"):
            await journal.prepare()
        assert await journal.safety.pending()


async def test_unknown_agent_schema_returns_nonrestartable_exit(tmp_path: Path) -> None:
    from senior_control.agent.composition import AgentConfig, run_agent

    path = tmp_path / "robot.db"
    async with Database(path) as database:
        journal = AgentJournal(database, robot_id="robot-1")
        await journal.prepare()
        await journal.safety.mark()
        await database.connection.execute("UPDATE agent_schema_version SET version = 2")
        await database.connection.commit()
    config = AgentConfig(
        robot_id="robot-1",
        cloud_url="wss://example.invalid/ws/agent",
        device_token="token",
        database_path=path,
        socket_path=tmp_path / "agent.sock",
    )
    assert await run_agent(config) == 78


async def test_completed_safety_receipt_survives_reset_and_reopen(tmp_path: Path) -> None:
    from senior_control.errors import DomainError, ErrorCode

    path = tmp_path / "robot.db"
    async with Database(path) as database:
        journal = AgentJournal(database, robot_id="robot")
        await journal.prepare()
        assert not await journal.safety.admit("first", "content")
        assert await journal.safety.pending()
        await journal.safety.complete("first")
        await journal.safety.clear_rejected()  # Actual confirmed recovery resolves obligation.
    async with Database(path) as database:
        journal = AgentJournal(database, robot_id="robot")
        await journal.prepare()
        assert await journal.safety.admit("first", "content")
        assert not await journal.safety.pending()
        with pytest.raises(DomainError) as error:
            await journal.safety.admit("first", "different-content")
        assert error.value.code is ErrorCode.CONFLICT
        assert not await journal.safety.pending()
        assert not await journal.safety.admit("second", "content")
        assert await journal.safety.pending()
    async with Database(path) as database:
        journal = AgentJournal(database, robot_id="robot")
        await journal.prepare()
        assert not await journal.safety.admit("second", "content")
        assert await journal.safety.pending()  # Unknown does not become completed on boot.
