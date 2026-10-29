"""Task 14 backend observability: redacted structured logs and bounded metric labels."""

from __future__ import annotations

import asyncio
import io
import json
import logging
from collections.abc import AsyncIterator, Iterator
from pathlib import Path

import pytest
from agent_fakes import (
    ROBOT,
    FakeConnector,
    FakeHome,
    FakeLink,
    FakeMaps,
    FakeMotion,
    FakeState,
    FakeTasks,
    operation,
    until,
)

from senior_control.agent import Agent
from senior_control.agent.journal import AgentJournal
from senior_control.errors import DomainError, ErrorCode
from senior_control.observability import JsonFormatter, Metrics
from senior_control.protocol import OPERATION_TYPES
from senior_control.storage.database import Database

SECRET = "s3cret-device-token-value"


@pytest.fixture
def log_output() -> Iterator[io.StringIO]:
    stream = io.StringIO()
    handler = logging.StreamHandler(stream)
    handler.setFormatter(JsonFormatter())
    logger = logging.getLogger("senior_control")
    previous = (logger.level, logger.propagate)
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)
    yield stream
    logger.removeHandler(handler)
    logger.setLevel(previous[0])
    logger.propagate = previous[1]


@pytest.fixture
async def agent_and_tasks(tmp_path: Path) -> AsyncIterator[tuple[Agent, FakeTasks, FakeLink]]:
    async with Database(tmp_path / "robot.db") as database:
        journal = AgentJournal(database, robot_id=ROBOT)
        await journal.prepare()
        tasks = FakeTasks()
        link = FakeLink(epoch=1)
        agent = Agent(
            ROBOT,
            connector=FakeConnector(link),
            journal=journal,
            tasks=tasks,
            home=FakeHome(),
            maps=FakeMaps(),
            state=FakeState(),
            motion=FakeMotion(),
            heartbeat_seconds=0.02,
            retry_delay=0.01,
        )
        runner = asyncio.ensure_future(agent.run())
        await until(lambda: agent.connected)
        yield agent, tasks, link
        agent.request_shutdown()
        await asyncio.wait_for(runner, timeout=2)


async def test_logs_redact_secrets_and_keep_operation_ids(
    agent_and_tasks: tuple[Agent, FakeTasks, FakeLink], log_output: io.StringIO
) -> None:
    agent, tasks, _ = agent_and_tasks
    await agent.dispatch(operation("task.create", "op-ok-1", {"device_token": SECRET}))
    tasks.fail = DomainError(ErrorCode.VALIDATION, f"bad token {SECRET}")
    with pytest.raises(DomainError):
        await agent.dispatch(operation("task.create", "op-bad-2", {"device_token": SECRET}))

    text = log_output.getvalue()
    assert SECRET not in text
    records = [json.loads(line) for line in text.splitlines()]
    by_event = {(r["event"], r.get("operation_id")) for r in records}
    assert ("operation_accepted", "op-ok-1") in by_event
    assert ("operation_completed", "op-ok-1") in by_event
    assert ("operation_failed", "op-bad-2") in by_event
    assert all(r.get("robot_id") == ROBOT for r in records if r["event"].startswith("operation_"))


async def test_metrics_counts_operations_by_type_and_result(
    agent_and_tasks: tuple[Agent, FakeTasks, FakeLink],
) -> None:
    agent, tasks, _ = agent_and_tasks
    await agent.dispatch(operation("task.create", "op-m1", {}))
    tasks.fail = DomainError(ErrorCode.CONFLICT, "x")
    with pytest.raises(DomainError):
        await agent.dispatch(operation("task.create", "op-m2", {}))
    series = agent.metrics.series()
    assert series[("agent_operations_total", "task.create", "succeeded")] == 1
    assert series[("agent_operations_total", "task.create", "conflict")] == 1


def test_metrics_labels_are_bounded() -> None:
    metrics = Metrics()
    for index in range(2000):
        metrics.inc("agent_operations_total", operation=f"user-{index}", result=f"r-{index}")
        metrics.inc("agent_operations_total", operation="task.create", result="succeeded")
    series = metrics.series()
    assert len(series) <= 2
    assert series[("agent_operations_total", "other", "other")] == 2000
    assert series[("agent_operations_total", "task.create", "succeeded")] == 2000
    assert not any("user-" in label for key in series for label in key)

    worst = Metrics()
    for kind in (*OPERATION_TYPES, "drive.stop", "drive.velocity", "other", "bogus"):
        for result in ("succeeded", "failed", "needs_review", "offline", "conflict", "zzz"):
            worst.inc("agent_operations_total", operation=kind, result=result)
    assert len(worst.series()) <= 40 * 12

    with pytest.raises(ValueError):
        metrics.inc("per_user_metric", operation="task.create", result="succeeded")
