# ruff: noqa: F811
"""Agent keeps delivery receipts, not a durable or mirrored software latch."""

from agent_fakes import ROBOT, FakeLink, simple
from test_agent import finish, rig, start  # noqa: F401


async def test_old_database_latch_is_inert_and_preserved(rig):
    database = rig.journal._database
    await database.connection.execute(
        "CREATE TABLE agent_safety_intent (robot_id TEXT PRIMARY KEY)"
    )
    await database.connection.execute("INSERT INTO agent_safety_intent VALUES (?)", (ROBOT,))
    await database.connection.commit()
    link = FakeLink(1)
    runner = await start(rig, link)
    try:
        assert not rig.agent._safety_blocked
        assert not any(event.startswith("estop:") for event in rig.motion.events)
        async with database.connection.execute("SELECT * FROM agent_safety_intent") as cursor:
            assert await cursor.fetchone() == (ROBOT,)
    finally:
        await finish(runner, rig)


async def test_delivery_receipt_is_idempotent_without_agent_latch(rig):
    link = FakeLink(1)
    runner = await start(rig, link)
    try:
        request = simple("safety.estop", seq=1)
        await rig.agent.dispatch(request)
        await rig.agent.dispatch(request)
        assert rig.motion.events.count(f"estop:{request.request_id}") == 1
        assert not rig.agent._safety_blocked
        assert not hasattr(rig.agent, "recovery_required")
        assert not hasattr(rig.journal, "safety")
    finally:
        await finish(runner, rig)
