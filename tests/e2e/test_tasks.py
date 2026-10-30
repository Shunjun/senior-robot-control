"""A disconnected scheduler stops active work and never backfills it."""

from datetime import datetime, timedelta, timezone

from conftest import SimulationProfile

from senior_control.models import MapRef
from senior_control.tasks.models import (
    ExecutionContext,
    ExecutionRecord,
    ExecutionStatus,
    IntentAction,
    TaskDefinition,
)
from senior_control.tasks.runner import TaskRunner
from senior_control.tasks.schedule import Schedule


def test_task_disconnect_stops_no_resume(simulation_profile: SimulationProfile) -> None:
    created = datetime(2026, 10, 30, 0, 0, tzinfo=timezone.utc)
    due = created + timedelta(minutes=30)
    map_ref = MapRef("yard", 1)
    task = TaskDefinition(
        "patrol",
        "robot-a",
        map_ref,
        ("home",),
        Schedule.interval(created, 1800, "UTC"),
        "UTC",
        True,
        0,
        created,
    )
    active = ExecutionRecord("run-1", "patrol", due, ExecutionStatus.RUNNING, True, False)
    offline = ExecutionContext(
        False,
        True,
        True,
        False,
        map_ref,
        last_tick_utc=due,
        last_tick_mono=100.0,
    )

    stopped = TaskRunner([task], [active]).tick(due + timedelta(seconds=1), 101.0, offline)
    assert len(stopped) == 1
    assert stopped[0].action is IntentAction.PAUSE
    assert stopped[0].reason == "offline"

    reconnected = ExecutionContext(
        True,
        True,
        True,
        False,
        map_ref,
        last_tick_utc=due + timedelta(seconds=1),
        last_tick_mono=101.0,
    )
    after_reconnect = TaskRunner([task], [active]).tick(
        due + timedelta(seconds=2),
        102.0,
        reconnected,
    )
    assert after_reconnect == []
    assert simulation_profile.api_port != simulation_profile.relay_port
