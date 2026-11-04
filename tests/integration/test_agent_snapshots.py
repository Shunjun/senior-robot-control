"""Task 14 fix: the relay accepts the agent's snapshot uplink and keeps it as a cache."""

from __future__ import annotations

import json
from typing import Any

import pytest
from agent_fakes import MAP, ROBOT, simple

from senior_control.agent.snapshots import (
    HomeSummary,
    RobotSnapshot,
    SnapshotCache,
    snapshot_envelope,
)
from senior_control.cloud.limits import CloudLimits
from senior_control.cloud.relay import CLOSE_BAD_FRAME, CLOSE_POLICY, Connection, Hub
from senior_control.protocol import SnapshotEnvelope, serialize_envelope, validate_frame

SNAPSHOT = RobotSnapshot(
    robot_id=ROBOT,
    connected=True,
    localized=True,
    map_ref=MAP,
    home=HomeSummary("home", MAP, "ready"),
    task_revision=6,
    runnable_task_ids=("patrol",),
    execution_status="idle",
    collected_at=1234.5,
)


class Rig:
    def __init__(self) -> None:
        self.now = [5000.0]
        self.hub = Hub(
            CloudLimits(), snapshots=SnapshotCache(stale_after=5.0, clock=lambda: self.now[0])
        )
        self.agent: Connection = self.hub.attach_agent(ROBOT)

    def frame(
        self, snapshot: RobotSnapshot = SNAPSHOT, *, epoch: int | None = None, robot_id: str = ROBOT
    ) -> bytes:
        envelope = snapshot_envelope(
            snapshot,
            robot_id=robot_id,
            session_id="agent-session",
            connection_epoch=self.agent.epoch if epoch is None else epoch,
            request_id="snap-1",
            seq=1,
            expires_at=9e9,
        )
        return serialize_envelope(envelope)


async def test_snapshot_frame_is_cached_with_last_synced_at() -> None:
    rig = Rig()
    rig.hub.handle_agent_frame(rig.agent, rig.frame())
    assert not rig.agent.outbox.closed
    cached = rig.hub.snapshots.get(ROBOT)
    assert cached is not None
    assert cached.snapshot == SNAPSHOT
    assert cached.last_synced_at == 5000.0 and cached.stale is False


async def test_snapshot_goes_stale_when_the_robot_goes_offline() -> None:
    rig = Rig()
    rig.hub.handle_agent_frame(rig.agent, rig.frame())
    rig.now[0] = 5001.0
    rig.hub.detach(rig.agent)
    cached = rig.hub.snapshots.get(ROBOT)
    assert cached is not None and cached.stale is True
    assert cached.last_synced_at == 5000.0 and cached.snapshot.task_revision == 6
    assert cached.snapshot.connected is False


async def test_superseded_socket_does_not_mark_the_new_one_offline() -> None:
    rig = Rig()
    newer = rig.hub.attach_agent(ROBOT)
    rig.hub.handle_agent_frame(newer, rig.frame(epoch=newer.epoch))
    rig.hub.detach(rig.agent)
    cached = rig.hub.snapshots.get(ROBOT)
    assert cached is not None and cached.stale is False


async def test_snapshot_from_an_older_epoch_is_dropped() -> None:
    rig = Rig()
    newer = rig.hub.attach_agent(ROBOT)
    rig.hub.handle_agent_frame(newer, rig.frame(epoch=rig.agent.epoch))
    assert rig.hub.snapshots.get(ROBOT) is None
    assert not newer.outbox.closed


async def test_snapshot_for_another_robot_closes_the_socket() -> None:
    rig = Rig()
    rig.hub.handle_agent_frame(rig.agent, rig.frame(robot_id="robot-2"))
    assert await rig.agent.outbox.get() == CLOSE_BAD_FRAME
    assert rig.hub.snapshots.get("robot-2") is None


async def test_command_frame_on_the_agent_socket_is_still_rejected() -> None:
    rig = Rig()
    rig.hub.handle_agent_frame(rig.agent, serialize_envelope(simple("drive.stop", seq=1)))
    assert await rig.agent.outbox.get() == CLOSE_POLICY
    assert rig.hub.snapshots.get(ROBOT) is None


async def test_a_controller_cannot_inject_a_snapshot() -> None:
    rig = Rig()
    controller = rig.hub.attach_controller(ROBOT, "session-hash")
    assert controller is not None
    rig.hub.handle_control_frame(controller, rig.frame())
    assert await controller.outbox.get() == CLOSE_POLICY
    assert rig.hub.snapshots.get(ROBOT) is None


def test_cache_requires_positive_staleness() -> None:
    with pytest.raises(ValueError):
        SnapshotCache(stale_after=0)


# --------------------------------------------------------------- push to the browser


def _pushed(controller: Connection) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    while controller.outbox.pending:
        item = controller.outbox._items.popleft()  # noqa: SLF001
        frames.append(json.loads(item[1]))
    return frames


def _controller(rig: Rig, robot_id: str = ROBOT) -> Connection:
    controller = rig.hub.attach_controller(robot_id, "session-hash")
    assert controller is not None
    return controller


async def test_a_new_controller_receives_the_cached_snapshot() -> None:
    rig = Rig()
    rig.hub.handle_agent_frame(rig.agent, rig.frame())
    rig.now[0] = 5002.0
    controller = _controller(rig)
    (frame,) = _pushed(controller)
    assert frame["type"] == "snapshot" and frame["robot_id"] == ROBOT
    assert frame["connection_epoch"] == rig.agent.epoch
    assert frame["payload"]["task_revision"] == 6
    assert frame["payload"]["stale"] is False
    assert frame["payload"]["last_synced_at"] == 5000.0
    assert frame["payload"]["connected"] is True


async def test_a_controller_with_no_cache_is_sent_nothing() -> None:
    rig = Rig()
    assert _pushed(_controller(rig)) == []


async def test_a_stale_cache_reaches_a_new_controller_as_stale_with_its_own_sync_time() -> None:
    rig = Rig()
    rig.hub.handle_agent_frame(rig.agent, rig.frame())
    rig.now[0] = 5100.0
    (frame,) = _pushed(_controller(rig))
    assert frame["payload"]["stale"] is True
    assert frame["payload"]["last_synced_at"] == 5000.0
    assert frame["payload"]["connected"] is False


async def test_a_cache_change_is_pushed_to_that_robots_controllers_only() -> None:
    rig = Rig()
    mine = _controller(rig)
    mine_too = _controller(rig)
    other = _controller(rig, "robot-2")
    rig.hub.handle_agent_frame(rig.agent, rig.frame())
    for controller in (mine, mine_too):
        (frame,) = _pushed(controller)
        assert frame["payload"]["stale"] is False
        assert frame["payload"]["last_synced_at"] == 5000.0
        assert frame["connection_epoch"] == rig.agent.epoch
    assert _pushed(other) == []


async def test_losing_the_agent_pushes_a_stale_frame_without_an_epoch() -> None:
    rig = Rig()
    controller = _controller(rig)
    rig.hub.handle_agent_frame(rig.agent, rig.frame())
    _pushed(controller)
    rig.now[0] = 5001.0
    rig.hub.detach(rig.agent)
    (frame,) = _pushed(controller)
    assert frame["payload"]["stale"] is True
    assert frame["payload"]["connected"] is False
    assert frame["payload"]["last_synced_at"] == 5000.0
    assert frame["connection_epoch"] == 0


async def test_a_new_agent_epoch_reaches_the_controllers_with_the_cache() -> None:
    rig = Rig()
    rig.hub.handle_agent_frame(rig.agent, rig.frame())
    controller = _controller(rig)
    _pushed(controller)
    newer = rig.hub.attach_agent(ROBOT)
    (frame,) = _pushed(controller)
    assert frame["connection_epoch"] == newer.epoch == rig.agent.epoch + 1


async def test_queued_snapshots_coalesce_to_the_newest() -> None:
    rig = Rig()
    controller = _controller(rig)
    rig.hub.handle_agent_frame(rig.agent, rig.frame())
    rig.now[0] = 5001.0
    rig.hub.handle_agent_frame(rig.agent, rig.frame())
    (frame,) = _pushed(controller)
    assert frame["payload"]["last_synced_at"] == 5001.0


def test_an_agent_uplink_may_omit_the_cache_fields() -> None:
    body = json.loads(Rig().frame())
    body["payload"].pop("stale", None)
    body["payload"].pop("last_synced_at", None)
    envelope = validate_frame(json.dumps(body).encode(), ROBOT)
    assert isinstance(envelope, SnapshotEnvelope)
    assert envelope.payload.stale is False and envelope.payload.last_synced_at is None
