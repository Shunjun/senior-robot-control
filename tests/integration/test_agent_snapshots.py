"""Task 14 fix: the relay accepts the agent's snapshot uplink and keeps it as a cache."""

from __future__ import annotations

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
from senior_control.protocol import serialize_envelope

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
