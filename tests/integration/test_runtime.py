"""Task 13 agent supervisor contract. No ROS import: every ROS-facing call is injected.

Interface assumed by these tests (the stubs only declare it):

* ``Supervisor`` is an async context manager that owns its heartbeat task, its background
  tasks, and the optional uplink retry loop. Constructing it starts nothing.
* ``await supervisor.request_ros(name)`` runs the injected ``call_ros`` without blocking
  the heartbeat.
* ``supervisor.submit_drive(vx, wz)`` is synchronous. It returns True when the value is
  accepted, False when motion is revoked. One pending slot holds the newest value; a newer
  submit, a stop included, replaces it. The call already inside ``drive_sink`` may finish.
* A failing background task revokes motion. ``publish_cmd_vel`` is never called by the
  supervisor; the safety chain owns that topic.
* ``AuthRejected`` from ``connect`` ends the uplink loop with state ``auth_failed``; other
  connection errors retry with delay.
"""

from __future__ import annotations

import asyncio
import sys

from senior_control.runtime.supervisor import AuthRejected, Supervisor


async def settle(seconds: float) -> None:
    await asyncio.sleep(seconds)


async def test_slow_ros_does_not_block_heartbeat() -> None:
    beats: list[float] = []
    release = asyncio.Event()

    async def heartbeat() -> None:
        beats.append(asyncio.get_running_loop().time())

    async def slow_ros(name: str) -> str:
        await release.wait()
        return name

    async def sink(vx: float, wz: float) -> None:
        return None

    async def publish_cmd_vel(vx: float, wz: float) -> None:
        raise AssertionError("supervisor must not publish /cmd_vel")

    supervisor = Supervisor(
        send_heartbeat=heartbeat,
        call_ros=slow_ros,
        drive_sink=sink,
        publish_cmd_vel=publish_cmd_vel,
        heartbeat_seconds=0.02,
    )
    async with supervisor:
        pending = asyncio.ensure_future(supervisor.request_ros("get_state"))
        await settle(0.25)
        assert not pending.done()
        assert len(beats) >= 5
        gaps = [b - a for a, b in zip(beats, beats[1:], strict=False)]
        assert max(gaps) < 0.1
        release.set()
        assert await asyncio.wait_for(pending, 1.0) == "get_state"
    assert "rclpy" not in sys.modules


async def test_background_failure_revokes_motion() -> None:
    delivered: list[tuple[float, float]] = []
    published: list[tuple[float, float]] = []
    fail = asyncio.Event()

    async def heartbeat() -> None:
        return None

    async def ros(name: str) -> None:
        return None

    async def sink(vx: float, wz: float) -> None:
        delivered.append((vx, wz))

    async def publish_cmd_vel(vx: float, wz: float) -> None:
        published.append((vx, wz))

    async def watcher() -> None:
        await fail.wait()
        raise RuntimeError("scan watcher died")

    supervisor = Supervisor(
        send_heartbeat=heartbeat,
        call_ros=ros,
        drive_sink=sink,
        publish_cmd_vel=publish_cmd_vel,
        background={"scan": watcher},
        heartbeat_seconds=0.02,
    )
    async with supervisor:
        assert supervisor.motion_allowed is True
        assert supervisor.submit_drive(0.2, 0.0) is True
        await settle(0.1)
        assert delivered == [(0.2, 0.0)]

        fail.set()
        await settle(0.1)

        assert supervisor.motion_allowed is False
        assert supervisor.submit_drive(0.3, 0.0) is False
        await settle(0.1)
        assert delivered == [(0.2, 0.0)]
        assert published == []


async def test_backpressure_bounds_memory() -> None:
    delivered: list[tuple[float, float]] = []
    gate = asyncio.Event()

    async def heartbeat() -> None:
        return None

    async def ros(name: str) -> None:
        return None

    async def blocked_sink(vx: float, wz: float) -> None:
        await gate.wait()
        delivered.append((vx, wz))

    async def publish_cmd_vel(vx: float, wz: float) -> None:
        raise AssertionError("supervisor must not publish /cmd_vel")

    supervisor = Supervisor(
        send_heartbeat=heartbeat,
        call_ros=ros,
        drive_sink=blocked_sink,
        publish_cmd_vel=publish_cmd_vel,
        heartbeat_seconds=0.02,
    )
    async with supervisor:
        accepted = [supervisor.submit_drive(i / 1000, 0.0) for i in range(1000)]
        assert any(accepted)
        assert supervisor.pending == 1

        gate.set()
        await settle(0.2)

        assert supervisor.pending == 0
        assert delivered == [(0.999, 0.0)]

        # One slot: while a call is inside the sink, a newer submit replaces the pending
        # value, and a stop replaces a pending non-zero command instead of queueing behind it.
        gate.clear()
        before = len(delivered)
        assert supervisor.submit_drive(0.1, 0.0) is True
        await settle(0.05)
        for vx in (0.2, 0.5, 0.3):
            assert supervisor.submit_drive(vx, 0.0) is True
        assert supervisor.submit_drive(0.0, 0.0) is True
        assert supervisor.pending == 1

        gate.set()
        await settle(0.2)

        assert supervisor.pending == 0
        assert delivered[before:] == [(0.1, 0.0), (0.0, 0.0)]


async def test_auth_failure_does_not_retry_forever() -> None:
    attempts = {"auth": 0, "network": 0}

    async def heartbeat() -> None:
        return None

    async def ros(name: str) -> None:
        return None

    async def sink(vx: float, wz: float) -> None:
        return None

    async def publish_cmd_vel(vx: float, wz: float) -> None:
        return None

    async def rejected() -> None:
        attempts["auth"] += 1
        raise AuthRejected("device credential revoked")

    async def flaky() -> None:
        attempts["network"] += 1
        raise ConnectionError("cloud unreachable")

    denied = Supervisor(
        send_heartbeat=heartbeat,
        call_ros=ros,
        drive_sink=sink,
        publish_cmd_vel=publish_cmd_vel,
        connect=rejected,
        heartbeat_seconds=0.02,
        retry_delay=0.01,
    )
    async with denied:
        await settle(0.3)
        assert attempts["auth"] == 1
        assert denied.state == "auth_failed"
        assert denied.motion_allowed is False

    retrying = Supervisor(
        send_heartbeat=heartbeat,
        call_ros=ros,
        drive_sink=sink,
        publish_cmd_vel=publish_cmd_vel,
        connect=flaky,
        heartbeat_seconds=0.02,
        retry_delay=0.01,
    )
    async with retrying:
        await settle(0.3)
        assert attempts["network"] >= 3
        assert retrying.state == "reconnecting"
