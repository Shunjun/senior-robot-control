"""Task 13 agent supervisor contract. No ROS import: every ROS-facing call is injected.

Interface assumed by these tests (the stubs only declare it):

* ``Supervisor`` is an async context manager that owns its heartbeat task, its background
  tasks, and the optional uplink retry loop. Constructing it starts nothing.
* ``await supervisor.request_ros(name)`` runs the injected ``call_ros`` without blocking
  the heartbeat.
* A failing background task revokes the runtime and notifies the Agent.
* ``AuthRejected`` from ``connect`` ends the uplink loop with state ``auth_failed``; other
  connection errors retry with delay.
"""

from __future__ import annotations

import asyncio
import sys

import pytest
from vehicle_manager.agent.supervisor import AuthRejected, Supervisor


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
        heartbeat_ms=20,
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


async def test_background_failure_stops_only_its_watcher() -> None:
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
        background={"scan": watcher},
        heartbeat_ms=20,
    )
    async with supervisor:
        assert supervisor.motion_allowed is True
        await settle(0.1)
        assert delivered == []

        fail.set()
        await settle(0.1)

        assert supervisor.motion_allowed is True
        await settle(0.1)
        assert delivered == []
        assert published == []


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
        connect=rejected,
        heartbeat_ms=20,
        retry_delay_ms=10,
    )
    async with denied:
        await settle(0.3)
        assert attempts["auth"] == 1
        assert denied.state == "auth_failed"
        assert denied.motion_allowed is False

    retrying = Supervisor(
        send_heartbeat=heartbeat,
        call_ros=ros,
        connect=flaky,
        heartbeat_ms=20,
        retry_delay_ms=10,
        max_retry_delay=0.02,
    )
    async with retrying:
        await settle(0.3)
        assert attempts["network"] >= 6
        assert retrying.state == "reconnecting"


async def test_shutdown_when_heartbeat_cancellation_is_swallowed() -> None:
    entered = asyncio.Event()
    release = asyncio.Event()

    async def beat() -> None:
        entered.set()
        try:
            await release.wait()
        except asyncio.CancelledError:
            return  # A transport may finish while shutdown cancels the send.

    async def ros(name: str) -> None:
        pass

    supervisor = Supervisor(send_heartbeat=beat, call_ros=ros, heartbeat_ms=60000)
    await supervisor.__aenter__()
    await entered.wait()
    close = asyncio.create_task(supervisor.__aexit__(None, None, None))
    done, _ = await asyncio.wait({close}, timeout=0.5)
    assert close in done
    await close


async def test_shutdown_reports_stubborn_background_within_deadline() -> None:
    entered = asyncio.Event()
    release = asyncio.Event()

    async def stubborn() -> None:
        entered.set()
        while not release.is_set():
            try:
                await release.wait()
            except asyncio.CancelledError:
                pass

    async def beat() -> None:
        pass

    async def ros(name: str) -> None:
        pass

    async def sink(vx: float, wz: float) -> None:
        pass

    supervisor = Supervisor(
        send_heartbeat=beat,
        call_ros=ros,
        background={"stubborn": stubborn},
        shutdown_timeout_ms=50,
    )
    await supervisor.__aenter__()
    await entered.wait()
    tasks = list(supervisor._tasks)
    try:
        with pytest.raises(RuntimeError, match="shutdown deadline"):
            await asyncio.wait_for(supervisor.__aexit__(None, None, None), 0.5)
    finally:
        release.set()
        await asyncio.gather(*tasks, return_exceptions=True)
