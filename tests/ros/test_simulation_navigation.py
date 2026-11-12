"""Opt-in route acceptance against a dedicated, isolated Gazebo launch."""

import asyncio
import os
import time
from pathlib import Path
from uuid import uuid4

import pytest

from senior_control.runtime.ipc import IpcClient, IpcError


@pytest.mark.asyncio
async def test_gazebo_route_order_and_authority_loss() -> None:
    socket = os.getenv("SENIOR_SIM_NAV_SOCKET")
    if not socket:
        pytest.skip("requires the dedicated qualified Gazebo simulation")
    client = IpcClient(Path(socket), timeout=8)
    heartbeat = IpcClient(Path(socket))
    epoch = 1001
    renewing = True

    async def renew() -> None:
        while renewing:
            try:
                await heartbeat.request(
                    "server_heartbeat", connection_epoch=epoch, deadline=time.monotonic() + 1
                )
            except IpcError:
                pass
            await asyncio.sleep(0.15)

    async def status(execution_id: str) -> dict:
        reply = await client.request("navigation.task_status", execution_id=execution_id)
        progress = reply.get("progress")
        assert reply.get("ok") is True and isinstance(progress, dict), reply
        return progress

    async def wait_for_route(execution_id: str, expected: set[str], timeout: float) -> set[int]:
        deadline = time.monotonic() + timeout
        active = []
        while time.monotonic() < deadline:
            progress = await status(execution_id)
            index = progress.get("active_waypoint_index")
            if index is not None and (not active or active[-1] != index):
                active.append(index)
                assert active == sorted(set(active)), active
            if progress["status"] in {"succeeded", "failed", "canceled"}:
                assert progress["status"] in expected, (progress, active)
                return set(active)
            await asyncio.sleep(0.05)
        pytest.fail(f"route {execution_id} did not finish, observed waypoints {active}")

    renewer = asyncio.create_task(renew())
    execution_id = ""
    try:

        async def ready_state() -> dict:
            for _ in range(300):
                try:
                    state = await client.request("state.get")
                except IpcError:
                    state = {}
                if state.get("navigation_ready") is True:
                    return state
                await asyncio.sleep(0.1)
            diagnostic = {key: value for key, value in state.items() if key != "scan_points"}
            diagnostic["scan_points_count"] = len(state.get("scan_points", []))
            pytest.fail(f"simulation did not qualify navigation: {diagnostic}")

        await ready_state()
        pose = await client.request("pose.get")
        assert pose["ok"] is True, pose

        async def start(poses: list[list[float]]) -> str:
            state = await ready_state()
            fields = {
                name: state[name] for name in ("map_id", "reference_revision", "map_generation")
            }
            new_id = uuid4().hex
            reply = await client.request(
                "navigation.task_start",
                execution_id=new_id,
                plan={"execution_id": new_id, **fields, "poses": poses},
            )
            assert reply.get("ok") is True, reply
            return new_id

        x, y = pose["x"], pose["y"]
        execution_id = await start([[x + distance, y, 0.0] for distance in (0.35, 0.7, 1.05)])
        assert await wait_for_route(execution_id, {"succeeded"}, 90) == {0, 1, 2}
        final_pose = await client.request("pose.get")
        assert final_pose["x"] >= x + 0.8, final_pose

        execution_id = await start([[x, y, 0.0]])
        for _ in range(200):
            if (await status(execution_id)).get("active_waypoint_index") == 0:
                break
            await asyncio.sleep(0.05)
        else:
            pytest.fail("second route never sent a goal")
        renewing = False
        await renewer
        reply = await client.request("disable_motion", connection_epoch=epoch)
        assert reply.get("ok") is True, reply
        await wait_for_route(execution_id, {"canceled", "failed"}, 20)
        epoch += 1
        assert (
            await heartbeat.request(
                "server_heartbeat", connection_epoch=epoch, deadline=time.monotonic() + 1
            )
        )["ok"] is True
        await asyncio.sleep(0.5)
        assert (await status(execution_id))["status"] in {"canceled", "failed"}
    finally:
        renewing = False
        await renewer
        if execution_id:
            try:
                await client.request("navigation.task_cancel", execution_id=execution_id)
            except IpcError:
                pass
        await client.close()
        await heartbeat.close()
