"""Opt-in manual drive acceptance through the real isolated safety output."""

import asyncio
import os
import time
from pathlib import Path

import pytest

from senior_control.runtime.ipc import IpcClient, IpcError


async def test_gazebo_manual_motion_and_stop():
    socket = os.getenv("SENIOR_SIM_NAV_SOCKET")
    if not socket:
        pytest.skip("requires the dedicated qualified Gazebo simulation")
    rclpy = pytest.importorskip("rclpy")
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("manual_acceptance_observer")
    outputs = []
    diagnostics = []

    def record_status(message):
        for status in message.status:
            if status.name == "safety_gate":
                diagnostics.append(
                    (status.message, {item.key: item.value for item in status.values})
                )
                del diagnostics[:-20]

    node.create_subscription(DiagnosticArray, "/safety/status", record_status, 10)
    node.create_subscription(
        Twist,
        "/cmd_vel",
        lambda msg: outputs.append((time.monotonic(), msg.linear.x, msg.angular.z)),
        10,
    )
    running = True

    async def spin():
        while running:
            rclpy.spin_once(node, timeout_sec=0)
            await asyncio.sleep(0.01)

    observer = asyncio.create_task(spin())
    client = IpcClient(Path(socket), timeout=3)
    heartbeat = IpcClient(Path(socket))
    epoch = 901

    async def renew():
        while running:
            reply = await heartbeat.request(
                "server_heartbeat", connection_epoch=epoch, deadline=time.monotonic() + 1
            )
            assert reply.get("ok") is True, reply
            await asyncio.sleep(0.15)

    renewer = asyncio.create_task(renew())

    async def send(kind, sequence, payload):
        envelope = {
            "v": 1,
            "robot_id": "route-sim",
            "session_id": "manual-acceptance",
            "connection_epoch": epoch,
            "request_id": f"manual-{sequence}",
            "seq": sequence,
            "expires_at": 0.0,
            "type": kind,
            "payload": payload,
        }
        import json

        now = time.monotonic()
        if kind == "drive.velocity":
            envelope["payload"] = {
                **payload,
                "gesture": 1,
                "sampled_at": now,
                "server_issued_at": now,
                "server_deadline": now + 0.3,
            }
        if renewer.done():
            await renewer
        result = await client.request(
            "command", envelope=json.dumps(envelope), received_at=now, local_deadline=now + 0.2
        )
        assert result.get("ok") is True, result

    try:
        state = {}
        for _ in range(300):
            state = await client.request("state.get")
            if state.get("localized") and state.get("safety_clear") and state.get("robot_pose"):
                break
            await asyncio.sleep(0.1)
        else:
            pytest.fail(f"simulation did not qualify motion: {state}")
        start = state["robot_pose"]
        assert state["scan_points"], state
        await send("control.acquire", 1, {})
        for sequence in range(2, 32):
            await send("drive.velocity", sequence, {"vx": 0.08, "wz": 0.0})
            await asyncio.sleep(0.1)
        motion_diagnostics = list(diagnostics)
        await send("drive.stop", 32, {})
        stopped_at = time.monotonic()
        for _ in range(50):
            if any(at >= stopped_at and vx == 0 and wz == 0 for at, vx, wz in outputs):
                break
            await asyncio.sleep(0.01)
        else:
            pytest.fail("safety output did not return to zero within 500 ms")
        first_zero = next(at for at, vx, wz in outputs if at >= stopped_at and vx == 0 and wz == 0)
        await asyncio.sleep(0.5)
        after_stop = [(vx, wz) for at, vx, wz in outputs if at >= first_zero]
        assert len(after_stop) >= 2 and all(vx == 0 and wz == 0 for vx, wz in after_stop)
        assert any(vx > 0.01 for _, vx, _ in outputs), (
            "safety output never allowed motion",
            motion_diagnostics,
        )
        final = await client.request("state.get")
        pose = final.get("robot_pose")
        assert pose is not None, final
        assert pose["x"] - start["x"] >= 0.02, (start, pose)
        print(f"Observed motion {pose['x'] - start['x']:.3f} m and actual zero output")
    finally:
        running = False
        try:
            await client.request("disable_motion", connection_epoch=epoch)
        except (IpcError, OSError):
            pass
        observer.cancel()
        renewer.cancel()
        await asyncio.gather(observer, renewer, return_exceptions=True)
        await client.close()
        await heartbeat.close()
        node.destroy_node()
        rclpy.shutdown()
