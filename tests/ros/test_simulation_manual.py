"""Opt-in manual drive acceptance through the real isolated safety output."""

import asyncio
import math
import os
import time
from collections import deque
from pathlib import Path
from statistics import median

import pytest

from senior_control.runtime.ipc import IpcClient, IpcError


async def test_gazebo_manual_motion_and_stop():
    socket = os.getenv("SENIOR_SIM_NAV_SOCKET")
    if not socket:
        pytest.skip("requires the dedicated qualified Gazebo simulation")
    rclpy = pytest.importorskip("rclpy")
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("manual_acceptance_observer")
    outputs = deque(maxlen=4096)
    body_motion = deque(maxlen=4096)
    diagnostics = []

    def record_body(message):
        twist = message.twist.twist
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        body_motion.append(
            (
                time.monotonic(),
                stamp,
                math.hypot(twist.linear.x, twist.linear.y),
                abs(twist.angular.z),
            )
        )

    node.create_subscription(Odometry, "/ground_truth/odom", record_body, qos_profile_sensor_data)

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
            if (
                state.get("localized")
                and state.get("safety_clear")
                and state.get("robot_pose")
                and state.get("scan_points")
            ):
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
        # Derive observation cadence from fresh source stamps, not an assumed sensor rate.
        source_periods = [
            later[1] - earlier[1]
            for earlier, later in zip(list(body_motion), list(body_motion)[1:], strict=False)
            if 0 < later[1] - earlier[1] <= 0.2
        ]
        assert len(source_periods) >= 3, "insufficient fresh ground-truth odometry"
        body_period = median(source_periods)
        # Allow two dropped samples, but never bridge an observation gap over 200 ms.
        max_body_gap = min(0.2, max(0.03, 3 * body_period))
        last_body_stamp = body_motion[-1][1]
        await send("drive.stop", 32, {})
        stopped_at = time.monotonic()
        for _ in range(50):
            if any(at >= stopped_at and vx == 0 and wz == 0 for at, vx, wz in outputs):
                break
            await asyncio.sleep(0.01)
        else:
            pytest.fail("safety output did not return to zero within 500 ms")
        first_zero = next(at for at, vx, wz in outputs if at >= stopped_at and vx == 0 and wz == 0)
        # This is a bounded observation window, not a physical-stop product requirement.
        # Simulation measurement tolerance: <=5 mm/s planar and <=0.01 rad/s yaw.
        body_stop_at = None
        sustained_body = []
        observation_deadline = stopped_at + 2.0
        while time.monotonic() < observation_deadline:
            sustained_body = []
            previous = None
            for sample in list(body_motion):
                at, stamp, speed, yaw_rate = sample
                if at < stopped_at or stamp <= last_body_stamp:
                    continue
                if previous is not None and stamp <= previous[1]:
                    continue
                if (
                    speed > 0.005
                    or yaw_rate > 0.01
                    or (
                        previous is not None
                        and (at - previous[0] > max_body_gap or stamp - previous[1] > max_body_gap)
                    )
                ):
                    sustained_body = []
                if speed <= 0.005 and yaw_rate <= 0.01:
                    sustained_body.append(sample)
                previous = sample
            if (
                len(sustained_body) >= 3
                and sustained_body[-1][0] - sustained_body[0][0] >= 0.2
                and sustained_body[-1][1] - sustained_body[0][1] >= 0.2
                and time.monotonic() - sustained_body[-1][0] <= max_body_gap
            ):
                body_stop_at = sustained_body[0][0]
                break
            await asyncio.sleep(0.01)
        assert body_stop_at is not None, (
            "ground-truth body motion did not hold near zero for 200 ms in the 2 s observation",
            list(body_motion)[-30:],
            motion_diagnostics,
        )
        await asyncio.sleep(max(0, first_zero + 0.5 - time.monotonic()))
        after_stop = [(vx, wz) for at, vx, wz in outputs if at >= first_zero]
        assert len(after_stop) >= 2 and all(vx == 0 and wz == 0 for vx, wz in after_stop)
        assert any(vx > 0.01 for _, vx, _ in outputs), (
            "safety output never allowed motion",
            motion_diagnostics,
        )
        final = await client.request("state.get")
        pose = final.get("robot_pose")
        assert pose is not None, final
        print(
            f"Observed displacement={pose['x'] - start['x']:.3f} m; "
            f"output_zero_latency={(first_zero - stopped_at) * 1000:.1f} ms; "
            f"measured_body_stop_latency={(body_stop_at - stopped_at) * 1000:.1f} ms; "
            f"body_near_zero_duration={sustained_body[-1][0] - body_stop_at:.3f} s; "
            f"odom_period={body_period * 1000:.1f} ms"
        )
        assert pose["x"] - start["x"] >= 0.02, (start, pose)
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
