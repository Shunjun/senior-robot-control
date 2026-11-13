"""Four opt-in real output cases. Requires separately started fresh isolated world.

Truth is evaluation only. Candidate thresholds below are declared test targets,
not frozen physical guarantees. No process/service is started by this test.
"""

import asyncio
import csv
import json
import math
import os
import time
from pathlib import Path
from uuid import uuid4

import pytest

from senior_control.protocol import MotionRuntimeState
from senior_control.runtime.ipc import IpcClient

CASES = {
    "low_start_stop": ("low", 1.0, 0.0, 12.0),
    "high_start_stop": ("high", 1.0, 0.0, 12.0),
    "low_turn": ("low", 0.25, 0.35, 8.0),
    "high_turn": ("high", 0.25, 0.35, 8.0),
}
DRIVING = dict(
    low_linear_mps=5 / 9,
    high_linear_mps=25 / 18,
    low_angular_rps=1.0,
    high_angular_rps=1.0,
    reverse_linear_mps=0.3,
)


@pytest.mark.parametrize("case", CASES)
async def test_motion_core_normal(case):
    if os.getenv("SENIOR_MOTION_CORE_ACCEPTANCE") != "1":
        pytest.skip("opt-in, fresh isolated Linux Fortress candidate world required")
    assert os.getenv("ROS_LOCALHOST_ONLY") == "1" and os.getenv("ROS_DOMAIN_ID") == "173"
    socket = os.environ["SENIOR_SIM_NAV_SOCKET"]
    output = Path(os.environ["SENIOR_MOTION_CORE_EVIDENCE"]) / case
    output.mkdir(parents=True, exist_ok=False)
    rclpy = pytest.importorskip("rclpy")
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("motion_core_acceptance_observer")
    runtime = None
    rows = []
    truth = []
    commands = []
    start = time.monotonic()

    def record_command(m):
        commands.append((time.monotonic(), m.linear.x, m.angular.z))
        rows.append(("command", time.monotonic(), "", m.linear.x, 0.0, m.angular.z, "", "", ""))

    def record_truth(m):
        p, v = m.pose.pose.position, m.twist.twist
        t = time.monotonic()
        truth.append((t, p.x, p.y, math.hypot(v.linear.x, v.linear.y), v.angular.z))
        rows.append(
            (
                "truth_eval_only",
                t,
                m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                v.linear.x,
                v.linear.y,
                v.angular.z,
                p.x,
                p.y,
                "",
            )
        )

    def record_runtime(m):
        nonlocal runtime
        endpoints = node.get_publishers_info_by_topic("/motion/runtime")
        if len(endpoints) != 1 or endpoints[0].node_name != "safety_gate":
            return
        runtime = MotionRuntimeState.model_validate_json(m.data)
        rows.append(("runtime", time.monotonic(), "", 0.0, 0.0, 0.0, "", "", m.data))

    subscriptions = [
        node.create_subscription(Twist, "/cmd_vel", record_command, 10),
        node.create_subscription(
            Odometry, "/ground_truth/odom", record_truth, qos_profile_sensor_data
        ),
        node.create_subscription(String, "/motion/runtime", record_runtime, 10),
    ]
    client = IpcClient(Path(socket))
    authority = None
    session = uuid4().hex
    seq = 0
    gear, linear, turn, active = CASES[case]

    async def sample():
        assert time.monotonic() - start < 90, "case external deadline exceeded"
        for _ in range(16):
            rclpy.spin_once(node, timeout_sec=0)
        await asyncio.sleep(0.01)

    async def request(kind, payload):
        nonlocal seq
        seq += 1
        now = time.monotonic()
        envelope = dict(
            v=1,
            type=kind,
            robot_id="route-sim",
            session_id=session,
            connection_epoch=1,
            controller_generation=1,
            request_id=f"{session}-{seq}",
            seq=seq,
            expires_at=0,
            payload=payload,
        )
        rows.append(
            (
                "request",
                now,
                "",
                0.0,
                0.0,
                0.0,
                "",
                "",
                json.dumps(
                    {
                        "type": kind,
                        "seq": seq,
                        "payload": payload,
                        "received_steady": now,
                        "deadline_steady": now + 0.2,
                    }
                ),
            )
        )
        response = await client.request(
            "manual",
            envelope=json.dumps(envelope),
            driving=DRIVING,
            received_steady=now,
            deadline_steady=now + 0.2,
        )
        rows.append(
            (
                "response",
                time.monotonic(),
                "",
                0.0,
                0.0,
                0.0,
                "",
                "",
                json.dumps({"seq": seq, "response": response}),
            )
        )
        return response

    try:
        until = time.monotonic() + 25
        while time.monotonic() < until:
            await sample()
            if (
                runtime
                and runtime.valid_until > time.monotonic()
                and runtime.healthy
                and runtime.stationary
            ):
                break
        assert runtime and runtime.healthy and runtime.stationary, (
            "no real fresh healthy/still state"
        )
        assert truth and abs(truth[-1][3]) < 0.02 and abs(truth[-1][4]) < 0.03
        response = await request("control.acquire", {})
        assert response.get("ok"), response
        authority = response["authority_id"]
        intent_seq = 1
        response = await request(
            "drive.intent",
            dict(
                authority_id=authority,
                gesture=1,
                sequence=intent_seq,
                linear=0.0,
                turn=0.0,
                gear=gear,
            ),
        )
        assert response.get("accepted"), response
        began = time.monotonic()
        while time.monotonic() - began < active:
            intent_seq += 1
            response = await request(
                "drive.intent",
                dict(
                    authority_id=authority,
                    gesture=1,
                    sequence=intent_seq,
                    linear=linear,
                    turn=turn,
                    gear=gear,
                ),
            )
            assert response.get("accepted"), response
            for _ in range(5):
                await sample()
        released = time.monotonic()
        response = await request("control.release", dict(authority_id=authority))
        assert response.get("accepted"), response
        authority = None
        until = time.monotonic() + 15
        while time.monotonic() < until:
            await sample()
            recent = [r for r in truth if r[0] > time.monotonic() - 0.5]
            if len(recent) >= 5 and all(abs(r[3]) < 0.02 and abs(r[4]) < 0.03 for r in recent):
                break
        assert truth and len(commands) > 10, "missing native truth/output streams"
        moving = [r for r in truth if began < r[0] < released]
        expected = DRIVING[f"{gear}_linear_mps"] * linear
        # Fixed candidate targets, not tuned after a run. Failure retained verbatim.
        assert max(r[3] for r in moving) >= expected * 0.8, "requested body response not reached"
        if turn:
            assert max(r[4] for r in moving) >= 0.25, "turning body response not reached"
        assert math.hypot(truth[-1][1] - truth[0][1], truth[-1][2] - truth[0][2]) > 0.05
        if not turn:
            assert truth[-1][1] - truth[0][1] > 0.05, "wrong straight travel direction"
        recent = [r for r in truth if r[0] > time.monotonic() - 0.5]
        assert len(recent) >= 5 and all(abs(r[3]) < 0.02 and abs(r[4]) < 0.03 for r in recent), (
            "body did not stop"
        )
        post = [r for r in commands if released < r[0] < released + 0.2]
        assert any(r[1] > 0 for r in post), "ordinary release skipped continuous braking"
    finally:
        if authority:
            try:
                await request("control.release", dict(authority_id=authority))
            except Exception as exc:
                # Preserve curves even if the producer is gone; original intent
                # deadline remains bounded and is never extended by cleanup.
                rows.append(
                    (
                        "cleanup_failed",
                        time.monotonic(),
                        "",
                        0.0,
                        0.0,
                        0.0,
                        "",
                        "",
                        type(exc).__name__,
                    )
                )
        with (output / "curve.csv").open("w") as f:
            writer = csv.writer(f)
            writer.writerow(["kind", "steady", "ros", "vx", "vy", "wz", "x", "y", "runtime"])
            writer.writerows(rows)
        (output / "case.json").write_text(
            json.dumps(
                dict(
                    case=case,
                    profile="fortress-candidate-unvalidated-v1",
                    synthetic=False,
                    truth_evaluation_only=True,
                    active_seconds=active,
                ),
                indent=2,
            )
        )
        await client.close()
        for sub in subscriptions:
            node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()
