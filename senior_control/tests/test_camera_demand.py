"""Demand ownership uses real child processes; no ROS or motion command is started."""

import asyncio
import sys
import tempfile
from pathlib import Path

import pytest
from vehicle_manager.media.camera_demand import CameraDemand


@pytest.fixture
def service_root():
    with tempfile.TemporaryDirectory(prefix="camera-", dir="/tmp") as directory:
        yield Path(directory).resolve()


async def wait_until(condition):
    for _ in range(200):
        if condition():
            return
        await asyncio.sleep(0.01)
    raise AssertionError("camera ownership did not converge")


async def test_camera_shared_until_last_consumer_releases():
    children = []

    async def start(component):
        child = await asyncio.create_subprocess_exec(
            sys.executable, "-c", "import time; time.sleep(60)"
        )
        children.append(child)
        return child

    async def stop(child):
        if child.returncode is None:
            child.terminate()
        await child.wait()

    owner = CameraDemand(start=start, stop=stop)
    task = asyncio.create_task(owner.run())
    try:
        await asyncio.sleep(0.03)
        assert not children
        assert owner.request("camera.acquire", "preview")["ok"]
        await wait_until(lambda: len(children) == 1)
        assert owner.request("camera.acquire", "depth-consumer")["ok"]
        owner.request("camera.release", "preview")
        await asyncio.sleep(0.03)
        assert children[0].returncode is None
        assert len(children) == 1
        owner.request("camera.release", "depth-consumer")
        await wait_until(lambda: children[0].returncode is not None)
    finally:
        await owner.close()
        await task


async def test_abandoned_camera_lease_expires():
    children = []

    async def start(component):
        child = await asyncio.create_subprocess_exec(
            sys.executable, "-c", "import time; time.sleep(60)"
        )
        children.append(child)
        return child

    async def stop(child):
        if child.returncode is None:
            child.terminate()
        await child.wait()

    owner = CameraDemand(start=start, stop=stop, lease_seconds=0.08)
    task = asyncio.create_task(owner.run())
    try:
        owner.request("camera.acquire", "lost-agent")
        await wait_until(lambda: len(children) == 1)
        await wait_until(lambda: children[0].returncode is not None)
        assert not owner.request("camera.renew", "lost-agent")["ok"]
    finally:
        await owner.close()
        await task


async def test_compression_consumer_does_not_restart_shared_capture():
    children = {}

    async def start(component):
        child = await asyncio.create_subprocess_exec(
            sys.executable, "-c", "import time; time.sleep(60)"
        )
        children[component] = child
        return child

    async def stop(child):
        if child.returncode is None:
            child.terminate()
        await child.wait()

    owner = CameraDemand(start=start, stop=stop)
    task = asyncio.create_task(owner.run())
    try:
        owner.request("camera.acquire", "preview", ["color"])
        await wait_until(lambda: "capture" in children)
        capture = children["capture"]
        owner.request("camera.acquire", "recorder", ["jpeg", "depth_compressed"])
        await wait_until(lambda: len(children) == 3)
        owner.request("camera.release", "preview")
        await asyncio.sleep(0.03)
        assert capture.returncode is None
        owner.request("camera.acquire", "depth-reader", ["depth"])
        owner.request("camera.release", "recorder")
        await wait_until(
            lambda: all(children[k].returncode is not None for k in ["jpeg", "depth_compressed"])
        )
        assert children["capture"] is capture
        assert capture.returncode is None
    finally:
        await owner.close()
        await task
    assert capture.returncode is not None


async def test_production_preview_releases_lease_source_and_encoder(service_root, monkeypatch):
    import threading

    from senior_robot import ros_video
    from vehicle_manager.agent.config import AgentConfig
    from vehicle_manager.agent.production import _media_binding
    from vehicle_manager.media.stream import SessionDescription, VideoSession

    from vehicle_core.ipc import IpcServer

    aiortc = pytest.importorskip("aiortc")
    sources = []
    children = []

    class Source:
        def __init__(self, *args, **kwargs):
            self.closed = False
            sources.append(self)

        async def next_frame(self, timeout):
            await asyncio.sleep(timeout)
            return None

        async def close(self):
            self.closed = True

    async def start(component):
        child = await asyncio.create_subprocess_exec(
            sys.executable, "-c", "import time; time.sleep(60)"
        )
        children.append(child)
        return child

    async def stop(child):
        if child.returncode is None:
            child.terminate()
        await child.wait()

    owner = CameraDemand(start=start, stop=stop)
    task = asyncio.create_task(owner.run())

    async def handle(request):
        return owner.request(request["type"], request.get("token"), request.get("capabilities"))

    server = IpcServer(service_root / "service.sock", handle)
    await server.start()
    monkeypatch.setattr(ros_video, "RosCameraSource", Source)
    binding = _media_binding(
        AgentConfig(
            robot_id="robot-a",
            cloud_url="wss://example.test",
            device_token="test-only",
            database_path=service_root / "agent.db",
            socket_path=service_root / "agent.sock",
            ros_video_topic="/camera/color/image_raw",
        )
    )
    before = set(threading.enumerate())
    client = aiortc.RTCPeerConnection()
    client.addTransceiver("video", direction="recvonly")
    try:
        session = binding.session()
        assert isinstance(session, VideoSession)
        await client.setLocalDescription(await client.createOffer())
        await session.offer("robot-a", SessionDescription("offer", client.localDescription.sdp))
        await wait_until(lambda: bool(children))
        workers = [t for t in threading.enumerate() if t not in before and t.name == "video-encode"]
        assert len(workers) == 1
        await session.close()
        await wait_until(lambda: children[0].returncode is not None)
        assert sources[0].closed
        assert not workers[0].is_alive()
    finally:
        await binding.close()
        await client.close()
        await server.close()
        await owner.close()
        await task


def test_compression_gate_limits_work_to_fifteen_fps_and_keeps_latest_payload():
    from senior_robot.camera_rate_limit import LatestImageGate

    gate = LatestImageGate()
    emitted = []
    for index in range(30):
        gate.offer(f"original-serialized-header-and-pixels:{index}".encode())
        image = gate.take(index / 30)
        if image is not None:
            emitted.append((index / 30, image))
    assert 14 <= len(emitted) <= 15
    assert all(b[0] - a[0] >= 1 / 15 - 1e-9 for a, b in zip(emitted, emitted[1:], strict=False))
    gate.offer(b"superseded-frame")
    gate.offer(b"newest-original-header-and-pixels")
    assert gate.take(0.98) is None
    assert gate.take(1.04) == b"newest-original-header-and-pixels"
    assert gate.take(1.2) is None
