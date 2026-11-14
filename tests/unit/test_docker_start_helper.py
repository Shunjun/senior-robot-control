"""Exercise the container wrapper with real child processes and local sockets."""

import asyncio
import os
import sys
import tempfile
from pathlib import Path

import pytest

from senior_control.runtime.ipc import IpcClient


@pytest.fixture
def service_root():
    with tempfile.TemporaryDirectory(prefix="senior-service-") as directory:
        yield Path(directory).resolve()


async def wait_for_socket(root, task):
    for _ in range(200):
        if task.done():
            await task
            pytest.fail("service exited before readiness")
        if (root / "service.sock").exists():
            return
        await asyncio.sleep(0.01)
    pytest.fail("service did not become ready")


async def test_control_stop_reaps_owned_child_and_preserves_data(service_root):
    from senior_control.docker_start_helper import supervise

    marker = service_root / "agent.db"
    marker.write_text("existing database")
    child = service_root / "child.py"
    child.write_text("import time\ntime.sleep(60)\n")
    task = asyncio.create_task(
        supervise(service_root, [sys.executable, str(child)], dict(os.environ))
    )
    try:
        await wait_for_socket(service_root, task)
        client = IpcClient(service_root / "service.sock")
        try:
            assert (await client.request("status"))["running"] is True
            assert (await client.request("stop"))["ok"] is True
        finally:
            await client.close()
        assert await asyncio.wait_for(task, 5) == 0
        assert not (service_root / "service.sock").exists()
        assert marker.read_text() == "existing database"
    finally:
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


async def test_child_failure_propagates_and_socket_is_removed(service_root):
    from senior_control.docker_start_helper import supervise

    result = await supervise(
        service_root, [sys.executable, "-c", "raise SystemExit(78)"], dict(os.environ)
    )
    assert result == 78
    assert not (service_root / "service.sock").exists()


async def test_second_owner_cannot_replace_running_control_socket(service_root):
    from senior_control.docker_start_helper import supervise

    task = asyncio.create_task(
        supervise(
            service_root, [sys.executable, "-c", "import time; time.sleep(60)"], dict(os.environ)
        )
    )
    try:
        await wait_for_socket(service_root, task)
        with pytest.raises(BlockingIOError):
            await supervise(
                service_root, [sys.executable, "-c", "raise SystemExit(0)"], dict(os.environ)
            )
        client = IpcClient(service_root / "service.sock")
        try:
            assert (await client.request("stop"))["ok"] is True
        finally:
            await client.close()
        assert await asyncio.wait_for(task, 5) == 0
    finally:
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
