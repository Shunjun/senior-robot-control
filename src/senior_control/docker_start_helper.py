"""Own an Agent child inside Docker, with a private, PID-independent stop endpoint.

The host's systemd service runs this wrapper through ``docker exec``. ExecStop
contacts its private socket so stopping the Docker client cannot orphan an Agent.
It never stops the container or starts hardware/ROS nodes.
"""

import argparse
import asyncio
import fcntl
import os
import signal
import sys
from pathlib import Path
from typing import Any

from senior_control.runtime.ipc import IpcClient, IpcError, IpcServer
from senior_control.vehicle_deploy import private_directory


async def supervise(root: Path, command: list[str], environment: dict[str, str]) -> int:
    private_directory(root)
    fd = os.open(root / ".agent-service.lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, "wb") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        stopping = asyncio.Event()
        process: asyncio.subprocess.Process | None = None

        async def handle(request: dict[str, Any]) -> dict[str, Any]:
            if request["type"] == "status":
                return {"ok": True, "running": process is not None and process.returncode is None}
            if request["type"] == "stop":
                stopping.set()
                return {"ok": True}
            return {"ok": False, "error": "unsupported"}

        server = IpcServer(root / "service.sock", handle)
        loop = asyncio.get_running_loop()
        signals = (signal.SIGINT, signal.SIGTERM)
        waiters: list[asyncio.Task[Any]] = []
        try:
            await server.start()
            for number in signals:
                loop.add_signal_handler(number, stopping.set)
            process = await asyncio.create_subprocess_exec(*command, env=environment)
            exited = asyncio.create_task(process.wait())
            requested = asyncio.create_task(stopping.wait())
            waiters = [exited, requested]
            await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
            if stopping.is_set():
                return 0
            return await exited
        finally:
            for number in signals:
                loop.remove_signal_handler(number)
            if process is not None and process.returncode is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(process.wait(), timeout=20)
                except asyncio.TimeoutError:
                    process.kill()
                    await process.wait()
            for task in waiters:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*waiters, return_exceptions=True)
            await server.close()


async def stop(root: Path) -> int:
    private_directory(root)
    client = IpcClient(root / "service.sock", timeout=3)
    try:
        reply = await client.request("stop")
        if reply.get("ok") is not True:
            return 1
    finally:
        await client.close()
    # Wait for the owner to reap its child and remove the endpoint before systemd restarts it.
    for _ in range(250):
        if not (root / "service.sock").exists():
            return 0
        await asyncio.sleep(0.1)
    return 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("run", "stop"))
    parser.add_argument("--control-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.action == "stop":
            return asyncio.run(stop(args.control_dir))
        env = dict(os.environ)
        expected = {
            "SENIOR_CREDENTIALS_FILE": "device.json",
            "SENIOR_DATABASE_PATH": "agent.db",
            "SENIOR_AGENT_SOCKET": "agent.sock",
        }
        for key, name in expected.items():
            if env.get(key) != str(args.control_dir / name):
                raise ValueError("Agent paths must use the selected directory")
        return asyncio.run(
            supervise(
                args.control_dir,
                [sys.executable, "-m", "senior_control.agent.main"],
                env,
            )
        )
    except (OSError, ValueError, IpcError):
        print(
            "Agent service failed. Check the private directory, configuration and service owner.",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
