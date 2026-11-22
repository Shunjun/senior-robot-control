import tempfile
from pathlib import Path

import pytest

from vehicle_core.ipc import IpcClient, IpcError, IpcServer


async def test_each_new_connection_checks_identity_before_business_or_stream():
    with tempfile.TemporaryDirectory(prefix="identity-") as directory:
        path = Path(directory) / "a.sock"
        identity = [True]
        calls = []

        async def handle(message):
            calls.append(message["type"])
            if message["type"] == "identity":
                return {"ok": identity[0], "ros_domain": 17}
            return {"ok": True}

        server = IpcServer(path, handle)
        await server.start()
        client = IpcClient(path, robot_id="robot", ros_domain=17)
        intent = IpcClient(path, robot_id="robot", ros_domain=17)
        try:
            assert (await client.request("state.get"))["ok"]
            assert calls == ["identity", "state.get"]
            await intent.send("manual_intent")
            for _ in range(50):
                import asyncio

                if "manual_intent" in calls:
                    break
                await asyncio.sleep(0.001)
            assert calls == ["identity", "state.get", "identity", "manual_intent"]
            await client.close()
            identity[0] = False
            with pytest.raises(IpcError):
                await client.request("state.get")
            assert calls[-1] == "identity" and calls.count("state.get") == 1
        finally:
            await client.close()
            await intent.close()
            await server.close()


async def test_owned_adapter_must_have_current_peer_verification_record():
    with tempfile.TemporaryDirectory(prefix="identity-") as directory:
        path = Path(directory) / "a.sock"
        calls = []

        async def handle(message):
            calls.append(message["type"])
            return {"ok": True}

        server = IpcServer(path, handle)
        await server.start()
        client = IpcClient(path, robot_id="robot", owner_record="current-run")
        try:
            with pytest.raises(IpcError):
                await client.request("manual")
            assert not calls and path.exists()
            path.with_suffix(".owner.json").write_text("[1,2,3]")
            with pytest.raises(IpcError):
                await client.request("manual")
            assert not calls and path.exists()
            from vehicle_core.ipc import _record

            _record(path, run_id="old-run")
            with pytest.raises(IpcError):
                await client.request("manual")
            assert not calls and path.exists()
            _record(path, run_id="current-run")
            assert (await client.request("manual"))["ok"]
            assert calls == ["identity", "manual"]
        finally:
            await client.close()
            await server.close()
