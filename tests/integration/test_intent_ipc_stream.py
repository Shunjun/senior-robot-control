"""Real local framed IPC: notification stream never waits for handler receipts."""

import asyncio
from pathlib import Path

import pytest

from senior_control.runtime.ipc import IpcClient, IpcError, IpcServer


async def wait_for(predicate):
    async def poll():
        while not predicate():
            await asyncio.sleep(0.005)

    await asyncio.wait_for(poll(), 2)


async def test_held_intent_handler_does_not_block_stream_or_request_channel(tmp_path):
    # Keep Unix path below macOS's 104-byte sun_path limit.
    import tempfile

    with tempfile.TemporaryDirectory(prefix="is-", dir="/tmp") as directory:
        path = Path(directory) / "s"
        entered, release = asyncio.Event(), asyncio.Event()
        seen = []

        async def handler(frame):
            if frame["type"] == "manual_intent":
                seen.append(frame)
                entered.set()
                if len(seen) == 1:
                    await release.wait()
            return {"ok": True, "kind": frame["type"]}

        server = IpcServer(path, handler)
        intents, requests = IpcClient(path), IpcClient(path)
        await server.start()
        try:
            await intents.send(
                "manual_intent", sequence=1, received_steady=10, deadline_steady=10.3
            )
            await asyncio.wait_for(entered.wait(), 1)
            for sequence in range(2, 101):
                await intents.send(
                    "manual_intent", sequence=sequence, received_steady=10.1, deadline_steady=10.4
                )
            response = await requests.request("state")
            assert response == {"v": 1, "ok": True, "kind": "state"}
            assert len(seen) == 1
            with pytest.raises(IpcError, match="cannot carry requests"):
                await intents.request("state")
            with pytest.raises(IpcError, match="dedicated intent"):
                await requests.send("manual_intent")
            await asyncio.sleep(0.02)  # Drain the dedicated reader while the handler is held.
            release.set()
            await wait_for(lambda: len(seen) == 2)
            assert [f["sequence"] for f in seen] == [1, 100]
            assert seen[0]["deadline_steady"] == 10.3
            assert seen[-1]["received_steady"] == 10.1
            assert seen[-1]["deadline_steady"] == 10.4
            # No execution ACK is written on the notification stream.
            with pytest.raises(asyncio.TimeoutError):
                await asyncio.wait_for(intents._reader.read(1), 0.03)
        finally:
            release.set()
            await intents.close()
            await requests.close()
            await server.close()
        assert not path.exists() and not server._connections
