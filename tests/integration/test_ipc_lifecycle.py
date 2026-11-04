"""IPC startup and cleanup preserve ownership of filesystem socket paths."""

import asyncio
import os
import socket
from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from senior_control.runtime.ipc import IpcClient, IpcServer


@pytest.fixture
def socket_dir():
    with TemporaryDirectory(prefix="sr-ipc-", dir="/tmp") as directory:
        yield Path(directory)


async def answer(_message):
    return {"ok": True}


async def test_bind_then_chmod_failure_closes_listener_and_removes_socket(socket_dir, monkeypatch):
    async def run():
        path = socket_dir / "ipc.sock"
        server = IpcServer(path, answer)
        listener = None
        real_start = asyncio.start_unix_server

        async def capture(*args, **kwargs):
            nonlocal listener
            listener = await real_start(*args, **kwargs)
            return listener

        def fail_chmod(*_args):
            raise PermissionError("chmod failed")

        monkeypatch.setattr(asyncio, "start_unix_server", capture)
        monkeypatch.setattr(os, "chmod", fail_chmod)
        with pytest.raises(PermissionError, match="chmod failed"):
            await server.start()
        assert listener is not None and not listener.is_serving()
        assert not path.exists()
        await server.close()

    await run()


async def test_close_before_start_or_after_bind_failure_keeps_regular_file(socket_dir):
    async def run():
        path = socket_dir / "occupied"
        path.write_text("keep")
        server = IpcServer(path, answer)
        await server.close()
        assert path.read_text() == "keep"
        with pytest.raises(OSError):
            await server.start()
        await server.close()
        assert path.read_text() == "keep"

    await run()


@pytest.mark.parametrize("replacement", ["file", "socket"])
async def test_close_does_not_remove_replaced_path(socket_dir, replacement):
    async def run():
        path = socket_dir / "ipc.sock"
        server = IpcServer(path, answer)
        await server.start()
        path.unlink()
        other = None
        if replacement == "file":
            path.write_text("replacement")
        else:
            other = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            other.bind(str(path))
        try:
            inode = path.lstat().st_ino
            await server.close()
            await server.close()
            assert path.lstat().st_ino == inode
            if replacement == "file":
                assert path.read_text() == "replacement"
        finally:
            if other is not None:
                other.close()
            path.unlink(missing_ok=True)

    await run()


async def test_start_close_restart_and_cancelled_start(socket_dir, monkeypatch):
    async def run():
        path = socket_dir / "ipc.sock"
        server = IpcServer(path, answer)
        for _ in range(2):
            await server.start()
            client = IpcClient(path)
            assert (await client.request("ping"))["ok"] is True
            await client.close()
            await server.close()
            assert not path.exists()
        real_chmod = os.chmod

        def cancel(*_args):
            raise asyncio.CancelledError()

        monkeypatch.setattr(os, "chmod", cancel)
        with pytest.raises(asyncio.CancelledError):
            await server.start()
        assert not path.exists()
        monkeypatch.setattr(os, "chmod", real_chmod)
        await server.start()
        await server.close()
        assert not path.exists()

    await run()


async def test_cancelled_close_removes_owned_path_and_stops_listener(socket_dir):
    async def run():
        path = socket_dir / "ipc.sock"
        entered = asyncio.Event()
        cancelled = asyncio.Event()

        async def waiting_handler(_message):
            entered.set()
            try:
                await asyncio.Event().wait()
            finally:
                cancelled.set()
                await asyncio.Event().wait()

        server = IpcServer(path, waiting_handler)
        await server.start()
        client = IpcClient(path)
        request = asyncio.create_task(client.request("ping"))
        await asyncio.wait_for(entered.wait(), 2)
        closing = asyncio.create_task(server.close())
        await asyncio.wait_for(cancelled.wait(), 2)
        closing.cancel()
        with pytest.raises(asyncio.CancelledError):
            await closing
        request.cancel()
        await asyncio.gather(request, return_exceptions=True)
        await client.close()
        assert not path.exists()
        with pytest.raises(OSError):
            await asyncio.open_unix_connection(str(path))
        await server.close()

    await run()


async def test_concurrent_servers_close_only_their_own_socket(socket_dir):
    async def run():
        path = socket_dir / "ipc.sock"
        first = IpcServer(path, answer)
        second = IpcServer(path, answer)
        try:
            await asyncio.gather(first.start(), second.start())
            await first.close()
            assert path.exists()
            client = IpcClient(path)
            try:
                assert (await client.request("ping"))["ok"] is True
            finally:
                await client.close()
            await second.close()
            assert not path.exists()
        finally:
            await first.close()
            await second.close()

    await run()


async def test_concurrent_start_on_same_server_is_rejected_and_cancel_cleans_socket(
    socket_dir, monkeypatch
):
    async def run():
        path = socket_dir / "ipc.sock"
        server = IpcServer(path, answer)
        entered = asyncio.Event()
        real_start = asyncio.start_unix_server
        captured = None

        async def pending(*args, **kwargs):
            nonlocal captured
            captured = kwargs.get("sock")
            entered.set()
            await asyncio.Event().wait()
            return await real_start(*args, **kwargs)

        monkeypatch.setattr(asyncio, "start_unix_server", pending)
        starting = asyncio.create_task(server.start())
        await asyncio.wait_for(entered.wait(), 2)
        try:
            with pytest.raises(RuntimeError, match="started|starting"):
                await asyncio.wait_for(server.start(), 0.5)
        finally:
            starting.cancel()
            with pytest.raises(asyncio.CancelledError):
                await starting
        assert captured is not None and captured.fileno() == -1
        assert not path.exists()
        await server.close()
        monkeypatch.setattr(asyncio, "start_unix_server", real_start)
        await server.start()
        await server.close()

    await run()


async def test_close_cancels_owned_pending_start_without_resurrection(socket_dir, monkeypatch):
    async def run():
        path = socket_dir / "ipc.sock"
        server = IpcServer(path, answer)
        entered = asyncio.Event()
        release = asyncio.Event()
        real_start = asyncio.start_unix_server
        pending_socket = None

        async def blocked(*args, **kwargs):
            nonlocal pending_socket
            pending_socket = kwargs["sock"]
            entered.set()
            await release.wait()
            return await real_start(*args, **kwargs)

        monkeypatch.setattr(asyncio, "start_unix_server", blocked)
        starting = asyncio.create_task(server.start())
        await asyncio.wait_for(entered.wait(), 2)
        try:
            await asyncio.wait_for(server.close(), 2)
            release.set()
            result = await asyncio.wait_for(asyncio.gather(starting, return_exceptions=True), 2)
            assert isinstance(result[0], asyncio.CancelledError)
            assert pending_socket is not None and pending_socket.fileno() == -1
            assert not path.exists()
            with pytest.raises(OSError):
                await asyncio.open_unix_connection(str(path))
        finally:
            starting.cancel()
            await asyncio.gather(starting, return_exceptions=True)
            await server.close()
        monkeypatch.setattr(asyncio, "start_unix_server", real_start)
        await server.start()
        await server.close()
        assert not path.exists()

    await run()


@pytest.mark.parametrize("failure", ["error", "cancel"])
async def test_start_failure_preserves_original_error_when_cleanup_also_fails(
    socket_dir, monkeypatch, failure
):
    async def run():
        path = socket_dir / "ipc.sock"
        server = IpcServer(path, answer)
        real_unlink = os.unlink

        def fail_chmod(*_args):
            if failure == "cancel":
                raise asyncio.CancelledError("original cancellation")
            raise PermissionError("original chmod failure")

        def fail_unlink(*_args, **_kwargs):
            raise OSError("cleanup unlink failure")

        monkeypatch.setattr(os, "chmod", fail_chmod)
        monkeypatch.setattr(os, "unlink", fail_unlink)
        try:
            expected = asyncio.CancelledError if failure == "cancel" else PermissionError
            with pytest.raises(expected, match="original") as error:
                await server.start()
            assert isinstance(error.value.__cause__, OSError)
            assert str(error.value.__cause__) == "cleanup unlink failure"
        finally:
            monkeypatch.setattr(os, "unlink", real_unlink)
            if path.exists():
                path.unlink()
            await server.close()

    await run()


async def test_close_does_not_cancel_the_external_start_caller(socket_dir, monkeypatch):
    async def run():
        server = IpcServer(socket_dir / "ipc.sock", answer)
        entered = asyncio.Event()

        async def blocked(*_args, **_kwargs):
            entered.set()
            await asyncio.Event().wait()

        monkeypatch.setattr(asyncio, "start_unix_server", blocked)

        async def caller():
            try:
                await server.start()
            except asyncio.CancelledError:
                return "caller continued"
            raise AssertionError("pending start should stop")

        caller_task = asyncio.create_task(caller())
        await asyncio.wait_for(entered.wait(), 2)
        await asyncio.wait_for(server.close(), 2)
        assert await asyncio.wait_for(caller_task, 2) == "caller continued"

    await run()
