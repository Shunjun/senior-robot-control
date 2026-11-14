"""Exclusive adapter binding preserves paths installed after simulation preflight."""

import errno
import socket
from pathlib import Path
from tempfile import TemporaryDirectory

import pytest

from senior_control.runtime.ipc import IpcClient, IpcServer


async def answer(_message):
    return {"ok": True, "owner": "replacement"}


@pytest.mark.parametrize("replacement", ["socket", "symlink", "file"])
async def test_exclusive_bind_preserves_path_replaced_after_preflight(replacement):
    with TemporaryDirectory(prefix="sr-exclusive-", dir="/tmp") as directory:
        root = Path(directory)
        path = root / "adapter.sock"
        assert not path.exists()  # The earlier simulation preflight saw no occupied path.
        candidate = IpcServer(path, answer, replace_existing=False)
        incumbent_path = root / "other.sock" if replacement == "symlink" else path
        incumbent = None
        if replacement == "file":
            path.write_text("foreign replacement")
        else:
            incumbent = IpcServer(incumbent_path, answer)
            await incumbent.start()
            if replacement == "symlink":
                path.symlink_to(incumbent_path)
        identity = path.lstat()
        try:
            with pytest.raises(OSError) as error:
                await candidate.start()
            assert error.value.errno == errno.EADDRINUSE
            await candidate.close()
            current = path.lstat()
            assert (current.st_dev, current.st_ino) == (identity.st_dev, identity.st_ino)
            if replacement == "file":
                assert path.read_text() == "foreign replacement"
            else:
                client = IpcClient(incumbent_path)
                try:
                    assert (await client.request("identity"))["owner"] == "replacement"
                finally:
                    await client.close()
        finally:
            await candidate.close()
            if incumbent is not None:
                await incumbent.close()


async def test_exclusive_bind_preserves_socket_inserted_at_bind_boundary(monkeypatch):
    from senior_control.runtime import ipc

    with TemporaryDirectory(prefix="sr-exclusive-", dir="/tmp") as directory:
        path = Path(directory) / "adapter.sock"
        real_socket = socket.socket
        replacement = None
        replacement_identity = None

        class RacingSocket(real_socket):
            def bind(self, address):
                nonlocal replacement, replacement_identity
                if address == str(path) and replacement is None:
                    replacement = real_socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    replacement.bind(address)
                    replacement.listen(1)
                    replacement_identity = path.lstat()
                return super().bind(address)

        server = IpcServer(path, answer, replace_existing=False)
        monkeypatch.setattr(ipc.socket, "socket", RacingSocket)
        try:
            with pytest.raises(OSError) as error:
                await server.start()
            assert error.value.errno == errno.EADDRINUSE
            await server.close()
            assert replacement_identity is not None
            assert path.lstat().st_ino == replacement_identity.st_ino
        finally:
            await server.close()
            if replacement is not None:
                replacement.close()


@pytest.mark.parametrize("replace_existing", ["false", "true", 0, 1, None])
def test_ipc_replacement_policy_requires_boolean(replace_existing):
    with pytest.raises(ValueError, match="must be a boolean"):
        IpcServer(Path("/tmp/unused-policy.sock"), answer, replace_existing=replace_existing)


async def test_exclusive_bind_creates_and_cleans_its_own_fresh_socket():
    with TemporaryDirectory(prefix="sr-exclusive-", dir="/tmp") as directory:
        path = Path(directory) / "adapter.sock"
        server = IpcServer(path, answer, replace_existing=False)
        client = IpcClient(path)
        try:
            await server.start()
            assert (await client.request("identity"))["owner"] == "replacement"
        finally:
            await client.close()
            await server.close()
        assert not path.exists()
