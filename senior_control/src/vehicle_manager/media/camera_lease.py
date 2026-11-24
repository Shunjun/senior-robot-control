"""Renewable consumer lease on the service's private camera endpoint."""

import asyncio
import secrets
from collections.abc import Awaitable, Callable, Sequence
from pathlib import Path

from vehicle_core.ipc import IpcClient, IpcError
from vehicle_manager.media.stream import VideoUnavailable


class CameraLease:
    def __init__(
        self,
        socket: Path,
        *,
        capabilities: Sequence[str] = ("color",),
    ) -> None:
        self._client = IpcClient(socket, timeout_ms=1500)
        self._capabilities = list(capabilities)
        self._token = secrets.token_urlsafe(24)
        self._renewal: asyncio.Task[None] | None = None
        self._closed = False

    async def start(self, on_lost: Callable[[], Awaitable[None]]) -> None:
        try:
            reply = await self._client.request(
                "camera.acquire", token=self._token, capabilities=self._capabilities
            )
            if reply.get("ok") is not True:
                raise VideoUnavailable("camera_lease_unavailable")
        except (IpcError, OSError) as exc:
            raise VideoUnavailable("camera_service_unavailable") from exc
        self._renewal = asyncio.create_task(self._renew(on_lost))

    async def _renew(self, on_lost: Callable[[], Awaitable[None]]) -> None:
        try:
            while not self._closed:
                await asyncio.sleep(3)
                reply = await self._client.request("camera.renew", token=self._token)
                if reply.get("ok") is not True:
                    break
        except (IpcError, OSError):
            pass
        if not self._closed:
            await on_lost()

    async def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        renewal, self._renewal = self._renewal, None
        if renewal is not None and renewal is not asyncio.current_task():
            renewal.cancel()
            await asyncio.gather(renewal, return_exceptions=True)
        try:
            # Release even if acquisition was cancelled before its reply arrived.
            await self._client.request("camera.release", token=self._token)
        except (IpcError, OSError):
            pass  # The service expires abandoned leases even when transport is gone.
        finally:
            await self._client.close()
