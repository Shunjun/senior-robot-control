"""Shared camera leases; capture stays whole while optional compression is independent."""

import asyncio
import logging
import time
from collections.abc import Awaitable, Callable
from typing import Any

logger = logging.getLogger(__name__)
CAPABILITIES = frozenset({"color", "depth", "ir", "point_cloud", "jpeg", "depth_compressed"})
COMPONENTS = ("capture", "jpeg", "depth_compressed")
LEASE_SECONDS = 12.0


class CameraDemand:
    """One service owns all camera children; consumer tokens own only their demands."""

    def __init__(
        self,
        *,
        start: Callable[[str], Awaitable[asyncio.subprocess.Process]],
        stop: Callable[[asyncio.subprocess.Process], Awaitable[None]],
        ready: asyncio.Event | None = None,
        lease_seconds: float = LEASE_SECONDS,
    ) -> None:
        self._start = start
        self._stop = stop
        self._ready = ready
        self._lease_seconds = lease_seconds
        self._leases: dict[str, tuple[float, frozenset[str]]] = {}
        self._children: dict[str, asyncio.subprocess.Process] = {}
        self._retry_at: dict[str, float] = {}
        self._wake = asyncio.Event()
        self._closing = False
        self._finished = asyncio.Event()

    def request(self, kind: str, token: Any, capabilities: Any = None) -> dict[str, Any]:
        if self._closing or not isinstance(token, str) or not 1 <= len(token) <= 128:
            return {"ok": False, "error": "invalid_camera_lease"}
        if kind == "camera.acquire":
            requested = ["color"] if capabilities is None else capabilities
            if (
                not isinstance(requested, list)
                or not requested
                or any(not isinstance(item, str) or item not in CAPABILITIES for item in requested)
            ):
                return {"ok": False, "error": "invalid_camera_capabilities"}
            if token not in self._leases and len(self._leases) >= 64:
                return {"ok": False, "error": "camera_consumers_full"}
            resources = frozenset({"capture", *(set(requested) & {"jpeg", "depth_compressed"})})
            self._leases[token] = (time.monotonic() + self._lease_seconds, resources)
        elif kind == "camera.renew":
            existing = self._leases.get(token)
            if existing is None or existing[0] <= time.monotonic():
                return {"ok": False, "error": "camera_lease_expired"}
            self._leases[token] = (time.monotonic() + self._lease_seconds, existing[1])
        elif kind == "camera.release":
            self._leases.pop(token, None)
        else:
            return {"ok": False, "error": "unsupported"}
        self._wake.set()
        return {"ok": True}

    async def run(self) -> None:
        try:
            while not self._closing:
                self._wake.clear()
                now = time.monotonic()
                self._leases = {key: lease for key, lease in self._leases.items() if lease[0] > now}
                wanted = set().union(*(lease[1] for lease in self._leases.values()))
                if self._ready is not None and not self._ready.is_set():
                    wanted.clear()
                # Stop dependent processors first. Capture is never restarted on consumer changes.
                for name in reversed(COMPONENTS):
                    child = self._children.get(name)
                    if child is not None and (name not in wanted or child.returncode is not None):
                        await self._stop(child)
                        del self._children[name]
                        self._retry_at[name] = time.monotonic() + 2 if name in wanted else 0
                for name in COMPONENTS:
                    if self._closing:
                        break
                    if name not in wanted or name in self._children:
                        continue
                    if name != "capture" and "capture" not in self._children:
                        continue
                    if time.monotonic() < self._retry_at.get(name, 0):
                        continue
                    try:
                        self._children[name] = await self._start(name)
                    except (OSError, ValueError):
                        logger.warning("camera component %s unavailable", name, exc_info=True)
                        self._retry_at[name] = time.monotonic() + 2
                try:
                    await asyncio.wait_for(self._wake.wait(), min(0.25, self._lease_seconds / 2))
                except asyncio.TimeoutError:
                    pass
        finally:
            try:
                for name in reversed(COMPONENTS):
                    child = self._children.pop(name, None)
                    if child is not None:
                        await self._stop(child)
            finally:
                self._finished.set()

    async def close(self) -> None:
        self._closing = True
        self._leases.clear()
        self._wake.set()
        await self._finished.wait()
