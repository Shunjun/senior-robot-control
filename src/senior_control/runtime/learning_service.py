"""Run-supervisor component. Filesystem commits use the dedicated ProcessWorker."""

import asyncio
import contextlib
import logging
import os
import socket
import struct
from pathlib import Path

from senior_control.motion.learning_record import MAX_RECORD_BYTES
from senior_control.motion.navigation_clock import monotonic_domain
from senior_control.runtime.learning_owner import LearningRecordOwner
from senior_control.runtime.learning_wire import (
    REQUEST,
    ActionReady,
    ActionReport,
    Attach,
    Complete,
    GeometryReport,
    PhaseReport,
    Reply,
    Reserve,
    State,
)

_LOG = logging.getLogger(__name__)


def address(run_id: str) -> str:
    # Abstract Linux socket avoids root-path length limits and stale socket files.
    return "\0senior-learning-" + run_id


class LearningService:
    def __init__(self, path: Path):
        self.path = path
        self.owner: LearningRecordOwner | None = None
        self.listener: socket.socket | None = None
        self.stop = asyncio.Event()
        self.task: asyncio.Task[None] | None = None
        self.peers: set[asyncio.Task[None]] = set()

    async def start(self) -> None:
        run_id = self.path.name.removeprefix(".startup-learning-")
        try:
            listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            self.listener = listener
            listener.setblocking(False)
            listener.bind(address(run_id))
            listener.listen(8)
            self.owner = LearningRecordOwner(self.path, run_id, monotonic_domain(), new_run=True)
            self.task = asyncio.create_task(self._run())
        except (OSError, ValueError, RuntimeError) as exc:
            _LOG.warning("Learning service unavailable: %s", exc)
            if self.listener is not None:
                self.listener.close()
                self.listener = None
            if self.owner is not None:
                self.owner.close()
                self.owner = None

    async def _run(self) -> None:
        assert self.owner is not None and self.listener is not None
        loop = asyncio.get_running_loop()
        accepting = asyncio.create_task(loop.sock_accept(self.listener))
        try:
            while not self.stop.is_set():
                try:
                    self.owner.poll()
                    if accepting.done():
                        connection, _ = accepting.result()
                        if len(self.peers) >= 8:
                            connection.close()
                        else:
                            peer = asyncio.create_task(self._serve(connection))
                            self.peers.add(peer)
                            peer.add_done_callback(self.peers.discard)
                        accepting = asyncio.create_task(loop.sock_accept(self.listener))
                except (OSError, ValueError, RuntimeError) as exc:
                    _LOG.warning("Learning owner component error: %s", exc)
                with contextlib.suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(self.stop.wait(), 0.01)
        finally:
            accepting.cancel()
            await asyncio.gather(accepting, return_exceptions=True)

    async def _serve(self, connection: socket.socket) -> None:
        try:
            # The endpoint is a private same-user, same-run capability, not a
            # cloud route. Peer credentials are kernel supplied on target Linux.
            peer_credential_option = getattr(socket, "SO_PEERCRED", None)
            if peer_credential_option is None:
                raise ValueError("learning_peer_credentials_unavailable")
            credentials = connection.getsockopt(socket.SOL_SOCKET, int(peer_credential_option), 12)
            _, uid, _ = struct.unpack("3i", credentials)
            if uid != os.geteuid():
                connection.close()
                return
            raw = await asyncio.wait_for(
                asyncio.get_running_loop().sock_recv(connection, MAX_RECORD_BYTES + 1), 2
            )
            if len(raw) > MAX_RECORD_BYTES:
                raise ValueError("learning_request_size")
            assert self.owner is not None
            owner = self.owner
            request = REQUEST.validate_json(raw)
            owner.poll()
            revision = None
            ready = False
            session = None
            if isinstance(request, Attach):
                session, _ = owner.begin_consumer()
            elif isinstance(request, State):
                pass
            elif isinstance(request, Reserve):
                revision = owner.reserve_action(request.session)
            elif isinstance(request, ActionReady):
                ready = owner.action_ready(request.session, request.revision)
            elif isinstance(request, ActionReport):
                owner.report_action(
                    request.session,
                    request.start_ns,
                    request.interval_ns,
                    finished=request.finished,
                )
            elif isinstance(request, GeometryReport):
                owner.set_geometry(request.session, request.geometry)
            elif isinstance(request, PhaseReport):
                owner.set_phase(request.session, request.phase, request.reason, request.segment_id)
            elif isinstance(request, Complete):
                revision = owner.complete_model(request.session, request.points)
            reply = Reply(
                ok=True,
                error=owner.error,
                record=owner.public_record,
                session=session,
                revision=revision,
                ready=ready,
                durable_revision=owner.durable_revision,
            )
        except asyncio.CancelledError:
            connection.close()
            raise
        except (OSError, ValueError, RuntimeError, asyncio.TimeoutError) as exc:
            reply = Reply(ok=False, error=str(exc)[:256])
        try:
            payload = reply.model_dump_json().encode()
            # At most one bounded reply. A slow peer cannot block the owner loop.
            reply_until = asyncio.get_running_loop().time() + 2.0
            while True:
                try:
                    if connection.send(payload) != len(payload):
                        raise OSError("learning_reply_partial")
                    break
                except BlockingIOError:
                    if asyncio.get_running_loop().time() >= reply_until:
                        raise OSError("learning_reply_timeout") from None
                    await asyncio.sleep(0.01)
        except (OSError, asyncio.CancelledError):
            pass
        finally:
            connection.close()

    async def close(self) -> None:
        self.stop.set()
        if self.task is not None:
            await asyncio.gather(self.task, return_exceptions=True)
        if self.listener is not None:
            self.listener.close()
        for peer in self.peers:
            peer.cancel()
        await asyncio.gather(*self.peers, return_exceptions=True)
        if self.owner is not None:
            # Orderly shutdown is outside control scheduling, with the same
            # bounded resource cleanup horizon as ProcessWorker.close().
            until = asyncio.get_running_loop().time() + 1.0
            while asyncio.get_running_loop().time() < until:
                if self.owner.prepare_close():
                    break
                await asyncio.sleep(0.01)
            self.owner.close()
