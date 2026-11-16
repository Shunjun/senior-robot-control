"""Dedicated bounded record IPC thread. Control reads snapshots without waiting."""

import logging
import queue
import socket
import threading
from pathlib import Path

from senior_control.motion.learning_record import (
    MAX_RECORD_BYTES,
    LearningGeometry,
    LearningRecord,
    Phase,
)
from senior_control.runtime.learning_service import address
from senior_control.runtime.learning_wire import (
    ActionReady,
    ActionReport,
    Attach,
    Complete,
    GeometryReport,
    PhaseReport,
    Reply,
    Request,
    Reserve,
    SessionRequest,
    State,
)
from senior_control.safety.learning_view import LearningFilterView

_LOG = logging.getLogger(__name__)


def request_once(run_id: str, request: Request) -> Reply:
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as channel:
        channel.settimeout(2)
        channel.connect(address(run_id))
        channel.sendall(request.model_dump_json().encode())
        raw = channel.recv(MAX_RECORD_BYTES + 1)
        if len(raw) > MAX_RECORD_BYTES:
            raise ValueError("learning_reply_truncated")
        return Reply.model_validate_json(raw)


class LearningClient:
    def __init__(self, path: Path) -> None:
        self.path, self.run_id = path, path.name.removeprefix(".startup-learning-")
        self._commands: queue.Queue[Request] = queue.Queue(maxsize=16)
        self._replies: queue.Queue[tuple[Request, Reply]] = queue.Queue(maxsize=16)
        self._stop = threading.Event()
        self._session: str | None = None
        self._record: LearningRecord | None = None
        self._revision: int | None = None
        self._action_ready = False
        self._attaching = True
        self.view: LearningFilterView | None = None
        self.error: str | None = None
        self._complete_wait = False
        self._reserving = False
        self._checking_action = False
        self._state_pending = False
        self._last_geometry: LearningGeometry | None = None
        self._pending_geometry: LearningGeometry | None = None
        self._last_phase: tuple[Phase, str, str | None] | None = None
        self._thread = threading.Thread(
            target=self._run, name="learning-record-client", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        pending: Request | None = Attach()
        while not self._stop.is_set():
            if pending is None:
                try:
                    pending = self._commands.get(timeout=0.05)
                except queue.Empty:
                    continue
            try:
                reply = request_once(self.run_id, pending)
            except (OSError, ValueError, RuntimeError) as exc:
                reply = Reply(ok=False, error=str(exc)[:256])
            while not self._stop.is_set():
                try:
                    self._replies.put((pending, reply), timeout=0.05)
                    break
                except queue.Full:
                    continue
            # A single request/reply is in flight; no command echo/retry can
            # replay an action after an ambiguous transport failure.
            pending = None

    def poll(self) -> None:
        for _ in range(16):
            try:
                request, reply = self._replies.get_nowait()
            except queue.Empty:
                break
            if isinstance(request, SessionRequest) and request.session != self._session:
                continue
            if isinstance(request, ActionReady):
                self._checking_action = False
            elif isinstance(request, State):
                self._state_pending = False
            if (
                not reply.ok
                and isinstance(request, Attach)
                and reply.error in {"learning_consumer_not_ready", "learning_model_not_durable"}
            ):
                self._enqueue(Attach())
                continue
            if not reply.ok:
                self.error = reply.error or "learning_record_unavailable"
                self._attaching = False
                _LOG.warning("Learning record client unavailable: %s", self.error)
                continue
            self.error = reply.error
            if reply.record is not None:
                if reply.record.run_id != self.run_id:
                    self.error = "learning_reply_run_mismatch"
                    continue
                self._record = reply.record
            if isinstance(request, Attach):
                self._session, self._attaching = reply.session, False
                record = self._record
                if record is not None and record.model is not None and record.geometry is not None:
                    self.view = LearningFilterView(record.model, record.geometry)
                elif self._pending_geometry is not None and self._session is not None:
                    self.geometry(self._pending_geometry)
            if reply.revision is not None:
                self._revision = reply.revision
            if isinstance(request, ActionReady) and reply.ready:
                self._action_ready = True
            if (
                self._revision is not None
                and not self._complete_wait
                and not self._action_ready
                and self._session is not None
                and not self._checking_action
            ):
                self._checking_action = self._enqueue(
                    ActionReady(session=self._session, revision=self._revision)
                )
            if (
                self._complete_wait
                and self._record is not None
                and self._record.model is not None
                and reply.durable_revision >= self._revision_or_max()
            ):
                self._complete_wait = False
                self.view, self._session = None, None
                self._attaching = True
                self._revision, self._action_ready, self._reserving = None, False, False
                self._enqueue(Attach())
            elif self._complete_wait and not self._state_pending:
                self._state_pending = self._enqueue(State())

    def _revision_or_max(self) -> int:
        return self._revision if self._revision is not None else (1 << 63)

    def _enqueue(self, request: Request) -> bool:
        if self._stop.is_set():
            return False
        try:
            self._commands.put_nowait(request)
            return True
        except queue.Full:
            self.error = "learning_record_client_busy"
            return False

    @property
    def record(self) -> LearningRecord | None:
        self.poll()
        return self._record

    @property
    def ready(self) -> bool:
        self.poll()
        return (
            (self._session is not None or self.view is not None)
            and not self._attaching
            and not self._complete_wait
            and self.error is None
        )

    def reserve_action(self) -> bool:
        self.poll()
        if self._action_ready:
            return True
        if self._session is not None and self._revision is None and not self._reserving:
            self._reserving = self._enqueue(Reserve(session=self._session))
        return False

    def geometry(self, geometry: LearningGeometry) -> None:
        self._pending_geometry = geometry
        if self.view is not None:
            return
        if self._session is not None and self._last_geometry != geometry:
            if self._enqueue(GeometryReport(session=self._session, geometry=geometry)):
                self._last_geometry = geometry

    def phase(self, phase: Phase, reason: str, segment_id: str | None = None) -> None:
        key = phase, reason, segment_id
        if self._session is not None and self._last_phase != key:
            if self._enqueue(
                PhaseReport(
                    session=self._session, phase=phase, reason=reason, segment_id=segment_id
                )
            ):
                self._last_phase = key

    def action(self, start_ns: int, interval_ns: int, *, finished: bool) -> None:
        if self._session is not None:
            self._enqueue(
                ActionReport(
                    session=self._session,
                    start_ns=start_ns,
                    interval_ns=interval_ns,
                    finished=finished,
                )
            )

    def complete(self, points: tuple[tuple[float, float], ...]) -> None:
        if self._session is not None and not self._complete_wait:
            self._revision = None
            self._complete_wait = self._enqueue(Complete(session=self._session, points=points))

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.1)
