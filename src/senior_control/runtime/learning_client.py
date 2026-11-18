"""Bounded control RPC and independent raw FIFO; neither performs learning CPU work."""

import math
import queue
import socket
import threading
import time
from collections import deque
from pathlib import Path
from uuid import uuid4

from senior_control.motion.learning_record import (
    MAX_RECORD_BYTES,
    LearningGeometry,
    LearningRecord,
    Phase,
    attempt_identity,
)
from senior_control.runtime.learning_service import address
from senior_control.runtime.learning_wire import (
    AcceptAttempt,
    ActionReady,
    ActionReport,
    Attach,
    FrameBatch,
    GeometryReport,
    MotionObservedThrough,
    PhaseReport,
    RecordedFrame,
    Reply,
    Request,
    Reserve,
    SessionRequest,
    State,
)
from senior_control.safety.learning_view import LearningFilterView
from senior_control.vision.raw_capture import capture_from_message


def request_once(run_id: str, request: Request) -> Reply:
    payload = request.model_dump_json().encode()
    if len(payload) > MAX_RECORD_BYTES:
        raise ValueError("learning_request_size")
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as channel:
        channel.settimeout(2)
        channel.connect(address(run_id))
        channel.sendall(payload)
        raw = channel.recv(MAX_RECORD_BYTES + 1)
        if len(raw) > MAX_RECORD_BYTES:
            raise ValueError("learning_reply_truncated")
        return Reply.model_validate_json(raw)


class LearningClient:
    def __init__(self, path: Path, *, read_only: bool = False) -> None:
        self.path, self.run_id = path, path.name.removeprefix(".startup-learning-")
        self.read_only = read_only
        self._commands: queue.Queue[Request] = queue.Queue(maxsize=16)
        self._replies: queue.Queue[tuple[Request, Reply]] = queue.Queue(maxsize=16)
        self._stop = threading.Event()
        self._session: str | None = None
        self._record: LearningRecord | None = None
        self._revision: int | None = None
        self._action_ready = False
        self._reserving = False
        self._checking_action = False
        self._attaching = not read_only
        self.view: LearningFilterView | None = None
        self.active_revision = 0
        self.proposal = None
        self.accepted_frames = 0
        self.error: str | None = None
        self._last_phase = None
        self._last_geometry = None
        self._observed: MotionObservedThrough | None = None
        self._observed_lock = threading.Lock()
        self._takeover_sent = False
        self._thread = threading.Thread(target=self._run, name="learning-control", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        pending: Request | None = State() if self.read_only else Attach()
        while not self._stop.is_set():
            if pending is None:
                try:
                    pending = self._commands.get(timeout=0.05)
                except queue.Empty:
                    with self._observed_lock:
                        pending, self._observed = self._observed, None
                    if pending is None:
                        pending = State()
            if isinstance(pending, State):
                pending = State(
                    record_revision=-1 if self._record is None else self._record.revision,
                    active_revision=self.active_revision,
                )
            elif isinstance(pending, Attach):
                pending = Attach(active_revision=self.active_revision)
            elif isinstance(pending, SessionRequest):
                pending = pending.model_copy(
                    update={
                        "record_revision": -1 if self._record is None else self._record.revision,
                        "active_revision": self.active_revision,
                    }
                )
            try:
                reply = request_once(self.run_id, pending)
            except (OSError, ValueError, RuntimeError) as exc:
                reply = Reply(ok=False, error=str(exc)[:256])
            while not self._stop.is_set():
                try:
                    self._replies.put((pending, reply), timeout=0.05)
                    break
                except queue.Full:
                    pass
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
            if not reply.ok:
                if isinstance(request, Attach) and reply.error == "learning_consumer_not_ready":
                    self._enqueue(Attach())
                    continue
                self.error = reply.error or "learning_record_unavailable"
                continue
            self.error = reply.error
            if reply.record is not None:
                record = reply.record
                if record.run_id != self.run_id:
                    self.error = "learning_reply_run_mismatch"
                    continue
                changed = self._record is not None and self._record.attempt_id != record.attempt_id
                self._record = record
                if changed and not self.read_only:
                    self._takeover_sent = False
                    self._session = None
                    self._revision = None
                    self._action_ready = self._reserving = self._checking_action = False
                    self._last_geometry = self._last_phase = None
                    self._attaching = True
                    self._enqueue(Attach())
            self.accepted_frames = reply.accepted_frames
            self.proposal = reply.proposal
            if reply.manifest is not None and reply.manifest.active_revision > self.active_revision:
                active = reply.active_record
                if active is not None and active.model is not None and active.geometry is not None:
                    self.view = LearningFilterView(active.model, active.geometry)
                    self.active_revision = reply.manifest.active_revision
            if self.read_only:
                continue
            if isinstance(request, Attach):
                self._session, self._attaching = reply.session, False
            if isinstance(request, Reserve):
                self._revision = reply.revision
            if isinstance(request, ActionReady) and reply.ready:
                self._action_ready = True
            if (
                self._revision is not None
                and not self._action_ready
                and not self._checking_action
                and self._session
            ):
                self._checking_action = self._enqueue(
                    ActionReady(session=self._session, revision=self._revision)
                )

    def _enqueue(self, request: Request) -> bool:
        if self._stop.is_set():
            return False
        try:
            self._commands.put_nowait(request)
            return True
        except queue.Full:
            self.error = "learning_control_queue_full"
            return False

    @property
    def record(self) -> LearningRecord | None:
        self.poll()
        return self._record

    @property
    def ready(self) -> bool:
        self.poll()
        return self._session is not None and not self._attaching and self.error is None

    def reserve_action(self) -> bool:
        self.poll()
        if self.read_only:
            return False
        if self._action_ready:
            return True
        if self._session is not None and not self._reserving:
            self._reserving = self._enqueue(Reserve(session=self._session))
        return False

    def accept_attempt(self, accepted: bool, reason: str) -> None:
        if self._session and self._record:
            self._enqueue(
                AcceptAttempt(
                    session=self._session,
                    attempt_id=self._record.attempt_id,
                    accepted=accepted,
                    reason=reason,
                )
            )

    def geometry(self, geometry: LearningGeometry) -> None:
        if not self.read_only and self._session and self._last_geometry != geometry:
            if self._enqueue(GeometryReport(session=self._session, geometry=geometry)):
                self._last_geometry = geometry

    def phase(self, phase: Phase, reason: str, segment_id: str | None = None) -> None:
        key = phase, reason, segment_id
        if not self.read_only and self._session and self._last_phase != key:
            if self._enqueue(
                PhaseReport(
                    session=self._session, phase=phase, reason=reason, segment_id=segment_id
                )
            ):
                self._last_phase = key

    def action(self, start_ns: int, interval_ns: int, *, finished: bool) -> None:
        if not self.read_only and self._session:
            self._enqueue(
                ActionReport(
                    session=self._session,
                    start_ns=start_ns,
                    interval_ns=interval_ns,
                    finished=finished,
                )
            )

    def observed(
        self,
        *,
        segment_id: str,
        now_ns: int,
        validation_after_ns: int | None,
        first_takeover_ns: int | None,
    ) -> None:
        if self.read_only or self._session is None or self._record is None:
            return
        # Latest observation contains the immutable FIRST takeover fact, so coalescing
        # observations cannot erase a cancellation or reorder control commands.
        value = MotionObservedThrough(
            session=self._session,
            attempt_id=self._record.attempt_id,
            segment_id=segment_id,
            through_steady_ns=now_ns,
            validation_after_steady_ns=validation_after_ns,
            first_takeover_ns=first_takeover_ns,
        )
        if first_takeover_ns is not None and not self._takeover_sent:
            self._takeover_sent = self._enqueue(value)
        else:
            with self._observed_lock:
                self._observed = value

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.1)


class LearningFrames:
    """Reception-side finite original-frame recorder with one ordered sender."""

    def __init__(self, path: Path, profile, body_frame: str) -> None:
        self.run_id = path.name.removeprefix(".startup-learning-")
        self.profile, self.body_frame = profile, body_frame
        self._attempt = attempt_identity(self.run_id)
        self._stream = uuid4().hex
        self._index = 0
        self._frames: deque[RecordedFrame] = deque()
        self._gap = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="learning-raw-frames", daemon=True)
        self._thread.start()

    def mark_gap(self) -> None:
        with self._lock:
            self._gap = True

    def offer(self, raw) -> None:
        m = raw.message
        capture = capture_from_message(m.raw_capture)
        x, y = self.profile.length / 2, self.profile.width / 2
        geometry = LearningGeometry(
            body_frame=self.body_frame,
            sensor_frame=m.frame_id,
            physical_polygon=((-x, -y), (x, -y), (x, y), (-x, y)),
            sensor_to_body=(m.sensor_x, m.sensor_y, m.sensor_yaw),
            angle_min=m.angle_min,
            angle_increment=m.angle_increment,
            count=len(m.ranges),
            range_min=m.range_min,
            range_max=m.range_max,
        )
        ranges = tuple(
            v if math.isfinite(v) else "nan" if math.isnan(v) else "inf" if v > 0 else "-inf"
            for v in m.ranges
        )
        with self._lock:
            self._index += 1
            if len(self._frames) >= 16:
                self._gap = True
                return
            self._frames.append(
                RecordedFrame(
                    capture=capture,
                    recorded_steady_ns=time.monotonic_ns(),
                    index=self._index,
                    geometry=geometry,
                    ranges=ranges,
                    transform_stamp=m.transform_stamp,
                )
            )

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                frames = tuple(self._frames.popleft() for _ in range(min(4, len(self._frames))))
                through = (
                    self._frames[0].recorded_steady_ns - 1 if self._frames else time.monotonic_ns()
                )
                request = FrameBatch(
                    attempt_id=self._attempt,
                    stream_id=self._stream,
                    frames=frames,
                    gap=self._gap,
                    through_steady_ns=through,
                )
            try:
                reply = request_once(self.run_id, request)
                if not reply.ok:
                    raise ValueError(reply.error)
                if reply.attempt_id is not None and reply.attempt_id != self._attempt:
                    with self._lock:
                        self._attempt = reply.attempt_id
                        self._gap = False
            except (OSError, ValueError, RuntimeError):
                if frames:
                    self.mark_gap()
            self._stop.wait(0.01 if frames else 0.05)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.1)
