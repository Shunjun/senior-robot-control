"""Run-supervisor component. Filesystem commits use the dedicated ProcessWorker."""

import asyncio
import contextlib
import logging
import os
import socket
import struct
from collections import deque
from collections.abc import Iterable
from pathlib import Path

from senior_perception.learning_compute import ComputeRequest, LearningCompute
from senior_perception.learning_owner import LearningRecordOwner
from senior_perception.learning_record import MAX_RECORD_BYTES, CaptureSummary, attempt_identity
from senior_perception.learning_wire import (
    REQUEST,
    AcceptAttempt,
    ActionReady,
    ActionReport,
    Attach,
    BeginAttempt,
    DeclineAttempt,
    FrameBatch,
    GeometryReport,
    MotionObservedThrough,
    PhaseReport,
    QueryAttempt,
    RecordedFrame,
    Reply,
    Reserve,
    RetireMotionOwner,
    SessionRequest,
    State,
)
from senior_perception.self_learning import LEARNING_SAMPLE_COUNT
from vehicle_core.navigation_clock import monotonic_domain

_LOG = logging.getLogger("senior_perception.learning_service")  # //Compatible: log filters.


def address(run_id: str) -> str:
    # Abstract Linux socket avoids root-path length limits and stale socket files.
    return "\0senior-learning-" + run_id


def select_motion_frames(
    frames: Iterable[RecordedFrame], start_ns: int, end_ns: int
) -> tuple[RecordedFrame, ...]:
    """Select distinct temporal quartiles, preserving original acquisition order."""
    motion = tuple(
        frame for frame in frames if start_ns <= frame.capture.time.sample_steady_ns < end_ns
    )
    if len(motion) < LEARNING_SAMPLE_COUNT:
        raise ValueError("learning_samples_insufficient")
    first = motion[0].capture.time.sample_steady_ns
    span = motion[-1].capture.time.sample_steady_ns - first
    selected, previous = [motion[0]], 0
    for slot in range(1, LEARNING_SAMPLE_COUNT - 1):
        # Integer comparisons keep nanosecond sample identities exact. Leave
        # enough later acquisitions for the remaining distinct sample slots.
        stop = len(motion) - (LEARNING_SAMPLE_COUNT - slot - 1)
        index = min(
            range(previous + 1, stop),
            key=lambda i: abs(
                (motion[i].capture.time.sample_steady_ns - first) * (LEARNING_SAMPLE_COUNT - 1)
                - span * slot
            ),
        )
        selected.append(motion[index])
        previous = index
    return (*selected, motion[-1])


class LearningService:
    def __init__(self, path: Path):
        self.path = path
        self.owner: LearningRecordOwner | None = None
        self.listener: socket.socket | None = None
        self.stop = asyncio.Event()
        self.task: asyncio.Task[None] | None = None
        self.peers: dict[asyncio.Task[None], int] = {}
        self._peer_sequence = 0
        self.compute: LearningCompute | None = None
        self.proposal: BeginAttempt | None = None
        self._reset_capture()

    def _reset_capture(self) -> None:
        self.frames: deque[RecordedFrame] = deque()
        self.ingress_through = 0
        self.motion_through = 0
        self._stationary = False
        self.first_takeover = None
        self.validation_after = None
        self.accepted_frames = 0
        self._frame_origin = None
        self._last_measurement = None
        self._compute_submitted = False
        self._pending_failure = None

    def _accept_frames(self, batch: FrameBatch) -> None:
        assert self.owner is not None
        owner, record = self.owner, self.owner.record
        if batch.attempt_id != record.attempt_id:
            return
        if record.capture_summary.state != "open" or record.phase in {
            "completed",
            "failed",
            "unconfirmed",
        }:
            return
        for frame in batch.frames:
            if frame.recorded_steady_ns < record.created_steady_ns:
                continue
            capture = frame.capture
            origin = (
                capture.bridge_boot_id,
                capture.source_epoch,
                capture.raw_gid,
                capture.capture_generation,
                capture.time.clock_generation,
            )
            if self._frame_origin is not None and origin != self._frame_origin:
                self._fail_capture("learning_source_changed")
                return
            self._frame_origin = origin
            measurement = capture.stamp_ns, capture.time.sample_steady_ns, capture.sequence
            if self._last_measurement is not None:
                if measurement[0] == self._last_measurement[0]:
                    continue  # A duplicate acquisition cannot supply another sample.
                if any(b < a for a, b in zip(self._last_measurement, measurement, strict=True)):
                    self._fail_capture("learning_capture_clock_regressed")
                    return
            self._last_measurement = measurement
            if record.geometry is None:
                owner._revise(geometry=frame.geometry)
                record = owner.record
            elif not record.geometry.same_structure(frame.geometry):
                self._fail_capture("learning_geometry_changed")
                return
            if len(self.frames) >= 128:
                # This is a sampling buffer, not a lossless frame log. Compact
                # dense data in time order, preserving both temporal endpoints.
                rows = tuple(self.frames)
                self.frames = deque(rows[i * (len(rows) - 1) // 63] for i in range(64))
            self.frames.append(frame)
            self.accepted_frames += 1
            if not record.action.consumed:
                self.frames = deque((frame,))
        self.ingress_through = max(self.ingress_through, batch.through_steady_ns)
        self._advance_capture()

    def _advance_capture(self) -> None:
        assert self.owner is not None
        owner, record = self.owner, self.owner.record
        if record.capture_summary.state != "open" or record.phase in {
            "completed",
            "failed",
            "unconfirmed",
        }:
            return
        start, end = record.action.first_submitted_steady_ns, record.action.deadline_steady_ns
        stopped = self.validation_after
        if (
            start is not None
            and end is not None
            and stopped is not None
            and stopped >= end
            and self._stationary
            and self.motion_through >= stopped
            and self.ingress_through >= stopped
            and (self.first_takeover is None or stopped < self.first_takeover)
        ):
            # No extra post-stop frames: only actual motion-window samples are
            # used. Discard arrivals recorded after takeover, even if acquired earlier.
            self.frames = deque(
                frame
                for frame in self.frames
                if start <= frame.capture.time.sample_steady_ns < end
                and (self.first_takeover is None or frame.recorded_steady_ns < self.first_takeover)
            )
            if len(self.frames) < LEARNING_SAMPLE_COUNT:
                self._fail_capture("learning_samples_insufficient")
                return
            owner.capture(
                CaptureSummary(
                    state="complete",
                    accepted_frames=self.accepted_frames,
                    validation_after_steady_ns=stopped,
                    closed_steady_ns=stopped,
                )
            )
            return
        if self.first_takeover is not None and self.ingress_through >= self.first_takeover:
            self._fail_capture(self._pending_failure or "control_takeover")

    def _fail_capture(self, reason: str) -> None:
        assert self.owner is not None
        self.owner.fail_capture(
            reason,
            CaptureSummary(
                state="incomplete",
                accepted_frames=self.accepted_frames,
            ),
        )
        self.frames.clear()

    def _pump_compute(self) -> None:
        assert self.owner is not None
        if self.compute is None:
            return
        owner = self.owner
        result = self.compute.poll()
        if result is not None and result.attempt_id == owner.record.attempt_id:
            if result.error is not None:
                self._fail_capture(result.error)
            else:
                summary = owner.record.capture_summary.model_copy(update={"compute_complete": True})
                owner.capture(summary)
                if result.points is None:
                    if owner._session is not None:
                        owner.set_phase(owner._session, "unconfirmed", "classification_unverified")
                else:
                    if owner._session is not None:
                        owner.complete_model(owner._session, result.points)
        record = owner.record
        if self.compute.flight is not None or self._compute_submitted:
            return
        if record.capture_summary.state == "incomplete" or record.phase in {
            "failed",
            "unconfirmed",
            "completed",
        }:
            return
        start, end = record.action.first_submitted_steady_ns, record.action.deadline_steady_ns
        if start is None or end is None or record.geometry is None or record.segment_id is None:
            return
        if record.capture_summary.state != "complete":
            return  # Motion and capture proceed independently of CPU computation.
        try:
            samples = select_motion_frames(self.frames, start, end)
        except ValueError as exc:
            self._fail_capture(str(exc))
            return
        self.compute.submit(ComputeRequest(record.attempt_id, record.geometry, samples))
        self.frames.clear()
        self._compute_submitted = True

    async def _lookup(self, attempt_id):
        assert self.owner is not None
        until = asyncio.get_running_loop().time() + 1.5
        while asyncio.get_running_loop().time() < until:
            done, record = self.owner.lookup(attempt_id)
            if done:
                return record
            await asyncio.sleep(0.005)
        raise ValueError("learning_lookup_pending")

    def _status(self, selected=None):
        assert self.owner is not None
        owner = self.owner
        record = owner.record if selected is None else selected
        active = owner.active_record
        public = owner.public_record if record.attempt_id == owner.record.attempt_id else record
        manifest = owner.public_manifest
        return dict(
            run_id=record.run_id,
            active_revision=manifest.active_revision,
            active_model=None
            if active is None or active.model is None
            else dict(
                attempt_id=active.attempt_id,
                model_id=active.model.model_id,
                point_count=len(active.model.anchors),
            ),
            attempt_id=record.attempt_id,
            phase=public.phase,
            reason=public.reason,
            motion_complete=record.action.stage == "window_finished",
            capture_complete=record.capture_summary.state == "complete",
            compute_complete=record.capture_summary.compute_complete,
            durable=record.model is not None
            and (active == record or record.attempt_id != owner.record.attempt_id),
            can_start=not owner.busy,
            cannot_start_reason="learning_busy" if owner.busy else None,
        )

    async def start(self) -> None:
        run_id = self.path.name.removeprefix(".startup-learning-")
        try:
            listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            self.listener = listener
            listener.setblocking(False)
            listener.bind(address(run_id))
            listener.listen(8)
            self.owner = LearningRecordOwner(self.path, run_id, monotonic_domain(), new_run=True)
            self.compute = LearningCompute()
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
                    self._advance_capture()
                    self._pump_compute()
                    if accepting.done():
                        connection, _ = accepting.result()
                        if len(self.peers) >= 8:
                            connection.close()
                        else:
                            self._peer_sequence += 1
                            peer = asyncio.create_task(self._serve(connection, self._peer_sequence))
                            self.peers[peer] = self._peer_sequence
                            peer.add_done_callback(lambda done: self.peers.pop(done, None))
                        accepting = asyncio.create_task(loop.sock_accept(self.listener))
                except (OSError, ValueError, RuntimeError) as exc:
                    _LOG.warning("Learning owner component error: %s", exc)
                with contextlib.suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(self.stop.wait(), 0.01)
        finally:
            accepting.cancel()
            await asyncio.gather(accepting, return_exceptions=True)

    async def handle(self, request, *, earlier=()) -> Reply:
        assert self.owner is not None
        owner = self.owner
        owner.poll()
        data = None
        revision = None
        ready = False
        session = None
        if isinstance(request, Attach):
            if owner.record.trigger == "manual" and owner.record.model is None:
                proposal = self.proposal
                if (
                    proposal is None
                    or request.attempt_id != owner.record.attempt_id
                    or request.output_boot_id != proposal.output_boot_id
                    or request.generation != proposal.generation
                    or owner.record.action.consumed
                ):
                    raise ValueError("learning_attempt_identity")
            session, _ = owner.begin_consumer()
        elif isinstance(request, State):
            pass
        elif isinstance(request, Reserve):
            if owner.record.trigger == "manual" and self.proposal is not None:
                raise ValueError("learning_attempt_unconfirmed")
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
            if owner.record.capture_summary.state == "complete":
                pass
            elif request.phase == "failed":
                self._pending_failure = request.reason
                self._advance_capture()
            else:
                owner.set_phase(request.session, request.phase, request.reason, request.segment_id)
        elif isinstance(request, FrameBatch):
            self._accept_frames(request)
        elif isinstance(request, MotionObservedThrough):
            owner._check(request.session)
            if request.attempt_id != owner.record.attempt_id:
                raise ValueError("learning_attempt_identity")
            if request.through_steady_ns >= self.motion_through:
                self.motion_through = request.through_steady_ns
                self._stationary = request.stationary
            if request.gap:
                self._fail_capture("learning_motion_facts_incomplete")
                return Reply(ok=True, recording=False)
            if request.validation_after_steady_ns is not None:
                self.validation_after = request.validation_after_steady_ns
            if request.first_takeover_ns is not None:
                if self._pending_failure is None:
                    self._pending_failure = request.reason
                self.first_takeover = (
                    request.first_takeover_ns
                    if self.first_takeover is None
                    else min(self.first_takeover, request.first_takeover_ns)
                )
            self._advance_capture()
        elif isinstance(request, BeginAttempt):
            identity = attempt_identity(owner.record.run_id, request.start_request_id)
            existing = await self._lookup(identity)
            if existing is not None:
                if (
                    existing.request_fingerprint != request.fingerprint
                    or existing.start_request_id != request.start_request_id
                ):
                    raise ValueError("learning_request_conflict")
                state = "accepted"
                attempt = existing.attempt_id
            else:
                if request.output_boot_id is None:
                    raise ValueError("learning_active_conflict")
                previous_attempt = owner.record.attempt_id
                state = owner.new_attempt(
                    request.start_request_id,
                    request.fingerprint,
                    request.payload.expected_active_revision,
                    request.admitted,
                    output_boot_id=request.output_boot_id,
                )
                attempt = owner.record.attempt_id
                if state == "accepted" and previous_attempt != attempt:
                    self._reset_capture()
                    self.proposal = request
            data = dict(start_request_id=request.start_request_id, attempt_id=attempt, state=state)
        elif isinstance(request, AcceptAttempt):
            owner._check(request.session)
            if request.attempt_id != owner.record.attempt_id:
                raise ValueError("learning_attempt_identity")
            self.proposal = None
            if not request.accepted:
                self._fail_capture(request.reason)
        elif isinstance(request, RetireMotionOwner):
            # Each sender has one flight. Settle earlier accepted connections, not
            # CPU work or unseen history; both raw and closing facts may be in flight.
            if earlier:
                await asyncio.gather(*earlier, return_exceptions=True)
            record = owner.record
            if (
                request.attempt_id == record.attempt_id
                and request.owner_epoch == record.owner_epoch
                and request.motion_owner_id == record.motion_owner_id
                and request.motion_generation == record.motion_generation
                and record.expected_output_boot_id != request.replacement_output_boot_id
            ):
                self._advance_capture()
                if owner.record.capture_summary.state == "open":
                    self._fail_capture("motion_owner_replaced")
        elif isinstance(request, DeclineAttempt):
            record = owner.record
            if (
                request.attempt_id == record.attempt_id
                and request.output_boot_id == record.expected_output_boot_id
                and not record.action.consumed
            ):
                self._fail_capture(request.reason)
                self.proposal = None
        elif isinstance(request, QueryAttempt):
            identity = request.payload.attempt_id
            if request.payload.start_request_id is not None:
                identity = attempt_identity(owner.record.run_id, request.payload.start_request_id)
            selected = owner.record if identity is None else await self._lookup(identity)
            if selected is None:
                raise ValueError("learning_request_unknown")
            data = self._status(selected)
        record = (
            owner.public_record
            if not isinstance(request, FrameBatch)
            and (
                getattr(request, "record_revision", -1) != owner.record.revision
                or getattr(request, "record_attempt_id", None) != owner.record.attempt_id
            )
            else None
        )
        active = (
            owner.active_record
            if isinstance(request, (State, Attach, SessionRequest))
            and request.active_revision != owner.public_manifest.active_revision
            else None
        )
        # The current completed attempt IS the active record: transmit its catalog once.
        if record == active and active is not None:
            record = None
        return Reply(
            ok=True,
            recording=owner.record.capture_summary.state == "open"
            and owner.record.phase not in {"completed", "failed", "unconfirmed"},
            error=owner.error,
            record=record,
            manifest=owner.public_manifest,
            attempt_id=owner.record.attempt_id,
            active_record=active,
            proposal=self.proposal,
            data=data,
            accepted_frames=self.accepted_frames,
            session=session,
            revision=revision,
            ready=ready,
            durable_revision=owner.durable_revision,
        )

    async def _serve(self, connection: socket.socket, sequence: int) -> None:
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
            request = REQUEST.validate_json(raw)
            earlier = (
                tuple(task for task, order in self.peers.items() if order < sequence)
                if isinstance(request, RetireMotionOwner)
                else ()
            )
            reply = await self.handle(request, earlier=earlier)
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
        if self.compute is not None:
            self.compute.close()
        if self.owner is not None:
            # Orderly shutdown is outside control scheduling, with the same
            # bounded resource cleanup horizon as ProcessWorker.close().
            until = asyncio.get_running_loop().time() + 1.0
            while asyncio.get_running_loop().time() < until:
                if self.owner.prepare_close():
                    break
                await asyncio.sleep(0.01)
            self.owner.close()
