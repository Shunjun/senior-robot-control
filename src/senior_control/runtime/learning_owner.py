"""Run-owned learning facts; bounded asynchronous persistence, never control I/O."""

import logging
import time
from collections.abc import Callable
from concurrent.futures import Future
from functools import partial
from pathlib import Path
from typing import Protocol
from uuid import uuid4

from senior_control.motion.learning_record import (
    ActiveReference,
    CaptureSummary,
    LearnedCatalog,
    LearningAction,
    LearningAnchor,
    LearningGeometry,
    LearningRecord,
    Phase,
    ProgramManifest,
    attempt_identity,
)
from senior_control.motion.process_resources import ProcessWorker
from senior_control.runtime.learning_store import (
    RecordWriterLock,
    attempt_path,
    read_manifest,
    read_record,
    write_record,
)

_LOG = logging.getLogger(__name__)


CommitResult = int | LearningRecord | None


class CommitWorker(Protocol):
    def submit(self, task: Callable[[], CommitResult]) -> Future[CommitResult]: ...
    def close(self) -> None: ...


def _worker() -> CommitWorker:
    return ProcessWorker[CommitResult](name="learning-record-io")


def _read_optional(path: Path) -> LearningRecord | None:
    try:
        return read_record(path)
    except FileNotFoundError:
        return None


class LearningRecordOwner:
    def __init__(
        self,
        path: Path,
        run_id: str,
        clock_domain: str,
        *,
        new_run: bool,
        worker_factory: Callable[[], CommitWorker] = _worker,
    ) -> None:
        self.path = path
        self.record = LearningRecord(
            run_id=run_id,
            attempt_id=attempt_identity(run_id),
            clock_domain=clock_domain,
            owner_epoch=uuid4().hex,
            motion_owner_id=uuid4().hex,
        )
        self.manifest = ProgramManifest(
            run_id=run_id,
            owner_epoch=self.record.owner_epoch,
            latest_attempt_id=self.record.attempt_id,
        )
        self.active_record: LearningRecord | None = None
        self._published_manifest = self.manifest
        self._manifest_durable = -1
        self.durable_revision = -1
        self._durable_model: str | None = None
        self.error: str | None = None
        self._lock: RecordWriterLock | None = None
        self._worker: CommitWorker | None = None
        self._future: Future[CommitResult] | None = None
        self._lookup_future: Future[CommitResult] | None = None
        self._lookup_target: str | None = None
        self._writing: LearningRecord | ProgramManifest | None = None
        self._session: str | None = None
        self._reservation: tuple[str, int] | None = None
        self._closed = False
        self._closing = False
        self._io_failed = False
        try:
            if path.name != ".startup-learning-" + run_id:
                raise ValueError("learning_record_path_identity")
            self._lock = RecordWriterLock(path)
            try:
                manifest = read_manifest(path)
                previous = read_record(attempt_path(path, manifest.latest_attempt_id))
                self.manifest = manifest.model_copy(
                    update={
                        "owner_epoch": self.record.owner_epoch,
                        "revision": manifest.revision + 1,
                    }
                )
                if manifest.active_ref is not None:
                    active = read_record(attempt_path(path, manifest.active_ref.attempt_id))
                    reference = manifest.active_ref
                    if (
                        active.run_id != run_id
                        or active.attempt_id != reference.attempt_id
                        or active.revision != reference.record_revision
                        or active.model is None
                        or active.model.model_id != reference.model_id
                    ):
                        raise ValueError("learning_active_identity")
                    self.active_record = active
            except FileNotFoundError:
                if not new_run:
                    raise ValueError("learning_record_missing") from None
            else:
                if new_run:
                    raise ValueError("learning_run_already_exists")
                if previous.run_id != run_id or previous.clock_domain != clock_domain:
                    raise ValueError("learning_record_identity")
                updates: dict[str, object] = {
                    "owner_epoch": self.record.owner_epoch,
                    "revision": previous.revision + 1,
                }
                self.record = LearningRecord.model_validate({**previous.model_dump(), **updates})
            self._activation_revision = self.record.revision
            self._worker = worker_factory()
            self.poll()
        except (OSError, ValueError, RuntimeError) as exc:
            self._activation_revision = self.record.revision
            self._fault(str(exc))

    def _fault(self, reason: str) -> None:
        message = reason[:256] or "learning_record_unavailable"
        if self.error != message:
            _LOG.warning("Learning record unavailable: %s", message)
        self.error = message

    def _revise(self, **updates: object) -> None:
        self.record = LearningRecord.model_validate(
            {
                **self.record.model_dump(),
                **updates,
                "revision": self.record.revision + 1,
                "committed_steady_ns": time.monotonic_ns(),
            }
        )

    @property
    def ready(self) -> bool:
        return (
            self._worker is not None
            and self.durable_revision >= self._activation_revision
            and self._manifest_durable >= 0
            and not self._closed
            and not self._closing
        )

    def poll(self) -> None:
        if self._closed:
            return
        if self._future is not None and self._future.done():
            try:
                revision = self._future.result()
                if (
                    type(revision) is not int
                    or self._writing is None
                    or revision != self._writing.revision
                ):
                    raise ValueError("learning_commit_identity")
                if isinstance(self._writing, ProgramManifest):
                    self._manifest_durable = revision
                    self._published_manifest = self._writing
                    if self._writing.active_ref is not None:
                        self.active_record = self.record
                else:
                    self.durable_revision = max(self.durable_revision, revision)
                    self._durable_model = (
                        None if self._writing.model is None else self._writing.model.model_id
                    )
            except (OSError, ValueError, RuntimeError) as exc:
                self._io_failed = True
                self._fault(str(exc))
            self._future, self._writing = None, None
        if (
            self._worker is not None
            and self._future is None
            and self._lookup_future is None
            and not self._io_failed
            and (
                self.record.revision > self.durable_revision
                or self.manifest.revision > self._manifest_durable
                or (self.record.model is not None and self.active_record != self.record)
            )
        ):
            try:
                if self.record.revision > self.durable_revision:
                    self._writing = self.record
                    destination = attempt_path(self.path, self.record.attempt_id)
                else:
                    if self.record.model is not None and self.active_record != self.record:
                        reference = ActiveReference(
                            attempt_id=self.record.attempt_id,
                            model_id=self.record.model.model_id,
                            record_revision=self.record.revision,
                        )
                        if self.manifest.active_ref != reference:
                            self.manifest = self.manifest.model_copy(
                                update={
                                    "revision": self.manifest.revision + 1,
                                    "active_revision": self.manifest.active_revision + 1,
                                    "active_ref": reference,
                                }
                            )
                    self._writing = self.manifest
                    destination = self.path
                self._future = self._worker.submit(
                    partial(write_record, destination, self._writing)
                )
            except (OSError, ValueError, RuntimeError) as exc:
                self._writing = None
                self._io_failed = True
                self._fault(str(exc))

    @property
    def public_record(self) -> LearningRecord:
        model = self.record.model
        if model is not None and (
            model.model_id != self._durable_model or self.active_record != self.record
        ):
            return LearningRecord.model_validate(
                {
                    **self.record.model_dump(),
                    "model": None,
                    "phase": "failed" if self._io_failed else "validating",
                    "reason": "learning_completion_not_durable"
                    if self._io_failed
                    else "learning_completion_pending_commit",
                }
            )
        return self.record

    def lookup(self, attempt_id: str) -> tuple[bool, LearningRecord | None]:
        if self.record.attempt_id == attempt_id:
            return True, self.record
        if self.active_record is not None and self.active_record.attempt_id == attempt_id:
            return True, self.active_record
        if self._lookup_future is not None:
            if self._lookup_target != attempt_id or not self._lookup_future.done():
                return False, None
            future, self._lookup_future = self._lookup_future, None
            self._lookup_target = None
            result = future.result()
            if result is not None and not isinstance(result, LearningRecord):
                raise ValueError("learning_lookup_identity")
            return True, result
        self.poll()
        if self._future is not None or self._worker is None:
            return False, None
        self._lookup_target = attempt_id
        self._lookup_future = self._worker.submit(
            partial(_read_optional, attempt_path(self.path, attempt_id))
        )
        return False, None

    @property
    def public_manifest(self) -> ProgramManifest:
        return self._published_manifest

    @property
    def busy(self) -> bool:
        return (
            self.durable_revision < self.record.revision
            or self._future is not None
            or self.record.phase not in {"completed", "failed", "unconfirmed"}
            or (self.record.model is not None and self.active_record != self.record)
        )

    def new_attempt(
        self,
        request_id: str,
        fingerprint: str,
        expected_revision: int,
        admitted: bool = True,
        *,
        output_boot_id: str | None = None,
    ) -> str:
        identity = attempt_identity(self.record.run_id, request_id)
        if self.record.attempt_id == identity:
            if (
                self.record.request_fingerprint != fingerprint
                or self.record.start_request_id != request_id
            ):
                raise ValueError("learning_request_conflict")
            return "accepted"
        if self.busy:
            return "busy"
        if not admitted or self.manifest.active_revision != expected_revision or self._io_failed:
            raise ValueError("learning_active_conflict")
        self.record = LearningRecord(
            run_id=self.record.run_id,
            attempt_id=identity,
            trigger="manual",
            motion_owner_id=uuid4().hex,
            motion_generation=self.manifest.revision + 1,
            expected_output_boot_id=output_boot_id,
            start_request_id=request_id,
            request_fingerprint=fingerprint,
            clock_domain=self.record.clock_domain,
            owner_epoch=self.record.owner_epoch,
        )
        self.manifest = self.manifest.model_copy(
            update={
                "latest_attempt_id": identity,
                "revision": self.manifest.revision + 1,
            }
        )
        self.durable_revision = -1
        self._activation_revision = 0
        self._durable_model = None
        self._session = None
        self._reservation = None
        self.poll()
        return "accepted"

    def capture(self, summary: CaptureSummary) -> None:
        if self.record.capture_summary.state == "complete" and summary.state != "complete":
            return
        if self.record.capture_summary != summary:
            self._revise(capture_summary=summary)
            self.poll()

    def begin_consumer(self) -> tuple[str | None, LearningRecord]:
        """Completed results are read-only; the existing owner retains single writing."""
        self.poll()
        if not self.ready:
            raise ValueError("learning_consumer_not_ready")
        if self.record.model is not None:
            if (
                self.record.model.model_id != self._durable_model
                or self.active_record != self.record
            ):
                raise ValueError("learning_model_not_durable")
            return None, self.record
        if self._session is None:
            self._session = uuid4().hex
        return self._session, self.record

    def _check(self, session: str) -> None:
        self.poll()
        if session != self._session or not self.ready or self.record.model is not None:
            raise ValueError("learning_consumer_identity")

    def reserve_action(self, session: str) -> int | None:
        self._check(session)
        if self.record.action.consumed or self._io_failed or self.record.phase == "completed":
            return None
        self._revise(action=LearningAction(consumed=True, stage="reserved_may_have_output"))
        self._reservation = session, self.record.revision
        self.poll()
        return self.record.revision

    def action_ready(self, session: str, revision: int) -> bool:
        self._check(session)
        return (
            self._reservation == (session, revision)
            and self.durable_revision >= revision
            and not self._io_failed
        )

    def report_action(
        self, session: str, start_ns: int, interval_ns: int, *, finished: bool
    ) -> None:
        self._check(session)
        if not self.record.action.consumed:
            raise ValueError("learning_action_not_consumed")
        old = self.record.action
        if old.first_submitted_steady_ns is not None and old.first_submitted_steady_ns != start_ns:
            raise ValueError("learning_action_start_changed")
        if interval_ns < old.reported_nonzero_interval_ns:
            raise ValueError("learning_action_duration_regressed")
        action = LearningAction(
            consumed=True,
            stage="window_finished" if finished else "running",
            first_submitted_steady_ns=start_ns,
            deadline_steady_ns=start_ns + 3_000_000_000,
            reported_nonzero_interval_ns=interval_ns,
        )
        self._revise(action=action)
        self.poll()

    def set_geometry(self, session: str, geometry: LearningGeometry) -> None:
        self._check(session)
        if self.record.geometry == geometry:
            return
        self._revise(geometry=geometry)
        self.poll()

    def set_phase(
        self, session: str, phase: Phase, reason: str, segment_id: str | None = None
    ) -> None:
        if self.record.phase in {"completed", "failed", "unconfirmed"}:
            return
        self._check(session)
        if phase == "completed":
            raise ValueError("learning_completion_requires_model")
        if (self.record.phase, self.record.reason, self.record.segment_id) != (
            phase,
            reason,
            segment_id,
        ):
            self._revise(phase=phase, reason=reason, segment_id=segment_id)
            self.poll()

    def fail_capture(self, reason: str, summary: CaptureSummary) -> None:
        """The recording owner can close an orphan without minting a motion session."""
        if self.record.phase in {"completed", "failed", "unconfirmed"}:
            return
        if self.record.capture_summary.state == "complete":
            summary = self.record.capture_summary
        self._revise(phase="failed", reason=reason, capture_summary=summary)
        self.poll()

    def complete_model(self, session: str, points: tuple[tuple[float, float], ...]) -> int:
        self._check(session)
        if (
            self.record.geometry is None
            or self.record.model is not None
            or self._io_failed
            or self.record.capture_summary.state != "complete"
        ):
            raise ValueError("learning_model_not_admissible")
        unique = tuple(dict.fromkeys(points))
        model = LearnedCatalog(
            model_id=uuid4().hex,
            anchors=tuple(LearningAnchor(id=i, point=p) for i, p in enumerate(unique)),
        )
        self._revise(model=model, phase="completed", reason="classified_body_returns")
        self.poll()
        return self.record.revision

    def prepare_close(self) -> bool:
        self.poll()
        if self._io_failed:
            return False
        self._closing = True
        return (
            self.durable_revision >= self.record.revision
            and self._manifest_durable >= self.manifest.revision
        )

    def close(self) -> None:
        if self._closed:
            return
        self.poll()
        if self._worker is not None:
            self._worker.close()
        if self._lock is not None:
            self._lock.close()
        self._closed = True
