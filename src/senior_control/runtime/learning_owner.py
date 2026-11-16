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
    LearnedCatalog,
    LearningAction,
    LearningAnchor,
    LearningGeometry,
    LearningRecord,
    Phase,
)
from senior_control.motion.process_resources import ProcessWorker
from senior_control.runtime.learning_store import RecordWriterLock, read_record, write_record

_LOG = logging.getLogger(__name__)


class CommitWorker(Protocol):
    def submit(self, task: Callable[[], int]) -> Future[int]: ...
    def close(self) -> None: ...


def _worker() -> CommitWorker:
    return ProcessWorker[int](name="learning-record-io")


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
            run_id=run_id, clock_domain=clock_domain, owner_epoch=uuid4().hex
        )
        self.durable_revision = -1
        self._durable_model: str | None = None
        self.error: str | None = None
        self._lock: RecordWriterLock | None = None
        self._worker: CommitWorker | None = None
        self._future: Future[int] | None = None
        self._writing: LearningRecord | None = None
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
                previous = read_record(path)
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
            and not self._closed
            and not self._closing
        )

    def poll(self) -> None:
        if self._closed:
            return
        if self._future is not None and self._future.done():
            try:
                revision = self._future.result()
                if self._writing is None or revision != self._writing.revision:
                    raise ValueError("learning_commit_identity")
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
            and not self._io_failed
            and self.record.revision > self.durable_revision
        ):
            try:
                self._writing = self.record
                self._future = self._worker.submit(partial(write_record, self.path, self.record))
            except (OSError, ValueError, RuntimeError) as exc:
                self._writing = None
                self._io_failed = True
                self._fault(str(exc))

    @property
    def public_record(self) -> LearningRecord:
        model = self.record.model
        if model is not None and model.model_id != self._durable_model:
            return LearningRecord.model_validate(
                {
                    **self.record.model_dump(),
                    "model": None,
                    "phase": "validating",
                    "reason": "learning_completion_pending_commit",
                }
            )
        return self.record

    def begin_consumer(self) -> tuple[str | None, LearningRecord]:
        """Completed results are read-only; the existing owner retains single writing."""
        self.poll()
        if not self.ready:
            raise ValueError("learning_consumer_not_ready")
        if self.record.model is not None:
            if self.record.model.model_id != self._durable_model:
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
        if self.record.phase == "completed":
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

    def complete_model(self, session: str, points: tuple[tuple[float, float], ...]) -> int:
        self._check(session)
        if self.record.geometry is None or self.record.model is not None or self._io_failed:
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
        return self.durable_revision >= self.record.revision

    def close(self) -> None:
        if self._closed:
            return
        self.poll()
        if self._worker is not None:
            self._worker.close()
        if self._lock is not None:
            self._lock.close()
        self._closed = True
