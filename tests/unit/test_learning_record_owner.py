"""Real schema/atomic storage/owner with explicitly held test commit futures."""

from concurrent.futures import Future

import pytest
from learning_test_support import DOMAIN, ImmediateWorker, TestRun, geometry
from test_startup_learning_feasibility import BOUNDARY

from senior_control.motion.learning_record import (
    MAX_RECORD_BYTES,
    CaptureSummary,
    LearningAction,
    LearningRecord,
    parse_record,
)
from senior_control.runtime.learning_owner import LearningRecordOwner
from senior_control.runtime.learning_store import read_record, write_record


class HeldWorker:
    def __init__(self):
        self.future = None
        self.task = None
        self.calls = 0

    def submit(self, task):
        assert self.future is None or self.future.done()
        self.future = Future()
        self.task = task
        self.calls += 1
        return self.future

    def finish(self):
        future, task = self.future, self.task
        future.set_result(task())

    def close(self):
        pass


def finish_writes(owner, worker):
    for _ in range(8):
        if owner._future is None:
            break
        worker.finish()
        owner.poll()


def test_strict_record_roundtrip_and_corrupt_or_unknown_inputs():
    record = LearningRecord(
        attempt_id="0" * 32, run_id="a" * 32, clock_domain=DOMAIN, owner_epoch="b" * 32
    )
    assert parse_record(record.to_bytes()) == record
    for data in (
        b'{"schema_version":1,"schema_version":1}',
        b'{"value":NaN}',
        b'{"schema_version":2}',
        b"{",
        b"x" * (MAX_RECORD_BYTES + 1),
    ):
        with pytest.raises(ValueError):
            parse_record(data)
    with pytest.raises(ValueError):
        LearningAction(
            consumed=True,
            stage="running",
            first_submitted_steady_ns=1,
            deadline_steady_ns=30_000_000_001,
        )


def test_corrupt_record_and_duplicate_writer_are_nonfatal_objects(tmp_path):
    run = TestRun(tmp_path / "duplicate")
    try:
        duplicate = LearningRecordOwner(
            run.path,
            run.owner.record.run_id,
            DOMAIN,
            new_run=False,
            worker_factory=ImmediateWorker,
        )
        assert duplicate.error and not duplicate.ready
        duplicate.close()
    finally:
        run.close()
    broken = tmp_path / (".startup-learning-" + "d" * 32)
    broken.write_text("attempt_claimed_before_motion\n")
    owner = LearningRecordOwner(
        broken, "d" * 32, DOMAIN, new_run=False, worker_factory=ImmediateWorker
    )
    assert owner.error and not owner.ready
    owner.close()


def test_atomic_write_failure_keeps_previous_complete_snapshot(tmp_path, monkeypatch):
    from senior_control.runtime import learning_store

    path = tmp_path / (".startup-learning-" + "e" * 32)
    original = LearningRecord(
        attempt_id="0" * 32, run_id="e" * 32, clock_domain=DOMAIN, owner_epoch="f" * 32
    )
    write_record(path, original)
    newer = LearningRecord.model_validate({**original.model_dump(), "revision": 1})
    actual = learning_store.os.replace

    def fail_replace(*args, **kwargs):
        raise OSError("injected replacement failure")

    monkeypatch.setattr(learning_store.os, "replace", fail_replace)
    with pytest.raises(OSError, match="replacement failure"):
        write_record(path, newer)
    assert read_record(path) == original
    # A later successful atomic commit publishes one entire revision.
    monkeypatch.setattr(learning_store.os, "replace", actual)
    write_record(path, newer)
    assert read_record(path) == newer


def test_pending_symlink_never_overwrites_unrelated_file(tmp_path):
    path = tmp_path / (".startup-learning-" + "e" * 32)
    other = tmp_path / "unrelated"
    other.write_text("keep")
    path.with_name(path.name + ".pending").symlink_to(other)
    record = LearningRecord(
        attempt_id="0" * 32, run_id="e" * 32, clock_domain=DOMAIN, owner_epoch="f" * 32
    )
    with pytest.raises(OSError):
        write_record(path, record)
    assert other.read_text() == "keep" and not path.exists()


def test_commit_failure_is_local_and_never_confirms_action_consumption(tmp_path):
    worker = HeldWorker()
    owner = LearningRecordOwner(
        tmp_path / (".startup-learning-" + "9" * 32),
        "9" * 32,
        DOMAIN,
        new_run=True,
        worker_factory=lambda: worker,
    )
    try:
        worker.future.set_exception(OSError("injected fsync failure"))
        owner.poll()
        assert not owner.ready and "fsync failure" in owner.error
        assert worker.calls == 1  # No unbounded automatic retry queue.
        with pytest.raises(ValueError):
            owner.begin_consumer()
    finally:
        owner.close()


def test_action_consumption_requires_commit_and_survives_client_rebind(tmp_path):
    worker = HeldWorker()
    path = tmp_path / (".startup-learning-" + "a" * 32)
    owner = LearningRecordOwner(path, "a" * 32, DOMAIN, new_run=True, worker_factory=lambda: worker)
    try:
        with pytest.raises(ValueError):
            owner.begin_consumer()
        finish_writes(owner, worker)
        session, _ = owner.begin_consumer()
        revision = owner.reserve_action(session)
        assert revision is not None and not owner.action_ready(session, revision)
        worker.finish()
        owner.poll()
        assert owner.action_ready(session, revision)
        same_session, record = owner.begin_consumer()
        assert same_session == session and record.action.consumed
        assert owner.reserve_action(same_session) is None
    finally:
        owner.close()


def test_completed_catalog_is_published_only_after_commit_and_attach_is_read_only(tmp_path):
    worker = HeldWorker()
    path = tmp_path / (".startup-learning-" + "c" * 32)
    owner = LearningRecordOwner(path, "c" * 32, DOMAIN, new_run=True, worker_factory=lambda: worker)
    try:
        finish_writes(owner, worker)
        session, _ = owner.begin_consumer()
        owner.set_geometry(session, geometry(BOUNDARY))
        worker.finish()
        owner.poll()
        owner.capture(CaptureSummary(state="complete"))
        owner.complete_model(session, ((0.0, 0.15), (0.1, 0.1), (0.4, 0.1)))
        assert owner.public_record.model is None
        with pytest.raises(ValueError, match="model_not_durable"):
            owner.begin_consumer()
        finish_writes(owner, worker)
        readonly, record = owner.begin_consumer()
        assert readonly is None and record.model.points == ((0.0, 0.15), (0.1, 0.1), (0.4, 0.1))
        from senior_control.runtime.learning_store import attempt_path

        assert read_record(attempt_path(path, record.attempt_id)).model == record.model
        with pytest.raises(ValueError, match="consumer_identity"):
            owner.reserve_action(session)
        calls, before = worker.calls, path.read_bytes()
        for _ in range(1000):
            owner.poll()
            assert owner.begin_consumer() == (None, record)
        assert worker.calls == calls and path.read_bytes() == before
    finally:
        owner.close()


def test_unclean_owner_exit_preserves_saved_frozen_model_and_consumed_action(tmp_path):
    run = TestRun(tmp_path / "unclean")
    client = run.client()
    client.geometry(geometry(BOUNDARY))
    assert client.reserve_action()
    run.owner.capture(CaptureSummary(state="complete"))
    run.owner.complete_model(client.session, ((0.0, 0.15),))
    run.pump()
    original = client.record.model
    client.close()
    run.owner.close()
    restored = LearningRecordOwner(
        run.path, run.owner.record.run_id, DOMAIN, new_run=False, worker_factory=ImmediateWorker
    )
    try:
        for _ in range(4):
            restored.poll()
        assert restored.record.model == original and restored.record.phase == "completed"
        assert restored.record.action.consumed
        assert restored.begin_consumer()[0] is None
    finally:
        restored.close()


def test_explicit_attempt_requests_are_idempotent_busy_and_preserve_old_action(tmp_path):
    from senior_control.protocol import LearningStartPayload
    from senior_control.runtime.learning_store import attempt_path
    from senior_control.runtime.learning_wire import BeginAttempt

    run = TestRun(tmp_path / "attempts")
    try:
        session, _ = run.owner.begin_consumer()
        run.owner.reserve_action(session)
        run.pump()
        run.owner.set_phase(session, "failed", "control_takeover")
        run.pump()
        old = run.owner.record
        request = BeginAttempt(
            start_request_id="manual",
            fingerprint="a" * 64,
            payload=LearningStartPayload(expected_active_revision=0, expected_authority_id=None),
            owner_boot_id="b" * 32,
            generation=1,
            deadline_steady=100.0,
            admitted=True,
        )
        accepted = run.rpc(request)
        identity = accepted.data["attempt_id"]
        assert accepted.data["state"] == "accepted"
        duplicate = run.rpc(request.model_copy(update={"deadline_steady": 200.0, "generation": 2}))
        assert duplicate.data == accepted.data
        assert run.service.proposal.deadline_steady == 100.0
        busy = run.rpc(request.model_copy(update={"start_request_id": "another"}))
        assert busy.data["state"] == "busy" and busy.data["attempt_id"] == identity
        assert run.owner.record.attempt_id == identity and not run.owner.record.action.consumed
        assert read_record(attempt_path(run.path, old.attempt_id)).action == old.action
        from senior_control.protocol import LearningStatusPayload
        from senior_control.runtime.learning_wire import QueryAttempt

        queried = run.rpc(QueryAttempt(payload=LearningStatusPayload(attempt_id=old.attempt_id)))
        assert queried.data["phase"] == "failed" and queried.data["reason"] == "control_takeover"

    finally:
        run.close()
