"""Actual client thread observes the service's single-writer records; no model upload API."""

import threading
import time

from learning_test_support import TestRun, geometry
from senior_perception import learning_client
from senior_perception.learning_record import CaptureSummary
from senior_perception.learning_wire import Reply, State
from test_learning_record_owner import HeldWorker, finish_writes
from test_startup_learning import side_observation
from test_startup_learning_feasibility import BOUNDARY


def wire_owner(monkeypatch, run):
    lock, requests = threading.Lock(), []

    def request_once(run_id, request):
        assert run_id == run.owner.record.run_id
        with lock:
            requests.append(type(request))
            run.pump()
            try:
                return run.rpc(request)
            except ValueError as exc:
                return Reply(ok=False, error=str(exc))

    monkeypatch.setattr(learning_client, "request_once", request_once)
    return requests, lock


def until(client, predicate):
    end = time.monotonic() + 3
    while time.monotonic() < end:
        client.poll()
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError(f"client did not converge: {client.error}")


def test_actual_control_client_reserves_once_and_rebinds_only_to_durable_view(
    tmp_path, monkeypatch
):
    run = TestRun(tmp_path / "client")
    _, lock = wire_owner(monkeypatch, run)
    clients = []
    try:
        client = learning_client.LearningClient(run.path)
        clients.append(client)
        until(client, lambda: client.ready)
        client.geometry(geometry(BOUNDARY))
        until(client, lambda: client.record.geometry is not None)
        until(client, client.reserve_action)
        with lock:
            run.owner.capture(CaptureSummary(state="complete", compute_complete=True))
            run.owner.complete_model(client._session, ((0.0, 0.15),))
        until(client, lambda: client.view is not None)
        assert client.view.filter(side_observation(0.15)).rays[180].kind == "self"
        assert client.view.filter(side_observation(0.10)).rays[180].kind == "hit"
        client.close()
        restored = learning_client.LearningClient(run.path)
        clients.append(restored)
        until(restored, lambda: restored.view is not None)
        assert restored._session is None and not restored.reserve_action()
        assert restored.record.action.consumed
        assert restored.view.catalog == client.view.catalog
    finally:
        for client in clients:
            client.close()
        run.close()


def test_read_only_observer_keeps_old_view_until_new_active_manifest_is_durable(
    tmp_path, monkeypatch
):
    run = TestRun(tmp_path / "observer")
    requests, lock = wire_owner(monkeypatch, run)
    observer = learning_client.LearningClient(run.path, read_only=True)
    try:
        until(observer, lambda: observer.record is not None)
        assert observer.view is None and run.owner._session is None
        with lock:
            session, _ = run.owner.begin_consumer()
            run.owner.set_geometry(session, geometry(BOUNDARY))
            run.owner.capture(CaptureSummary(state="complete", compute_complete=True))
            run.owner.complete_model(session, ((0.0, 0.15),))
        until(observer, lambda: observer.view is not None)
        old = observer.view
        with lock:
            run.pump()
            assert run.owner.new_attempt("manual-request", "f" * 64, 1) == "accepted"
            run.pump()
            session, _ = run.owner.begin_consumer()
            run.owner.set_geometry(session, geometry(BOUNDARY))
            run.owner.capture(CaptureSummary(state="complete", compute_complete=True))
            run.pump()
            held = HeldWorker()
            run.owner._worker = held
            run.owner.complete_model(session, ((0.0, 0.10),))
        until(observer, lambda: observer.record.phase == "validating")
        assert observer.view is old
        with lock:
            held.finish()  # candidate durable, active reference still pending
            run.owner.poll()
        time.sleep(0.06)
        observer.poll()
        assert observer.view is old
        with lock:
            finish_writes(run.owner, held)
        until(observer, lambda: observer.active_revision == 2)
        replacement = observer.view
        assert replacement is not old and replacement.catalog.points == ((0.0, 0.10),)
        until(observer, lambda: len(requests) > 5)
        assert observer.view is replacement
        assert observer._session is None and not observer.reserve_action()
        assert run.owner._session == session
        assert requests and all(request is State for request in requests)
    finally:
        observer.close()
        run.close()


def test_raw_sender_reopens_when_attempt_changes(tmp_path, monkeypatch):
    from dataclasses import asdict
    from types import SimpleNamespace as N

    from senior_perception.learning_wire import BeginAttempt
    from sensor_time_test_support import capture_fields
    from test_learning_compute import recorded

    from senior_control.protocol import LearningStartPayload

    run = TestRun(tmp_path / "sender-switch")
    _, lock = wire_owner(monkeypatch, run)
    sender = learning_client.LearningFrames(run.path, run.profile, "base_link")
    monitor = N(poll=lambda: None, error=None)

    def offer(index):
        frame = recorded(index, time.monotonic())
        header = asdict(frame.capture)
        header.pop("time")
        geometry = frame.geometry
        message = N(
            frame_id=geometry.sensor_frame,
            raw_capture=N(**header, **capture_fields(time=frame.capture.time)),
            ranges=[float(value) for value in frame.ranges],
            angle_min=geometry.angle_min,
            angle_increment=geometry.angle_increment,
            range_min=geometry.range_min,
            range_max=geometry.range_max,
            sensor_x=0.0,
            sensor_y=0.0,
            sensor_yaw=0.0,
            transform_stamp=frame.transform_stamp,
        )
        sender.offer(N(message=message))

    try:
        offer(1)
        offer(2)
        until(monitor, lambda: run.service.accepted_frames == 2)
        with lock:
            session, _ = run.owner.begin_consumer()
            run.owner.set_phase(session, "failed", "control_takeover")
            run.pump()
            request = BeginAttempt(
                start_request_id="new-round",
                fingerprint="a" * 64,
                payload=LearningStartPayload(
                    expected_active_revision=0, expected_authority_id=None
                ),
                owner_boot_id="b" * 32,
                output_boot_id="c" * 32,
                generation=1,
                deadline_steady=time.monotonic() + 5,
                admitted=True,
            )
            identity = run.rpc(request).data["attempt_id"]
        until(monitor, lambda: sender._attempt == identity and sender._recording)
        offer(3)
        until(monitor, lambda: run.service.accepted_frames == 1)
        assert run.service.frames[0].capture.sequence == 3
        assert run.service.frames[0].ranges[35] == "inf"
    finally:
        sender.close()
        run.close()


def test_live_motion_client_can_observe_next_attempt_after_its_catalog_is_durable(
    tmp_path, monkeypatch
):
    from senior_perception.learning_wire import BeginAttempt

    from senior_control.protocol import LearningStartPayload

    run = TestRun(tmp_path / "live-client-next")
    requests, lock = wire_owner(monkeypatch, run)
    client = learning_client.LearningClient(run.path)
    try:
        until(client, lambda: client.ready)
        client.geometry(geometry(BOUNDARY))
        until(client, lambda: client.record.geometry is not None)
        until(client, client.reserve_action)
        old_session = client._session
        with lock:
            run.owner.capture(CaptureSummary(state="complete", compute_complete=True))
            run.owner.complete_model(old_session, ((0.0, 0.15),))
        until(client, lambda: client.view is not None)
        assert not client.reserve_action()
        old = client.record
        client.close()
        client = learning_client.LearningClient(run.path, read_only=True)
        until(client, lambda: client.view is not None)
        with lock:
            observed_from = len(requests)

            request = BeginAttempt(
                start_request_id="next",
                fingerprint="a" * 64,
                payload=LearningStartPayload(
                    expected_active_revision=1, expected_authority_id=None
                ),
                owner_boot_id="b" * 32,
                output_boot_id="c" * 32,
                generation=1,
                deadline_steady=time.monotonic() + 5,
                admitted=True,
            )
            identity = run.rpc(request).data["attempt_id"]

        until(client, lambda: client.record.attempt_id == identity)
        assert client.read_only and client._session is None and not client.ready
        with lock:
            assert all(request is State for request in requests[observed_from:])
        assert client.bind_attempt(identity, "c" * 32, 1) is False

        def advanced():
            client.observed(
                segment_id="c" * 32,
                now_ns=time.monotonic_ns(),
                validation_after_ns=None,
                first_takeover_ns=None,
                stationary=True,
            )
            return client.record.attempt_id == identity and client.ready

        until(client, advanced)
        assert client._session != old_session
        assert not client.record.action.consumed and not client._action_ready
        assert client.view.catalog == old.model
        assert client.error is None
    finally:
        client.close()
        run.close()


def test_raw_sender_stops_after_completion_and_reopens_only_for_new_attempt(tmp_path, monkeypatch):
    from types import SimpleNamespace

    from senior_perception.learning_wire import BeginAttempt

    from senior_control.protocol import LearningStartPayload

    run = TestRun(tmp_path / "sender-complete")
    _, lock = wire_owner(monkeypatch, run)
    sender = learning_client.LearningFrames(run.path, run.profile, "base_link")
    monitor = SimpleNamespace(poll=lambda: None, error=None)
    try:
        with lock:
            session, _ = run.owner.begin_consumer()
            run.owner.set_geometry(session, geometry(BOUNDARY))
            run.owner.reserve_action(session)
            run.owner.capture(CaptureSummary(state="complete", compute_complete=True))
            run.owner.complete_model(session, ((0.0, 0.15),))
            run.pump()
            run.pump()
        until(monitor, lambda: not sender._recording)
        # A stopped recorder must return before looking at message/header/ranges.
        sender.offer(object())
        with sender._lock:
            assert not sender._frames
        with lock:
            reply = run.rpc(
                BeginAttempt(
                    start_request_id="after-completion",
                    fingerprint="d" * 64,
                    payload=LearningStartPayload(
                        expected_active_revision=run.owner.public_manifest.active_revision,
                        expected_authority_id=None,
                    ),
                    owner_boot_id="b" * 32,
                    output_boot_id="c" * 32,
                    generation=1,
                    deadline_steady=time.monotonic() + 5,
                    admitted=True,
                )
            )
            identity = reply.data["attempt_id"]
        until(monitor, lambda: sender._attempt == identity and sender._recording)
    finally:
        sender.close()
        run.close()
