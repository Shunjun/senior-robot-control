"""Real bounded client thread against an in-process owner transport adapter.

Kernel peer-credential/SEQPACKET behavior remains covered only by target Linux test.
"""

import threading
import time

from learning_test_support import TestRun, geometry
from test_startup_learning import side_observation
from test_startup_learning_feasibility import BOUNDARY

from senior_control.runtime import learning_client
from senior_control.runtime.learning_wire import (
    ActionReady,
    ActionReport,
    Attach,
    Complete,
    GeometryReport,
    PhaseReport,
    Reply,
    Reserve,
)


def wire_owner(monkeypatch, run):
    lock = threading.Lock()

    def request_once(run_id, request):
        assert run_id == run.owner.record.run_id
        with lock:
            owner = run.owner
            owner.poll()
            revision = session = None
            ready = False
            try:
                if isinstance(request, Attach):
                    session, _ = owner.begin_consumer()
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
                    owner.set_phase(
                        request.session, request.phase, request.reason, request.segment_id
                    )
                elif isinstance(request, Complete):
                    revision = owner.complete_model(request.session, request.points)
                return Reply(
                    ok=True,
                    error=owner.error,
                    record=owner.public_record,
                    session=session,
                    revision=revision,
                    ready=ready,
                    durable_revision=owner.durable_revision,
                )
            except ValueError as exc:
                return Reply(ok=False, error=str(exc))

    monkeypatch.setattr(learning_client, "request_once", request_once)


def until(client, predicate):
    end = time.monotonic() + 3
    while time.monotonic() < end:
        client.poll()
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError(f"client did not converge: {client.error}")


def test_real_client_thread_completion_fixed_filter_and_same_run_rebind(tmp_path, monkeypatch):
    run = TestRun(tmp_path / "client")
    wire_owner(monkeypatch, run)
    clients = []
    try:
        client = learning_client.LearningClient(run.path)
        clients.append(client)
        until(client, lambda: client.ready)
        manifest = geometry(BOUNDARY)
        client.geometry(manifest)
        until(client, lambda: client.record.geometry == manifest)
        until(client, client.reserve_action)
        client.complete(((0.0, 0.15),))
        until(client, lambda: client.view is not None)
        assert client.view.filter(side_observation(0.15)).rays[180].kind == "self"
        assert client.view.filter(side_observation(0.10)).rays[180].kind == "hit"
        client.close()
        restored = learning_client.LearningClient(run.path)
        clients.append(restored)
        until(restored, lambda: restored.view is not None)
        assert restored.record.model == client.record.model
        assert restored._session is None
        assert restored.record.action.consumed and not restored.reserve_action()
        assert restored.view.filter(side_observation(0.15)).rays[180].kind == "self"
    finally:
        for client in clients:
            client.close()
        run.close()


def test_completed_model_ignores_runtime_geometry_reports(tmp_path, monkeypatch):
    run = TestRun(tmp_path / "geometry-client")
    wire_owner(monkeypatch, run)
    client = learning_client.LearningClient(run.path)
    try:
        until(client, lambda: client.ready)
        manifest = geometry(BOUNDARY)
        client.geometry(manifest)
        until(client, lambda: client.record.geometry == manifest)
        client.complete(((0.0, 0.15),))
        until(client, lambda: client.view is not None)
        old_view, before = client.view, run.path.read_bytes()
        changed = manifest.model_copy(update={"sensor_to_body": (0.01, 0.0, 0.0)})
        client.geometry(changed)
        assert client.view is old_view and client.record.geometry == manifest
        assert client.record.model is not None and client.error is None
        assert run.path.read_bytes() == before
    finally:
        client.close()
        run.close()
