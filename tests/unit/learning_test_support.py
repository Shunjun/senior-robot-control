"""Real record owner/storage/mailbox with deterministic test-only commit resources."""

import hashlib
from concurrent.futures import Future
from pathlib import Path

import pytest
from source_time_test_support import DOMAIN

from senior_control.motion.startup_learning import StartupLearning
from senior_control.runtime.learning_owner import LearningRecordOwner
from senior_control.safety.learning_view import LearningFilterView


class ImmediateWorker:
    def submit(self, task):
        future = Future()
        try:
            future.set_result(task())
        except Exception as exc:
            future.set_exception(exc)
        return future

    def close(self):
        pass


class TestRun:
    __test__ = False

    def __init__(self, label):
        identity = hashlib.sha256(str(label).encode()).hexdigest()[:32]
        self.path = Path(label).parent / (".startup-learning-" + identity)
        self.owner = LearningRecordOwner(
            self.path,
            identity,
            DOMAIN,
            new_run=True,
            worker_factory=ImmediateWorker,
        )
        self.clients = []
        self.pump()

    def pump(self):
        for _ in range(4):
            self.owner.poll()

    def client(self):
        client = InProcessClient(self)
        self.clients.append(client)
        return client

    def close(self):
        for client in self.clients:
            client.close()
        self.pump()
        self.owner.prepare_close()
        self.pump()
        self.owner.close()


class InProcessClient:
    def __init__(self, run):
        self.run = run
        self.closed = False
        self._reservation = None
        self.view = None
        self._attach()

    def _attach(self):
        self.run.pump()
        self.session, record = self.run.owner.begin_consumer()
        model = record.model
        self.view = None if model is None else LearningFilterView(model, record.geometry)

    @property
    def error(self):
        return self.run.owner.error

    def poll(self):
        self.run.pump()

    @property
    def record(self):
        self.poll()
        return self.run.owner.public_record

    @property
    def ready(self):
        self.poll()
        return not self.closed and self.run.owner.ready and self.error is None

    def reserve_action(self):
        if self.session is None:
            return False
        if self._reservation is None:
            self._reservation = self.run.owner.reserve_action(self.session)
        self.poll()
        return self._reservation is not None and self.run.owner.action_ready(
            self.session, self._reservation
        )

    def geometry(self, geometry):
        if self.view is None:
            self.run.owner.set_geometry(self.session, geometry)
            self.poll()

    def phase(self, phase, reason, segment_id=None):
        if self.session is not None:
            self.run.owner.set_phase(self.session, phase, reason, segment_id)
        self.poll()

    def action(self, start_ns, interval_ns, *, finished):
        self.run.owner.report_action(self.session, start_ns, interval_ns, finished=finished)
        self.poll()

    def complete(self, points):
        self.run.owner.complete_model(self.session, points)
        self.poll()
        self._attach()

    def close(self):
        if not self.closed:
            self.poll()
            self.closed = True


_RUNS = {}


@pytest.fixture(autouse=True)
def learning_runs():
    yield
    for run in _RUNS.values():
        run.close()
    _RUNS.clear()


def client_for(label):
    key = str(label)
    if key not in _RUNS:
        _RUNS[key] = TestRun(label)
    return _RUNS[key].client()


def startup_for(label):
    if label is None:
        return StartupLearning(None)
    client = client_for(label)
    from test_startup_learning_feasibility import BOUNDARY

    client.geometry(geometry(BOUNDARY))
    return StartupLearning(client)


def bind_core(core, label):
    client = client_for(label)
    core._learning_client = client
    core.startup = StartupLearning(client)
    return client


def geometry(
    boundary,
    *,
    sensor=(0.0, 0.0, 0.0),
    angle_min=0.0,
    step=0.01,
    count=1,
    range_min=0.05,
    range_max=5.0,
):
    from senior_control.motion.learning_record import LearningGeometry

    return LearningGeometry(
        body_frame="base_link",
        sensor_frame="laser_link",
        source_kind="scan",
        physical_polygon=tuple(boundary),
        sensor_to_body=sensor,
        angle_min=angle_min,
        angle_increment=step,
        count=count,
        range_min=range_min,
        range_max=range_max,
    )


def begin_action(startup, now):
    previous = startup.state
    startup.state = "moving"
    startup.advance(0.0, now=now, stationary=False)
    assert startup.prepare_action()
    startup.motion_emitted(now)
    if previous not in {"disabled", "initializing", "sampling", "moving"}:
        startup.state = previous


def write_geometry_record(path, boundary):
    from senior_control.motion.learning_record import LearningRecord
    from senior_control.runtime.learning_store import write_record

    record = LearningRecord(
        run_id="f" * 32,
        clock_domain=DOMAIN,
        owner_epoch="e" * 32,
        geometry=geometry(boundary),
    )
    write_record(path, record)


def simulation_point(source, stamp, received, values, sequence=1, *, measurement_steady=None):
    """Explicit synthetic 1x source; production never derives physical time here."""
    from dataclasses import replace

    from source_time_test_support import capture_time
    from test_unified_motion_runtime import point

    host = received if measurement_steady is None else measurement_steady
    return replace(
        point(source, stamp, received, values, sequence, measurement_steady=host),
        measurement=capture_time(host, sequence=sequence).measurement,
    )
