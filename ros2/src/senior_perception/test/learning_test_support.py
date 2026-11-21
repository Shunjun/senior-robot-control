"""Real record owner/storage/mailbox with deterministic test-only commit resources."""

import hashlib
from concurrent.futures import Future
from pathlib import Path

import pytest
from senior_perception.learning_owner import LearningRecordOwner
from senior_perception.startup_learning import StartupLearning
from sensor_time_test_support import sample_time

from senior_control.safety.learning_view import LearningFilterView

DOMAIN = "00000000-0000-0000-0000-000000000001"


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


class InlineCompute:
    """Only transport is inline; the real one-shot compute function is used."""

    def __init__(self):
        self.flight = None
        self.held = False
        self.requests = []

    def submit(self, request):
        assert self.flight is None
        self.requests.append(request)
        self.flight = request

    def poll(self):
        from senior_perception.learning_compute import compute_samples

        if self.flight is None or self.held:
            return None
        request, self.flight = self.flight, None
        return compute_samples(request)


class TestRun:
    __test__ = False

    def __init__(self, label):
        from senior_perception.learning_service import LearningService

        identity = hashlib.sha256(str(label).encode()).hexdigest()[:32]
        self.path = Path(label).parent / (".startup-learning-" + identity)
        self.owner = LearningRecordOwner(
            self.path, identity, DOMAIN, new_run=True, worker_factory=ImmediateWorker
        )
        self.owner.record = self.owner.record.model_copy(update={"created_steady_ns": 0})
        self.service = LearningService(self.path)
        self.service.owner = self.owner
        self.service.compute = InlineCompute()
        self.clients = []
        self.hold_ingress = False
        self.profile = type("Profile", (), {"length": 0.6, "width": 0.4})()
        self.pump()

    def rpc(self, request):
        import asyncio

        return asyncio.run(self.service.handle(request))

    def pump(self):
        for _ in range(4):
            self.owner.poll()
            self.service._advance_capture()
            self.service._pump_compute()

    def offer(self, raw):
        import threading
        from collections import deque
        from unittest.mock import patch

        from senior_perception.learning_client import LearningFrames
        from senior_perception.learning_wire import FrameBatch

        recorder = object.__new__(LearningFrames)
        recorder.profile, recorder.body_frame = self.profile, "base_link"
        recorder._frames = deque(maxlen=16)
        recorder._lock = threading.Lock()
        recorder._recording, recorder._attempt = True, self.owner.record.attempt_id
        with patch(
            "senior_perception.learning_client.time.monotonic_ns",
            return_value=round(raw.received * 1e9),
        ):
            recorder.offer(raw)
        batch = FrameBatch(
            attempt_id=self.owner.record.attempt_id,
            frames=tuple(recorder._frames),
            through_steady_ns=round(raw.received * 1e9),
        )
        if not self.hold_ingress:
            self.service._accept_frames(batch)
        return batch

    def heartbeat(self, now_ns):
        from senior_perception.learning_wire import FrameBatch

        if not self.hold_ingress:
            self.service._accept_frames(
                FrameBatch(
                    attempt_id=self.owner.record.attempt_id,
                    frames=(),
                    through_steady_ns=now_ns,
                )
            )

    def client(self, *, read_only=False):
        client = InProcessClient(self, read_only=read_only)
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
    def __init__(self, run, *, read_only=False):
        self.read_only = read_only
        self.session = None
        self.run, self.closed, self._reservation = run, False, None
        self.view = None
        self._attempt = run.owner.record.attempt_id
        self.active_revision = 0
        if not read_only:
            self._attach()

    def _attach(self):
        self.run.pump()
        self.session, _ = self.run.owner.begin_consumer()
        self.poll()

    @property
    def proposal(self):
        return self.run.service.proposal

    @property
    def accepted_frames(self):
        return self.run.service.accepted_frames

    @property
    def error(self):
        return self.run.owner.error

    def poll(self):
        self.run.pump()
        if self._attempt != self.run.owner.record.attempt_id:
            self._attempt = self.run.owner.record.attempt_id
            self._reservation = None
            self.session = None
        record = self.run.owner.active_record
        if record is not None and record.model is not None:
            if self.active_revision != self.run.owner.public_manifest.active_revision:
                self.view = LearningFilterView(record.model, record.geometry)
                self.active_revision = self.run.owner.public_manifest.active_revision

    @property
    def record(self):
        self.poll()
        return self.run.owner.public_record

    @property
    def ready(self):
        self.poll()
        return (
            self.session is not None
            and not self.closed
            and self.run.owner.ready
            and self.error is None
        )

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
        if self.session is not None:
            self.run.owner.set_geometry(self.session, geometry)
            self.poll()

    def phase(self, phase, reason, segment_id=None):
        from senior_perception.learning_wire import PhaseReport

        if self.session is not None:
            self.run.rpc(
                PhaseReport(session=self.session, phase=phase, reason=reason, segment_id=segment_id)
            )
        self.poll()

    def action(self, start_ns, interval_ns, *, finished):
        self.run.owner.report_action(self.session, start_ns, interval_ns, finished=finished)
        self.poll()

    def observed(
        self, *, segment_id, now_ns, validation_after_ns, first_takeover_ns, stationary, reason=None
    ):
        from senior_perception.learning_wire import MotionObservedThrough

        if self.session is not None:
            self.run.rpc(
                MotionObservedThrough(
                    session=self.session,
                    attempt_id=self.run.owner.record.attempt_id,
                    segment_id=segment_id,
                    through_steady_ns=now_ns,
                    validation_after_steady_ns=validation_after_ns,
                    first_takeover_ns=first_takeover_ns,
                    reason=reason,
                    stationary=stationary,
                )
            )
            self.run.heartbeat(now_ns)
        self.poll()

    def bind_attempt(self, attempt_id, output_boot_id, generation):
        from senior_perception.learning_wire import Attach

        if self.session is None:
            reply = self.run.rpc(
                Attach(attempt_id=attempt_id, output_boot_id=output_boot_id, generation=generation)
            )
            self.session, self.read_only = reply.session, False
        return self.ready

    def retire_motion_owner(self, record, output_boot_id):
        from senior_perception.learning_wire import RetireMotionOwner

        self.run.rpc(
            RetireMotionOwner(
                attempt_id=record.attempt_id,
                owner_epoch=record.owner_epoch,
                motion_owner_id=record.motion_owner_id,
                motion_generation=record.motion_generation,
                replacement_output_boot_id=output_boot_id,
            )
        )
        return True

    def decline_attempt(self, record, output_boot_id, reason):
        from senior_perception.learning_wire import DeclineAttempt

        self.run.rpc(
            DeclineAttempt(
                attempt_id=record.attempt_id, output_boot_id=output_boot_id, reason=reason
            )
        )
        return True

    def accept_attempt(self, accepted, reason):
        from senior_perception.learning_wire import AcceptAttempt

        self.run.rpc(
            AcceptAttempt(
                session=self.session,
                attempt_id=self.run.owner.record.attempt_id,
                accepted=accepted,
                reason=reason,
            )
        )

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
    core.learning._learning_client = client
    core.learning.startup = StartupLearning(client)
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
    from senior_perception.learning_record import LearningGeometry

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
    from senior_perception.learning_record import LearningRecord
    from senior_perception.learning_store import write_record

    record = LearningRecord(
        run_id="f" * 32,
        attempt_id="d" * 32,
        clock_domain=DOMAIN,
        motion_owner_id="9" * 32,
        owner_epoch="e" * 32,
        geometry=geometry(boundary),
    )
    write_record(path, record)


def learning_point(source, stamp_ns, received_ros_ns, received_steady_ns, values, sequence=1):
    """Explicit point input with real mapping, no environment-specific admission."""
    from senior_control.motion.observations import Frame, SourceStatus
    from senior_control.motion.sample_types import PointObservation

    return PointObservation(
        source=source,
        frame=Frame.BODY,
        time=sample_time(
            stamp_ns=stamp_ns,
            received_ros_ns=received_ros_ns,
            received_steady_ns=received_steady_ns,
        ),
        epoch="test-producer",
        sequence=sequence,
        quality=1.0,
        status=SourceStatus.VALID,
        values=values,
        covariance=None,
        model_id=None,
        body_from_sensor=None,
    )


def save_completed(run, points):
    """Persist a frozen model through the actual owner, for restoration-only tests."""
    from senior_perception.learning_record import CaptureSummary

    session = run.owner._session
    if session is None:
        session, _ = run.owner.begin_consumer()
    if not run.owner.record.action.consumed:
        run.owner.reserve_action(session)
    run.owner.capture(CaptureSummary(state="complete", compute_complete=True))
    run.owner.complete_model(session, tuple(points))
    for _ in range(3):
        run.pump()


def raw_scan(stamp, sequence):
    """A real-shaped raw observation without the retired ScanGeometry test fixture."""
    import math
    from dataclasses import asdict
    from types import SimpleNamespace

    from senior_perception.raw_safety_observation import RawSafetyObservation
    from sensor_time_test_support import capture_fields
    from test_learning_compute import recorded

    capture = recorded(sequence, stamp).capture
    fields = asdict(capture)
    fields.pop("time")
    message = SimpleNamespace(
        source_epoch="raw",
        sequence=sequence,
        frame_id="laser_link",
        ranges=[2.0] * 720,
        angle_min=-math.pi,
        angle_increment=math.tau / 720,
        range_min=0.05,
        range_max=20.0,
        sensor_x=0.0,
        sensor_y=0.0,
        sensor_yaw=0.0,
        transform_stamp=stamp,
        raw_capture=SimpleNamespace(**fields, **capture_fields(time=capture.time)),
    )
    return RawSafetyObservation(message, stamp, stamp, stamp, stamp)
