"""Real spawn/mailbox behavior without ROS, services or output transport."""

import gc
import multiprocessing as mp
import os
import time
from functools import partial

import pytest
from control_process_test_support import generic_motion_profile
from senior_motion_control.cadence import ControlCadence

from vehicle_core.process_resources import LatestSnapshot, ProcessWorker


@pytest.fixture(autouse=True)
def release_closed_future_owner_cycles():
    yield
    # Future -> owner -> flight cycles otherwise defer semaphore finalizers
    # until an unrelated allocation inside multiprocessing.resource_tracker.
    gc.collect()


def compute_pid(value):
    return os.getpid(), value


def held_task(seconds):
    time.sleep(seconds)
    return "finished"


def fail_task():
    raise ValueError("deliberate test failure")


def wait_done(future, timeout=5):
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, "spawn worker did not complete"
        time.sleep(0.005)
    return future


def test_absolute_slots_skip_50ms_after_80ms_work_and_skip_long_backlog():
    cadence = ControlCadence()
    assert cadence.due(0)
    cadence.finished(0.080)
    assert not cadence.due(0.080)
    assert not cadence.due(0.099)
    assert cadence.due(0.100)
    cadence.finished(1.280)
    assert not cadence.due(1.280)
    assert cadence.due(1.300)
    cadence.finished(1.301)
    assert not cadence.due(1.301)


def test_mailbox_latest_is_private_bounded_and_busy_lock_does_not_wait():
    slot = LatestSnapshot(mp.get_context("spawn"), capacity=256)
    source = {"rows": [1]}
    assert slot.put(source)
    source["rows"].append(2)
    version, result = slot.take(0)
    assert result == {"rows": [1]}
    result["rows"].append(3)
    assert slot.take(0)[1] == {"rows": [1]}
    assert slot.take(version) is None
    with pytest.raises(ValueError, match="snapshot_capacity_exceeded"):
        slot.put(bytes(300))
    assert slot.take(0)[1] == {"rows": [1]}
    slot._lock.acquire()
    try:
        start = time.monotonic()
        assert not slot.put({"rows": [4]})
        assert slot.take(0) is None
        assert time.monotonic() - start < 0.05
    finally:
        slot._lock.release()
    for i in range(100):
        assert slot.put(i)
    assert slot.take(version)[1] == 99


def test_real_spawn_serializes_tasks_and_results_and_cleans_up():
    worker = ProcessWorker(name="batch-test-geometry")
    process = worker.process
    try:
        future = worker.submit(partial(compute_pid, {"version": 7}))
        pid, result = wait_done(future).result()
        assert pid != os.getpid() and result == {"version": 7}
        future = worker.submit(partial(held_task, 0.1))
        with pytest.raises(RuntimeError, match="geometry_process_busy"):
            worker.submit(partial(compute_pid, 8))
        assert wait_done(future).result() == "finished"
        future = worker.submit(fail_task)
        with pytest.raises(RuntimeError, match="deliberate test failure"):
            wait_done(future).result()
        assert wait_done(worker.submit(partial(compute_pid, 9))).result()[1] == 9
    finally:
        worker.close()
    assert process._closed
    assert all(p.pid != pid for p in mp.active_children())


def test_busy_request_mailbox_retains_one_flight_without_blocking_or_losing_task():
    worker = ProcessWorker(name="batch-test-contended-geometry")
    try:
        worker.requests._lock.acquire()
        try:
            started = time.monotonic()
            future = worker.submit(partial(compute_pid, 7))
            assert not future.done()
            assert time.monotonic() - started < 0.05
            with pytest.raises(RuntimeError, match="geometry_process_busy"):
                worker.submit(partial(compute_pid, 8))
            assert worker.flight is future
        finally:
            worker.requests._lock.release()
        pid, result = wait_done(future).result()
        assert pid != os.getpid() and result == 7
        assert wait_done(worker.submit(partial(compute_pid, 9))).result()[1] == 9
    finally:
        worker.close()


def test_process_death_fails_pending_future_and_close_reaps_it():
    worker = ProcessWorker(name="batch-test-killed-geometry")
    future = worker.submit(partial(held_task, 10))
    worker.process.terminate()
    worker.process.join(2)
    try:
        with pytest.raises(RuntimeError, match="geometry_process_exited"):
            wait_done(future).result()
        with pytest.raises(RuntimeError, match="geometry_process_unavailable"):
            worker.submit(partial(compute_pid, 1))
    finally:
        worker.close()


def test_real_spawn_carries_geometry_and_complete_sensor_snapshot():
    from control_process_test_support import generic_motion_profile
    from learning_test_support import learning_point
    from senior_motion_estimation.motion_sensor_runtime import MotionSensorRuntime
    from senior_motion_estimation.observations import Source
    from test_directional_constraints import engine, snapshot

    from vehicle_core.diagnostics import RuntimeDiagnostics

    sensors = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
    sample = learning_point(
        Source.WHEEL, 10_000_000_000, 10_000_000_000, 10_000_000_000, (0.2, 0.0), sequence=1
    )
    assert sensors.ingest(sample, 10, 10)
    estimate = sensors.estimate(10, 10)
    reference = ({key: estimate[0][key] for key in ("average", "mode")}, None, None)
    state = sensors.decision_snapshot()
    worker = ProcessWorker(name="batch-test-real-geometry")
    try:
        result = wait_done(worker.submit(partial(engine().refine, snapshot(), 0))).result()
        assert result.permit is not None
        assert result.permit.deadline_steady > 0
        pid, transferred = wait_done(
            worker.submit(partial(compute_pid, (state, (), False, 1, reference)))
        ).result()
        assert pid != os.getpid()
        assert transferred[0].latest == state.latest
        reader = MotionSensorRuntime(generic_motion_profile(), RuntimeDiagnostics(lambda *_: None))
        reader.apply_decision_snapshot(transferred[0], transferred[4])
        original = reader.latest
        reader.apply_decision_snapshot(transferred[0], transferred[4])
        assert reader.latest == original
        assert reader.latest.computed_steady == state.latest.computed_steady
        assert reader.latest.generation == state.latest.generation
        assert reader.latest.healthy and not reader.latest.vy.observed
        assert reader.raw_wheel == state.raw_wheel and reader.raw_imu is None
        assert reader.raw_wheel.roots[0].time == sample.time
        assert reader.wheel_linear(10.5) == pytest.approx(0.2)
        reader.apply_decision_snapshot(transferred[0], transferred[4])
        assert reader.wheel_linear(11.0) is None  # Replaying the snapshot cannot renew the root.
        assert reader.input_reference == reference
        assert transferred[1:] == ((), False, 1, reference)
    finally:
        worker.close()
        worker.close()  # Shutdown may run from both partial-init cleanup and explicit owner close.


def busy_task(seconds):
    started = time.monotonic()
    iterations = 0
    while time.monotonic() - started < seconds:
        iterations += 1
    return os.getpid(), iterations


def test_cpu_busy_spawn_does_not_hold_parent_snapshot_or_heartbeat():
    worker = ProcessWorker(name="batch-test-cpu-isolation")
    slot = LatestSnapshot(mp.get_context("spawn"), capacity=256)
    try:
        # Establish the separate interpreter before measuring concurrent work.
        wait_done(worker.submit(partial(compute_pid, 0))).result()
        future = worker.submit(partial(busy_task, 0.2))
        updates = 0
        deadline = time.monotonic() + 5
        while not future.done():
            assert time.monotonic() < deadline
            assert slot.put({"heartbeat": updates})
            updates += 1
            time.sleep(0.005)
        pid, iterations = future.result()
        assert pid != os.getpid() and iterations > 0 and updates >= 5
        assert slot.take(0)[1] == {"heartbeat": updates - 1}
    finally:
        worker.close()


def test_reception_child_inherits_typed_sensor_remaps_without_twist_or_untyped_fallback(
    monkeypatch,
):
    from types import SimpleNamespace as N

    import senior_motion_control.control_inputs as control_inputs

    captured = {}

    def process(**kwargs):
        captured.update(kwargs)
        return N(start=lambda: captured.update(started=True))

    context = N(Event=lambda: object(), Process=process)
    monkeypatch.setattr(control_inputs.mp, "get_context", lambda _: context)
    monkeypatch.setattr(control_inputs, "LatestSnapshot", lambda _: object())
    names = []
    node = N(
        _motion_core=N(profile=generic_motion_profile()),
        get_parameters_by_prefix=lambda prefix: {},
        resolve_topic_name=lambda topic: names.append(topic) or "/isolated" + topic,
    )
    channel = control_inputs.ControlInputs(node)
    mappings = dict(captured["args"][1])
    assert captured["started"]
    assert mappings["/wheel/odom"] == "/isolated/wheel/odom"
    assert mappings["/imu/data"] == "/isolated/imu/data"
    assert channel.slot is captured["args"][2]
