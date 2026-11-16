"""Real spawn/mailbox behavior without ROS, services or output transport."""

import gc
import multiprocessing as mp
import os
import time
from functools import partial

import pytest

from senior_control.motion.cadence import ControlCadence
from senior_control.motion.process_resources import LatestSnapshot, ProcessWorker


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
    from senior_robot.motion_sensor_runtime import MotionSensorRuntime
    from senior_robot.simulation_motion_profile import SimulationMotionProfile
    from test_directional_constraints import engine, snapshot

    from senior_control.runtime.diagnostics import RuntimeDiagnostics

    sensors = MotionSensorRuntime(SimulationMotionProfile(), RuntimeDiagnostics(lambda *_: None))
    reference = sensors.estimate(10, 10)
    state = {k: v for k, v in vars(sensors).items() if k not in {"profile", "diagnostics"}}
    worker = ProcessWorker(name="batch-test-real-geometry")
    try:
        result = wait_done(worker.submit(partial(engine().refine, snapshot(), 0))).result()
        assert result.permit is not None
        assert result.permit.deadline_steady > 0
        pid, transferred = wait_done(
            worker.submit(partial(compute_pid, (state, (), False, 1, reference)))
        ).result()
        assert pid != os.getpid()
        assert transferred[0]["latest"] == state["latest"]
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

    from senior_robot import control_inputs

    captured = {}

    def process(**kwargs):
        captured.update(kwargs)
        return N(start=lambda: captured.update(started=True))

    context = N(Event=lambda: object(), Process=process)
    monkeypatch.setattr(control_inputs.mp, "get_context", lambda _: context)
    monkeypatch.setattr(control_inputs, "LatestSnapshot", lambda _: object())
    names = []
    node = N(
        list_parameters=lambda *a, **kw: N(names=[]),
        resolve_topic_name=lambda topic: names.append(topic) or "/isolated" + topic,
    )
    channel = control_inputs.ControlInputs(node)
    mappings = dict(captured["args"][1])
    assert captured["started"]
    assert mappings["/wheel/odom/timed"] == "/isolated/wheel/odom/timed"
    assert mappings["/imu/data/timed"] == "/isolated/imu/data/timed"
    assert "/wheel/odom" not in mappings and "/imu/data" not in mappings
    assert channel.slot is captured["args"][2]
