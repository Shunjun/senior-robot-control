import json
import multiprocessing
import os
import signal
import threading
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from senior_control.config import DeviceProfile
from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.safety.async_guard import (
    BoundedSafetyWorker,
    RawSafetyObservation,
    SafetyTicket,
    SafetyWork,
    _worker_main,
    evaluate_safety_work,
)
from senior_control.safety.motion_runtime import (
    StampedObservation,
    TimedPoseBuffer,
    TimedPoseSample,
)
from senior_control.safety.self_filter import ScanObservation, filter_self
from senior_control.safety.velocity_uncertainty import IntervalVelocityBounds


def _wait_for_release(job):
    started, released = map(Path, job)
    started.write_text("started")
    while not released.exists():
        time.sleep(0.01)
    return "finished"


def _large_reply(_job):
    return b"x" * (8 * 1024 * 1024)


def _echo_work(job):
    return job


class _ReadyBlockingQueue:
    def __init__(self, ready, queue):
        self.ready = ready
        self.queue = queue

    def get(self):
        self.ready.set()
        return self.queue.get()


def test_worker_sigint_exits_cleanly_while_idle():
    context = multiprocessing.get_context("spawn")
    ready = context.Event()
    jobs, replies = context.Queue(), context.Queue()
    process = context.Process(
        target=_worker_main,
        args=(
            _ReadyBlockingQueue(ready, jobs),
            replies,
            _large_reply,
        ),
    )
    try:
        process.start()
        assert ready.wait(timeout=10), "owned worker did not reach its input wait"
        os.kill(process.pid, signal.SIGINT)
        process.join(timeout=3)
        assert process.exitcode == 0, "worker did not exit cleanly on SIGINT"
    finally:
        if process.is_alive():
            process.terminate()
            process.join(timeout=3)
        for queue in (jobs, replies):
            queue.cancel_join_thread()
            queue.close()


def _ticket():
    return SafetyTicket(1, 2, 3, "epoch-a", Velocity(0.05, 0.0), 10.0, 10.2, 20.0, 20.2)


def _permits(ticket, **changes):
    arguments = dict(
        command_generation=1,
        scan_generation=2,
        motion_generation=3,
        epoch="epoch-a",
        command=Velocity(0.05, 0.0),
        steady=10.1,
        ros=20.1,
    )
    arguments.update(changes)
    return ticket.permits(**arguments)


@pytest.mark.parametrize(
    "change",
    [
        {"command": Velocity(0, 0), "command_generation": 2},
        {"scan_generation": 3},
        {"motion_generation": 4},
        {"epoch": "epoch-b"},
        {"configuration": ("changed",)},
        {"steady": 10.201},
        {"ros": 20.201},
        {"ros": 19.9},
    ],
)
def test_ticket_rejects_stop_new_input_old_epoch_or_expired_horizon(change):
    ticket = _ticket()
    assert _permits(ticket)
    assert not _permits(ticket, **change)


def test_ticket_rejects_geometry_horizon_even_while_sensor_is_within_age_limit():
    # The underlying scan is still younger than its 0.8 s freshness limit,
    # but its swept uncertainty was only calculated through ROS time 20.2.
    assert 20.21 - 20.0 < 0.8
    assert not _permits(_ticket(), steady=10.1, ros=20.21)


def test_warmup_starts_idle_worker_without_pending_work_or_replies():
    replies = []
    worker = BoundedSafetyWorker(_echo_work, on_result=replies.append)
    try:
        assert worker._process is None and worker._supervisor is None
        assert worker.warmup()
        supervisor = worker._supervisor
        assert worker.warmup() and worker._supervisor is supervisor
        deadline = time.monotonic() + 10
        while (
            worker._process is None or not worker._process.is_alive()
        ) and time.monotonic() < deadline:
            time.sleep(0.01)
        assert worker._process is not None and worker._process.is_alive()
        assert not worker.pending and worker.poll() is None and not replies
        assert worker.submit(_ticket(), "real submitted work")
        deadline = time.monotonic() + 10
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll()
            time.sleep(0.01)
        assert result is not None and result.error is None
        assert result.value == "real submitted work" and result.ticket == _ticket()
        assert not worker.pending
    finally:
        worker.close()


def test_warmed_worker_is_reaped_when_closed_without_any_submission():
    worker = BoundedSafetyWorker(_echo_work)
    try:
        assert worker.warmup()
        deadline = time.monotonic() + 10
        while (
            worker._process is None or not worker._process.is_alive()
        ) and time.monotonic() < deadline:
            time.sleep(0.01)
        process = worker._process
        assert process is not None and process.is_alive()
        assert not worker.pending and worker.poll() is None
    finally:
        worker.close()
    assert worker._supervisor is not None
    worker._supervisor.join(timeout=3)
    assert not worker._supervisor.is_alive() and not process.is_alive()
    assert not worker.warmup() and not worker.submit(_ticket(), None)


def test_spawned_worker_is_bounded_and_late_result_cannot_authorize_stop(tmp_path):
    started, released = tmp_path / "started", tmp_path / "released"
    ticket = _ticket()
    ready = threading.Event()
    worker = BoundedSafetyWorker(_wait_for_release, on_result=lambda _reply: ready.set())
    try:
        assert worker.submit(ticket, (str(started), str(released)))
        deadline = time.monotonic() + 10
        while not started.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert started.exists()
        before = time.monotonic()
        assert worker.poll() is None
        assert not worker.submit(ticket, (str(started), str(released)))
        assert time.monotonic() - before < 0.1
        # A stop, an invalid scan, and a frozen-clock steady expiry all revoke
        # the outstanding ticket even while the computation is blocked.
        assert not _permits(ticket, command=Velocity(0, 0), command_generation=2)
        assert not _permits(ticket, scan_generation=3)
        assert not _permits(ticket, steady=10.201, ros=20.0)
        released.write_text("release")
        assert ready.wait(timeout=10), "the queued reply did not wake the consumer"
        reply = None
        deadline = time.monotonic() + 10
        while reply is None and time.monotonic() < deadline:
            reply = worker.poll()
            time.sleep(0.01)
        assert reply is not None and reply.error is None and reply.value == "finished"
        assert not _permits(reply.ticket, command=Velocity(0, 0), command_generation=2)
    finally:
        worker.close()


def test_spawned_worker_processes_real_scan_with_future_motion_allowance():
    fixture = json.loads(
        (Path(__file__).parents[1] / "fixtures/fortress_scan_seam.json").read_text()
    )
    raw = fixture["scan"]
    stamp = raw["stamp"]
    profile = DeviceProfile(
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)],
        self_mask_calibrated=True,
        braking_deceleration=0.3,
        lateral_braking_deceleration=0.3,
        yaw_braking_deceleration=0.5,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.8,
    )
    scan = ScanObservation(
        tuple(raw["ranges"]),
        raw["angle_min"],
        raw["angle_increment"],
        raw["range_min"],
        raw["range_max"],
        stamp,
        "ros",
        Pose2D(*map(float, fixture["sensor_origin"])),
        0.0,
        "base_link",
        stamp,
        True,
        0.01,
        False,
        True,
    )
    observation = filter_self(scan, profile)
    history = TimedPoseBuffer(max_gap=0.4, max_samples=4, max_variance=0.1)
    assert history.add(TimedPoseSample(stamp, Pose2D(0, 0, 0), (0.001,) * 3, "epoch-a"))
    steady = time.monotonic()
    work = SafetyWork(
        Velocity(0.05, 0),
        BodyTwist(0, 0.0, 0),
        profile,
        (StampedObservation(observation, steady),),
        history.snapshot(),
        stamp + 0.2,
        steady,
        0.4,
        0.3,
        0.5,
        0.5,
        False,
        interval_bounds=IntervalVelocityBounds(6.0, 0.25),
        physical_cap_qualified=True,
    )
    assert (
        evaluate_safety_work(replace(work, physical_cap_qualified=False)).decision.reason
        == "motion_uncertainty_unqualified"
    )
    missing_brake = profile.model_copy(update={"lateral_braking_deceleration": None})
    assert (
        evaluate_safety_work(replace(work, profile=missing_brake)).decision.reason
        == "motion_braking_unqualified"
    )
    worker = BoundedSafetyWorker()
    try:
        assert worker.submit(_ticket(), work)
        result = None
        deadline = time.monotonic() + 10
        while result is None and time.monotonic() < deadline:
            result = worker.poll()
            time.sleep(0.01)
        assert result is not None and result.error is None
        assert result.value.decision.allowed
        assert result.value.uncertainty_margin >= (0.3 + 0.5 * 0.5) * 0.199
        assert result.value.reference_stamp == stamp
        first_details = result.value.diagnostics
        assert first_details["interval_linear_upper"] == 6.0
        assert first_details["assumed_instantaneous_linear_cap"] == 0.3
        assert first_details["motion_uncertainty_sweep_margin"] > 0.42
        # A narrow interval-average estimate does not shrink the instantaneous cap.
        assert worker.submit(
            _ticket(), replace(work, interval_bounds=IntervalVelocityBounds(0.001, 0.001))
        )
        again = None
        deadline = time.monotonic() + 10
        while again is None and time.monotonic() < deadline:
            again = worker.poll()
            time.sleep(0.01)
        assert again is not None and again.error is None and again.value.decision.allowed
        assert again.value.diagnostics["interval_linear_upper"] == 0.001
        assert (
            again.value.diagnostics["motion_uncertainty_sweep_margin"]
            == first_details["motion_uncertainty_sweep_margin"]
        )
    finally:
        worker.close()


def test_spawned_worker_decodes_bounded_raw_scan_and_retains_sensor_transform():
    fixture = json.loads(
        (Path(__file__).parents[1] / "fixtures/fortress_scan_seam.json").read_text()
    )
    raw = fixture["scan"]
    stamp = raw["stamp"]
    profile = DeviceProfile(
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)],
        self_mask_calibrated=True,
        braking_deceleration=0.3,
        lateral_braking_deceleration=0.3,
        yaw_braking_deceleration=0.5,
        reaction_time=0.2,
        hard_margin=0.02,
        observation_max_age=0.8,
    )
    sensor = tuple(map(float, fixture["sensor_origin"]))
    message = SimpleNamespace(
        source_epoch="raw-boot",
        sequence=1,
        ranges=raw["ranges"],
        angle_min=raw["angle_min"],
        angle_increment=raw["angle_increment"],
        range_min=raw["range_min"],
        range_max=raw["range_max"],
        sensor_x=sensor[0],
        sensor_y=sensor[1],
        sensor_yaw=sensor[2],
        transform_stamp=0.0,
    )
    history = TimedPoseBuffer(max_gap=0.4, max_samples=4, max_variance=0.1)
    assert history.add(TimedPoseSample(stamp, Pose2D(0, 0, 0), (0.001,) * 3, "epoch-a"))
    steady = time.monotonic()
    sample = RawSafetyObservation(message, steady, stamp, steady, stamp)
    work = SafetyWork(
        Velocity(0.05, 0),
        BodyTwist(0, 0, 0),
        profile,
        (sample,),
        history.snapshot(),
        stamp + 0.2,
        steady,
        0.4,
        0.3,
        0.5,
        0.5,
        False,
        interpolate=True,
        max_angular_gap=0.01,
        transform_static=True,
        interval_bounds=IntervalVelocityBounds(6.0, 0.25),
        physical_cap_qualified=True,
    )
    worker = BoundedSafetyWorker()
    try:
        assert worker.submit(_ticket(), work)
        deadline = time.monotonic() + 10
        reply = None
        while reply is None and time.monotonic() < deadline:
            reply = worker.poll()
            time.sleep(0.01)
        assert reply is not None and reply.error is None
        assert reply.value.decision.allowed
        assert reply.value.selected_stamp == stamp
        assert reply.value.selected_received == steady
        assert reply.value.sensor_origin == pytest.approx(sensor)
    finally:
        worker.close()


@pytest.mark.parametrize("prewarm", [False, True], ids=["lazy-submit", "warmup"])
def test_process_start_and_large_result_transport_do_not_block_safety_thread(prewarm):
    started, release = threading.Event(), threading.Event()

    class SlowProcess:
        pid = None

        def start(self):
            started.set()
            release.wait(timeout=5)

    class Channel:
        def cancel_join_thread(self):
            pass

        def close(self):
            pass

    class SlowContext:
        def Queue(self, maxsize):
            return Channel()

        def Process(self, **kwargs):
            return SlowProcess()

    worker = BoundedSafetyWorker(_large_reply)
    worker._context = SlowContext()
    try:
        before = time.monotonic()
        assert worker.warmup() if prewarm else worker.submit(_ticket(), None)
        assert time.monotonic() - before < 0.1
        assert started.wait(timeout=2)
        before = time.monotonic()
        assert worker.poll() is None
        assert time.monotonic() - before < 0.1
    finally:
        worker.close()
        release.set()

    worker = BoundedSafetyWorker(_large_reply)
    try:
        assert worker.submit(_ticket(), None)
        deadline = time.monotonic() + 10
        reply = None
        while reply is None and time.monotonic() < deadline:
            before = time.monotonic()
            reply = worker.poll()
            assert time.monotonic() - before < 0.1
            time.sleep(0.01)
        assert reply is not None and len(reply.value) == 8 * 1024 * 1024
    finally:
        worker.close()


def test_hung_worker_shutdown_is_bounded(tmp_path):
    started, released = tmp_path / "started", tmp_path / "never-released"
    worker = BoundedSafetyWorker(_wait_for_release)
    assert worker.submit(_ticket(), (str(started), str(released)))
    deadline = time.monotonic() + 10
    while not started.exists() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert started.exists()
    before = time.monotonic()
    worker.close()
    assert time.monotonic() - before < 2.0
    deadline = time.monotonic() + 3
    while worker._process.is_alive() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert not worker._process.is_alive()
