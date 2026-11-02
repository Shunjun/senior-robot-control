import time

from test_independent_motion import frame, scene

from senior_control.vision.independent_motion import Profile
from senior_control.vision.rgbd_spawn_worker import SpawnWorker


def test_spawn_worker_returns_small_result_without_pixels():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        item = frame(1, image, depth)
        assert worker.submit(
            item,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        deadline = time.monotonic() + 10
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None
        assert result.reason == "initializing"
        assert not hasattr(result, "pair")
        assert result.hard_error_qualified is False
    finally:
        worker.close()


def test_pending_replaced_and_invalidation_revokes_inflight():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        for seq in (1, 2, 3):
            assert worker.submit(
                frame(seq, image, depth),
                now_ros_ns=1_400_000_000,
                now_steady=1.4,
                ros_deadline_ns=1_400_000_000,
                steady_deadline=1.4,
            )
        assert worker.dropped == 1
        worker.invalidate()
        assert worker.dropped == 2
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            assert worker.poll(now_ros_ns=1_400_000_000, now_steady=1.4) is None
            time.sleep(0.01)
    finally:
        worker.close()


def test_hard_error_flag_cannot_be_set_by_caller():
    import pytest

    from senior_control.vision.rgbd_spawn_worker import CompactResult

    with pytest.raises(TypeError):
        CompactResult(1, 1, "candidate", (), None, 0, 0, None, None, hard_error_qualified=True)


def test_candidate_preserves_pair_dependencies_without_pixel_bodies():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        for seq in (1, 2):
            assert worker.submit(
                frame(seq, image, depth),
                now_ros_ns=1_200_000_000,
                now_steady=1.2,
                ros_deadline_ns=1_300_000_000,
                steady_deadline=1.3,
            )
            deadline = time.monotonic() + 10
            result = None
            while result is None and time.monotonic() < deadline:
                result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
                time.sleep(0.01)
            assert result is not None
        assert result.reason == "candidate"
        assert result.tf_epochs == ("tf", "tf")
        assert result.dependencies == (("camera",), ("camera",))
        assert len(result.headers) == 2
    finally:
        worker.close()


def test_worker_death_produces_explicit_invalid_and_closes_admission():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        assert worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        worker._process.terminate()
        worker._process.join(2)
        result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
        assert result is not None and result.reason == "worker_failed"
        assert not worker.submit(
            frame(2, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
    finally:
        worker.close()


def test_inflight_timeout_reports_invalid_without_waiting_for_worker():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        assert worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        result = worker.poll(now_ros_ns=1_300_000_001, now_steady=1.31)
        assert result is not None and result.reason == "worker_timeout"
        assert result.hard_error_qualified is False
    finally:
        worker.close()


def test_parent_supplies_monotonic_generation_and_sequence_floor():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=42, sequence_floor=100)
    try:
        assert worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        deadline = time.monotonic() + 10
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None and (result.generation, result.sequence) == (42, 101)
    finally:
        worker.close()


def test_parent_can_retain_last_issued_sequence_after_submission():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=4, sequence_floor=8)
    try:
        assert worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        assert worker.sequence_frontier == 9
    finally:
        worker.close()


def test_submission_cannot_extend_original_capture_deadlines():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        assert not worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=5_000_000_000,
            steady_deadline=5.0,
        )
    finally:
        worker.close()


def test_candidate_expires_when_first_capture_deadline_passes():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        assert worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_400_000_000,
            steady_deadline=1.4,
        )
        deadline = time.monotonic() + 10
        first = None
        while first is None and time.monotonic() < deadline:
            first = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert first is not None
        assert worker.submit(
            frame(2, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_500_000_000,
            steady_deadline=1.5,
        )
        time.sleep(0.5)
        result = worker.poll(now_ros_ns=1_410_000_000, now_steady=1.41)
        assert result is not None and result.reason == "capture_expired"
    finally:
        worker.close()


def test_malformed_worker_output_fails_closed_instead_of_raising():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        worker._results.put_nowait("malformed")
        deadline = time.monotonic() + 3
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None and result.reason == "malformed_worker_result"
    finally:
        worker.close()


def test_wrong_ticket_does_not_release_live_worker_slot():
    from senior_control.vision.rgbd_spawn_worker import CompactResult

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        assert worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        worker._results.put_nowait(CompactResult(1, 99, "candidate", (), None, 0, 0, None, None))
        deadline = time.monotonic() + 3
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None and result.reason == "malformed_worker_result"
        assert worker._inflight is not None
    finally:
        worker.close()


def test_worker_cannot_claim_hard_error_qualification():
    from senior_control.vision.rgbd_spawn_worker import CompactResult

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        assert worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        forged = CompactResult(1, 1, "candidate", (), None, 0, 0, None, None)
        object.__setattr__(forged, "hard_error_qualified", True)
        worker._results.put_nowait(forged)
        deadline = time.monotonic() + 3
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None and not result.hard_error_qualified
        assert result.reason == "malformed_worker_result"
    finally:
        worker.close()


def test_nonfinite_candidate_result_is_rejected():
    from senior_control.vision.rgbd_spawn_worker import CompactResult

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        assert worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        bogus = (
            (float("nan"), 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        worker._results.put_nowait(CompactResult(1, 1, "candidate", (), bogus, 1, 1, 0.1, 0.1))
        deadline = time.monotonic() + 3
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None and result.reason == "malformed_worker_result"
    finally:
        worker.close()


def test_finite_nonrigid_candidate_is_rejected():
    from senior_control.vision.rgbd_spawn_worker import CompactResult

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        first, second = frame(1, image, depth), frame(2, image, depth)
        assert worker.submit(
            first,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        groups = tuple(
            (f.rgb_header, f.depth_header, f.rgb_info_header, f.depth_info_header)
            for f in (first, second)
        )
        scale = (
            (2.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        worker._results.put_nowait(CompactResult(1, 1, "candidate", groups, scale, 1, 1, 0.1, 0.1))
        deadline = time.monotonic() + 3
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None and result.reason == "malformed_worker_result"
    finally:
        worker.close()


def test_candidate_must_name_exact_capture_issued_to_worker():
    from senior_control.vision.rgbd_spawn_worker import CompactResult

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        first, second = frame(1, image, depth), frame(2, image, depth)
        assert worker.submit(
            first,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        groups = tuple(
            (f.rgb_header, f.depth_header, f.rgb_info_header, f.depth_info_header)
            for f in (first, second)
        )
        identity = (
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        worker._results.put_nowait(
            CompactResult(1, 1, "candidate", groups, identity, 1, 1, 0.1, 0.1)
        )
        deadline = time.monotonic() + 3
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None and result.reason == "malformed_worker_result"
    finally:
        worker.close()


def test_candidate_rejects_fit_counts_above_feature_budget():
    from senior_control.vision.rgbd_spawn_worker import CompactResult

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    try:
        first, second = frame(1, image, depth), frame(2, image, depth)
        assert worker.submit(
            second,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        groups = tuple(
            (f.rgb_header, f.depth_header, f.rgb_info_header, f.depth_info_header)
            for f in (first, second)
        )
        identity = (
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        worker._results.put_nowait(
            CompactResult(1, 1, "candidate", groups, identity, 501, 1, 0.1, 0.1)
        )
        deadline = time.monotonic() + 3
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None and result.reason == "malformed_worker_result"
    finally:
        worker.close()


def test_hung_worker_timeout_restarts_with_fresh_capture_floor():
    import os
    import signal

    from senior_control.vision.rgbd_spawn_worker import CompactResult

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    old = worker._process
    try:
        os.kill(old.pid, signal.SIGSTOP)
        assert worker.submit(
            frame(1, image, depth),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
            ros_deadline_ns=1_300_000_000,
            steady_deadline=1.3,
        )
        worker._results.put_nowait(CompactResult(1, 1, "initializing", (), None, 0, 0, None, None))
        result = worker.poll(now_ros_ns=1_300_000_001, now_steady=1.31)
        assert result is not None and result.reason == "worker_timeout"
        assert not old.is_alive()
        assert worker._process.pid != old.pid
        assert not worker.submit(
            frame(2, image, depth),
            now_ros_ns=1_300_000_001,
            now_steady=1.31,
            ros_deadline_ns=1_400_000_000,
            steady_deadline=1.4,
        )
        assert worker.submit(
            frame(4, image, depth),
            now_ros_ns=1_400_000_000,
            now_steady=1.4,
            ros_deadline_ns=1_700_000_000,
            steady_deadline=1.7,
        )
        deadline = time.monotonic() + 10
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_400_000_000, now_steady=1.4)
            time.sleep(0.01)
        assert result is not None and result.reason == "initializing"
        assert result.generation > 1 and result.sequence > 1
        replacement = worker._process
        worker.close()
        assert not replacement.is_alive()
    finally:
        if old.is_alive():
            os.kill(old.pid, signal.SIGCONT)
            old.terminate()
            old.join(2)
        worker.close()


def test_repeated_hung_large_frames_reclaim_ipc_threads_and_descriptors():
    import multiprocessing as mp
    import os
    import signal
    import threading

    import numpy as np

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image = np.ones((480, 640), dtype=np.uint8)
    depth = np.ones((480, 640), dtype=np.float32)
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    initial_threads = len(threading.enumerate())
    initial_fds = len(os.listdir("/dev/fd"))
    try:
        for i in range(10):
            seq = 1 + i * 10
            ros = 1_000_000_000 + seq * 100_000_000
            steady = 1.0 + seq * 0.1
            old = worker._process
            os.kill(old.pid, signal.SIGSTOP)
            assert worker.submit(
                frame(seq, image, depth),
                now_ros_ns=ros,
                now_steady=steady,
                ros_deadline_ns=ros + 300_000_000,
                steady_deadline=steady + 0.3,
            )
            time.sleep(0.05)
            result = worker.poll(now_ros_ns=ros + 300_000_001, now_steady=steady + 0.31)
            assert result is not None and result.reason == "worker_timeout"
            assert not old.is_alive()
        assert len(threading.enumerate()) <= initial_threads + 1
        assert len(os.listdir("/dev/fd")) <= initial_fds + 4
        assert len(mp.active_children()) <= 1
    finally:
        worker.close()


def test_worker_failure_restarts_with_fresh_capture_floor():
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    old = worker._process
    try:
        old.terminate()
        old.join(2)
        result = worker.poll(now_ros_ns=1_300_000_001, now_steady=1.31)
        assert result is not None and result.reason == "worker_failed"
        assert worker._process.pid != old.pid
        assert not worker.submit(
            frame(2, image, depth),
            now_ros_ns=1_300_000_001,
            now_steady=1.31,
            ros_deadline_ns=1_400_000_000,
            steady_deadline=1.4,
        )
        assert worker.submit(
            frame(4, image, depth),
            now_ros_ns=1_400_000_000,
            now_steady=1.4,
            ros_deadline_ns=1_700_000_000,
            steady_deadline=1.7,
        )
        deadline = time.monotonic() + 10
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_400_000_000, now_steady=1.4)
            time.sleep(0.01)
        assert result is not None and result.reason == "initializing"
    finally:
        worker.close()


def _partial_result_worker(profile, work, output):
    import os
    import signal
    import struct

    os.write(output.fileno(), struct.pack("!i", 1000) + b"x")
    os.kill(os.getpid(), signal.SIGSTOP)


def test_partial_result_cannot_block_poll_or_worker_restart(monkeypatch):
    import os
    import signal
    import threading

    import senior_control.vision.rgbd_spawn_worker as module

    original = module._run
    monkeypatch.setattr(module, "_run", _partial_result_worker)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    old = worker._process
    old_reader = worker._reader_thread
    polling = None
    try:
        _, status = os.waitpid(old.pid, os.WUNTRACED)
        assert os.WIFSTOPPED(status)
        results = []
        polling = threading.Thread(
            target=lambda: results.append(worker.poll(now_ros_ns=1_200_000_000, now_steady=1.2)),
            daemon=True,
        )
        polling.start()
        polling.join(0.3)
        assert not polling.is_alive(), "poll blocked on an incomplete result"
        assert results == [None]
        monkeypatch.setattr(module, "_run", original)
        os.kill(old.pid, signal.SIGKILL)
        old.join(2)
        result = worker.poll(now_ros_ns=1_300_000_000, now_steady=1.3)
        assert result is not None and result.reason == "worker_failed"
        assert worker._process.pid != old.pid
        assert not old_reader.is_alive()
        image, depth = scene()
        assert worker.submit(
            frame(4, image, depth),
            now_ros_ns=1_400_000_000,
            now_steady=1.4,
            ros_deadline_ns=1_700_000_000,
            steady_deadline=1.7,
        )
        deadline = time.monotonic() + 10
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_400_000_000, now_steady=1.4)
            time.sleep(0.01)
        assert result is not None and result.reason == "initializing"
    finally:
        if old.is_alive():
            os.kill(old.pid, signal.SIGKILL)
            old.join(2)
        # Release the old implementation's blocked reader on failure.
        if polling is not None and polling.is_alive():
            polling.join(2)
        worker.close()


def test_reader_thread_start_failure_reaps_spawned_child_and_ipc(monkeypatch):
    import multiprocessing as mp
    import os
    import threading

    import pytest

    import senior_control.vision.rgbd_spawn_worker as module

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    warmup = SpawnWorker(profile, generation=1, sequence_floor=0)
    warmup.close()
    original_children = {child.pid for child in mp.active_children()}
    original_fds = len(os.listdir("/dev/fd"))

    class FailingThread(threading.Thread):
        def start(self):
            raise RuntimeError("reader start failed")

    monkeypatch.setattr(module, "Thread", FailingThread)
    try:
        with pytest.raises(RuntimeError, match="reader start failed"):
            SpawnWorker(profile, generation=1, sequence_floor=0)
        leaked = [child for child in mp.active_children() if child.pid not in original_children]
        assert not leaked
        assert len(os.listdir("/dev/fd")) <= original_fds
    finally:
        for child in mp.active_children():
            if child.pid not in original_children:
                child.kill()
                child.join(2)


def _corrupt_result_worker(profile, work, output):
    import os
    import signal

    output.send_bytes(b"not a pickle")
    os.kill(os.getpid(), signal.SIGSTOP)


def test_invalid_serialized_result_reports_failure_and_restarts(monkeypatch):
    import os
    import signal

    import senior_control.vision.rgbd_spawn_worker as module

    original = module._run
    monkeypatch.setattr(module, "_run", _corrupt_result_worker)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    old = worker._process
    try:
        _, status = os.waitpid(old.pid, os.WUNTRACED)
        assert os.WIFSTOPPED(status)
        monkeypatch.setattr(module, "_run", original)
        deadline = time.monotonic() + 5
        result = None
        while result is None and time.monotonic() < deadline:
            result = worker.poll(now_ros_ns=1_300_000_000, now_steady=1.3)
            time.sleep(0.01)
        assert result is not None and result.reason == "worker_failed"
        assert not old.is_alive()
        assert worker._process.pid != old.pid
    finally:
        if old.is_alive():
            os.kill(old.pid, signal.SIGKILL)
            old.join(2)
        worker.close()


def test_reader_start_failure_on_restart_reaps_child_and_closes_admission(monkeypatch):
    import multiprocessing as mp
    import threading

    import senior_control.vision.rgbd_spawn_worker as module

    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    image, depth = scene()
    worker = SpawnWorker(profile, generation=1, sequence_floor=0)
    old = worker._process
    existing = {child.pid for child in mp.active_children()} - {old.pid}

    class FailingThread(threading.Thread):
        def start(self):
            raise RuntimeError("reader start failed")

    monkeypatch.setattr(module, "Thread", FailingThread)
    try:
        old.terminate()
        old.join(2)
        result = worker.poll(now_ros_ns=1_300_000_000, now_steady=1.3)
        assert result is not None and result.reason == "worker_failed"
        assert not [child for child in mp.active_children() if child.pid not in existing]
        assert not worker.submit(
            frame(4, image, depth),
            now_ros_ns=1_400_000_000,
            now_steady=1.4,
            ros_deadline_ns=1_700_000_000,
            steady_deadline=1.7,
        )
    finally:
        for child in mp.active_children():
            if child.pid not in existing:
                child.kill()
                child.join(2)
        worker.close()


def test_process_group_sigint_during_spawn_startup_keeps_child_clean(tmp_path):
    import os
    import select
    import signal
    import subprocess
    import sys
    import textwrap

    marker = tmp_path / "child_started"
    (tmp_path / "sitecustomize.py").write_text(
        textwrap.dedent("""
        import os
        import sys
        import time
        if '--multiprocessing-fork' in sys.argv:
            with open(os.environ['RGBD_SIGNAL_MARKER'], 'w') as f:
                f.write('starting')
            time.sleep(2)
    """)
    )
    script = tmp_path / "signal_group.py"
    script.write_text(
        textwrap.dedent("""
        import os
        import signal
        import threading
        import time
        from senior_control.vision.independent_motion import Profile
        from senior_control.vision.rgbd_spawn_worker import SpawnWorker

        caught = []
        def handler(signum, frame):
            caught.append(signum)

        if __name__ == '__main__':
            signal.signal(signal.SIGINT, handler)
            workers, masks = [], []
            def launch():
                before = signal.pthread_sigmask(signal.SIG_BLOCK, set())
                try:
                    profile = Profile('sim', 'fixture', True, 'texture', 'cal', .1, .1, 2., .1, .1)
                    workers.append(SpawnWorker(profile, generation=1, sequence_floor=0))
                finally:
                    masks.append((before, signal.pthread_sigmask(signal.SIG_BLOCK, set())))
            coordinator = threading.Thread(target=launch)
            coordinator.start()
            coordinator.join()
            worker = workers[0]
            try:
                deadline = time.monotonic() + 10
                marker = os.environ['RGBD_SIGNAL_MARKER']
                while not os.path.exists(marker) and time.monotonic() < deadline:
                    time.sleep(.01)
                print('READY', flush=True)
                time.sleep(2.5)
                print('RUNTIME', worker._process.is_alive(), flush=True)
                deadline = time.monotonic() + 5
                while len(caught) < 2 and time.monotonic() < deadline:
                    time.sleep(.01)
                same_handler = signal.getsignal(signal.SIGINT) is handler
                same_mask = masks[0][0] == masks[0][1]
                print('RESULT', len(caught), worker._process.is_alive(),
                      same_handler, same_mask, flush=True)
            finally:
                worker.close()
                print('CLOSED', worker._process.is_alive(), worker._process.exitcode, flush=True)
    """)
    )
    environment = os.environ.copy()
    environment["RGBD_SIGNAL_MARKER"] = str(marker)
    environment["PYTHONPATH"] = os.pathsep.join((str(tmp_path), os.path.abspath("src")))
    process = subprocess.Popen(
        [sys.executable, str(script)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=environment,
        start_new_session=True,
    )
    try:
        assert process.stdout is not None
        ready, _, _ = select.select([process.stdout], [], [], 10)
        assert ready and process.stdout.readline().strip() == "READY"
        assert marker.exists()
        os.killpg(process.pid, signal.SIGINT)
        ready, _, _ = select.select([process.stdout], [], [], 10)
        assert ready and process.stdout.readline().strip() == "RUNTIME True"
        os.killpg(process.pid, signal.SIGINT)
        stdout, stderr = process.communicate(timeout=10)
        assert process.returncode == 0
        assert "RESULT 2 True True True" in stdout
        assert f"CLOSED False {-signal.SIGTERM}" in stdout
        assert "KeyboardInterrupt" not in stderr
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def test_spawn_error_restores_caller_signal_mask_and_propagates(monkeypatch):
    import multiprocessing as mp
    import signal

    import pytest

    process_type = type(mp.get_context("spawn").Process())
    initial_mask = signal.pthread_sigmask(signal.SIG_BLOCK, set())
    initial_handler = signal.getsignal(signal.SIGINT)

    def fail_start(process):
        assert signal.SIGINT in signal.pthread_sigmask(signal.SIG_BLOCK, set())
        raise RuntimeError("spawn failed")

    monkeypatch.setattr(process_type, "start", fail_start)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.1, 0.1, 2.0, 0.1, 0.1)
    with pytest.raises(RuntimeError, match="spawn failed"):
        SpawnWorker(profile, generation=1, sequence_floor=0)
    assert signal.pthread_sigmask(signal.SIG_BLOCK, set()) == initial_mask
    assert signal.getsignal(signal.SIGINT) is initial_handler
