"""Slow scan jobs retain original proofs and cannot cross source/frame/filter scopes."""

from concurrent.futures import Future
from dataclasses import replace
from functools import partial

import pytest
from senior_robot.motion_core_perception import ScanGeometry, prepare_scan_geometry
from test_directional_constraints import engine, snapshot
from test_motion_core_perception import observation

from senior_control.config import DeviceProfile
from senior_control.motion.constraints import NoReturnPolicy
from senior_control.safety.self_filter import filter_self


class ScheduledWorker:
    def __init__(self):
        self.jobs = []

    def submit(self, task):
        assert not self.jobs or self.jobs[-1][0].done(), "only one geometry flight is allowed"
        future = Future()
        self.jobs.append((future, task))
        return future

    def finish(self):
        future, task = self.jobs[-1]
        future.set_result(task())

    def close(self):
        pass


def offer(scheduler, key, stamp):
    import math

    raw = replace(
        observation([10.0] * 720).scan,
        angle_increment=2 * math.pi / 720,
        stamp=stamp,
        reference_stamp=stamp,
        transform_stamp=stamp,
    )
    filtered = filter_self(raw, DeviceProfile())
    task = partial(
        prepare_scan_geometry,
        filtered,
        0.02,
        NoReturnPolicy.UNKNOWN,
        dict(
            stamp_steady=stamp,
            deadline_steady=stamp + 1,
            source_epoch=key[0],
            frame_id="local",
            revision=key[1],
            forecast_horizon_s=10,
            basis="synthetic full scan",
        ),
    )
    return scheduler.update(key, task, snapshot=raw)


@pytest.mark.parametrize("changed", ["source", "frame", "filter"])
def test_pending_is_bounded_and_old_scope_is_discarded_before_eventual_legal_output(changed):
    worker = ScheduledWorker()
    scheduler = ScanGeometry(worker)
    first = ("scan", 1, "frame-1", "filter-1")
    latest = ("scan", 3, "frame-1", "filter-1")
    assert offer(scheduler, first, 0) is None
    assert offer(scheduler, ("scan", 2, "frame-1", "filter-1"), 0.05) is None
    assert offer(scheduler, latest, 0.1) is None
    assert len(worker.jobs) == 1 and scheduler.pending[0] == latest
    worker.finish()
    original = offer(scheduler, latest, 0.1)
    assert original.key == first and original.perception.deadline_steady == 1
    assert original.snapshot.stamp == 0
    assert len(worker.jobs) == 2  # Completed old proof is usable while the latest job starts.
    scope = {
        "source": ("replacement", 4, "frame-1", "filter-1"),
        "frame": ("scan", 4, "frame-2", "filter-1"),
        "filter": ("scan", 4, "frame-1", "filter-2"),
    }[changed]
    assert offer(scheduler, scope, 0.15) is None
    worker.finish()  # A result from the retired scope arrives late.
    current = (scope[0], 5, scope[2], scope[3])
    assert offer(scheduler, current, 0.2) is None
    assert scheduler.ready is None and len(worker.jobs) == 3
    worker.finish()
    result = offer(scheduler, current, 0.2)
    assert result.key == current and result.snapshot.stamp == 0.2
    assert result.perception.deadline_steady == 1.2
    ctx = replace(snapshot(0.2), perception=result.perception)
    assert engine().refine(ctx, 0.2).permit is not None
