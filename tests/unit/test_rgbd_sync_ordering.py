"""Out-of-order buffers are supported; a stored frame must be consumed once."""

from types import SimpleNamespace as N

import pytest

from senior_control.runtime.diagnostics import RuntimeDiagnostics
from senior_control.vision.synchronizer import DepthFrame, ImageFrame, RgbdSynchronizer


def rgb(stamp):
    return ImageFrame(stamp, 1, 1, "rgb8", bytes(3))


def depth(stamp):
    return DepthFrame(stamp, 1, 1, [1.0])


@pytest.mark.parametrize("rgb_first", [True, False])
def test_out_of_order_consumes_match_and_older_but_preserves_newer(rgb_first):
    s = RgbdSynchronizer(max_skew_ms=10)
    offer = s.offer_rgb if rgb_first else s.offer_depth
    match = s.offer_depth if rgb_first else s.offer_rgb
    frame = rgb if rgb_first else depth
    counterpart = depth if rgb_first else rgb
    for t in (2.0, 0.5, 1.0):
        assert offer(frame(t)) is None
    pair = match(counterpart(1.0))
    assert pair is not None and pair.rgb.stamp == pair.depth.stamp == 1.0
    assert match(counterpart(1.0)) is None
    newer = match(counterpart(2.0))
    assert newer is not None and newer.rgb.stamp == newer.depth.stamp == 2.0
    assert match(counterpart(2.0)) is None


def test_queue_depth_wait_and_drops_survive_arbitrary_index_consumption(monkeypatch):
    import vehicle_core.diagnostics as module

    clock = [0.0]
    d = RuntimeDiagnostics(lambda *_: None)
    monkeypatch.setattr(module, "process_diagnostics", lambda: d)
    monkeypatch.setattr(module, "time", N(monotonic=lambda: clock[0]))
    s = RgbdSynchronizer(max_skew_ms=10)
    s.offer_rgb(rgb(2.0))
    clock[0] = 1.0
    s.offer_rgb(rgb(0.5))
    s.offer_rgb(rgb(1.0))
    clock[0] = 2.0
    assert s.offer_depth(depth(1.0)) is not None
    state = d.snapshot("vision.rgb_depth_pair.rgb")
    assert state.pending == 1 and state.dropped_total == 1 and state.overwritten_total == 0
    assert state.oldest_wait_ms == 2000.0
    assert s.offer_depth(depth(2.0)) is not None
    assert d.snapshot("vision.rgb_depth_pair.rgb").pending == 0


def test_reoffered_same_object_is_consumed_once_and_duplicate_slot_is_dropped(monkeypatch):
    import vehicle_core.diagnostics as module

    d = RuntimeDiagnostics(lambda *_: None)
    monkeypatch.setattr(module, "process_diagnostics", lambda: d)
    s = RgbdSynchronizer(max_skew_ms=10)
    s.offer_rgb(rgb(2.0))
    same = rgb(1.0)
    s.offer_rgb(same)
    s.offer_rgb(same)
    assert s.offer_depth(depth(1.0)) is not None
    assert s.offer_depth(depth(1.0)) is None
    state = d.snapshot("vision.rgb_depth_pair.rgb")
    assert state.pending == 1 and state.dropped_total == 1
    assert s.offer_depth(depth(2.0)) is not None


def test_unusable_nan_slot_is_not_retained_as_newer_than_the_match(monkeypatch):
    import vehicle_core.diagnostics as module

    d = RuntimeDiagnostics(lambda *_: None)
    monkeypatch.setattr(module, "process_diagnostics", lambda: d)
    s = RgbdSynchronizer(max_skew_ms=10)
    for stamp in (2.0, float("nan"), 1.0):
        s.offer_rgb(rgb(stamp))
    assert s.offer_depth(depth(1.0)) is not None
    state = d.snapshot("vision.rgb_depth_pair.rgb")
    assert state.pending == 1 and state.dropped_total == 1
    assert s.offer_depth(depth(2.0)) is not None
    assert d.snapshot("vision.rgb_depth_pair.rgb").pending == 0
