"""Original nonempty scan inputs through the complete learning/output chain."""

import pytest
from learning_test_support import learning_runs  # noqa: F401
from test_motion_core_handoff import core  # noqa: F401
from test_startup_learning_cycle import (
    test_full_core_cycle_with_delayed_and_reordered_raw_scans as run_cycle,
)


@pytest.mark.parametrize("gap", ["none", "left", "right", "both"])
def test_nonempty_complete_core_action_saves_and_filters_reference(
    core,  # noqa: F811
    tmp_path,
    monkeypatch,
    gap,
):
    snapshots = []
    snapshot = core._snapshot

    def capture(*args, **kwargs):
        value = snapshot(*args, **kwargs)
        snapshots.append((core.startup.active, core.target.source, value.startup_visibility))
        return value

    monkeypatch.setattr(core, "_snapshot", capture)
    original = core._filtered_scan

    def with_attachment(raw):
        raw.message.ranges[0] = 0.15
        if gap in {"left", "both"}:
            raw.message.ranges[1] = float("nan")
        if gap in {"right", "both"}:
            raw.message.ranges[-1] = float("nan")
        return original(raw)

    monkeypatch.setattr(core, "_filtered_scan", with_attachment)
    run_cycle(core, tmp_path, monkeypatch, (0.0,), False, False, "completed")
    assert any(
        active and source == "startup_learning" and visibility is not None
        for active, source, visibility in snapshots
    )
    assert any(not active and visibility is None for active, source, visibility in snapshots)
    assert all(0 <= v.vx <= 0.05 and v.wz == 0 for v, _ in core.node.emitted)
    record = core._learning_client.record
    assert record.action.consumed and record.model.points
    assert (
        record.action.deadline_steady_ns - record.action.first_submitted_steady_ns == 3_000_000_000
    )
    assert core._latest_masked.rays[0].kind == "self"
    restored = core._learning_client.run.client()
    assert restored.record.model == record.model and not restored.reserve_action()


@pytest.mark.parametrize("direction", ["front", "rear"])
def test_loaded_disabled_direction_allows_over_50_percent_nonempty_learning(
    core,  # noqa: F811
    tmp_path,
    monkeypatch,
    direction,
):
    import math

    from control_process_test_support import generic_motion_profile
    from senior_motion_control.motion_profile import configure_startup_learning

    profile = generic_motion_profile()
    core.profile = configure_startup_learning(
        profile, front_stop=direction != "front", rear_stop=direction != "rear"
    )
    seen = []
    snapshot = core._snapshot

    def capture(*args, **kwargs):
        value = snapshot(*args, **kwargs)
        if value.startup_visibility is not None:
            seen.append(value.startup_visibility)
        return value

    monkeypatch.setattr(core, "_snapshot", capture)
    original = core._filtered_scan

    def with_attachment(raw):
        message = raw.message
        center = 0 if direction == "front" else math.pi
        for i in range(len(message.ranges)):
            angle = message.sensor_yaw + message.angle_min + i * message.angle_increment
            if abs(math.remainder(angle - center, math.tau)) < math.pi / 10:
                message.ranges[i] = 0.15
        return original(raw)

    monkeypatch.setattr(core, "_filtered_scan", with_attachment)
    run_cycle(core, tmp_path, monkeypatch, (0.0,), False, False, "completed")
    assert (
        core._learning_client.record.action.consumed and core._learning_client.record.model.points
    )
    assert seen
    assert any((p.front_occluded if direction == "front" else p.rear_occluded) > 0.5 for p in seen)
    assert all(not (p.front_stop if direction == "front" else p.rear_stop) for p in seen)


def test_false_switches_still_block_real_obstacle_in_actual_core(
    core,  # noqa: F811
    tmp_path,
    monkeypatch,
):
    from senior_motion_control.motion_profile import configure_startup_learning

    core.profile = configure_startup_learning(core.profile, front_stop=False, rear_stop=False)
    original = core._filtered_scan

    def with_obstacle(raw):
        raw.message.ranges[0] = 0.15  # Approved small self occlusion behind.
        raw.message.ranges[360] = 0.38  # Actual obstacle outside the physical body.
        return original(raw)

    monkeypatch.setattr(core, "_filtered_scan", with_obstacle)
    tick = core.tick
    calls = [0]

    class ProbeFinished(Exception):
        pass

    def bounded_tick():
        tick()
        calls[0] += 1
        if calls[0] == 22:
            raise ProbeFinished

    monkeypatch.setattr(core, "tick", bounded_tick)
    with pytest.raises(ProbeFinished):
        run_cycle(core, tmp_path, monkeypatch, (0.0,), False, False, "completed")
    assert core._latest_masked.rays[360].kind == "hit"
    assert core.node.emitted and all(v.vx == v.wz == 0 for v, _ in core.node.emitted)
    assert any("stop_envelope_blocked" in reason for _, reason in core.node.emitted)
    assert not core._learning_client.record.action.consumed
