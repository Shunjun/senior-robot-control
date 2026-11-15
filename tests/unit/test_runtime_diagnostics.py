import logging

import pytest

from senior_control.runtime.diagnostics import QueueSnapshot, RuntimeDiagnostics
from senior_control.sensors.policy import default_policy


def test_queue_warning_immediate_limited_and_recovers_without_control_side_effects():
    emitted = []
    d = RuntimeDiagnostics(lambda level, data: emitted.append((level, data)))
    d.queue(QueueSnapshot("native", 13, 16, oldest_wait_ms=1), 0.0)
    assert emitted == []  # queue mutation performs no log IO
    d.pump(0.0)
    assert emitted[-1][0] == logging.WARNING
    d.queue(QueueSnapshot("native", 14, 16, oldest_wait_ms=1), 1.0)
    d.pump(1.0)
    assert len(emitted) == 1
    d.pump(5.0)
    assert len([level for level, _ in emitted if level == logging.WARNING]) == 2
    d.queue(QueueSnapshot("native", 0, 16, oldest_wait_ms=0), 6.0)
    d.pump(10.0)
    assert any(data["event"] == "queue_recovered" for _, data in emitted)


def test_single_slot_unknown_depth_and_exact_drop_fields():
    out = []
    d = RuntimeDiagnostics(lambda level, data: out.append(data))
    d.queue(QueueSnapshot("latest", 1, 1, oldest_wait_ms=10, overwritten_total=2), 0)
    d.queue(QueueSnapshot("dds", None, 4), 0)
    d.pump(0)
    assert out == []
    d.queue(QueueSnapshot("latest", 1, 1, oldest_wait_ms=301, overwritten_total=3), 1)
    d.pump(1)
    assert out[-1]["pending"] == 1 and out[-1]["overwritten_total"] == 3
    assert out[-1]["dropped_total"] is None
    assert d.snapshot("dds").pending is None


def test_age_raw_dedup_separate_compute_and_five_second_debug():
    out = []
    d = RuntimeDiagnostics(lambda level, data: out.append((level, data)))
    p = default_policy()
    age = p.age(stamp_ros=9.5, received_ros=10, received_steady=20, now_ros=10, now_steady=20)
    d.age("wheel", age, key=("boot", 1))
    d.age("wheel", age, key=("boot", 1))
    d.duration("compute", 0.002)
    d.pump(0)
    assert not out
    d.pump(5)
    payload = next(data for level, data in out if level == logging.DEBUG)
    assert payload["streams"]["sensor.raw.wheel"]["count"] == 1
    assert payload["streams"]["sensor.raw.wheel"]["over_soft"] == 1
    assert payload["streams"]["duration.compute"]["max"] == 0.002


def test_pending_counter_waits_overwrites_and_running_are_distinct():
    from senior_control.runtime.diagnostics import PendingCounter

    d = RuntimeDiagnostics(lambda *_: None)
    p = PendingCounter("ipc", 4, diagnostics=d)
    p.enqueue("a", 0.0)
    p.enqueue("b", 0.1)
    p.start("a", 0.2)
    snap = p.snapshot(0.3)
    assert snap.pending == 1 and snap.running == 1
    assert snap.oldest_wait_ms == pytest.approx(200.0)
    p.discard("b", 0.4, overwritten=True)
    p.finish("a", 0.5)
    assert p.snapshot(0.5).pending == 0
    assert p.snapshot(0.5).overwritten_total == 1


def test_transient_crossing_is_not_lost_when_queue_drains_before_reporter():
    out = []
    d = RuntimeDiagnostics(lambda level, data: out.append((level, data)))
    d.queue(QueueSnapshot("burst", 4, 4, oldest_wait_ms=1), 0)
    d.queue(QueueSnapshot("burst", 0, 4, oldest_wait_ms=0), 0.01)
    d.pump(0.1)
    assert out[0][0] == logging.WARNING
    assert (
        out[0][1]["pending"] == 0 and out[0][1]["trigger_pending"] == 4 and out[0][1]["peak"] == 4
    )
    d.pump(5.1)
    assert any(data["event"] == "queue_recovered" for _, data in out)


@pytest.mark.parametrize("capacity,threshold", [(4, 3), (16, 12), (64, 32), (1024, 32)])
def test_queue_count_thresholds_are_diagnostic_only(capacity, threshold):
    from senior_control.runtime.diagnostics import QueueWarningPolicy

    out = []
    d = RuntimeDiagnostics(lambda level, data: out.append((level, data)))
    assert QueueWarningPolicy().threshold(capacity) == threshold
    d.queue(QueueSnapshot("boundary", threshold, capacity), 0)
    d.pump(0)
    assert not out
    d.queue(QueueSnapshot("boundary", threshold + 1, capacity), 0.1)
    d.pump(0.1)
    assert out[-1][0] == logging.WARNING


def test_single_slot_wait_grows_without_new_enqueues_and_logger_cannot_recurse():
    out = []

    def sink(level, data):
        out.append(data)
        d.queue(QueueSnapshot("logs", 99, 1024), 0.4)
        d.pump(0.4)

    d = RuntimeDiagnostics(sink)
    d.queue(QueueSnapshot("latest", 1, 1, oldest_wait_ms=0), 0)
    d.pump(0.4)
    assert len(out) == 1 and out[0]["queue"] == "latest"
    assert out[0]["oldest_wait_ms"] == 400


def test_multiple_owners_use_worst_queue_and_released_owner_is_unknown():
    import gc

    from senior_control.runtime.diagnostics import PendingCounter

    out = []
    d = RuntimeDiagnostics(lambda level, data: out.append((level, data)))
    busy = PendingCounter("shared", 4, diagnostics=d)
    quiet = PendingCounter("shared", 4, diagnostics=d)
    for n in range(4):
        busy.enqueue(n, 0.0)
    quiet.enqueue("a", 0.0)
    quiet.finish("a", 0.0)
    d.pump(0.0)
    assert out[-1][1]["pending"] == 4
    del busy
    gc.collect()
    d.pump(5.0)
    assert not any(level == logging.WARNING for level, _ in out[1:])
    del quiet
    gc.collect()
    d.pump(10.0)
    assert d.snapshot("shared").pending is None


def test_diagnostics_configuration_rejects_unknown_fields_and_reaches_actual_agent_env():
    from pydantic import TypeAdapter, ValidationError

    from senior_control.agent.composition import AgentConfig
    from senior_control.runtime.diagnostics import DiagnosticsSettings, diagnostics_from_environment

    settings = diagnostics_from_environment(
        {
            "SENIOR_DIAGNOSTICS__QUEUE__COUNT": "7",
            "SENIOR_DIAGNOSTICS__QUEUE__RATIO": ".5",
            "SENIOR_DIAGNOSTICS__PERIOD_S": "3",
        }
    )
    assert settings.queue.count == 7 and settings.queue.ratio == 0.5 and settings.period_s == 3
    with pytest.raises(ValidationError):
        TypeAdapter(DiagnosticsSettings).validate_python({"queue": {"typo": 3}})
    with pytest.raises(ValidationError):
        diagnostics_from_environment({"SENIOR_DIAGNOSTICS__QUEUE__COUNT": "0"})
    config = AgentConfig.from_environment(
        {
            "SENIOR_ROBOT_ID": "robot",
            "SENIOR_CLOUD_URL": "wss://localhost",
            "SENIOR_DEVICE_TOKEN": "test",
            "SENIOR_DATABASE_PATH": "/tmp/diagnostic-test.db",
            "SENIOR_AGENT_SOCKET": "/tmp/diagnostic-test.sock",
            "SENIOR_DIAGNOSTICS__QUEUE__COUNT": "7",
        }
    )
    assert config.diagnostics.queue.count == 7
