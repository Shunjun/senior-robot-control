"""Evidence-based diagnostics, with normal and incomplete-data controls."""

import json
from pathlib import Path

from tools.vehicle_motion_analyze import analyze_trace

FIXTURE = Path(__file__).parents[1] / "fixtures/vehicle_motion_trace.json"


def analyze(tmp_path, samples):
    path = tmp_path / "trace.jsonl"
    path.write_text("\n".join(json.dumps(item) for item in samples) + "\n")
    return analyze_trace(path)


def sample(stream, steady, ros, **data):
    return {
        "kind": "sample",
        "stream": stream,
        "received_steady": steady,
        "stamp_ros": ros,
        "data": data,
    }


def test_fixture_distinguishes_measured_failure_windows():
    result = analyze_trace(FIXTURE)
    assert set(result["findings"]) == {
        "clock_pause",
        "alignment_wait",
        "wheel_body_disagreement",
        "permit_starvation",
    }
    assert result["physical_stop"] == "not_evaluated"
    assert result["source_sha256"]
    assert all(item["sample_indices"] for item in result["evidence"])


def test_healthy_interval_aligned_data_is_not_a_fault(tmp_path):
    records = []
    for step in range(7):
        t = step / 10
        records += [
            sample("clock", t, t),
            sample("wheel", t, t, vx=0.5),
            sample("truth", t, t, vx=0.5),
            sample("scan_motion", t, t, vx=0.5, interval_start_ros=t - 0.1, valid=True),
        ]
    assert analyze(tmp_path, records)["findings"] == []


def test_single_mismatch_or_text_is_not_a_diagnosis(tmp_path):
    records = [
        sample("wheel", 1, 1, vx=1.0),
        sample("truth", 1, 1, vx=0.0),
        sample("scan_motion", 1, 1, vx=0.0, interval_start_ros=0.9, valid=True),
        sample("ipc", 1, None, reason="alignment_wait permit_starvation"),
    ]
    assert analyze(tmp_path, records)["findings"] == []


def test_accelerating_wheel_is_averaged_over_scan_interval(tmp_path):
    records = []
    for step in range(11):
        t = step / 10
        records += [sample("wheel", t, t, vx=t), sample("truth", t, t, vx=t)]
        if step >= 2:
            records.append(
                sample("scan_motion", t, t, vx=t - 0.1, interval_start_ros=t - 0.2, valid=True)
            )
    assert "wheel_body_disagreement" not in analyze(tmp_path, records)["findings"]


def test_missing_truth_or_stale_scan_cannot_prove_body_disagreement(tmp_path):
    records = [sample("wheel", i / 10, i / 10, vx=1.0) for i in range(8)]
    records += [
        sample("scan_motion", i / 10, -1, vx=0, interval_start_ros=-1.1, valid=True)
        for i in range(8)
    ]
    result = analyze(tmp_path, records)
    assert "wheel_body_disagreement" not in result["findings"]
    assert "truth" in result["missing_streams"]
    assert result["physical_stop"] == "not_evaluated"


def test_out_of_order_stamps_break_evidence_window(tmp_path):
    records = [sample("clock", i / 10, 1.0) for i in range(3)]
    records += [sample("clock", 0.15, 1.0), sample("clock", 0.3, 0.0)]
    result = analyze(tmp_path, records)
    assert "clock_pause" not in result["findings"]
    assert result["invalid_samples"]


def test_unknown_alignment_or_only_one_expired_result_is_insufficient(tmp_path):
    records = [
        sample("fusion_diagnostic", 1, None, alignment_wait_s=0.258),
        sample(
            "worker_diagnostic",
            1,
            None,
            submitted=10,
            completed=1,
            rejected_expired=1,
            admitted=0,
            pending=True,
        ),
    ]
    assert analyze(tmp_path, records)["findings"] == []


def test_capture_summary_does_not_invent_ros_streams():
    from tools.vehicle_motion_capture import capture_summary

    result = capture_summary({"ipc": 10}, ros_error="rclpy unavailable")
    assert result["complete"] is False
    assert "truth" in result["missing_streams"]
    assert "wheel" in result["missing_streams"]
    assert result["ros_error"] == "rclpy unavailable"


def test_capture_serializes_ros_stamp_without_using_receive_time_as_source():
    from tools.vehicle_motion_capture import ros_sample

    result = ros_sample(
        "wheel",
        {"header": {"stamp": {"sec": 2, "nanosec": 5}}, "twist": {"twist": {"linear": {"x": 0.2}}}},
        100.0,
    )
    assert result["stamp_ros"] == 2.000000005
    assert result["received_steady"] == 100.0
    assert result["data"]["vx"] == 0.2
    missing = ros_sample("wheel", {}, 100.0)
    assert missing["stamp_ros"] is None


def test_source_changes_cannot_complete_a_pause_window(tmp_path):
    records = [sample("clock", i / 10, 1.0, source_epoch=str(i // 3)) for i in range(8)]
    assert "clock_pause" not in analyze(tmp_path, records)["findings"]


def test_capture_failure_is_bounded_and_reports_actual_absence(tmp_path):
    import subprocess
    import sys

    output = tmp_path / "capture.jsonl"
    completed = subprocess.run(
        [
            sys.executable,
            "tools/vehicle_motion_capture.py",
            "--socket",
            "/tmp/vehicle-motion-absent.sock",
            "--seconds",
            "0.03",
            "--output",
            str(output),
        ],
        capture_output=True,
        timeout=3,
    )
    assert completed.returncode == 2
    summary = json.loads(output.read_text().splitlines()[-1])
    assert summary["counts"] == {}
    assert summary["complete"] is False
    assert set(summary["missing_streams"]) >= {"truth", "ipc", "scan"}


def test_worker_success_or_counter_restart_is_not_expiry_starvation(tmp_path):
    records = [
        sample(
            "worker_diagnostic",
            i / 10,
            None,
            pending=True,
            submitted=i + 1,
            completed=i,
            rejected_expired=i,
            admitted=i,
        )
        for i in range(8)
    ]
    assert "permit_starvation" not in analyze(tmp_path, records)["findings"]
    for row in records[4:]:
        row["data"].update(submitted=1, completed=0, rejected_expired=0, admitted=0)
    assert "permit_starvation" not in analyze(tmp_path, records)["findings"]


def test_capture_preserves_nonfinite_sensor_semantics_and_redacts_keys():
    from tools.vehicle_motion_capture import ros_sample

    row = ros_sample(
        "scan", {"ranges": [float("inf"), float("nan")], "credentials": "must-not-appear"}, 1
    )
    assert row["raw"]["ranges"] == ["inf", "nan"]
    assert row["raw"]["credentials"] == "[redacted]"


def test_same_dds_new_producer_epoch_breaks_window_and_interval():
    from tools.vehicle_motion_analyze import Sample, _interval_average

    records = [
        sample("clock", i / 10, 1.0, publisher_gid="same", producer_epoch=str(i // 3))
        for i in range(8)
    ]
    assert "clock_pause" not in analyze_trace_records(records)["findings"]
    wheel = [
        Sample(
            i,
            "wheel",
            i / 10,
            i / 10,
            {"vx": 0.3, "publisher_gid": "same", "producer_epoch": str(i // 3)},
        )
        for i in range(7)
    ]
    assert _interval_average(wheel, 0.1, 0.5) is None


def analyze_trace_records(records):
    from tools.vehicle_motion_analyze import _parse, _windows

    streams, _ = _parse(records)
    return {"findings": ["clock_pause"] if _windows(streams["clock"], lambda row: True) else []}


def test_interval_cannot_stitch_two_ros_clock_passes_or_invalid_rows():
    from tools.vehicle_motion_analyze import Sample, _interval_average

    def rows(stamps):
        return [Sample(i, "wheel", i / 10, stamp, {"vx": 0.2}) for i, stamp in enumerate(stamps)]

    # Neither pass independently covers [0,.3]; they must not be assembled.
    assert _interval_average(rows([0.0, 0.1, 0.2, 0.1, 0.2, 0.3]), 0.0, 0.3) is None
    invalid = rows([0.0, 0.1, None, 0.1, 0.2, 0.3])
    assert _interval_average(invalid, 0.0, 0.3) is None


def test_capture_preserves_producer_epoch_alongside_dds_identity():
    from tools.vehicle_motion_capture import attach_dds_identity

    record = {"data": {"source_epoch": "algorithm-2"}}
    attach_dds_identity(record, publisher_gid="dds-a", source_ns=1, received_ns=2)
    assert record["data"]["source_epoch"] == "algorithm-2"
    assert record["data"]["producer_epoch"] == "algorithm-2"
    assert record["data"]["publisher_gid"] == "dds-a"


def test_parse_keeps_invalid_sample_as_a_continuity_barrier():
    from tools.vehicle_motion_analyze import _interval_average, _parse

    records = [sample("wheel", i / 10, i / 10, vx=0.2) for i in range(4)]
    records.insert(
        2,
        {
            "kind": "sample",
            "stream": "wheel",
            "received_steady": 0.15,
            "stamp_ros": 0.15,
            "data": "invalid",
        },
    )
    streams, invalid = _parse(records)
    assert invalid == [2]
    assert _interval_average(streams["wheel"], 0, 0.3) is None


def test_ros_capture_uses_its_context_executor_and_humble_message_info(monkeypatch):
    import io
    import sys
    from types import ModuleType, SimpleNamespace

    from tools.vehicle_motion_capture import RosCapture, TraceWriter

    callbacks = []
    context = object()
    node = SimpleNamespace(
        create_subscription=lambda kind, topic, cb, qos: callbacks.append(cb),
        destroy_node=lambda: None,
    )
    calls = []

    class Executor:
        def __init__(self, *, context):
            calls.append(context)

        def add_node(self, node):
            pass

        def spin_once(self, *, timeout_sec):
            calls.append("spin")

        def shutdown(self):
            calls.append("shutdown")

    def module(name, **values):
        result = ModuleType(name)
        result.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, result)

    module(
        "rclpy",
        Context=lambda: context,
        init=lambda **kw: None,
        create_node=lambda *a, **kw: node,
        shutdown=lambda **kw: None,
        spin_once=lambda *a, **kw: (_ for _ in ()).throw(AssertionError("default executor")),
    )
    module("rclpy.executors", SingleThreadedExecutor=Executor)

    def metadata_callback(callback):
        callback._senior_dds_info = True
        return callback

    module(
        "senior_robot.motion_lifecycle", MotionExecutor=Executor, with_dds_info=metadata_callback
    )
    module("rclpy.qos", qos_profile_sensor_data=object())
    module("rosidl_runtime_py.convert", message_to_ordereddict=lambda msg: msg)
    module("rosidl_runtime_py.utilities", get_message=lambda name: object())
    writer = TraceWriter(io.StringIO())
    capture = RosCapture(writer)
    capture.spin()
    assert getattr(callbacks[2], "_senior_dds_info", False)
    callbacks[2](
        {"source_epoch": "producer"},
        {"publisher_gid": [1, 2], "source_timestamp": 1, "received_timestamp": 2},
    )
    capture.close()
    assert calls == [context, "spin", "shutdown"]
    assert writer.counts["scan_motion"] == 1
    callbacks[2]({"source_epoch": "producer"}, {"source_timestamp": 3, "received_timestamp": 4})
    assert json.loads(writer.stream.getvalue().splitlines()[-1])["dds"]["publisher_gid"] is None


def test_fresh_scan_after_reset_cannot_use_old_matching_ros_stamps(tmp_path):
    records = []
    for i in range(11):
        records.append(sample("wheel", i / 10, i / 10, vx=1.0))
    # Auxiliary stream is invalidated at reset; only an incomplete new run follows.
    records.append(sample("wheel", 9.9, None))
    records.append(sample("wheel", 10.0, 0.0, vx=0.0))
    for i in range(11):
        t = i / 10
        records += [sample("clock", 10.0 + t, t), sample("truth", 10.0 + t, t, vx=0.0)]
        if i:
            records.append(
                sample("scan_motion", 10.0 + t, t, vx=0.0, interval_start_ros=t - 0.1, valid=True)
            )
    assert "wheel_body_disagreement" not in analyze(tmp_path, records)["findings"]
