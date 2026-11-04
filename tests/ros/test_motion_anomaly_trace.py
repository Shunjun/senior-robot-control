"""Checks the bounded first-anomaly collection used by owned live tests."""

from motion_anomaly_trace import AnomalyTrace


def test_capture_retains_first_event_and_bounded_before_after_samples():
    trace = AnomalyTrace(max_samples=2, post_seconds=1.0)
    for index in range(4):
        trace.record("wheel", float(index), index, {"sample": index})
    trace.record("motion", 4.0, 4, {"sample": "first"}, anomaly=True)
    trace.record("wheel", 4.2, 5, {"sample": "after"})
    trace.record("motion", 4.3, 6, {"sample": "later"}, anomaly=True)
    trace.record("wheel", 5.1, 7, {"sample": "too late"})
    for index in range(10):
        trace.record("wheel", 6.0 + index, index, {"sample": index + 100})

    document = trace.document(lambda _topic, sample: sample)
    assert document is not None and document["first_anomaly_steady"] == 4.0
    assert [item["sample"]["sample"] for item in document["topics"]["wheel"]] == [2, 3, "after"]
    assert document["topics"]["wheel"][-1]["observed_clock_ns"] == 5
    assert [item["sample"]["sample"] for item in document["topics"]["motion"]] == [
        "first",
        "later",
    ]


def test_no_anomaly_produces_no_claimed_first_event():
    trace = AnomalyTrace()
    trace.record("motion", 1.0, 2, {"sample": "healthy"})
    assert trace.document(lambda _topic, sample: sample) is None
