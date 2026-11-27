"""The shared consumer rule, independent of transport and algorithm quality."""

from vehicle_core.source_order import SourceOrder


def test_same_generation_accepts_only_newer_capture_timestamps():
    order = SourceOrder()
    assert order.accept("boot-a", 1, 100)
    assert not order.accept("boot-a", 1, 100)
    assert not order.accept("boot-a", 1, 99)
    assert order.accept("boot-a", 1, 101)


def test_new_generation_replaces_timestamp_frontier_and_rejects_old_batch():
    order = SourceOrder()
    assert order.accept("boot-a", 3, 100)
    assert order.observe_generation("boot-a", 4)
    assert not order.accept("boot-a", 3, 200)
    assert order.accept("boot-a", 4, 10)
    assert not order.accept("boot-a", 3, 200)
    assert not order.accept("boot-a", 4, 10)
    assert order.accept("boot-a", 4, 11)


def test_new_boot_requires_explicit_confirmation_and_never_switches_back_on_data():
    order = SourceOrder()
    assert order.accept("boot-a", 2, 100)
    assert not order.accept("boot-b", 1, 101)
    order.confirm_boot("boot-b")
    assert order.accept("boot-b", 1, 10)
    assert not order.accept("boot-a", 2, 200)
    order.confirm_boot("boot-b")
    assert not order.accept("boot-b", 1, 10)
    assert order.accept("boot-b", 1, 11)

    received = SourceOrder()
    assert received.accept_capture("boot-a", 2, 100, 1000)
    assert not received.accept_capture("boot-b", 1, 101, 1000)
    assert received.accept_capture("boot-b", 1, 10, 1001)
    assert not received.accept_capture("boot-a", 2, 200, 1000)
    assert received.accept_capture("boot-b", 1, 11, 1002)


def test_invalid_metadata_does_not_bind_or_reset_a_source():
    order = SourceOrder()
    assert not order.accept("", 1, 100)
    assert not order.accept("bad", 0, 100)
    assert not order.accept("bad", 1, -1)
    assert order.accept("boot-a", 1, 100)
    assert not order.accept("boot-b", 99, -1)
    assert order.accept("boot-a", 1, 101)
