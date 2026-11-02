from senior_control.vision.rgbd_bounded_inbox import BoundedInbox


def test_overflow_revokes_old_inflight_and_discards_old_queue_before_new():
    inbox = BoundedInbox(capacity=2)
    assert inbox.offer("inflight")
    issued, value, reason = inbox.take()
    assert value == "inflight" and not reason
    assert inbox.offer("old-1") and inbox.offer("old-2")
    assert not inbox.offer("overflow")
    assert not inbox.current(issued)
    assert inbox.offer("new")
    generation, value, reason = inbox.take()
    assert generation != issued and value is None and reason == "overflow"
    _, value, reason = inbox.take()
    assert value == "new" and not reason


def test_explicit_revoke_is_a_barrier_not_a_new_capture():
    inbox = BoundedInbox(capacity=2)
    assert inbox.offer("old")
    inbox.revoke("source_changed")
    _, value, reason = inbox.take()
    assert value is None and reason == "source_changed"
    assert inbox.take() is None


def test_idle_poll_snapshot_is_invalidated_by_concurrent_overflow():
    inbox = BoundedInbox(capacity=1)
    generation = inbox.snapshot()
    assert inbox.offer("first")
    assert not inbox.offer("overflow")
    assert not inbox.current(generation)


def test_publish_commit_and_invalidation_share_one_order():
    inbox = BoundedInbox(capacity=1)
    published = []
    first = inbox.snapshot()
    inbox.revoke("source_changed")
    assert not inbox.commit_if_current(first, lambda: published.append("old"))
    current = inbox.snapshot()
    assert not inbox.commit_if_current(current, lambda: published.append("early"))
    assert inbox.take()[2] == "source_changed"
    assert inbox.commit_if_current(current, lambda: published.append("new"))
    assert published == ["new"]


def test_publish_waits_for_queued_status_without_retiring_generation():
    inbox = BoundedInbox(capacity=2)
    generation = inbox.snapshot()
    published = []
    assert inbox.offer("unexamined status")
    assert not inbox.commit_if_current(generation, lambda: published.append("premature"))
    assert inbox.take()[1] == "unexamined status"
    assert inbox.current(generation)
    assert inbox.commit_if_current(generation, lambda: published.append("checked"))
    assert published == ["checked"]
