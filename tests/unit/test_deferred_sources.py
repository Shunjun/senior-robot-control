"""Soft source invalidation does not blacklist future input."""

from senior_motion_control.source_fences import SourceEpochFence


def test_source_invalidation_recovers_without_blacklist_or_exhaustion():
    fence = SourceEpochFence(max_retired=2)
    assert fence.accept("active")
    fence.retire_active()
    fence.retire_unadmitted("deferred")
    assert fence.active is None
    assert not fence.is_retired("active")
    assert fence.accept("active")
    assert fence.accept("deferred")
    fence.retire_unadmitted("overflow")
    assert fence.accept("fresh")
