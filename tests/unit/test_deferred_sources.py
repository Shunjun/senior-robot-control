"""Retired wall-clock future queues have no runtime or compatibility branch."""

from senior_control.motion.source_fences import SourceEpochFence


def test_replaced_deferred_epoch_cannot_revive_or_exhaustion_be_forgotten():
    fence = SourceEpochFence(max_retired=2)
    assert fence.accept("active")
    fence.retire_active()
    fence.retire_unadmitted("deferred")
    assert fence.active is None
    assert fence.is_retired("active") and fence.is_retired("deferred")
    assert not fence.accept("active") and not fence.accept("deferred")
    fence.retire_unadmitted("overflow")
    assert not fence.accept("fresh")
