"""Learned mask revocation crosses serialized copies and only subtracts points."""

import pickle
from dataclasses import replace

import pytest
from test_motion_core_handoff import core  # noqa: F401
from test_startup_learning import BOUNDARY, record, side_observation

from senior_control.models import Pose2D
from senior_control.safety.self_learning import SelfReturnModel


def learned_model():
    model = SelfReturnModel(BOUNDARY, error_m=0.005)
    for i in range(12):
        record(model, side_observation(0.15), Pose2D(i * 0.015, 0, 0))
    assert model.validate(independent_motion=True) == 1
    return model


@pytest.mark.parametrize("change", ["new_obstacle", "device_removed"])
def test_copied_revocation_reaches_owner_and_returning_echo_is_not_masked(change):
    owner = learned_model()
    copied = pickle.loads(pickle.dumps(owner))
    current = side_observation(0.10 if change == "new_obstacle" else 1.0)
    filtered, event = copied.filter_snapshot(current, provisional=False)
    assert filtered.rays[180].kind == "hit"
    assert owner.learned and copied.learned
    event = pickle.loads(pickle.dumps(event))
    assert owner.apply_revocation(event)
    assert owner.learned == () and owner.revision > copied.revision
    assert not owner.apply_revocation(event)  # Duplicate removal cannot resurrect anything.
    returned, _ = owner.filter_snapshot(side_observation(0.15), provisional=False)
    assert returned.rays[180].kind == "hit"
    _, old_event = copied.filter_snapshot(side_observation(0.15), provisional=False)
    assert not owner.apply_revocation(old_event)
    assert owner.learned == ()


@pytest.mark.parametrize("invalid", ["foreign_model", "future_version", "invented_member"])
def test_owner_rejects_invalid_cross_process_revocations(invalid):
    owner = learned_model()
    _, event = owner.filter_snapshot(side_observation(1.0), provisional=False)
    invalid_event = {
        "foreign_model": replace(event, model_id="other"),
        "future_version": replace(event, base_revision=event.base_revision + 1),
        "invented_member": replace(event, removed=((100.0, 100.0),)),
    }[invalid]
    before = owner.learned
    with pytest.raises(ValueError, match="invalid_self_return_revocation"):
        owner.apply_revocation(invalid_event)
    assert owner.learned == before


@pytest.mark.parametrize("changed_range", [0.10, 1.0])
def test_core_latest_scan_revokes_owner_model_before_geometry_can_copy_it(core, changed_range):  # noqa: F811
    from test_continuous_scan_control import scan

    from senior_control.config import DeviceProfile

    core.node._profile = lambda: DeviceProfile(footprint=BOUNDARY)
    core.startup.state = "completed"
    core.self_returns = learned_model()
    first = scan(10.0, 1)
    first.message.ranges[540] = 0.15
    _, masked = core._filtered_scan(first)
    assert masked.rays[540].kind == "self"
    changed = scan(10.05, 2)
    changed.message.ranges[540] = changed_range
    _, masked = core._filtered_scan(changed)
    assert masked.rays[540].kind == "hit" and core.self_returns.learned == ()
    returned = scan(10.1, 3)
    returned.message.ranges[540] = 0.15
    _, masked = core._filtered_scan(returned)
    assert masked.rays[540].kind == "hit" and core.self_returns.learned == ()
