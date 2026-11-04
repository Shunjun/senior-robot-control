"""Identity and transition checks for the live owner-loss observation."""

from copy import deepcopy
from types import SimpleNamespace

import pytest
from motion_ingress_loss import owner_loss_observed


def samples():
    before = SimpleNamespace(
        valid=True,
        owner_lost=False,
        upstream_invalid=False,
        reason="",
        owner_gid="owner",
        owner_epoch="epoch",
        owner_generation=1,
        owner_sequence=10,
        relay_gid="relay",
        relay_boot="boot",
        relay_sequence=58,
    )
    lost = deepcopy(before)
    lost.valid = False
    lost.owner_lost = True
    lost.reason = "owner_lost"
    lost.owner_generation = 2
    lost.owner_sequence = 0
    lost.relay_sequence = 59
    return before, lost, [], [("scan_receipt", "/", "relay")]


def test_owner_loss_from_original_owner_and_live_relay():
    assert owner_loss_observed(*samples())


@pytest.mark.parametrize(
    "fault",
    [
        "owner",
        "epoch",
        "relay",
        "boot",
        "generation",
        "sequence",
        "not_lost",
        "valid",
        "upstream",
        "reason",
        "owner_sequence",
        "owner_present",
        "relay_gone",
        "relay_endpoint_mismatch",
        "wrong_relay_name",
        "extra_relay",
    ],
)
def test_owner_loss_rejects_inconsistent_evidence(fault):
    before, lost, owners, relays = samples()
    if fault == "owner":
        lost.owner_gid = "other"
    elif fault == "epoch":
        lost.owner_epoch = "other"
    elif fault == "relay":
        lost.relay_gid = "other"
    elif fault == "boot":
        lost.relay_boot = "other"
    elif fault == "generation":
        lost.owner_generation = before.owner_generation
    elif fault == "sequence":
        lost.relay_sequence = before.relay_sequence
    elif fault == "not_lost":
        lost.owner_lost = False
    elif fault == "valid":
        lost.valid = True
    elif fault == "upstream":
        lost.upstream_invalid = True
    elif fault == "reason":
        lost.reason = "other"
    elif fault == "owner_sequence":
        lost.owner_sequence = 1
    elif fault == "owner_present":
        owners.append(("safety_scan_ingress", "/", "owner"))
    elif fault == "relay_gone":
        relays.clear()
    elif fault == "relay_endpoint_mismatch":
        relays[0] = ("scan_receipt", "/", "other")
    elif fault == "wrong_relay_name":
        relays[0] = ("other", "/", "relay")
    elif fault == "extra_relay":
        relays.append(("other", "/", "other"))
    assert not owner_loss_observed(before, lost, owners, relays)
