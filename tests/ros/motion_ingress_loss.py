"""Checks for the live safety scan ingress owner-loss event."""


def owner_loss_observed(before, lost, owner_endpoints, relay_endpoints):
    return (
        before.valid
        and not before.owner_lost
        and bool(before.owner_gid and before.owner_epoch and before.relay_gid and before.relay_boot)
        and not lost.valid
        and lost.owner_lost
        and not lost.upstream_invalid
        and lost.reason == "owner_lost"
        and lost.owner_sequence == 0
        and lost.owner_gid == before.owner_gid
        and lost.owner_epoch == before.owner_epoch
        and lost.owner_generation > before.owner_generation
        and lost.relay_gid == before.relay_gid
        and lost.relay_boot == before.relay_boot
        and lost.relay_sequence > before.relay_sequence
        and not owner_endpoints
        and relay_endpoints == [("scan_receipt", "/", before.relay_gid)]
    )
