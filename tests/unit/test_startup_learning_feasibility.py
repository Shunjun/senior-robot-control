"""Baseline feasibility probes, not acceptance of startup motion or learned filtering."""

from senior_control.config import DeviceProfile
from senior_control.models import Pose2D, Velocity
from senior_control.safety.self_filter import ScanObservation, filter_self
from test_directional_constraints import engine, snapshot


BOUNDARY = [(-0.3, -0.2), (0.3, -0.2), (0.3, 0.2), (-0.3, 0.2)]


def forward_scan(distance):
    return ScanObservation(
        (distance,), 0.0, 0.01, 0.05, 5.0, 10.0, "ros", Pose2D(0, 0, 0),
        10.0, "base_link", 10.0,
    )


def test_unlearned_interior_return_blocks_first_motion_even_with_external_free_space():
    observation = filter_self(forward_scan(0.15), DeviceProfile(footprint=BOUNDARY))
    assert observation.external_hits == observation.rays
    context = snapshot(
        target=Velocity(0.05, 0), current=0,
        hits=tuple(ray.endpoint for ray in observation.external_hits),
    )
    # The existing synthetic scene supplies generous free space: this isolates
    # occupied/sweep overlap rather than a separate unknown-space failure.
    result = engine().refine(context, 0)
    assert result.permit is None
    assert result.brake.reason == "stop_envelope_blocked"


def test_using_physical_polygon_as_existing_mask_would_hide_external_obstacle():
    observation = filter_self(
        forward_scan(1.0), DeviceProfile(footprint=BOUNDARY, self_mask=[BOUNDARY])
    )
    # This documents why substituting the footprint into the old ray-intersection
    # mask is unsafe: the endpoint is outside, but its ray starts inside.
    assert observation.rays[0].endpoint == (1.0, 0.0)
    assert observation.rays[0].kind == "self"
    assert not observation.external_hits
    assert not observation.free_rays


def test_same_resting_measurements_do_not_supply_attachment_evidence():
    profile = DeviceProfile(footprint=BOUNDARY)
    # Attached device and a separate object at the same body-relative location
    # produce identical stationary measurements. Neither is learned by baseline.
    attached = tuple(filter_self(forward_scan(0.15), profile) for _ in range(5))
    separate = tuple(filter_self(forward_scan(0.15), profile) for _ in range(5))
    assert attached == separate
    assert all(frame.external_hits and not frame.occluded_regions for frame in attached)
