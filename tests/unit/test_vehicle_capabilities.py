"""Approved unavailable production facade and unconfirmed-map scenarios."""

from senior_motion_control.vehicle_capabilities import VehicleCapabilities
from test_startup_learning_chain import rig  # noqa: F401


def test_production_facades_report_unavailable_without_motion_sink(rig):  # noqa: F811
    capabilities = rig.core.capabilities
    before = list(rig.core.node.emitted)
    for port in (
        capabilities.mapping,
        capabilities.localization,
        capabilities.navigation,
        capabilities.gps,
    ):
        result = port.read()
        assert result.status == "unavailable"
        assert result.value is None
    assert not hasattr(capabilities, "emit")
    assert rig.core.node.emitted == before


def test_unconfirmed_map_cannot_navigate():
    result = VehicleCapabilities().navigate((1.0, 2.0, 0.0))
    assert result.status == "unavailable"
    assert result.reason == "map_pose_not_confirmed"
    assert result.value is None
