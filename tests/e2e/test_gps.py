"""GPS and passive heading may estimate pose but never publish velocity."""

from dataclasses import fields

from conftest import SimulationProfile

from senior_control.localization.heading import HeadingObserver, MotionSample
from senior_control.models import Velocity


def test_gps_passive_heading_never_moves_robot(
    simulation_profile: SimulationProfile,
) -> None:
    observer = HeadingObserver(min_baseline_m=5.0, max_correction_rad=0.35)
    result = observer.observe(
        MotionSample(
            stamp=20.0,
            map_yaw=0.2,
            map_dx=8.0,
            map_dy=0.0,
            enu_east_m=8.0,
            enu_north_m=0.0,
            odom_forward_m=8.0,
            imu_yaw_delta_rad=0.01,
            gnss_trusted=True,
            georeferenced=True,
            commanded_forward=True,
            slip=False,
            jump=False,
        )
    )

    assert result is not None
    assert not isinstance(result, Velocity)
    assert {field.name for field in fields(result)} == {"correction_rad", "source", "ready"}
    assert getattr(result, "velocity", None) is None
    assert not hasattr(observer, "publish")
    assert len(simulation_profile.config_hash) == 64
