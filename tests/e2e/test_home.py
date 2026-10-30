"""Home remains the original persisted waypoint after process restart."""

from pathlib import Path

from conftest import SimulationProfile

from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, Pose2D
from senior_control.storage.database import Database
from senior_control.waypoints import HomeService, LocalizedPose, StartupReference

MAP = MapRef("yard", 3)


def localized(pose: Pose2D) -> LocalizedPose:
    return LocalizedPose(pose, MAP, "node-1", Pose2D(0.0, 0.0, 0.0), True)


async def test_home_returns_after_restart(
    tmp_path: Path,
    simulation_profile: SimulationProfile,
) -> None:
    database_path = tmp_path / "home.sqlite3"
    original = Pose2D(1.0, 2.0, 0.4)
    async with Database(database_path) as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "wheel_odom", 0.5),
            robot_id="robot-a",
        )
        await service.prepare()
        first = await service.initialize(StartupReference(False, True, True), localized(original))
        assert first.waypoint is not None
        waypoint_id = first.waypoint.id

    async with Database(database_path) as database:
        restarted = HomeService(
            database,
            MapReferenceStore(MAP, "wheel_odom", 0.5),
            robot_id="robot-a",
        )
        await restarted.prepare()
        result = await restarted.initialize(
            StartupReference(False, True, True),
            localized(Pose2D(9.0, 9.0, 1.0)),
        )

    assert result.waypoint is not None
    assert result.waypoint.id == waypoint_id
    assert result.waypoint.pose == original
    assert simulation_profile.database.name == "acceptance.sqlite3"
