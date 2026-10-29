"""Waypoint rows survive restart. Home stays one per robot."""

import asyncio
from pathlib import Path

import pytest

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.archive import BundleWaypoint
from senior_control.mapping.references import AnchorPose, AnchorSnapshot, GridExtent
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D
from senior_control.storage.database import Database
from senior_control.storage.migrations import migrate
from senior_control.storage.schema import DOMAIN_MIGRATIONS
from senior_control.waypoints import HomeService, LocalizedPose, StartupReference

MAP = MapRef("yard", 3)
ROBOT = "robot-a"
CELL = 0.5
STILL = StartupReference(False, True, True, None)


def localized(
    pose: Pose2D,
    *,
    healthy: bool = True,
    latitude: float | None = None,
    longitude: float | None = None,
) -> LocalizedPose:
    return LocalizedPose(
        pose,
        MAP,
        "node-1",
        Pose2D(0.0, 0.0, 0.0),
        healthy,
        latitude,
        longitude,
    )


async def test_schema_version_two_adds_waypoint_table(tmp_path: Path) -> None:
    async with Database(tmp_path / "schema.db") as database:
        await migrate(database, list(DOMAIN_MIGRATIONS))
        async with database.connection.execute("PRAGMA user_version") as cursor:
            row = await cursor.fetchone()
        assert row is not None
        assert row[0] == 2
        async with database.connection.execute(
            "SELECT name FROM sqlite_master WHERE type = 'table'"
        ) as cursor:
            names = {table[0] for table in await cursor.fetchall()}
        assert "waypoint" in names
        assert "map_home_link" in names
        async with database.connection.execute("PRAGMA table_info(map_home_link)") as cursor:
            columns = [info[1] for info in await cursor.fetchall()]
        assert columns == [
            "robot_id",
            "source_robot_id",
            "map_id",
            "reference_revision",
            "waypoint_id",
            "status",
            "adopted",
        ]


async def test_one_home_per_robot(tmp_path: Path) -> None:
    async with Database(tmp_path / "one.db") as database:
        left = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        right = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id="robot-b")
        await left.prepare()
        await right.prepare()
        first = await left.set_current(localized(Pose2D(1.0, 2.0, 0.4)))
        second = await left.set_current(localized(Pose2D(3.0, 4.0, 0.5)))
        assert second.pose == Pose2D(3.0, 4.0, 0.5)
        assert second.id == first.id
        assert second.kind == "home"
        other = await right.set_current(localized(Pose2D(8.0, 8.0, 0.1)))
        assert other.id != first.id
        async with database.connection.execute(
            "SELECT COUNT(*) FROM waypoint WHERE robot_id = ? AND kind = 'home'",
            (ROBOT,),
        ) as cursor:
            count = await cursor.fetchone()
        assert count is not None
        assert count[0] == 1


async def test_foreign_home_stays_pending(tmp_path: Path) -> None:
    async with Database(tmp_path / "foreign.db") as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        imported = await service.import_home(
            BundleWaypoint("home-b", "home", Pose2D(1.0, 1.0, 0.25), MAP, "node-1", "robot-b"),
            anchor_pose=Pose2D(0.0, 0.0, 0.0),
        )
        assert imported.status == "pending_confirmation"
        assert imported.source_robot_id == "robot-b"
        assert imported.pose == Pose2D(1.0, 1.0, 0.25)
        link = await service.home_link()
        assert link is not None
        assert link.adopted is False
        assert link.status == "pending_confirmation"
        assert link.waypoint_id == imported.id
        result = await service.initialize(
            StartupReference(True, False, False, None),
            localized(Pose2D(9.0, 9.0, 0.0)),
        )
        assert result.initialized is False
        kept = await service.home()
        assert kept is not None
        assert kept.pose == imported.pose
        assert kept.status == "pending_confirmation"
        await service.note_localization(localized(Pose2D(1.0, 1.0, 0.25)))
        with pytest.raises(DomainError) as caught:
            await service.return_home(MAP)
        assert caught.value.code == ErrorCode.CONFLICT
        assert str(caught.value) == "home is not adopted"


async def test_gps_does_not_rewrite_map_pose(tmp_path: Path) -> None:
    path = tmp_path / "gps.db"
    pose = Pose2D(1.5, 2.5, 0.3)
    async with Database(path) as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        waypoint = await service.set_current(
            localized(pose, latitude=31.2, longitude=121.4),
        )
        assert waypoint.pose == pose
        assert waypoint.latitude == pytest.approx(31.2)
        assert waypoint.longitude == pytest.approx(121.4)
    async with Database(path) as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        home = await service.home()
        assert home is not None
        assert home.pose == pose
        assert home.latitude == pytest.approx(31.2)
        assert home.longitude == pytest.approx(121.4)


async def test_waypoint_pose_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "points.db"
    async with Database(path) as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        saved = await service.save_waypoint(localized(Pose2D(3.0, 4.0, 0.5)), waypoint_id="gate")
        assert saved.id == "gate"
        assert saved.kind == "waypoint"
        assert saved.pose == Pose2D(3.0, 4.0, 0.5)
    async with Database(path) as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        found = await service.waypoint("gate")
        assert found is not None
        assert found.pose == Pose2D(3.0, 4.0, 0.5)
        assert found.kind == "waypoint"


async def test_reference_change_moves_home_and_survives_restart(tmp_path: Path) -> None:
    path = tmp_path / "shift.db"
    async with Database(path) as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        created = await service.initialize(STILL, localized(Pose2D(4.0, 5.0, 0.2)))
        assert created.waypoint is not None
        moved = await service.apply_anchor_update(
            AnchorSnapshot(
                MAP,
                "odom",
                (AnchorPose("node-1", Pose2D(1.0, 0.0, 0.0), True),),
                GridExtent(0, 0, 0.0, 0.0),
            )
        )
        assert moved.id == created.waypoint.id
        assert moved.pose.x == pytest.approx(5.0)
        assert moved.pose.y == pytest.approx(5.0)
        assert moved.pose.yaw == pytest.approx(0.2)
    async with Database(path) as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        home = await service.home()
        assert home is not None
        assert home.pose.x == pytest.approx(5.0)
        assert home.pose.y == pytest.approx(5.0)
        assert home.pose.yaw == pytest.approx(0.2)


class BlockingNavigation:
    def __init__(self) -> None:
        self.events: list[str] = []
        self.release = asyncio.Event()

    async def navigate(self, goal: object) -> str:
        del goal
        self.events.append("navigate")
        return f"exec-{self.events.count('navigate')}"

    async def cancel(self, execution_id: str) -> OperationResult:
        self.events.append("cancel")
        await self.release.wait()
        self.events.append("terminated")
        return OperationResult(execution_id, OperationStatus.SUCCEEDED)


async def test_cancel_waits_outside_write_transaction(tmp_path: Path) -> None:
    navigation = BlockingNavigation()
    async with Database(tmp_path / "lock.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        first = await service.return_home(MAP)
        assert first == "exec-1"
        navigation.release.clear()
        replacement = asyncio.create_task(service.return_home(MAP))
        for _ in range(50):
            if "cancel" in navigation.events:
                break
            await asyncio.sleep(0.01)
        assert "cancel" in navigation.events
        completed = False
        try:
            updated = await asyncio.wait_for(
                service.set_current(localized(Pose2D(2.0, 2.0, 0.4))),
                timeout=1,
            )
            completed = updated.pose == Pose2D(2.0, 2.0, 0.4)
        except TimeoutError:
            completed = False
        finally:
            navigation.release.set()
            await replacement
        assert completed
        assert navigation.events == ["navigate", "cancel", "terminated", "navigate"]


async def test_unadopted_home_is_rejected(tmp_path: Path) -> None:
    navigation = BlockingNavigation()
    async with Database(tmp_path / "unadopted.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        await database.connection.rollback()
        await database.connection.execute(
            """
            UPDATE map_home_link
            SET adopted = 0, status = 'unadopted'
            WHERE robot_id = ?
            """,
            (ROBOT,),
        )
        await database.connection.commit()
        with pytest.raises(DomainError) as caught:
            await service.return_home(MAP)
        assert caught.value.code == ErrorCode.CONFLICT
        assert str(caught.value) == "home is not adopted"
        assert navigation.events == []


async def test_disconnect_does_not_resume_navigation(tmp_path: Path) -> None:
    navigation = BlockingNavigation()
    async with Database(tmp_path / "offline.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        first = await service.return_home(MAP)
        assert first == "exec-1"
        await service.note_connection(False)
        with pytest.raises(DomainError) as caught:
            await service.return_home(MAP)
        assert caught.value.code == ErrorCode.OFFLINE
        assert str(caught.value) == "connection is offline"
        await service.note_connection(True)
        assert navigation.events == ["navigate"]


async def test_review_required_reference_rejects_return(tmp_path: Path) -> None:
    navigation = BlockingNavigation()
    async with Database(tmp_path / "review.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(4.0, 5.0, 0.2)))
        await service.note_localization(localized(Pose2D(4.0, 5.0, 0.2)))
        await service.apply_anchor_update(
            AnchorSnapshot(
                MAP,
                "odom",
                (AnchorPose("node-1", None, False),),
                GridExtent(0, 0, 0.0, 0.0),
            )
        )
        with pytest.raises(DomainError) as caught:
            await service.return_home(MAP)
        assert caught.value.code == ErrorCode.CONFLICT
        assert str(caught.value) == "coordinate reference needs review"
        assert navigation.events == []
