from senior_control.models import Velocity
from senior_control.safety.arbiter import Arbiter, FinalCommandFence


def test_final_command_fence_rejects_old_generation_and_deadline():

    fence = FinalCommandFence()
    fence.set_publisher(b"a")
    assert fence.accept(b"a", "boot-a", 1, 0, 5.0, Velocity(0.0, 0.0), 1.0)
    assert fence.accept(b"a", "boot-a", 1, 1, 2.0, Velocity(0.1, 0.0), 1.1)
    assert fence.usable(1.9)
    assert not fence.usable(2.0)
    assert not fence.accept(b"a", "boot-a", 3, 1, 5.0, Velocity(0.1, 0.0), 2.0)
    assert fence.accept(b"a", "boot-a", 3, 0, 5.0, Velocity(0.0, 0.0), 2.0)
    assert not fence.accept(b"a", "boot-a", 1, 2, 5.0, Velocity(0.1, 0.0), 2.1)
    assert not fence.usable(2.1)
    fence.set_publisher(b"b")
    assert not fence.accept(b"a", "boot-a", 3, 0, 5.0, Velocity(0.0, 0.0), 2.1)
    assert not fence.accept(b"b", "boot-b", 1, 1, 5.0, Velocity(0.1, 0.0), 2.1)
    assert fence.accept(b"b", "boot-b", 1, 0, 5.0, Velocity(0.0, 0.0), 2.1)


def test_final_fence_never_reaccepts_retired_publisher_gid():

    fence = FinalCommandFence()
    fence.set_publisher(b"a")
    assert fence.accept(b"a", "boot-a", 1, 0, 2.0, Velocity(0.0, 0.0), 1.0)
    fence.set_publisher(None)
    fence.set_publisher(b"a")
    assert not fence.accept(b"a", "boot-a", 1, 0, 2.0, Velocity(0.0, 0.0), 1.0)


def test_final_fence_rejects_retired_boot_on_new_publisher():

    fence = FinalCommandFence()
    fence.set_publisher(b"a")
    assert fence.accept(b"a", "boot-a", 1, 0, 2.0, Velocity(0.0, 0.0), 1.0)
    fence.set_publisher(b"b")
    assert not fence.accept(b"b", "boot-a", 1, 0, 2.0, Velocity(0.0, 0.0), 1.0)


def test_final_fence_cannot_relabel_session_within_generation():

    fence = FinalCommandFence()
    fence.set_publisher(b"a")
    assert fence.accept(b"a", "boot", 1, 0, 0.0, Velocity(0.0, 0.0), 1.0)
    assert fence.accept(b"a", "boot", 1, 2, 2.0, Velocity(0.1, 0.0), 1.1, session="page-a", epoch=1)
    assert not fence.accept(
        b"a", "boot", 1, 3, 2.0, Velocity(0.1, 0.0), 1.2, session="page-b", epoch=1
    )
    assert not fence.accept(
        b"a", "boot", 1, 3, 2.0, Velocity(0.1, 0.0), 1.2, session="page-a", epoch=2
    )


def test_task_samples_keep_local_deadline_without_a_remote_lease():
    owner = Arbiter("robot", 300)
    assert owner.begin_navigation("route", 10.0, task_epoch=1)
    assert owner.accept_navigation_sample("route", 1, Velocity(0.2, 0), 10.0, 10.1)
    assert owner.final_command(10.2) == (Velocity(0.2, 0), "route", 1, 1, 10.3)
    assert owner.final_command(10.3) is None
    assert not owner.accept_navigation_sample("route", 1, Velocity(0.2, 0), 10.2, 10.3)
    assert not owner.accept_navigation_sample("route", 2, Velocity(0.2, 0), 10.0, 10.3)
    owner.end_navigation("route")
    assert not owner.begin_navigation("route", 10.4, task_epoch=1)
    assert owner.begin_navigation("route", 10.4, task_epoch=2)


def test_estop_retires_task_and_rejects_wrong_robot_or_replayed_request():
    from senior_control.protocol import EmptyPayload, SimpleEnvelope

    owner = Arbiter("robot", 300)
    assert owner.begin_navigation("route", 10.0, task_epoch=1)
    stop = SimpleEnvelope(
        v=1,
        type="safety.estop",
        robot_id="other",
        session_id="page",
        connection_epoch=1,
        request_id="stop",
        seq=1,
        expires_at=0.0,
        payload=EmptyPayload(),
    )
    assert not owner.accept(stop, 10).accepted
    stop = stop.model_copy(update={"robot_id": "robot"})
    assert owner.accept(stop, 10).accepted
    assert owner.final_command(10) is None
    assert not owner.task_navigation_active
    assert not owner.accept(stop, 10.1).accepted
    assert not owner.begin_navigation("route", 10.1, task_epoch=1)
