"""Standard pinned localization retains raw age, source and map generations.

Migrates the generic protections from simulation-reference admission; world/tick
pairing and truth-based permission are retired with that production reference.
"""

import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest

from vehicle_core.sample_time import map_sample
from vehicle_core.values import Pose2D


@pytest.fixture
def reference(monkeypatch):
    lifecycle = ModuleType("senior_sensor_ingress.ros_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    receipt = ModuleType("senior_sensor_ingress.native_receipt")
    # Recording boundary for B's mandatory metadata accessor; no time remapping.
    receipt.with_dds_info = lifecycle.with_dds_info
    receipt.received_sample = lambda info: info["sample_time"]
    monkeypatch.setitem(sys.modules, receipt.__name__, receipt)
    path = Path("ros2/src/senior_localization/senior_localization/hardware_reference.py")
    spec = importlib.util.spec_from_file_location("standard_reference_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = N(ns=100_000_000_000)
    monkeypatch.setattr(module, "time", N(monotonic_ns=lambda: clock.ns))
    ref = module.HardwareReference.__new__(module.HardwareReference)
    ref.config = N(max_age_s=0.5, max_position_variance=0.2, max_yaw_variance=0.2)
    ref._grid = N(map_ref="map", grid_id="generation-a")
    ref._map_serial, ref._pose_map_serial = 1, -1
    ref._pose = ref._pose_time = ref._pose_identity = ref._pose_gid = None
    ref._pose_at = float("-inf")
    ref._pose_stamp_ns = -1
    ref._pose_sequence = 0
    ref._retired_pose_sources = set()
    logs = []
    ref._node = N(get_logger=lambda: N(info=logs.append))
    ref._pose_topic = "/amcl_pose"
    ref.config.pose_publisher = "amcl"
    sources = {"gid": b"pose-a"}
    ref._source = lambda _topic, _name, info=None: (
        sources["gid"] if info is None else info["publisher_gid"]
    )
    ref.grid = lambda: ref._grid

    def message(
        *,
        ns=10_000_000_000,
        received_ns=None,
        receipt_sequence=1,
        receipt_generation=1,
        clock_generation=1,
        gid=None,
        x=1.0,
    ):
        received_ns = ns if received_ns is None else received_ns
        sample = map_sample(
            stamp_ns=ns,
            received_ros_ns=received_ns,
            received_steady_ns=clock.ns,
            clock_generation=clock_generation,
        )
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = covariance[35] = 0.1
        msg = N(
            header=N(stamp=N(sec=ns // 10**9, nanosec=ns % 10**9), frame_id="map"),
            pose=N(
                pose=N(position=N(x=x, y=0.0, z=0.0), orientation=N(x=0, y=0, z=0, w=1)),
                covariance=covariance,
            ),
        )
        return msg, {
            "sample_time": sample,
            "publisher_gid": gid or sources["gid"],
            "receipt_generation": receipt_generation,
            "receipt_sequence": receipt_sequence,
        }

    return N(ref=ref, clock=clock, sources=sources, message=message, logs=logs)


def test_original_age_duplicate_and_queued_delivery_cannot_renew_pose(reference):
    r = reference
    message, info = r.message(received_ns=10_200_000_000)
    r.ref._on_pose(message, info)
    original = r.ref.pose()
    assert original == Pose2D(1, 0, 0)
    r.clock.ns += 100_000_000
    message.pose.pose.position.x = 99
    r.ref._on_pose(message, {**info, "receipt_sequence": 2})
    assert r.ref._pose_sequence == 2
    assert r.ref._pose_time is info["sample_time"]
    assert r.ref.pose() is original
    assert r.ref._pose_at == pytest.approx(99.8)
    r.clock.ns += 200_000_000
    assert r.ref.pose() is None  # Exactly the original 500ms deadline.
    r.ref._on_pose(message, {**info, "receipt_sequence": 3})
    assert r.ref._pose_sequence == 3
    assert r.ref._pose_time is info["sample_time"]
    assert r.ref._pose_at + r.ref.config.max_age_s == pytest.approx(100.3)
    assert r.ref.pose() is None


def test_increasing_native_sequence_with_older_stamp_withdraws_without_renewal(reference):
    r = reference
    original = r.message(received_ns=10_200_000_000)
    r.ref._on_pose(*original)
    r.clock.ns += 100_000_000
    r.ref._on_pose(
        *r.message(
            ns=9_900_000_000,
            received_ns=10_200_000_000,
            receipt_sequence=2,
            x=99,
        )
    )
    assert r.ref._pose_sequence == 2
    assert r.ref._pose is None and r.ref.pose() is None
    assert r.ref._pose_time is original[1]["sample_time"]
    assert r.ref._pose_at == pytest.approx(99.8)
    assert r.ref._pose_stamp_ns == 10_000_000_000


def test_unseen_expired_first_delivery_is_not_stored_and_fresh_sample_recovers(reference):
    r = reference
    original = r.message()
    r.ref._on_pose(*original)
    assert r.ref.pose() == Pose2D(1, 0, 0)
    r.clock.ns += 100_000_000
    queued = r.message(
        ns=10_100_000_000,
        received_ns=10_700_000_000,
        receipt_sequence=2,
        x=99,
    )
    r.ref._on_pose(*queued)
    assert r.ref._pose_sequence == 2
    assert r.ref._pose_stamp_ns == 10_100_000_000
    assert r.ref._pose is None  # Check admission, not only pose()'s independent TTL.
    assert r.ref._pose_time is original[1]["sample_time"]
    assert r.ref._pose_at == pytest.approx(100.0)
    r.clock.ns += 100_000_000
    fresh = r.message(ns=10_800_000_000, receipt_sequence=3, x=2)
    r.ref._on_pose(*fresh)
    assert r.ref.pose() == Pose2D(2, 0, 0)
    assert r.ref._pose_time is fresh[1]["sample_time"]
    assert r.ref._pose_at == pytest.approx(100.2)


def test_source_replacement_retires_queued_old_writer_without_restamping(reference):
    r = reference
    old = r.message()
    r.ref._on_pose(*old)
    r.sources["gid"] = b"pose-b"
    assert r.ref.pose() is None
    new = r.message(ns=10_100_000_000, receipt_generation=2, x=2)
    r.ref._on_pose(*new)
    replacement = r.ref.pose()
    r.ref._on_pose(*old)
    assert r.ref.pose() is replacement and replacement.x == 2
    assert r.ref._pose_time is new[1]["sample_time"]


def test_clock_segment_and_map_change_cannot_reuse_old_task_pose(reference):
    r = reference
    old = r.message()
    r.ref._on_pose(*old)
    new = r.message(ns=1_000_000_000, clock_generation=2, receipt_sequence=2, x=2)
    r.ref._on_pose(*new)
    assert r.ref.pose().x == 2
    assert r.ref._pose_identity == (b"pose-a", 1, 2)
    r.ref._on_pose(old[0], {**old[1], "receipt_sequence": 3})
    assert r.ref.pose().x == 2
    r.ref._invalidate_map()
    assert not r.ref.qualified("map", "generation-a")
    r.ref._on_pose(*new)  # A cached pose cannot qualify a replacement map.
    assert r.ref.pose() is None


@pytest.mark.parametrize("failure", ["frame", "quaternion", "stamp", "covariance", "position"])
def test_standard_pose_payload_validation_preserved(reference, failure):
    r = reference
    message, info = r.message()
    if failure == "frame":
        message.header.frame_id = "odom"
    elif failure == "quaternion":
        message.pose.pose.orientation.w = 0
    elif failure == "stamp":
        message.header.stamp.nanosec = 10**9
    elif failure == "covariance":
        message.pose.covariance[0] = float("nan")
    else:
        message.pose.pose.position.x = float("nan")
    r.ref._on_pose(message, info)
    assert r.ref.pose() is None


def test_standard_reference_pose_reply_satisfies_existing_home_ipc_schema(reference):
    from vehicle_manager.agent.pose_reply import PoseReply

    from vehicle_core.values import MapRef

    r = reference
    r.ref._grid.map_ref = MapRef("map", 1)
    r.ref.config.anchor_id = "home-anchor"
    r.ref.config.anchor_pose = Pose2D(0, 0, 0)
    r.ref._on_pose(*r.message(received_ns=10_200_000_000))
    reply = PoseReply.model_validate(r.ref.pose_reply())
    assert reply.v == 1 and reply.map_id == "map"
    assert reply.map_generation == "generation-a"
    assert reply.sampled_at == pytest.approx(99.8)


@pytest.fixture
def subscribed_reference(monkeypatch):
    """Real reference + native registration; only ROS types and pinned grid I/O replaced."""
    from senior_localization import hardware_reference as module
    from senior_sensor_ingress.native_receipt import NativeReceiptGroup

    for package, name in (
        ("geometry_msgs", "PoseWithCovarianceStamped"),
        ("nav_msgs", "OccupancyGrid"),
    ):
        messages = ModuleType(f"{package}.msg")
        setattr(messages, name, type(name, (), {"__module__": messages.__name__}))
        monkeypatch.setitem(sys.modules, messages.__name__, messages)
    clock = N(ns=100_000_000_000)
    monkeypatch.setattr(module, "time", N(monotonic_ns=lambda: clock.ns))
    grid = N(map_ref="map", grid_id="generation-a")
    config = N(
        grid=lambda: grid,
        map_publisher="map_server",
        pose_publisher="amcl",
        source_namespace="/",
        pose_topic="/amcl_pose",
        max_age_s=0.5,
        max_position_variance=0.2,
        max_yaw_variance=0.2,
    )
    monkeypatch.setattr(module, "load_navigation_reference", lambda _: config)
    gids = {"/map": b"map-a", "/amcl_pose": b"pose-a"}
    node = N(
        resolve_topic_name=lambda topic: topic,
        get_logger=lambda: N(info=lambda _: None),
        get_publishers_info_by_topic=lambda topic: [
            N(
                node_name="map_server" if topic == "/map" else "amcl",
                node_namespace="/",
                endpoint_gid=gids[topic],
            )
        ],
    )
    # Exercise subscribe/add without starting a ROS timer or native child.
    group = NativeReceiptGroup.__new__(NativeReceiptGroup)
    group.node, group.inputs, group._started_at = node, [], None
    node._native_receipt_group = group
    ref = module.HardwareReference(node, Path("/unused-pinned-reference.json"))
    ref._map_gid = gids["/map"]
    return N(ref=ref, clock=clock, group=group, gid=gids["/amcl_pose"])


def test_amcl_latejoin_requests_retained_sample_on_existing_native_channel(subscribed_reference):
    r = subscribed_reference
    map_input, pose_input = r.group.inputs
    assert (map_input.topic, map_input.qos, map_input.writer) == ("/map", "latched", "map_server")
    assert (pose_input.topic, pose_input.qos, pose_input.writer, pose_input.namespace) == (
        "/amcl_pose",
        "latched",
        "amcl",
        "/",
    )
    assert pose_input.callback == r.ref._on_pose
    assert pose_input.callback._senior_dds_info is True
    assert pose_input.invalidate == r.ref._invalidate_pose


def test_retained_pose_preserves_native_identity_and_original_age(subscribed_reference):
    r = subscribed_reference
    pose_input = r.group.inputs[1]
    sample = map_sample(
        stamp_ns=10_000_000_000,
        received_ros_ns=10_200_000_000,
        received_steady_ns=r.clock.ns,
        clock_generation=3,
    )
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = covariance[35] = 0.1
    msg = N(
        header=N(stamp=N(sec=10, nanosec=0), frame_id="map"),
        pose=N(
            pose=N(position=N(x=1.0, y=0.0, z=0.0), orientation=N(x=0, y=0, z=0, w=1)),
            covariance=covariance,
        ),
    )
    info = dict(
        publisher_gid=r.gid,
        native_receipt_authenticated=True,
        native_writer_name="amcl",
        native_writer_namespace="/",
        receipt_generation=2,
        receipt_sequence=7,
        sample_time=sample,
    )
    # Discovery alone cannot authenticate even a retained sample.
    pose_input.callback(msg, {**info, "publisher_gid": b""})
    assert r.ref.pose() is None
    pose_input.callback(msg, info)
    assert r.ref.pose() == Pose2D(1, 0, 0)
    assert r.ref._pose_time is sample
    assert r.ref._pose_identity == (b"pose-a", 2, 3)
    assert r.ref._pose_sequence == 7
    assert r.ref._pose_stamp_ns == 10_000_000_000
    assert r.ref._pose_at == pytest.approx(99.8)
    r.clock.ns += 300_000_000
    assert r.ref.pose() is None  # 500ms from measurement, not callback receipt.
    pose_input.callback(msg, {**info, "receipt_sequence": 8})
    assert r.ref._pose_time is sample and r.ref._pose_sequence == 8
    assert r.ref.pose() is None

    # An unseen historical sample first delivered at the deadline is not stored.
    pose_input.invalidate()
    older = map_sample(
        stamp_ns=10_100_000_000,
        received_ros_ns=10_600_000_000,
        received_steady_ns=r.clock.ns,
        clock_generation=3,
    )
    msg.header.stamp.nanosec = 100_000_000
    pose_input.callback(msg, {**info, "sample_time": older, "receipt_sequence": 9})
    assert r.ref._pose is None and r.ref.pose() is None
    assert r.ref._pose_time is sample
    assert r.ref._pose_at == pytest.approx(99.8)


def test_pose_diagnostics_preserve_receipt_deadline_and_bound_repetition(reference):
    r = reference
    message, info = r.message(received_ns=10_200_000_000)
    r.ref._on_pose(message, info)
    accepted = json.loads(r.logs[-1].split(" ", 1)[1])
    assert accepted["reason"] == "accepted"
    assert accepted["receipt_sequence"] == 1
    assert accepted["stamp_ns"] == 10_000_000_000
    assert accepted["sample_steady_ns"] == 99_800_000_000
    assert accepted["covariance"] == [0.1, 0.1, 0.1]
    assert accepted["frame"] == "map"
    assert accepted["publisher_gid"] == b"pose-a".hex()
    r.clock.ns += 300_000_000
    assert r.ref.pose() is None
    assert json.loads(r.logs[-1].split(" ", 1)[1])["reason"] == "age"
    before = len(r.logs)
    for _ in range(100):
        assert r.ref.pose() is None
    assert len(r.logs) == before
    assert r.ref._pose_time is info["sample_time"]
    assert r.ref._pose_at == 99.8


@pytest.mark.parametrize("snapshot_mode", ["state", "pending"])
async def test_static_service_timeout_preserves_failure_and_observes_state(
    monkeypatch, tmp_path, snapshot_mode
):
    import asyncio

    from shared_layers_native_fixture import NativeFixture

    msg, srv = ModuleType("lifecycle_msgs.msg"), ModuleType("lifecycle_msgs.srv")
    msg.State = N(PRIMARY_STATE_ACTIVE=3)
    srv.GetState = N(Request=lambda: None)
    monkeypatch.setitem(sys.modules, msg.__name__, msg)
    monkeypatch.setitem(sys.modules, srv.__name__, srv)
    fixture = NativeFixture(tmp_path)
    rows, destroyed, settled = [], [], []
    fixture.trace = N(emit=lambda stage, **fields: rows.append(dict(stage=stage, **fields)))
    service = N(service_is_ready=lambda: False)
    fixture.node = N(create_client=lambda *args: service, destroy_client=destroyed.append)
    failure = AssertionError("original service timeout")

    async def wait(predicate):
        for _ in range(4):
            await asyncio.sleep(0)
        raise failure

    async def request(kind):
        assert kind == "state.get"
        try:
            if snapshot_mode == "pending":
                await asyncio.Future()
            return {"ok": True, "navigation_ready": False, "localized": False}
        finally:
            settled.append(True)

    fixture.wait = wait
    with pytest.raises(AssertionError) as caught:
        await fixture.static_ready(N(request=request))
    assert caught.value is failure
    assert destroyed == [service]
    phases = [r["phase"] for r in rows if r["stage"] == "static_service"]
    assert phases == ["begin", "wait_error"]
    assert all(
        r["service"] == "/map_server/get_state" for r in rows if r["stage"] == "static_service"
    )
    assert settled == [True]
    snapshots = [r for r in rows if r["stage"] == "static_diagnostic_state"]
    if snapshot_mode == "state":
        assert snapshots[0]["record"]["navigation_ready"] is False
    else:
        assert snapshots == []


@pytest.mark.parametrize("failure", [None, "cells"])
def test_map_diagnostics_observe_exact_match_without_qualifying_rejection(reference, failure):
    r = reference
    r.ref.config.map_publisher = "map_server"
    r.ref._grid = N(width=2, height=1, resolution=0.1, cells=(0, 100), origin=N(x=0, y=0, yaw=0))
    message = N(
        header=N(frame_id="map", stamp=N(sec=1, nanosec=0)),
        info=N(
            width=2,
            height=1,
            resolution=0.1,
            origin=N(position=N(x=0, y=0, z=0), orientation=N(x=0, y=0, z=0, w=1)),
        ),
        data=[0, 100],
    )
    if failure == "cells":
        message.data[1] = 0
    r.ref._on_map(message, {"publisher_gid": b"map-writer"})
    row = json.loads(r.logs[-1].split(" ", 1)[1])
    assert row["reason"] == (failure or "accepted")
    assert (r.ref._map_gid is not None) == (failure is None)
    assert r.ref._map_serial == 2
    assert r.ref._pose is None
    if failure is None or failure == "cells":
        assert row["cells_match"] is (failure is None)


@pytest.mark.parametrize("active", [True, False])
async def test_static_service_observation_retains_active_assertion(monkeypatch, tmp_path, active):
    import asyncio

    from shared_layers_native_fixture import NativeFixture

    msg, srv = ModuleType("lifecycle_msgs.msg"), ModuleType("lifecycle_msgs.srv")
    msg.State = N(PRIMARY_STATE_ACTIVE=3)
    srv.GetState = N(Request=lambda: None)
    monkeypatch.setitem(sys.modules, msg.__name__, msg)
    monkeypatch.setitem(sys.modules, srv.__name__, srv)
    fixture = NativeFixture(tmp_path)
    rows, names, destroyed = [], [], []
    fixture.trace = N(emit=lambda stage, **fields: rows.append(dict(stage=stage, **fields)))
    future = N(done=lambda: True, result=lambda: N(current_state=N(id=3 if active else 2)))
    service = N(service_is_ready=lambda: True, call_async=lambda request: future)

    def create(kind, name):
        names.append(name)
        return service

    fixture.node = N(create_client=create, destroy_client=destroyed.append)

    async def wait(predicate):
        await asyncio.sleep(0)
        assert predicate()

    async def request(kind):
        return {"ok": True, "navigation_ready": True}

    fixture.wait = wait
    # This existing test isolates GetState diagnostics after initialization;
    # the two component tests below exercise the new initialization owner.
    fixture.localization_initialized = True
    result = await fixture.static_ready(N(request=request))
    assert bool(result) is active
    assert names == (
        [
            "/map_server/get_state",
            "/amcl/get_state",
            "/planner_server/get_state",
            "/controller_server/get_state",
            "/bt_navigator/get_state",
        ]
        if active
        else ["/map_server/get_state"]
    )
    assert len(destroyed) == len(names)
    for name in names:
        events = [r for r in rows if r.get("service") == name]
        assert [r["phase"] for r in events] == ["begin", "ready", "request", "response"]
        assert events[-1]["state_id"] == (3 if active else 2)


def test_diagnostic_sink_failure_preserves_pose_deadline_rejection_and_original_error(reference):
    r = reference

    def broken(*args, **kwargs):
        raise RuntimeError("diagnostic sink failed")

    r.ref._node = N(get_logger=lambda: N(info=broken))
    message, info = r.message(received_ns=10_200_000_000)
    r.ref._on_pose(message, info)
    pose = r.ref.pose()
    assert pose == Pose2D(1, 0, 0)
    assert r.ref._pose_time is info["sample_time"]
    assert r.ref._pose_at == 99.8
    r.clock.ns += 300_000_000
    assert r.ref.pose() is None
    rejected, rejected_info = r.message(ns=10_400_000_000, receipt_sequence=2)
    rejected.header.frame_id = "odom"
    r.ref._on_pose(rejected, rejected_info)
    assert r.ref._pose is None and r.ref._pose_stamp_ns == 10_400_000_000
    assert r.ref._pose_time is info["sample_time"] and r.ref._pose_at == 99.8
    original = RuntimeError("original grid failure")

    def grid():
        raise original

    r.ref.grid = grid
    with pytest.raises(RuntimeError) as caught:
        r.ref._on_pose(*r.message(ns=10_500_000_000, receipt_sequence=3))
    assert caught.value is original


def test_map_diagnostic_failure_preserves_acceptance_and_original_cell_error(reference):
    r = reference
    r.ref.config.map_publisher = "map_server"
    r.ref._grid = N(width=1, height=1, resolution=0.1, cells=(0,), origin=N(x=0, y=0, yaw=0))
    message = N(
        header=N(frame_id="map", stamp=N(sec=1, nanosec=0)),
        info=N(
            width=1,
            height=1,
            resolution=0.1,
            origin=N(position=N(x=0, y=0, z=0), orientation=N(x=0, y=0, z=0, w=1)),
        ),
        data=[0],
    )

    def broken(*args):
        raise RuntimeError("diagnostic sink failed")

    r.ref._node = N(get_logger=lambda: N(info=broken))
    r.ref._on_map(message, {"publisher_gid": b"map-writer"})
    assert r.ref._map_gid == b"map-writer" and r.ref._map_serial == 2
    message.data = [1]
    r.ref._on_map(message, {"publisher_gid": b"map-writer"})
    assert r.ref._map_gid is None and r.ref._map_serial == 3
    original = ValueError("original cell failure")

    class BadCell:
        def __int__(self):
            raise original

    message.data = [BadCell()]
    with pytest.raises(ValueError) as caught:
        r.ref._on_map(message, {"publisher_gid": b"map-writer"})
    assert caught.value is original


@pytest.mark.parametrize("boundary", ["fields", "json"])
def test_diagnostic_construction_failure_preserves_qualified_pose(reference, monkeypatch, boundary):
    r = reference
    diagnostic_globals = type(r.ref)._diagnose.__globals__
    if boundary == "json":

        def broken(*args, **kwargs):
            raise TypeError("diagnostic JSON failure")

        monkeypatch.setattr(diagnostic_globals["json"], "dumps", broken)
    else:

        class BadText:
            def __str__(self):
                raise ValueError("diagnostic field failure")

        message, info = r.message()
        # A diagnostic-only metadata field; receipt/source acceptance does not read it.
        info["native_writer_name"] = BadText()
        r.ref._on_pose(message, info)
        assert r.ref.pose() == Pose2D(1, 0, 0)

        class BadGid:
            def __bytes__(self):
                raise ValueError("diagnostic endpoint rendering failed")

        r.ref.config.source_namespace = "/"
        r.ref._node.get_publishers_info_by_topic = lambda topic: [
            N(node_name="other", node_namespace="/", endpoint_gid=BadGid())
        ]
        # A wrong writer is rejected before its GID is used by qualification.
        assert type(r.ref)._source(r.ref, "/amcl_pose", "amcl") is None
        original = RuntimeError("original topology query failure")

        def query(topic):
            raise original

        r.ref._node.get_publishers_info_by_topic = query
        with pytest.raises(RuntimeError) as caught:
            type(r.ref)._source(r.ref, "/amcl_pose", "amcl")
        assert caught.value is original
        return
    r.ref._on_pose(*r.message())
    assert r.ref.pose() == Pose2D(1, 0, 0)


@pytest.mark.parametrize("boundary", ["sink", "fields"])
async def test_warmer_diagnostic_failure_cannot_skip_warm(boundary):
    import ast
    import time

    path = Path("ros2/src/senior_robot/senior_robot/agent_adapter.py")
    tree = ast.parse(path.read_text())
    warmer = next(
        n
        for n in ast.walk(tree)
        if isinstance(n, ast.AsyncFunctionDef) and n.name == "warm_controller"
    )
    calls, rows = [], []
    namespace = dict(
        exiting=False,
        preparing_live=False,
        reference=N(pose=lambda: Pose2D(1, 0, 0)),
        time=time,
        json=json,
    )

    observations = 0

    def available():
        nonlocal observations
        observations += 1
        if boundary == "fields" and observations == 1:
            raise RuntimeError("diagnostic field failure")
        return bool(namespace["controller"]._owned)

    def log(text):
        if boundary == "sink" and not rows:
            rows.append(None)
            raise RuntimeError("diagnostic sink failure")
        rows.append(json.loads(text.split(" ", 1)[1]))

    async def warm():
        calls.append("warm")
        namespace["controller"]._owned.append("owned")
        namespace["exiting"] = True

    async def sleep(seconds):
        assert seconds == 2
        namespace["exiting"] = True

    namespace.update(
        controller=N(available=available, _owned=[], _closing=False, warm=warm),
        node=N(get_logger=lambda: N(info=log, warning=lambda text: None)),
        asyncio=N(sleep=sleep),
    )
    exec(compile(ast.Module(body=[warmer], type_ignores=[]), str(path), "exec"), namespace)
    await namespace["warm_controller"]()
    assert calls == ["warm"]
    returned = rows[-1]
    assert returned["phase"] == "warm_return"
    assert returned["controller_available"] is True and returned["controller_owned"] == 1
    assert returned["pose_available_at_entry"] is True


@pytest.mark.parametrize("phase", ["begin", "wait_error"])
@pytest.mark.parametrize("cancel", [False, True])
async def test_trace_failure_preserves_original_wait_and_settles_ipc(
    monkeypatch, tmp_path, phase, cancel
):
    import asyncio

    from shared_layers_native_fixture import NativeFixture

    from tools.shared_layers_system_trace import TraceWriter
    from vehicle_core.ipc import IpcClient

    msg, srv = ModuleType("lifecycle_msgs.msg"), ModuleType("lifecycle_msgs.srv")
    msg.State = N(PRIMARY_STATE_ACTIVE=3)
    srv.GetState = N(Request=lambda: None)
    monkeypatch.setitem(sys.modules, msg.__name__, msg)
    monkeypatch.setitem(sys.modules, srv.__name__, srv)
    fixture = NativeFixture(tmp_path)
    trace = TraceWriter(tmp_path / "trace.jsonl", {})
    fixture.trace = trace
    emit = trace.emit
    sink_error = RuntimeError("diagnostic sink failed")

    def broken(stage, **fields):
        if stage == "static_service" and fields.get("phase") == phase:
            raise sink_error
        emit(stage, **fields)

    monkeypatch.setattr(trace, "emit", broken)
    destroyed, settled = [], []
    service = N(service_is_ready=lambda: False)
    fixture.node = N(create_client=lambda *args: service, destroy_client=destroyed.append)
    client = IpcClient(tmp_path / "never-opened.sock")
    begun = asyncio.Event()
    original = (
        asyncio.CancelledError("original cancel") if cancel else AssertionError("original timeout")
    )

    async def exchange(frame):
        begun.set()
        try:
            await asyncio.Future()
        finally:
            settled.append(True)

    async def wait(predicate):
        await begun.wait()
        raise original

    client._exchange = exchange
    fixture.wait = wait
    before = asyncio.all_tasks()
    trace.__enter__()
    try:
        with pytest.raises(type(original)) as caught:
            await fixture.static_ready(client)
        assert caught.value is original
        assert destroyed == [service]
        assert settled == [True] and not client._lock.locked()
        assert asyncio.all_tasks() == before
        assert trace.error is sink_error
    finally:
        with pytest.raises(RuntimeError, match="trace writer failed") as trace_failure:
            trace.__exit__()
        assert trace_failure.value.__cause__ is sink_error
        assert not trace._thread.is_alive()


@pytest.fixture
def static_localization_fixture(monkeypatch, tmp_path):
    """Service boundary only: no ROS, native receipt or AMCL posterior emulation."""
    import asyncio

    from shared_layers_native_fixture import NativeFixture

    def initial_request():
        return N(pose=N(header=N(), pose=N(pose=N(position=N(), orientation=N()))))

    modules = {
        "lifecycle_msgs.msg": dict(State=N(PRIMARY_STATE_ACTIVE=3)),
        "lifecycle_msgs.srv": dict(GetState=N(Request=lambda: None)),
        "geometry_msgs.msg": dict(PoseWithCovarianceStamped=object),
        "nav2_msgs.srv": dict(SetInitialPose=N(Request=initial_request)),
        "sensor_msgs.msg": dict(LaserScan=object),
        "std_srvs.srv": dict(Empty=N(Request=lambda: None)),
        "rclpy.qos": dict(
            DurabilityPolicy=N(TRANSIENT_LOCAL=1),
            QoSProfile=lambda **kw: kw,
            qos_profile_sensor_data=object(),
        ),
    }
    for name, values in modules.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        monkeypatch.setitem(sys.modules, name, module)
    fixture = NativeFixture(tmp_path)
    fixture.startup = N(phase="static")
    rows, calls, clients, destroyed, subscriptions, removed = [], [], [], [], [], []
    fixture.trace = N(emit=lambda stage, **fields: rows.append(dict(stage=stage, **fields)))
    stamp = N(sec=123, nanosec=456)

    def create_client(kind, name):
        def call(request):
            future = asyncio.get_running_loop().create_future()
            calls.append(
                N(name=name, request=request, future=future, at=asyncio.get_running_loop().time())
            )
            if name.endswith("/get_state"):
                future.set_result(N(current_state=N(id=3)))
            elif name == "/set_initial_pose":
                future.set_result(N())  # An empty response cannot attest localization.
            return future

        service = N(
            name=name,
            service_is_ready=lambda: True,
            call_async=call,
            remove_pending_request=removed.append,
        )
        clients.append(service)
        return service

    def subscribe(kind, name, callback, qos):
        subscription = N(name=name, callback=callback)
        subscriptions.append(subscription)
        return subscription

    fixture.node = N(
        create_client=create_client,
        destroy_client=destroyed.append,
        create_subscription=subscribe,
        destroy_subscription=subscriptions.remove,
        get_clock=lambda: N(now=lambda: N(to_msg=lambda: stamp)),
    )
    return N(
        fixture=fixture,
        rows=rows,
        calls=calls,
        clients=clients,
        destroyed=destroyed,
        subscriptions=subscriptions,
        removed=removed,
        stamp=stamp,
    )


async def test_static_amcl_seed_uses_active_upstream_prior_without_admitting_ack(
    static_localization_fixture,
):
    import asyncio
    import math

    env = static_localization_fixture
    fixture = env.fixture
    ready = False

    async def request(kind):
        assert kind == "state.get"
        return dict(navigation_ready=ready)

    pending = asyncio.create_task(fixture.static_ready(N(request=request)))
    try:
        await fixture.wait(lambda: fixture.localization_initialized)
        assert not await pending  # Service acknowledgements alone leave static_ready false.
        names = [call.name for call in env.calls]
        assert names[:3] == ["/map_server/get_state", "/amcl/get_state", "/set_initial_pose"]
        assert names.index("/set_initial_pose") < names.index("/controller_server/get_state")
        seed = env.calls[2].request.pose
        assert seed.header.frame_id == "map" and seed.header.stamp is env.stamp
        assert (seed.pose.pose.position.x, seed.pose.pose.position.y) == (0.0, 0.0)
        assert seed.pose.pose.orientation.w == 1.0
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = 0.25
        covariance[35] = (math.pi / 12) ** 2
        assert seed.pose.covariance == covariance
        record = next(r for r in env.rows if r["stage"] == "static_amcl_service")
        assert "a097086719c88f781aa59788eca29ac6ca5e56db" in record["source"]
        assert record["source_sha256"] == (
            "29a6402bf643fc8ea0d2dcea29619d4cbdcaf7014c811332ed4e86e593d3f1af"
        )
        assert record["lifecycle"] == "map_server and amcl GetState ACTIVE observed"
        assert fixture.localization_pose_count == 0
        # Read-only observation retains even zero covariance and the original stamp;
        # neither observation nor the service response sets navigation_ready.
        fixture._observe_localization_pose(N(header=seed.header, pose=N(covariance=[0.0] * 36)))
        observed = env.rows[-1]
        assert observed["stage"] == "static_amcl_pose" and observed["stamp_ns"] == 123000000456
        assert observed["covariance"] == [0.0] * 36
        assert not await fixture.static_ready(N(request=request))
        assert sum(c.name == "/set_initial_pose" for c in env.calls) == 1
        ready = True  # Original IPC owner's result, not inferred from this component.
        assert (await fixture.static_ready(N(request=request)))["navigation_ready"]
        assert fixture.localization_task is None and not env.subscriptions
        assert len(env.destroyed) == len(env.clients)
    finally:
        pending.cancel()
        await asyncio.gather(pending, return_exceptions=True)
        await fixture._stop_static_localization()


async def test_static_amcl_new_scan_bounds_requests_and_settles_phase_exit_and_cancel(
    static_localization_fixture,
):
    import asyncio

    env = static_localization_fixture
    fixture = env.fixture

    def scan(stamp):
        fixture._observe_localization_scan(N(header=N(stamp=N(sec=stamp, nanosec=0))))

    def updates():
        return [c for c in env.calls if c.name == "/request_nomotion_update"]

    fixture.localization_task = asyncio.create_task(fixture._static_localization())
    try:
        await fixture.wait(lambda: fixture.localization_initialized)
        await asyncio.sleep(0.22)
        assert not updates()  # No scan, no busy requests.
        scan(1)
        await fixture.wait(lambda: len(updates()) == 1)
        for stamp in range(2, 20):
            scan(stamp)
        await asyncio.sleep(0.05)
        assert len(updates()) == 1 and not updates()[0].future.done()
        updates()[0].future.set_result(N())
        await fixture.wait(lambda: len(updates()) == 2)
        assert updates()[1].at - updates()[0].at >= 0.2
        assert fixture.localization_pose_count == 0  # ACK did not synthesize posterior.
        fixture.startup.phase = "stopping"  # Original owner is retiring the old AMCL.
        task = fixture.localization_task
        await asyncio.wait_for(asyncio.gather(task, return_exceptions=True), timeout=1)
        await fixture._stop_static_localization()
        assert task.cancelled() and updates()[1].future.cancelled()
        assert env.removed == [updates()[1].future]
        assert not env.subscriptions and len(env.destroyed) == len(env.clients)
        scan(20)
        await asyncio.sleep(0.22)
        assert len(updates()) == 2
        # A shutdown cancellation also settles an in-flight request before clients die.
        fixture.startup.phase = "static"
        fixture.localization_task = asyncio.create_task(fixture._static_localization())
        await fixture.wait(
            lambda: len(env.calls) >= 4 and env.calls[-1].name == "/set_initial_pose"
        )
        scan(21)
        await fixture.wait(lambda: len(updates()) == 3)
        fixture.shutdown_done = True
        await fixture._stop_static_localization()
        assert updates()[-1].future.cancelled() and not env.subscriptions
        assert len(env.destroyed) == len(env.clients)
        assert sum(r["stage"] == "static_amcl_settled" for r in env.rows) == 2
    finally:
        await fixture._stop_static_localization()
