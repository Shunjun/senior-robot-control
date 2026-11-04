"""Real Humble executor tests for producer failures and partial construction."""

import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from senior_robot.rgbd_candidate_producer import RgbdCandidateProducer  # noqa: E402


@pytest.mark.parametrize("sim_time", [False, True])
def test_background_failure_reaches_owned_executor(monkeypatch, sim_time):
    args = ["--ros-args", "-p", "use_sim_time:=true"] if sim_time else None
    rclpy.init(args=args)
    producer = RgbdCandidateProducer()
    executor = SingleThreadedExecutor()
    executor.add_node(producer)
    monkeypatch.setattr(
        producer, "_owner", lambda *_: (_ for _ in ()).throw(RuntimeError("clock failure"))
    )
    try:
        deadline = time.monotonic() + 2
        with pytest.raises(RuntimeError, match="clock failure"):
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
    finally:
        executor.shutdown()
        producer.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("operation", ["create_publisher", "create_subscription"])
def test_partial_construction_destroys_node_and_keeps_original_error(monkeypatch, operation):
    rclpy.init()
    destroyed = []
    original = Node.destroy_node

    def destroy(node):
        destroyed.append(True)
        return original(node)

    original_operation = getattr(Node, operation)

    def fail(node, *args, **kwargs):
        if operation == "create_publisher" and args[1] != "/rgbd_diagnostic/candidate":
            return original_operation(node, *args, **kwargs)
        raise RuntimeError("subscription failed")

    monkeypatch.setattr(Node, "destroy_node", destroy)
    monkeypatch.setattr(Node, operation, fail)
    try:
        with pytest.raises(RuntimeError, match="subscription failed"):
            RgbdCandidateProducer()
        assert destroyed
    finally:
        rclpy.shutdown()


def _commit_fixture(monkeypatch):
    from types import SimpleNamespace

    producer = RgbdCandidateProducer()
    producer._stop.set()
    producer._wake.set()
    producer._thread.join(timeout=5)
    now_ros = producer.get_clock().now().nanoseconds
    now_steady = time.monotonic()
    headers = tuple(
        tuple(
            SimpleNamespace(
                bridge_gid=f"raw-{index}",
                stamp_ns=now_ros,
                received_ros_ns=now_ros,
                received_steady=now_steady,
            )
            for index in range(4)
        )
        for _ in range(2)
    )
    tf_first = object()
    admission = (now_ros, now_steady)
    producer._binding = SimpleNamespace(
        current=SimpleNamespace(relay_gid="tf-owner"),
        first_receipt=tf_first,
        admitted_from=admission,
    )
    result = SimpleNamespace(
        reason="candidate",
        headers=headers,
        tf_receipts=(tf_first, tf_first),
        tf_admitted_from=(admission, admission),
    )
    output = SimpleNamespace(producer_gid="producer-owner")
    published, invalid = [], []
    monkeypatch.setattr(producer._publisher, "publish", published.append)
    monkeypatch.setattr(
        producer, "_publish_invalid_with_owner", lambda reason, _owner: invalid.append(reason)
    )
    return producer, result, output, published, invalid


@pytest.mark.parametrize("missing", [0, 1, 2, 3, "tf"])
def test_commit_rechecks_all_raw_and_tf_owners(monkeypatch, missing):
    rclpy.init()
    producer, result, output, published, invalid = _commit_fixture(monkeypatch)
    calls = []

    def owner(topic, name):
        calls.append(topic)
        if topic == producer._tf_topic:
            return "" if missing == "tf" else "tf-owner"
        if topic == producer._candidate_topic:
            return "producer-owner"
        index = producer._topics.index(topic)
        return "" if missing == index else f"raw-{index}"

    monkeypatch.setattr(producer, "_owner", owner)
    try:
        producer._commit_candidate(result, output, producer._inbox.snapshot())
        assert not published
        assert invalid
        assert (producer._tf_topic if missing == "tf" else producer._topics[missing]) in calls
    finally:
        producer.destroy_node()
        rclpy.shutdown()


def test_commit_checks_original_deadline_after_acquiring_lock(monkeypatch):
    import threading

    rclpy.init()
    producer, result, output, published, invalid = _commit_fixture(monkeypatch)

    def owner(topic, name):
        if topic == producer._tf_topic:
            return "tf-owner"
        if topic == producer._candidate_topic:
            return "producer-owner"
        return f"raw-{producer._topics.index(topic)}"

    monkeypatch.setattr(producer, "_owner", owner)
    token = producer._inbox.snapshot()
    producer._inbox._lock.acquire()
    thread = threading.Thread(target=lambda: producer._commit_candidate(result, output, token))
    try:
        thread.start()
        time.sleep(0.4)
    finally:
        producer._inbox._lock.release()
    thread.join(timeout=2)
    try:
        assert not thread.is_alive()
        assert not published
        assert "candidate_expired" in invalid
    finally:
        producer.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("old_owner", ["", "previous-relay"])
def test_transient_tf_retried_when_graph_discovery_arrives_later(monkeypatch, old_owner):
    import threading

    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from senior_motion_msgs.msg import BoundedStaticTf, StaticTransformEdge

    rclpy.init(
        args=["--ros-args", "-p", "static_background_declared:=true", "-p", "camera_frame:=optical"]
    )
    producer = RgbdCandidateProducer()
    source = rclpy.create_node("static_tf_receipt")
    pub = source.create_publisher(
        BoundedStaticTf,
        "/rgbd_diagnostic/tf",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    original = producer._owner
    graph_ready = threading.Event()
    saw_missing = threading.Event()

    def owner(topic, name):
        if topic == producer._tf_topic and not graph_ready.is_set():
            saw_missing.set()
            return old_owner
        return original(topic, name)

    take = producer._inbox.take

    def busy_inbox():
        item = take()
        if item is None and saw_missing.is_set():
            return producer._inbox.snapshot(), (0, None), ""
        return item

    monkeypatch.setattr(producer._inbox, "take", busy_inbox)
    monkeypatch.setattr(producer, "_owner", owner)
    executor = MultiThreadedExecutor()
    executor.add_node(producer)
    executor.add_node(source)
    spin = threading.Thread(target=executor.spin)
    spin.start()
    try:
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline and not pub.get_subscription_count():
            time.sleep(0.01)
        assert pub.get_subscription_count()
        owners = source.get_publishers_info_by_topic("/rgbd_diagnostic/tf")
        assert len(owners) == 1
        status = BoundedStaticTf()
        status.valid = status.admitted = True
        status.bridge_boot_id = "tfboot"
        status.bridge_gid = bytes(owners[0].endpoint_gid).hex()
        status.raw_gid = "01"
        status.generation = status.sequence = status.first_sequence = 1
        status.received_ros = status.first_received_ros = status.admitted_from_ros = (
            source.get_clock().now().to_msg()
        )
        status.received_steady = status.first_received_steady = status.admitted_from_steady = (
            time.monotonic()
        )
        edge = StaticTransformEdge()
        edge.parent_frame, edge.child_frame = "base_link", "optical"
        edge.rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
        status.edges = [edge]
        pub.publish(status)
        assert saw_missing.wait(2)
        graph_ready.set()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and producer._binding.current is None:
            time.sleep(0.01)
        assert producer._binding.current is not None
    finally:
        executor.shutdown()
        spin.join(timeout=5)
        producer.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_commit_rejects_missing_raw_owner_in_real_dds_graph(monkeypatch):
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import BoundedRawCameraInfo, BoundedRawImage, BoundedStaticTf

    rclpy.init()
    producer, result, output, published, invalid = _commit_fixture(monkeypatch)
    raw = rclpy.create_node("raw_receipt")
    tf = rclpy.create_node("static_tf_receipt")
    pubs = [
        raw.create_publisher(
            BoundedRawImage if index < 2 else BoundedRawCameraInfo,
            producer._topics[index],
            qos_profile_sensor_data,
        )
        for index in range(3)
    ]
    tf_pub = tf.create_publisher(BoundedStaticTf, producer._tf_topic, qos_profile_sensor_data)
    assert len(pubs) == 3 and tf_pub is not None
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end:
            raw_owners = [producer._owner(topic, "raw_receipt") for topic in producer._topics]
            tf_owner = producer._owner(producer._tf_topic, "static_tf_receipt")
            own_gid = producer._owner(producer._candidate_topic, "rgbd_candidate_producer")
            if all(raw_owners[:3]) and not raw_owners[3] and tf_owner and own_gid:
                break
            time.sleep(0.01)
        assert all(raw_owners[:3]) and not raw_owners[3] and tf_owner and own_gid
        producer._binding.current.relay_gid = tf_owner
        output.producer_gid = own_gid
        for group in result.headers:
            for index in range(3):
                group[index].bridge_gid = raw_owners[index]
        producer._commit_candidate(result, output, producer._inbox.snapshot())
        assert not published
        assert invalid == ["dependency_unavailable"]
    finally:
        producer.destroy_node()
        raw.destroy_node()
        tf.destroy_node()
        rclpy.shutdown()


def test_thread_start_failure_rolls_back_and_preserves_exception(monkeypatch):
    import threading

    rclpy.init()
    destroyed = []
    original_destroy = Node.destroy_node
    original_start = threading.Thread.start

    def destroy(node):
        destroyed.append(True)
        return original_destroy(node)

    def fail(thread):
        if thread.name == "rgbd-coordinator":
            raise RuntimeError("coordinator start failed")
        return original_start(thread)

    monkeypatch.setattr(Node, "destroy_node", destroy)
    monkeypatch.setattr(threading.Thread, "start", fail)
    try:
        with pytest.raises(RuntimeError, match="coordinator start failed"):
            RgbdCandidateProducer()
        assert destroyed
    finally:
        rclpy.shutdown()


def test_raw_owner_lost_before_final_query_is_rejected_with_real_graph(monkeypatch):
    import threading

    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import BoundedRawCameraInfo, BoundedRawImage, BoundedStaticTf

    rclpy.init()
    producer, result, output, published, invalid = _commit_fixture(monkeypatch)
    raw = rclpy.create_node("raw_receipt")
    tf = rclpy.create_node("static_tf_receipt")
    pubs = [
        raw.create_publisher(
            BoundedRawImage if index < 2 else BoundedRawCameraInfo,
            topic,
            qos_profile_sensor_data,
        )
        for index, topic in enumerate(producer._topics)
    ]
    tf_pub = tf.create_publisher(BoundedStaticTf, producer._tf_topic, qos_profile_sensor_data)
    assert tf_pub is not None
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end:
            raw_owners = [producer._owner(topic, "raw_receipt") for topic in producer._topics]
            tf_owner = producer._owner(producer._tf_topic, "static_tf_receipt")
            own_gid = producer._owner(producer._candidate_topic, "rgbd_candidate_producer")
            if all(raw_owners) and tf_owner and own_gid:
                break
            time.sleep(0.01)
        assert all(raw_owners) and tf_owner and own_gid
        producer._binding.current.relay_gid = tf_owner
        output.producer_gid = own_gid
        for group in result.headers:
            for index in range(4):
                group[index].bridge_gid = raw_owners[index]
        token = producer._inbox.snapshot()
        original_owner = producer._owner
        query_started = threading.Event()
        query_release = threading.Event()

        def delayed_owner(topic, name):
            if topic == producer._topics[3]:
                query_started.set()
                query_release.wait(3)
            return original_owner(topic, name)

        monkeypatch.setattr(producer, "_owner", delayed_owner)
        commit = threading.Thread(target=lambda: producer._commit_candidate(result, output, token))
        try:
            commit.start()
            assert query_started.wait(2)
            assert raw.destroy_publisher(pubs[3])
            end = time.monotonic() + 3
            while time.monotonic() < end and original_owner(producer._topics[3], "raw_receipt"):
                time.sleep(0.01)
            assert not original_owner(producer._topics[3], "raw_receipt")
        finally:
            query_release.set()
            commit.join(timeout=3)
        assert not commit.is_alive()
        assert not published
        assert invalid == ["dependency_unavailable"]
    finally:
        producer.destroy_node()
        raw.destroy_node()
        tf.destroy_node()
        rclpy.shutdown()


def test_tf_pending_cannot_cross_overflow_dropping_newer_invalid(monkeypatch):
    import copy
    import threading

    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from senior_motion_msgs.msg import BoundedStaticTf, StaticTransformEdge

    rclpy.init(
        args=["--ros-args", "-p", "static_background_declared:=true", "-p", "camera_frame:=optical"]
    )
    producer = RgbdCandidateProducer()
    source = rclpy.create_node("static_tf_receipt")
    pub = source.create_publisher(
        BoundedStaticTf,
        "/rgbd_diagnostic/tf",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    original = producer._owner
    entered = threading.Event()
    release = threading.Event()

    def owner(topic, name):
        if topic == producer._tf_topic and not release.is_set():
            entered.set()
            release.wait(3)
            return ""
        return original(topic, name)

    monkeypatch.setattr(producer, "_owner", owner)
    executor = MultiThreadedExecutor()
    executor.add_node(producer)
    executor.add_node(source)
    spin = threading.Thread(target=executor.spin)
    spin.start()
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end and not pub.get_subscription_count():
            time.sleep(0.01)
        assert pub.get_subscription_count()
        owners = source.get_publishers_info_by_topic("/rgbd_diagnostic/tf")
        assert len(owners) == 1
        status = BoundedStaticTf()
        status.valid = status.admitted = True
        status.bridge_boot_id = "tfboot"
        status.bridge_gid = bytes(owners[0].endpoint_gid).hex()
        status.raw_gid = "01"
        status.generation = status.sequence = status.first_sequence = 1
        status.received_ros = status.first_received_ros = status.admitted_from_ros = (
            source.get_clock().now().to_msg()
        )
        status.received_steady = status.first_received_steady = status.admitted_from_steady = (
            time.monotonic()
        )
        edge = StaticTransformEdge()
        edge.parent_frame, edge.child_frame = "base_link", "optical"
        edge.rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
        status.edges = [edge]
        pub.publish(status)
        assert entered.wait(3)
        for _ in range(16):
            producer._enqueue((0, None))
        invalid = copy.deepcopy(status)
        invalid.valid = False
        invalid.generation = 2
        pub.publish(invalid)
        producer._enqueue((4, producer._tf_epoch, invalid))
        release.set()
        time.sleep(0.4)
        assert producer._binding.current is None
        assert producer._engine is None
    finally:
        release.set()
        executor.shutdown()
        spin.join(timeout=5)
        producer.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


def test_slow_graph_lookup_does_not_block_ros_ingress(monkeypatch):
    import threading

    rclpy.init()
    producer, result, output, published, invalid = _commit_fixture(monkeypatch)
    entered = threading.Event()
    release = threading.Event()
    enqueued = threading.Event()

    def owner(topic, name):
        if topic == producer._candidate_topic:
            entered.set()
            release.wait(2)
            return "producer-owner"
        if topic == producer._tf_topic:
            return "tf-owner"
        return f"raw-{producer._topics.index(topic)}"

    monkeypatch.setattr(producer, "_owner", owner)
    token = producer._inbox.snapshot()
    commit = threading.Thread(target=lambda: producer._commit_candidate(result, output, token))
    ingress = threading.Thread(target=lambda: (producer._enqueue((0, None)), enqueued.set()))
    try:
        commit.start()
        assert entered.wait(2)
        ingress.start()
        assert enqueued.wait(0.2)
        release.set()
        commit.join(timeout=3)
        ingress.join(timeout=3)
        assert not published
    finally:
        release.set()
        commit.join(timeout=3)
        if ingress.ident is not None:
            ingress.join(timeout=3)
        producer.destroy_node()
        rclpy.shutdown()


def test_bounded_input_burst_is_drained_without_per_item_timer_delay():
    rclpy.init()
    producer = RgbdCandidateProducer()
    try:
        producer._enabled = True
        for _ in range(12):
            producer._enqueue((0, None))
        deadline = time.monotonic() + 0.15
        while time.monotonic() < deadline:
            with producer._inbox._lock:
                pending = len(producer._inbox._items)
            if not pending:
                break
            time.sleep(0.005)
        assert pending == 0
    finally:
        producer.destroy_node()
        rclpy.shutdown()


def test_overflow_reacquires_latched_valid_tf_without_relabeling_or_old_epoch():
    import copy
    import os
    import threading

    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from senior_motion_msgs.msg import BoundedStaticTf, StaticTransformEdge

    rclpy.init(
        args=["--ros-args", "-p", "static_background_declared:=true", "-p", "camera_frame:=optical"]
    )
    producer = RgbdCandidateProducer()
    source = rclpy.create_node("static_tf_receipt")
    pub = source.create_publisher(
        BoundedStaticTf,
        "/rgbd_diagnostic/tf",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    executor = MultiThreadedExecutor()
    executor.add_node(producer)
    executor.add_node(source)
    spin = threading.Thread(target=executor.spin)
    spin.start()
    try:
        end = time.monotonic() + 5
        while time.monotonic() < end and not pub.get_subscription_count():
            time.sleep(0.01)
        assert pub.get_subscription_count()
        owners = source.get_publishers_info_by_topic("/rgbd_diagnostic/tf")
        assert len(owners) == 1
        status = BoundedStaticTf()
        status.valid = status.admitted = True
        status.bridge_boot_id = "tfboot"
        status.bridge_gid = bytes(owners[0].endpoint_gid).hex()
        status.raw_gid = "01"
        status.generation = status.sequence = status.first_sequence = 1
        status.received_ros = status.first_received_ros = status.admitted_from_ros = (
            source.get_clock().now().to_msg()
        )
        status.received_steady = status.first_received_steady = status.admitted_from_steady = (
            time.monotonic()
        )
        edge = StaticTransformEdge()
        edge.parent_frame, edge.child_frame = "base_link", "optical"
        edge.rotation_xyzw = [0.0, 0.0, 0.0, 1.0]
        status.edges = [edge]
        pub.publish(status)
        end = time.monotonic() + 5
        while time.monotonic() < end and (
            producer._binding.current is None or producer._engine is None
        ):
            time.sleep(0.01)
        assert producer._binding.current is not None and producer._engine is not None
        first, admission = producer._binding.first_receipt, producer._binding.admitted_from
        baseline_threads = threading.active_count()
        baseline_fds = len(os.listdir("/proc/self/fd")) if os.path.isdir("/proc/self/fd") else None
        for _ in range(8):
            old_epoch, old_generation, old_engine = (
                producer._tf_epoch,
                producer._generation,
                producer._engine,
            )
            old_worker = old_engine.coordinator.worker
            old_process = old_worker._process
            old_worker_generation = old_worker.generation
            old_worker_sequence = old_worker.sequence_frontier
            old_ledger = old_engine.coordinator.ledger
            old_joiner = old_engine.joiner
            old_floor = (old_ledger._fresh_after_ros_ns, old_ledger._fresh_after_steady)
            producer._inbox.revoke("overflow")
            producer._wake.set()
            end = time.monotonic() + 5
            while time.monotonic() < end:
                if (
                    producer._tf_epoch > old_epoch
                    and producer._binding.current is not None
                    and producer._engine is not None
                    and producer._engine is not old_engine
                ):
                    break
                time.sleep(0.01)
            assert producer._tf_epoch > old_epoch and producer._generation > old_generation
            assert producer._engine is not None and producer._engine is not old_engine
            assert not old_process.is_alive()
            assert producer._engine.joiner is old_joiner
            assert producer._engine.coordinator.ledger is old_ledger
            assert producer._engine.coordinator.worker.generation > old_worker_generation
            assert producer._engine.coordinator.worker.sequence_frontier >= old_worker_sequence
            assert old_ledger._fresh_after_ros_ns >= old_floor[0]
            assert old_ledger._fresh_after_steady > old_floor[1]
            assert (
                producer._binding.first_receipt == first
                and producer._binding.admitted_from == admission
            )
            assert pub.get_subscription_count() == 1
            stale = copy.deepcopy(status)
            stale.valid = False
            stale.generation = 3
            successor_generation = producer._generation
            successor_engine = producer._engine
            for _ in range(32):
                producer._enqueue((4, old_epoch, stale))
            time.sleep(0.1)
            assert producer._generation == successor_generation
            assert producer._engine is successor_engine
            assert (
                producer._binding.first_receipt == first and producer._binding.current is not None
            )
        time.sleep(0.1)
        assert threading.active_count() <= baseline_threads + 2
        if baseline_fds is not None:
            assert len(os.listdir("/proc/self/fd")) <= baseline_fds + 3
    finally:
        executor.shutdown()
        spin.join(timeout=5)
        producer.destroy_node()
        source.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("failure", [RuntimeError, SystemExit])
def test_shutdown_surfaces_unreported_worker_fault(monkeypatch, failure):
    import threading

    entered = threading.Event()

    def work(producer):
        entered.set()
        assert producer._stop.wait(2)
        raise failure("worker shutdown failed")

    monkeypatch.setattr(RgbdCandidateProducer, "_work", work)
    rclpy.init()
    producer = RgbdCandidateProducer()
    try:
        assert entered.wait(2)
        with pytest.raises(RuntimeError) as error:
            producer.destroy_node()
        fault = error.value if failure is RuntimeError else error.value.__cause__
        assert isinstance(fault, failure)
        assert str(fault) == "worker shutdown failed"
        assert not producer._thread.is_alive()
    finally:
        rclpy.shutdown()


def test_clock_rollback_terminates_producer(monkeypatch):
    from types import SimpleNamespace

    ticks = iter((10, 9))
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=next(ticks, 9)))
    monkeypatch.setattr(RgbdCandidateProducer, "get_clock", lambda _node: clock)
    monkeypatch.setattr(RgbdCandidateProducer, "_invalid", lambda *_: True)
    rclpy.init()
    producer = RgbdCandidateProducer()
    executor = SingleThreadedExecutor()
    executor.add_node(producer)
    try:
        deadline = time.monotonic() + 2
        with pytest.raises(RuntimeError, match="RGB-D clock moved backwards"):
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
    finally:
        executor.shutdown()
        producer.destroy_node()
        rclpy.shutdown()
