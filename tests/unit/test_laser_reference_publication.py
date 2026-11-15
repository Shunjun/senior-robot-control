"""Real admission/fusion and handlers, ROS I/O and completion schedule only controlled."""

import copy
import json
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest
from motion_reference_test_support import ReadySubscription
from senior_robot import motion_core_output
from senior_robot.motion_observation_adapter import NativeObservationRosBridge
from senior_robot.simulation_motion_profile import SimulationMotionProfile


def native(value):
    return json.loads(json.dumps(value), object_hook=lambda fields: N(**fields))


def stamp(seconds):
    ns = round(seconds * 1e9)
    return {"sec": ns // 1_000_000_000, "nanosec": ns % 1_000_000_000}


class Node:
    def __init__(self, clock):
        self.clock = clock
        self.callbacks = {}
        self.packets = []
        self.events = []
        self.ready = None
        self.publisher_count = 0

    def create_publisher(self, _type, topic, _depth):
        if topic == "/motion/core_reference":
            self.publisher_count += 1

        def publish(message):
            if topic == "/motion/core_reference":
                self.events.append(("publish", self.clock.steady))
                self.packets.append(json.loads(message.data))
                if self.ready is not None:
                    self.ready.ready.append((message, {"publisher_gid": b"fusion"}))

        return N(publish=publish)

    def create_subscription(self, message_type, topic, callback, _qos):
        self.callbacks[topic] = callback
        sub = ReadySubscription(message_type, callback)
        if topic == "/motion/core_reference":
            self.ready = sub
        return sub

    def create_timer(self, period, callback, **_kwargs):
        assert period == 0.05
        return N(callback=callback)

    def resolve_topic_name(self, topic):
        return topic

    def get_parameter(self, name):
        return N(value="odom")

    def get_clock(self):
        return N(now=lambda: N(nanoseconds=round(self.clock.ros * 1e9)))

    def get_publishers_info_by_topic(self, topic):
        name, gid = {
            "/wheel/odom": ("drive", b"wheel"),
            "/imu/data": ("senior_sim_bridge", b"imu"),
            "/motion/scan": ("scan_motion_node", b"laser"),
            "/motion/core_reference": ("motion_fusion", b"fusion"),
        }[topic]
        return [N(node_name=name, node_namespace="/", endpoint_gid=gid)]

    def get_logger(self):
        return N(warning=lambda message: self.events.append(("warning", message)))


@pytest.fixture
def phase(monkeypatch):
    case = json.loads(Path("tests/fixtures/laser_publication_phase.json").read_text())
    clock = N(steady=0.0, ros=0.0)
    for name, fields in {
        "nav_msgs.msg": {"Odometry": N},
        "rclpy.clock": {"Clock": lambda **_: N(), "ClockType": N(STEADY_TIME=1)},
        "rclpy.qos": {"qos_profile_sensor_data": N()},
        "rclpy.time": {"Time": N},
        "sensor_msgs.msg": {"Imu": N},
        "senior_motion_msgs.msg": {"ScanMotion": N},
        "std_msgs.msg": {"String": lambda: N(data="")},
        "tf2_ros": {"TransformException": type("TransformException", (Exception,), {})},
        "senior_robot.motion_lifecycle": {"with_dds_info": lambda callback: callback},
    }.items():
        module = ModuleType(name)
        vars(module).update(fields)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setitem(sys.modules, "time", N(monotonic=lambda: clock.steady))
    monkeypatch.setattr(motion_core_output, "time", N(monotonic=lambda: clock.steady))
    monkeypatch.setattr("senior_robot.native_receipt.time", N(monotonic=lambda: clock.steady))
    monkeypatch.setattr(
        "senior_robot.motion_observation_adapter.time", N(monotonic=lambda: clock.steady)
    )
    node = Node(clock)
    bridge = NativeObservationRosBridge(node, SimulationMotionProfile())

    def points(end):
        wheel = N(
            child_frame_id="base_link",
            header=N(stamp=native(stamp(end)), frame_id="odom"),
            twist=N(twist=N(linear=N(x=0), angular=N(z=0))),
        )
        imu = N(header=N(stamp=native(stamp(end)), frame_id="base_link"), angular_velocity=N(z=0))
        node.callbacks["/wheel/odom"](wheel, {"publisher_gid": b"wheel"})
        node.callbacks["/imu/data"](imu, {"publisher_gid": b"imu"})

    # History for the legacy progress/diagnostic publisher; driving owns raw ingress.
    for index in range(4, 16):
        end = index / 10
        clock.steady, clock.ros = 128391.498658892 + end, end + 0.02
        points(end)
        message = copy.deepcopy(case["old_laser"])
        message["header"]["stamp"], message["interval_start"] = stamp(end), stamp(end - 0.1)
        node.callbacks["/motion/scan"](native(message), {"publisher_gid": b"laser"})
    clock.steady, clock.ros = case["old_snapshot"].values()
    bridge._timer.callback()
    assert node.packets[-1]["reference"]["end_ros"] == 1.5
    assert node.packets[-1]["reference"]["average"] is not None
    consumer = motion_core_output.MotionCoreOutput(
        node, SimulationMotionProfile(), ingress_factory=None
    )
    consumer._on_reference(N(data=json.dumps(node.packets[-1])), {"publisher_gid": b"fusion"})
    node.events.clear()
    clock.steady, clock.ros = case["new_received"].values()
    points(1.6)
    accepted = []
    before_return = []
    handler = bridge.core.laser

    def completed(*args):
        result = handler(*args)
        if before_return:
            before_return[0]()
        clock.steady, clock.ros = case["handler_finished"].values()
        node.events.append(("handler_return", clock.steady, result))
        accepted.append(result)
        return result

    callback = bridge._callback(completed, "/motion/scan")
    yield N(
        case=case,
        clock=clock,
        bridge=bridge,
        node=node,
        consumer=consumer,
        callback=callback,
        accepted=accepted,
        points=points,
        before_return=before_return,
    )
    consumer.close()


def test_accepted_laser_is_published_after_handler_return_before_next_timer(phase):
    before = len(phase.node.packets)
    phase.callback(native(phase.case["new_laser"]), {"publisher_gid": b"laser"})
    assert phase.accepted == [True]
    assert phase.node.events[0] == (
        "handler_return",
        phase.case["handler_finished"]["steady"],
        True,
    )
    # Sample actual core inputs at captured tick, without invoking the later producer timer.
    phase.clock.steady, phase.clock.ros = phase.case["tick"].values()
    # Explicit offline raw replay: production has no fused-snapshot subscription.
    phase.consumer._on_reference(
        N(data=json.dumps(phase.node.packets[-1])), {"publisher_gid": b"fusion"}
    )
    _, body, _ = phase.consumer._inputs(phase.clock.steady)
    assert body is not None, "handler accepted; consumer still has old end_ros=1.5 at ROS1.661"
    assert len(phase.node.packets) == before + 1
    assert phase.node.events[1][0] == "publish"
    assert phase.node.packets[-1]["reference"]["end_ros"] == 1.6
    assert body.stamp_steady == phase.case["new_received"]["steady"]
    roots = phase.consumer.sensors.latest.vx.roots
    assert body.deadline_steady <= min(
        r.received_steady + 1 - (r.received_ros - r.stamp_ros) for r in roots
    )
    assert phase.node.publisher_count == 1


def test_duplicate_and_rejected_laser_do_not_recompute_or_publish(phase):
    message = native(phase.case["new_laser"])
    phase.callback(message, {"publisher_gid": b"laser"})
    before = (len(phase.node.packets), phase.bridge.core._sequence)
    phase.callback(message, {"publisher_gid": b"laser"})
    assert phase.accepted == [True, False]
    assert (len(phase.node.packets), phase.bridge.core._sequence) == before
    phase.callback(message, {"publisher_gid": b"other"})
    assert phase.accepted == [True, False]  # Rejected origin never reaches admission.
    assert (len(phase.node.packets), phase.bridge.core._sequence) == before
    rejected = copy.deepcopy(phase.case["new_laser"])
    rejected["header"]["stamp"] = stamp(1.7)
    rejected["valid"] = False
    phase.clock.steady += 0.1
    phase.clock.ros = 1.72
    phase.case["handler_finished"] = dict(steady=phase.clock.steady, ros=phase.clock.ros)
    phase.callback(native(rejected), {"publisher_gid": b"laser"})
    assert phase.accepted[-1] is False
    assert (len(phase.node.packets), phase.bridge.core._sequence) == before
    phase.bridge._timer.callback()
    assert len(phase.node.packets) == before[0] + 1
    assert phase.node.packets[-1]["reference"]["mode"] not in {"laser_reference", "fused_reference"}


def test_wheel_and_imu_updates_leave_snapshot_work_to_periodic_timer(phase):
    before = (len(phase.node.packets), phase.bridge.core._sequence)
    for end in (1.601, 1.602):
        phase.clock.steady += 0.001
        phase.clock.ros += 0.001
        phase.points(end)
    assert (len(phase.node.packets), phase.bridge.core._sequence) == before
    phase.bridge._timer.callback()
    assert (len(phase.node.packets), phase.bridge.core._sequence) == (before[0] + 1, before[1] + 1)


def test_new_laser_epoch_does_not_add_old_five_sample_recovery_gate(phase):
    message = copy.deepcopy(phase.case["new_laser"])
    message["source_epoch"] = "new-laser-epoch"
    before = len(phase.node.packets)
    phase.callback(native(message), {"publisher_gid": b"laser"})
    assert phase.accepted == [True]
    assert len(phase.node.packets) == before + 1
    laser = next(s for s in phase.node.packets[-1]["sources"] if s["source"] == "laser")
    assert laser["ready"]
    assert phase.node.packets[-1]["reference"]["average"] is not None


def test_completion_after_control_sample_cannot_retroactively_supply_body(phase):
    earlier_body = []
    phase.case["handler_finished"] = dict(steady=phase.case["tick"]["steady"] + 0.001, ros=1.665)

    def independent_consumer_tick():
        # Producer handler has not returned to its own callback; consumer runs independently.
        phase.clock.steady, phase.clock.ros = phase.case["tick"].values()
        phase.consumer._consume_ready_reference()
        earlier_body.append(phase.consumer._inputs(phase.clock.steady)[1])

    phase.before_return.append(independent_consumer_tick)
    phase.callback(native(phase.case["new_laser"]), {"publisher_gid": b"laser"})
    assert phase.accepted == [True] and earlier_body[0] is not None
    old_sequence = phase.consumer.sensors.estimator._frontiers["laser"].roots[0].sequence
    assert old_sequence < phase.node.packets[-1]["sources"][0]["sample"]["sequence"]
    assert phase.node.events[-1] == ("publish", phase.case["handler_finished"]["steady"])
    phase.consumer._on_reference(
        N(data=json.dumps(phase.node.packets[-1])), {"publisher_gid": b"fusion"}
    )
    assert phase.consumer._inputs(phase.clock.steady)[1] is not None
    assert phase.consumer.sensors.estimator._frontiers["laser"].roots[0].sequence > old_sequence


def test_periodic_and_immediate_publications_share_sequence_and_original_deadline(phase):
    phase.callback(native(phase.case["new_laser"]), {"publisher_gid": b"laser"})
    immediate = phase.node.packets[-1]
    phase.clock.steady, phase.clock.ros = phase.case["next_periodic"].values()
    phase.bridge._timer.callback()
    periodic = phase.node.packets[-1]
    assert periodic["sequence"] == immediate["sequence"] + 1
    assert periodic["boot_id"] == immediate["boot_id"]
    assert periodic["reference"]["end_ros"] == immediate["reference"]["end_ros"] == 1.6

    def laser(packet):
        return next(s for s in packet["sources"] if s["source"] == "laser")

    assert laser(periodic)["sample"] == laser(immediate)["sample"]
    assert laser(periodic)["valid_until_steady"] == laser(immediate)["valid_until_steady"]
    assert phase.node.publisher_count == 1
