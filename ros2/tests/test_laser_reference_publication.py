"""Real admission/fusion and handlers, ROS I/O and completion schedule only controlled."""

import copy
import json
import sys
from pathlib import Path
from types import ModuleType
from types import SimpleNamespace as N

import pytest
import senior_motion_control.motion_core_output as motion_core_output
from control_process_test_support import (
    capture_inputs,
    generic_motion_profile,
    make_core,
    publish_inputs,
    read_inputs,
)
from motion_reference_test_support import ReadySubscription
from senior_motion_estimation.motion_observation_adapter import NativeObservationRosBridge
from sensor_time_test_support import capture_fields, sample_time

from vehicle_core.sample_time import SampleTime


def native(value):
    return json.loads(json.dumps(value), object_hook=lambda fields: N(**fields))


def laser_with_captures(value, *, first_time, last_time):
    """Geometry replay with explicit immutable original endpoint records."""
    value = {
        key: item
        for key, item in copy.deepcopy(value).items()
        if key not in {"first_time", "last_time"}
    }
    for name, time in (("first_capture", first_time), ("second_capture", last_time)):
        fields = capture_fields(time=time)
        value[name] = dict(
            **{key: vars(item) if isinstance(item, N) else item for key, item in fields.items()},
            frame_id="laser",
            raw_gid="cc" * 24,
            bridge_gid="dd" * 24,
            source_epoch="laser-source",
            bridge_boot_id="boot",
            capture_generation=1,
            sequence=max(1, time.stamp_ns),
            valid=True,
            reason="",
        )
    return native(value)


def receipt_info(gid, sample):
    return {
        "publisher_gid": gid,
        "native_receipt_authenticated": True,
        "native_writer_name": {
            b"wheel": "drive",
            b"imu": "senior_sim_bridge",
            b"laser": "scan_motion_node",
        }[gid],
        "native_writer_namespace": "/",
        "native_received_steady": sample.received_steady_ns / 1e9,
        "sample_time": sample,
        "receipt_generation": 1,
        "receipt_sequence": sample.stamp_ns,
    }


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
        "senior_interfaces.msg": {"ScanMotion": N},
        "std_msgs.msg": {"String": lambda: N(data="")},
        "tf2_ros": {"TransformException": type("TransformException", (Exception,), {})},
        "senior_sensor_ingress.ros_lifecycle": {"with_dds_info": lambda callback: callback},
    }.items():
        module = ModuleType(name)
        vars(module).update(fields)
        monkeypatch.setitem(sys.modules, name, module)
    test_time = N(monotonic=lambda: clock.steady, monotonic_ns=lambda: round(clock.steady * 1e9))
    monkeypatch.setitem(sys.modules, "time", test_time)
    monkeypatch.setattr(motion_core_output, "time", test_time)
    monkeypatch.setattr("senior_sensor_ingress.native_receipt.time", test_time)
    monkeypatch.setattr("senior_motion_estimation.motion_observation_adapter.time", test_time)

    # Replace only ROS I/O; handlers still receive full native receipt metadata.
    def subscribe(node, message_type, topic, callback, **_kwargs):
        return node.create_subscription(message_type, topic, callback, 10)

    monkeypatch.setattr("senior_sensor_ingress.native_receipt.subscribe", subscribe)
    node = Node(clock)
    bridge = NativeObservationRosBridge(node, generic_motion_profile())

    def points(end):
        wheel = N(
            child_frame_id="base_link",
            header=N(stamp=native(stamp(end)), frame_id="odom"),
            twist=N(twist=N(linear=N(x=0), angular=N(z=0))),
        )
        imu = N(header=N(stamp=native(stamp(end)), frame_id="base_link"), angular_velocity=N(z=0))
        sample = sample_time(
            stamp_ns=round(end * 1e9),
            received_ros_ns=round(clock.ros * 1e9),
            received_steady_ns=round(clock.steady * 1e9),
        )
        node.callbacks["/wheel/odom"](wheel, receipt_info(b"wheel", sample))
        node.callbacks["/imu/data"](imu, receipt_info(b"imu", sample))

    # History for the legacy progress/diagnostic publisher; driving owns raw ingress.
    originals = {}
    for index in range(3, 16):
        end = index / 10
        originals[round(end * 1e9)] = sample_time(
            stamp_ns=round(end * 1e9),
            received_ros_ns=round((end + 0.02) * 1e9),
            received_steady_ns=round((128391.498658892 + end) * 1e9),
        )
    # The shared endpoints override synthetic setup rows and are reused verbatim.
    for laser in (case["old_laser"], case["new_laser"]):
        for name in ("first_time", "last_time"):
            original = SampleTime(**laser[name])
            originals[original.stamp_ns] = original
    for index in range(4, 16):
        end = index / 10
        clock.steady, clock.ros = 128391.498658892 + end, end + 0.02
        points(end)
        message = copy.deepcopy(case["old_laser"])
        message["header"]["stamp"], message["interval_start"] = stamp(end), stamp(end - 0.1)
        node.callbacks["/motion/scan"](
            laser_with_captures(
                message,
                first_time=originals[round((end - 0.1) * 1e9)],
                last_time=originals[round(end * 1e9)],
            ),
            receipt_info(b"laser", originals[round(end * 1e9)]),
        )
    clock.steady, clock.ros = case["old_snapshot"].values()
    bridge._timer.callback()
    assert node.packets[-1]["reference"]["end_ros"] == 1.5
    assert node.packets[-1]["reference"]["average"] is not None
    consumer = make_core(monkeypatch, node, generic_motion_profile())
    capture_inputs(consumer, N(data=json.dumps(node.packets[-1])))
    node.events.clear()
    clock.steady, clock.ros = case["new_received"].values()
    points(1.6)
    first_time = originals[case["new_laser"]["first_time"]["stamp_ns"]]
    last_time = originals[case["new_laser"]["last_time"]["stamp_ns"]]

    def new_laser(value=None):
        return laser_with_captures(
            case["new_laser"] if value is None else value,
            first_time=first_time,
            last_time=last_time,
        )

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

    callback = bridge._callback(completed, "/motion/scan", "/motion/scan")
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
        new_laser=new_laser,
        laser_info=receipt_info(b"laser", last_time),
    )
    consumer.close()


def test_accepted_laser_is_published_after_handler_return_before_next_timer(phase):
    before = len(phase.node.packets)
    phase.callback(
        phase.new_laser(),
        phase.laser_info,
    )
    assert phase.accepted == [True]
    assert phase.node.events[0] == (
        "handler_return",
        phase.case["handler_finished"]["steady"],
        True,
    )
    # Sample actual core inputs at captured tick, without invoking the later producer timer.
    phase.clock.steady, phase.clock.ros = phase.case["tick"].values()
    # Explicit offline raw replay: production has no fused-snapshot subscription.
    capture_inputs(phase.consumer, N(data=json.dumps(phase.node.packets[-1])))
    _, body, _ = read_inputs(phase.consumer, phase.clock.steady)
    assert body is not None, "handler accepted; consumer still has old end_ros=1.5 at ROS1.661"
    assert len(phase.node.packets) == before + 1
    assert phase.node.events[1][0] == "publish"
    assert phase.node.packets[-1]["reference"]["end_ros"] == 1.6
    original = phase.laser_info["sample_time"]
    assert original.received_steady_ns - original.sample_steady_ns == 24_000_000
    assert body.stamp_steady == original.sample_steady_ns / 1e9
    roots = phase.consumer.sensors.latest.vx.roots
    assert body.stamp_steady == max(root.measurement_steady for root in roots)
    assert body.deadline_steady <= min(r.time.sample_steady_ns / 1e9 + 1 for r in roots)
    assert phase.node.publisher_count == 1


def test_duplicate_and_rejected_laser_do_not_recompute_or_publish(phase):
    message = phase.new_laser()
    phase.callback(message, phase.laser_info)
    before = (len(phase.node.packets), phase.bridge.core._sequence)
    phase.callback(message, phase.laser_info)
    assert phase.accepted == [True, False]
    assert (len(phase.node.packets), phase.bridge.core._sequence) == before
    phase.callback(
        message,
        {
            **phase.laser_info,
            "publisher_gid": b"other",
            "native_receipt_authenticated": False,
        },
    )
    assert phase.accepted == [True, False]  # Rejected origin never reaches admission.
    assert (len(phase.node.packets), phase.bridge.core._sequence) == before
    rejected = copy.deepcopy(phase.case["new_laser"])
    rejected["header"]["stamp"] = stamp(1.7)
    rejected["valid"] = False
    phase.clock.steady += 0.1
    phase.clock.ros = 1.72
    phase.case["handler_finished"] = dict(steady=phase.clock.steady, ros=phase.clock.ros)
    phase.callback(phase.new_laser(rejected), phase.laser_info)
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
    phase.callback(phase.new_laser(message), phase.laser_info)
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
        publish_inputs(phase.consumer)
        earlier_body.append(read_inputs(phase.consumer, phase.clock.steady)[1])

    phase.before_return.append(independent_consumer_tick)
    phase.callback(
        phase.new_laser(),
        phase.laser_info,
    )
    assert phase.accepted == [True] and earlier_body[0] is not None
    old_sequence = (
        phase.consumer.node._reception_test.sensors.estimator._frontiers["laser"].roots[0].sequence
    )
    assert old_sequence < phase.node.packets[-1]["sources"][0]["sample"]["sequence"]
    assert phase.node.events[-1] == ("publish", phase.case["handler_finished"]["steady"])
    capture_inputs(phase.consumer, N(data=json.dumps(phase.node.packets[-1])))
    assert read_inputs(phase.consumer, phase.clock.steady)[1] is not None
    assert (
        phase.consumer.node._reception_test.sensors.estimator._frontiers["laser"].roots[0].sequence
        > old_sequence
    )


def test_periodic_and_immediate_publications_share_sequence_and_original_deadline(phase):
    phase.callback(
        phase.new_laser(),
        phase.laser_info,
    )
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
