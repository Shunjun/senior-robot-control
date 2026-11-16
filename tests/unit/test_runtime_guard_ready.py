"""Actual CommandNode/Receiver and captured R5 states; ROS I/O and schedule only controlled."""

import copy
import json
from pathlib import Path
from types import SimpleNamespace as N

import pytest
from motion_reference_test_support import ReadySubscription
from test_navigation_command_clock import navigation  # noqa: F401

from senior_control.agent.manual_control import ManualIngress, MotionRuntimeReceiver
from senior_control.motion.manual_authority import ManualAuthorityState
from senior_control.safety.arbiter import Arbiter

CASE = json.loads(Path("tests/fixtures/runtime_guard_ready.json").read_text())


@pytest.fixture
def ready(request, monkeypatch):
    rig = request.getfixturevalue("navigation")
    node = rig.node
    import sys

    module = sys.modules[type(node).__module__]
    monkeypatch.setattr(module, "qos_profile_sensor_data", N(depth=5))
    monkeypatch.setattr(module, "FinalCommand", lambda: N())
    del node._publish_barrier  # Restore the real method hidden by the navigation I/O fixture.
    node._runtime_subscription = ReadySubscription(object, node._on_motion_runtime, depth=5)
    node._consuming_runtime = False
    node._motion_runtime = MotionRuntimeReceiver()
    node._arbiter = Arbiter("route-sim", 300)
    node._manual_ingress = ManualIngress("route-sim", "command-boot", ttl=0.3)
    node._manual_target = None
    node._navigation_source = None
    node._drive_sample = None
    node._owner_boot = "command-boot"
    node._last_close_source = ("idle", "", 0, 0.0, "", 0)
    node._close_reason = "prepared"
    node.get_parameter = lambda _: N(value="route-sim")
    node.get_publishers_info_by_topic = lambda _: [
        N(node_name="motion_reporter", node_namespace="/", endpoint_gid=b"gate")
    ]
    barriers = []
    node._final_publisher = N(publish=barriers.append)
    old = CASE["packets"][0]
    rig.clock.mono = CASE["receptions"][0]["admission_now"]
    node._on_motion_runtime(N(data=json.dumps(old)), {})
    assert node._motion_runtime.value.sequence == 198
    rig.clock.mono = CASE["guard"]["predicate_now"]
    yield N(
        node=node,
        clock=rig.clock,
        subscription=node._runtime_subscription,
        barriers=barriers,
        module=module,
    )


def enqueue(rig, packet, info=None):
    rig.subscription.ready.append((N(data=json.dumps(packet)), {} if info is None else info))


def test_report_is_historical_without_a_150ms_motion_gate(ready):
    cached = ready.node._motion_runtime.value
    assert ready.node._motion_runtime.current(ready.clock.mono) is cached
    assert not hasattr(cached, "valid_until")
    enqueue(ready, CASE["packets"][1])
    assert ready.node.task_motion_allowed(continuing=True)
    assert ready.subscription.takes == 0
    ready.node._consume_ready_runtime()
    assert ready.node._motion_runtime.value.sequence == 199


def test_missing_report_does_not_close_command_entry(ready):
    ready.node._motion_runtime.value = None
    assert ready.node.task_motion_allowed(continuing=True)


def test_duplicate_wrong_source_and_wrong_robot_cannot_replace_receiver(ready):
    packet = CASE["packets"][1]
    ready.node._on_motion_runtime(N(data=json.dumps(packet)), {})
    expected = ready.node._motion_runtime.value
    enqueue(ready, packet)
    enqueue(ready, packet, {"publisher_gid": b"other"})
    other = copy.deepcopy(packet)
    other.update(sequence=200, robot_id="other-robot")
    enqueue(ready, other)
    ready.node._consume_ready_runtime()
    assert ready.node._motion_runtime.value is expected


def manual_target(ready):
    from senior_control.motion.contracts import ManualIntent
    from senior_control.settings.models import simulation_settings_profile

    ingress = ready.node._manual_ingress
    now = ready.clock.mono
    ingress.bind(1, 1, "session", now)
    receipt = ingress.acquire("manual-test", now)
    target = ingress.accept(
        ManualIntent(receipt.authority_id, 1, 1, 0.0, 0.0, "low"),
        simulation_settings_profile().driving_defaults,
        now,
        now + 0.3,
        now,
    )
    assert target is not None
    ready.node._manual_target = target
    return target


def negative(mode):
    packet = copy.deepcopy(CASE["packets"][1])
    packet["mode"] = mode
    if mode == "execution_fault":
        packet["fault_revision"] = 1
    if mode == "estop":
        packet.update(estop_revision=1, confirmed_estop_status="software_latched")
    return packet


@pytest.mark.parametrize("mode", ["feedback_stop", "execution_fault", "estop"])
def test_negative_report_does_not_revoke_live_source_or_publish_barrier(ready, mode):
    target = manual_target(ready)
    enqueue(ready, negative(mode))
    ready.node._consume_ready_runtime()
    assert ready.node._manual_target is target
    assert ready.node._manual_ingress.view(ready.clock.mono).state is ManualAuthorityState.ACTIVE
    assert not ready.barriers
    assert ready.node._motion_runtime.value.mode == mode
    assert not ready.node.task_motion_allowed(continuing=True)  # Manual ownership still wins.


def test_reentrant_report_consumer_does_not_reenter(ready):
    original = ready.node._observe_retirement_core
    calls = []

    def observe(*args):
        calls.append(ready.subscription.takes)
        ready.node._consume_ready_runtime()
        original(*args)

    ready.node._observe_retirement_core = observe
    enqueue(ready, CASE["packets"][1])
    ready.node._consume_ready_runtime()
    assert calls == [1]
    assert not ready.node._consuming_runtime


def test_continuous_arrival_is_bounded_by_original_qos_depth(ready):
    calls = []

    def take(_type, _raw):
        packet = copy.deepcopy(CASE["packets"][1])
        packet["sequence"] += len(calls)
        calls.append(packet["sequence"])
        return N(data=json.dumps(packet)), {}

    ready.subscription.take_message = take
    ready.node._consume_ready_runtime()
    assert len(calls) == ready.module.qos_profile_sensor_data.depth == 5
    assert ready.node._motion_runtime.value.sequence == 203


def test_old_report_deadline_does_not_reappear_after_callback_work(ready):
    original = ready.node._observe_retirement_core

    def settle(*args):
        original(*args)
        ready.clock.mono += 60

    ready.node._observe_retirement_core = settle
    enqueue(ready, CASE["packets"][1])
    ready.node._consume_ready_runtime()
    assert ready.node.task_motion_allowed(continuing=True)
    assert ready.node._motion_runtime.current(ready.clock.mono).sequence == 199
