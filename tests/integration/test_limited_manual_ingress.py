"""Real T2/CommandNode with ROS I/O double; no independent body is invented."""

import pytest
import test_manual_wire as wire
from test_manual_wire import call, envelope, runtime


@pytest.fixture
def node(monkeypatch):
    return wire.node.__wrapped__(monkeypatch)


def test_explicit_limited_acquire_allows_new_neutral_without_stationary(node):
    frame = runtime(
        10.0, 2, mode="braking", stationary=False, body_reference=None, allowed_actions=["acquire"]
    )
    node.value._motion_runtime.accept(frame, b"t5", 10.0)
    prepared = call(node, envelope("control.acquire", {}))
    assert prepared["ok"]
    neutral = dict(
        authority_id=prepared["authority_id"],
        gesture=2,
        sequence=1,
        linear=0.0,
        turn=0.0,
        gear="low",
    )
    assert call(node, envelope("drive.intent", neutral, seq=2), 10.0)["accepted"]
    neutral["sequence"] = 2
    neutral["linear"] = 1.0
    assert call(node, envelope("drive.intent", neutral, seq=3), 10.0)["accepted"]
    assert call(
        node, envelope("control.release", dict(authority_id=prepared["authority_id"]), seq=4)
    )["accepted"]
    node.clock.now = 10.01
    node.value._motion_runtime.accept(
        runtime(
            10.01,
            3,
            mode="braking",
            stationary=False,
            body_reference=None,
            allowed_actions=["acquire"],
        ),
        b"t5",
        10.01,
    )
    old_intent = dict(neutral, sequence=999, gesture=9, linear=1.0)
    assert not call(node, envelope("drive.intent", old_intent, seq=5), 10.01)["accepted"]
    second = call(node, envelope("control.acquire", {}, seq=5, request="new-acquire"))
    assert second["authority_id"] != prepared["authority_id"]
    neutral.update(authority_id=second["authority_id"], gesture=3, sequence=1, linear=0.0)
    assert call(node, envelope("drive.intent", neutral, seq=6), 10.01)["accepted"]
    assert call(
        node, envelope("control.release", dict(authority_id=second["authority_id"]), seq=7)
    )["accepted"]
    assert node.value.task_motion_allowed(continuing=False)
    # RuntimeCheck for settings stays stationary=False; no handoff changes it.
    assert not node.value._motion_runtime.current(node.clock.now).stationary


def test_old_allowed_actions_does_not_gate_prepare(node):
    node.value._motion_runtime.accept(
        runtime(10.0, 2, mode="braking", stationary=False, allowed_actions=[]), b"t5", 10.0
    )
    assert call(node, envelope("control.acquire", {}))["ok"]


async def test_limited_action_does_not_open_settings_or_navigation(tmp_path, monkeypatch):
    from types import SimpleNamespace as N

    from vehicle_manager.agent.local_state import RuntimeState
    from vehicle_manager.agent.production_bindings import SettingsRuntime
    from vehicle_manager.settings.models import SettingsRequest, simulation_settings_profile
    from vehicle_manager.settings.service import RuntimeGate, SettingsService
    from vehicle_manager.settings.store import SettingsStore
    from vehicle_manager.storage.bindings import Database

    monkeypatch.setattr("vehicle_manager.agent.local_adapters.time.monotonic", lambda: 10.0)
    frame = runtime(
        10.0, mode="braking", stationary=False, body_reference=None, allowed_actions=["acquire"]
    )
    adapter = SettingsRuntime(None, None)
    adapter.live = N(motion_runtime=frame, manual_authority=None, manual_driving=False)
    adapter.executing = False
    async with Database(tmp_path / "limited-settings.db") as db:
        profile = simulation_settings_profile()
        service = SettingsService(SettingsStore(db), profile, RuntimeGate(adapter.check))
        await service.prepare()
        reply = await service.apply(
            SettingsRequest(
                request_id="limited",
                expected_version=0,
                group="driving",
                values=profile.driving_defaults.model_dump(),
            )
        )
        assert reply.reason == "stop_required"
    state = RuntimeState(
        localized=True,
        safety_clear=True,
        manual_driving=False,
        map_ref=None,
        execution_status="idle",
        navigation_ready=True,
        task_evidence_ready=True,
        motion_runtime=frame,
        motion_runtime_version=1,
    )
    assert state.for_tasks(True).admission_ready


def test_learning_admission_keeps_active_manual_target_and_requires_current_prepared_authority(
    node,
):
    prepared = call(node, envelope("control.acquire", {}))
    payload = {"expected_active_revision": 0, "expected_authority_id": prepared["authority_id"]}
    admitted = call(node, envelope("learning.start", payload, request="calibration"), 10.0)
    assert admitted["ok"] and admitted["can_start"]
    assert call(
        node,
        envelope(
            "drive.intent",
            dict(
                authority_id=prepared["authority_id"],
                gesture=1,
                sequence=1,
                linear=1.0,
                turn=0.0,
                gear="low",
            ),
            seq=2,
        ),
        10.0,
    )["accepted"]
    target = node.value._manual_target
    rejected = call(
        node, envelope("learning.start", payload, request="late-calibration", seq=3), 10.0
    )
    assert rejected["ok"] and not rejected["can_start"]
    assert node.value._manual_target is target
