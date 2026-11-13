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
    assert not node.value.task_motion_allowed(continuing=False)
    # RuntimeCheck for settings stays stationary=False; no handoff changes it.
    assert not node.value._motion_runtime.current(node.clock.now).stationary


def test_no_explicit_action_does_not_prepare_an_authority(node):
    node.value._motion_runtime.accept(
        runtime(10.0, 2, mode="braking", stationary=False, allowed_actions=[]), b"t5", 10.0
    )
    assert not call(node, envelope("control.acquire", {}))["ok"]


async def test_limited_action_does_not_open_settings_or_navigation(tmp_path, monkeypatch):
    from types import SimpleNamespace as N

    from senior_control.agent.ports import RuntimeState
    from senior_control.agent.settings_wire import SettingsRuntime
    from senior_control.settings.models import SettingsRequest, simulation_settings_profile
    from senior_control.settings.service import RuntimeGate, SettingsService
    from senior_control.settings.store import SettingsStore
    from senior_control.storage.database import Database

    monkeypatch.setattr("senior_control.agent.settings_wire.time.monotonic", lambda: 10.0)
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
    assert not state.for_tasks(True).admission_ready
