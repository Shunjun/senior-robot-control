"""Route qualification follows current source authority and preserves cancellation.

Retired SimulationReference diagnostic-reason permissions had no production
caller: AgentAdapter already qualifies with CommandNode.task_motion_allowed.
"""

import asyncio
import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from senior_control.models import OperationResult, OperationStatus
from senior_control.navigation import RoutePlan


@pytest.fixture
def navigation_evidence(monkeypatch):
    monkeypatch.syspath_prepend("ros2/src/senior_robot")
    lifecycle = ModuleType("senior_robot.motion_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)
    path = Path("ros2/src/senior_robot/senior_robot/agent_adapter.py")
    spec = importlib.util.spec_from_file_location("collision_adapter_under_test", path)
    adapter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(adapter)
    authority = SimpleNamespace(allowed=True)
    ref = SimpleNamespace(
        safety_reason="local authority unavailable",
        safety_ages="",
        pose=lambda: None,
        pose_reply=lambda: {"ok": False},
        state=lambda *_args, **kwargs: {
            "safety_clear": authority.allowed,
            "navigation_ready": kwargs.get("navigation_ready", False),
            "map_generation": "gen",
        },
    )
    return SimpleNamespace(ref=ref, adapter=adapter, authority=authority)


async def test_running_route_stops_when_current_source_authority_is_revoked(navigation_evidence):
    from senior_robot.navigation_adapter import NavigationAdapter, RouteService

    runtime = navigation_evidence
    canceled = []
    result = asyncio.get_running_loop().create_future()

    class Executor:
        async def start(self, goal):
            return "goal-1"

        async def result(self, execution):
            return await result

        async def cancel(self, execution):
            canceled.append(execution)
            return OperationResult(execution, OperationStatus.SUCCEEDED)

    service = RouteService(
        NavigationAdapter(Executor()), qualified=lambda *_: runtime.authority.allowed
    )
    plan = RoutePlan(
        execution_id="route-1",
        map_id="map",
        reference_revision=1,
        map_generation="gen",
        poses=[[1.0, 0.0, 0.0]],
        task_epoch=1,
        effect_id="plan",
        action_ids=["point-1"],
    )
    try:
        assert await service.start(plan)
        assert not await service.start(plan.model_copy(update={"execution_id": "replacement"}))
        runtime.authority.allowed = False
        for _ in range(20):
            if service.current is None:
                break
            await asyncio.sleep(0.01)
        assert canceled == ["goal-1"]
        assert service.status(plan.execution_id).status == "failed"
        assert not await service.start(plan.model_copy(update={"execution_id": "new-route"}))
    finally:
        if service.current is not None:
            await service.cancel(plan.execution_id)


def test_state_polling_does_not_log_running_route_qualification_loss(
    navigation_evidence,
    tmp_path,
    monkeypatch,
):
    from senior_control.models import MapRef
    from senior_control.runtime import ipc

    runtime = navigation_evidence
    logs = []
    captured = SimpleNamespace()
    params_file = tmp_path / "controller.yaml"
    params_file.touch()
    values = {
        "robot_id": "robot",
        "hardware_navigation_reference": "/unused/navigation-reference.json",
        "navigation_controller_params": str(params_file),
        "ipc_replace_existing_socket": False,
    }
    node = SimpleNamespace(
        declare_parameter=lambda name, default: values.setdefault(name, default),
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        get_logger=lambda: SimpleNamespace(warning=logs.append),
        context=SimpleNamespace(get_domain_id=lambda: 73),
        navigation_qualified=lambda: True,
        task_motion_allowed=lambda *, continuing: runtime.authority.allowed,
        motion_runtime=lambda: None,
        runtime_observation=lambda: {"status": "missing", "last_sampled_steady": None},
        manual_view=lambda: None,
        manual_request=lambda message: {"ok": False, "error": "unavailable"},
        motion_mode=lambda: "autonomous",
        submit=lambda *a, **k: None,
        disconnect=lambda *a: None,
        bind_navigation_source=lambda *a, **k: True,
        clear_navigation_source=lambda *a: None,
        complete_navigation_source=lambda *a: None,
        retire_navigation_source=lambda *a, **k: None,
        collect_terminal=lambda *a, **k: None,
        query_terminal_collection=lambda *a, **k: None,
        request_terminal_release=lambda *a, **k: None,
        terminal_release_status=lambda *a: None,
        collect_terminal_releases=lambda *a: None,
        navigation_source_matches=lambda *a: True,
        navigation_source_retired=lambda *a: False,
        shutdown_motion=lambda: None,
    )
    runtime.ref.grid = lambda: SimpleNamespace(map_ref=MapRef("map", 1), grid_id="gen")
    runtime.ref.qualified = lambda *args: True

    class Routes:
        def __init__(self, *args, **kwargs):
            captured.qualified = kwargs["qualified"]
            self.current = SimpleNamespace(
                status="running",
                execution_id="route-1",
                map_generation="gen",
                model_dump=lambda: {},
            )

        async def cancel(self, execution_id):
            self.current = None
            return True

    class Controller:
        def __init__(self, *args, **kwargs):
            pass

        def available(self):
            return True

        def open(self, *args):
            raise AssertionError("no controller starts in this test")

        async def close(self):
            pass

    class Core:
        def __init__(self, **kwargs):
            captured.state = kwargs["state_provider"]

        def release(self):
            pass

        async def handle(self, message):
            raise AssertionError("no IPC requests in this test")

    class Server:
        def __init__(self, *args, **kwargs):
            pass

        async def start(self):
            runtime.authority.allowed = True
            assert captured.qualified(MapRef("map", 1), "gen") is True
            state = captured.state()
            assert state["navigation_ready"] is True and state["safety_clear"] is True
            assert logs == []
            assert captured.qualified(MapRef("map", 1), "gen") is True
            assert logs == []
            runtime.authority.allowed = False
            assert captured.state()["navigation_ready"] is False and logs == []
            assert captured.qualified(MapRef("map", 1), "gen") is False
            assert len(logs) == 1 and "Route qualification lost" in logs[0]
            assert captured.state()["navigation_ready"] is False
            assert captured.qualified(MapRef("map", 1), "gen") is False
            assert len(logs) == 1

        async def close(self):
            pass

    def fake_module(name, **members):
        module = ModuleType(name)
        module.__dict__.update(members)
        monkeypatch.setitem(sys.modules, name, module)

    fake_module("rclpy", ok=lambda: False)
    fake_module("senior_robot.command_node", CommandNode=lambda **_: node)
    fake_module("senior_localization.hardware_reference", HardwareReference=lambda *a: runtime.ref)
    fake_module("senior_robot.dedicated_controller", DedicatedController=Controller)
    fake_module(
        "senior_robot.navigation_adapter",
        RouteService=Routes,
        create_nav2_adapter=lambda _: None,
        create_planner_readiness=lambda _: lambda: True,
    )
    lifecycle = sys.modules["senior_robot.motion_lifecycle"]
    lifecycle.MotionExecutor = object
    lifecycle.run = lambda factory, **kwargs: kwargs["spin"](
        factory(),
        SimpleNamespace(add_node=lambda _: None),
        lambda: False,
    )
    monkeypatch.setattr(runtime.adapter, "AdapterCore", Core)
    monkeypatch.setattr(ipc, "IpcServer", Server)
    monkeypatch.setenv("SENIOR_AGENT_SOCKET", str(tmp_path / "unused.sock"))
    runtime.adapter.main()
