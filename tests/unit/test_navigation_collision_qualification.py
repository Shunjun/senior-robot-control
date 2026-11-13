"""A blocked velocity does not invalidate a running route's independently fresh evidence."""

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
    root = Path("ros2/src/senior_robot")
    monkeypatch.syspath_prepend(str(root))
    lifecycle = ModuleType("senior_robot.motion_lifecycle")
    lifecycle.with_dds_info = lambda callback: callback
    monkeypatch.setitem(sys.modules, lifecycle.__name__, lifecycle)

    def load(name, filename):
        spec = importlib.util.spec_from_file_location(name, root / "senior_robot" / filename)
        module = importlib.util.module_from_spec(spec)
        monkeypatch.setitem(sys.modules, name, module)
        spec.loader.exec_module(module)
        return module

    reference = load("collision_reference_under_test", "simulation_reference.py")
    adapter = load("collision_adapter_under_test", "agent_adapter.py")
    clock = SimpleNamespace(now=100.0)
    source = SimpleNamespace(gid=b"gate")
    monkeypatch.setattr(reference, "time", SimpleNamespace(monotonic=lambda: clock.now))
    ref = reference.SimulationReference.__new__(reference.SimulationReference)
    ref._safety_at = ref._navigation_evidence_at = float("-inf")
    ref._safety_gid = None
    ref._source = lambda *_args, **_kwargs: source.gid
    ref.grid = lambda: None
    ref.pose = lambda: None

    def status(reason, *, certified=True):
        ref._on_safety(
            SimpleNamespace(
                status=[
                    SimpleNamespace(
                        name="safety_gate",
                        message=reason,
                        values=[
                            SimpleNamespace(
                                key="evidence_ready", value="true" if certified else "false"
                            )
                        ],
                    )
                ]
            ),
            {},
        )

    return SimpleNamespace(ref=ref, adapter=adapter, status=status, clock=clock, source=source)


def test_sweep_collision_keeps_evidence_but_not_ui_or_new_navigation_clear(navigation_evidence):
    runtime = navigation_evidence
    runtime.status("hard_sweep_collision")
    assert runtime.ref.navigation_evidence_ready() is True
    assert runtime.ref.safety_ready() is False
    assert runtime.ref.state("autonomous")["safety_clear"] is False
    routes = SimpleNamespace(current=SimpleNamespace(status="running"))
    assert runtime.adapter._route_safety_ready(runtime.ref, routes) is True
    assert runtime.adapter._route_safety_ready(runtime.ref, routes, continuing=False) is False
    for status in ("starting", "cancel_pending", "failed", "succeeded", "canceled"):
        routes.current.status = status
        assert runtime.adapter._route_safety_ready(runtime.ref, routes) is False


@pytest.mark.parametrize(
    "reason",
    [
        "hard_contact",
        "coverage_insufficient",
        "scan_stale",
        "progress_history_unqualified",
        "evidence_topology_unqualified",
        "unrecognized_reason",
    ],
)
def test_non_ready_reasons_never_qualify_continuation(navigation_evidence, reason):
    runtime = navigation_evidence
    runtime.status("hard_sweep_collision")
    runtime.status(reason)
    assert runtime.ref.navigation_evidence_ready() is False
    assert runtime.ref.safety_ready() is False


@pytest.mark.parametrize("failure", ["stale", "publisher_changed", "uncertified"])
def test_collision_evidence_requires_fresh_certified_same_source(navigation_evidence, failure):
    runtime = navigation_evidence
    runtime.status("hard_sweep_collision", certified=failure != "uncertified")
    if failure == "stale":
        runtime.clock.now += 1.251
    elif failure == "publisher_changed":
        runtime.source.gid = b"replacement"
    assert runtime.ref.navigation_evidence_ready() is False
    assert runtime.ref.safety_ready() is False


@pytest.mark.parametrize("failure", ["hard_contact", "stale", "publisher_changed", "uncertified"])
async def test_running_route_can_replan_collision_but_stops_on_lost_evidence(
    navigation_evidence,
    failure,
):
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
        NavigationAdapter(Executor()),
        qualified=lambda *_: runtime.adapter._route_safety_ready(runtime.ref, service),
    )
    plan = RoutePlan(
        execution_id="route-1",
        map_id="map",
        reference_revision=1,
        map_generation="gen",
        poses=[[1.0, 0.0, 0.0]],
    )
    runtime.status("hard_sweep_clear")
    try:
        assert await service.start(plan)
        runtime.status("hard_sweep_collision")
        await asyncio.sleep(0.07)
        assert service.current.status == "running" and canceled == []
        assert runtime.ref.state("autonomous")["safety_clear"] is False
        assert not await service.start(plan.model_copy(update={"execution_id": "replacement"}))
        assert not runtime.adapter._route_safety_ready(runtime.ref, service, continuing=False)
        runtime.status("hard_sweep_clear")
        assert runtime.ref.safety_ready() is True
        if failure == "hard_contact":
            runtime.status("hard_contact")
        elif failure == "stale":
            runtime.clock.now += 1.251
        elif failure == "publisher_changed":
            runtime.source.gid = b"replacement"
        else:
            runtime.status("hard_sweep_collision", certified=False)
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
        "simulation_qualified_reference": True,
        "simulation_baseline_world": "unused",
        "navigation_controller_params": str(params_file),
        "ipc_replace_existing_socket": False,
    }
    node = SimpleNamespace(
        declare_parameter=lambda name, default: values.setdefault(name, default),
        get_parameter=lambda name: SimpleNamespace(value=values[name]),
        get_logger=lambda: SimpleNamespace(warning=logs.append),
        context=SimpleNamespace(get_domain_id=lambda: 73),
        navigation_qualified=lambda: True,
        task_motion_allowed=lambda *, continuing: (
            runtime.ref.navigation_evidence_ready() if continuing else runtime.ref.safety_ready()
        ),
        motion_runtime=lambda: None,
        manual_view=lambda: None,
        manual_request=lambda message: {"ok": False, "error": "unavailable"},
        motion_mode=lambda: "autonomous",
        submit=lambda *a, **k: None,
        disconnect=lambda *a: None,
        server_heartbeat=lambda *a: True,
        take_drive_feedback=lambda: None,
        bind_navigation_source=lambda *a, **k: True,
        clear_navigation_source=lambda *a: None,
        navigation_source_matches=lambda *a: True,
        current_navigation_epoch=lambda: 1,
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
        def __init__(self, *args):
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
            runtime.status("hard_sweep_collision")
            assert captured.qualified(MapRef("map", 1), "gen") is True
            state = captured.state()
            assert state["navigation_ready"] is False and state["safety_clear"] is False
            assert logs == []
            assert captured.qualified(MapRef("map", 1), "gen") is True
            assert logs == []
            runtime.status("hard_contact")
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
    fake_module("senior_robot.simulation_reference", SimulationReference=lambda *a: runtime.ref)
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
    )
    monkeypatch.setattr(runtime.adapter, "AdapterCore", Core)
    monkeypatch.setattr(ipc, "IpcServer", Server)
    monkeypatch.setenv("SENIOR_AGENT_SOCKET", str(tmp_path / "unused.sock"))
    runtime.adapter.main()


@pytest.mark.parametrize(
    "reason",
    [
        "command_expired",
        "zero_requested",
        "evaluation_pending",
        "hard_sweep_clear",
        "warning_overlap",
    ],
)
def test_existing_ready_reasons_keep_both_qualifications(navigation_evidence, reason):
    runtime = navigation_evidence
    runtime.status(reason)
    assert runtime.ref.safety_ready() is True
    assert runtime.ref.navigation_evidence_ready() is True
