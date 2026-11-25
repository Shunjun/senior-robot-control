import importlib.util
from pathlib import Path

import pytest

from vehicle_core.values import Pose2D

_SPEC = importlib.util.spec_from_file_location(
    "world_reference", Path("ros2/src/senior_sim/scripts/world_reference.py")
)
_WORLD = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_WORLD)
baseline_grid, map_to_odom = _WORLD.baseline_grid, _WORLD.map_to_odom


def test_baseline_grid_matches_the_pinned_world(tmp_path: Path) -> None:
    world = Path("ros2/src/senior_sim/worlds/baseline.sdf")
    grid = baseline_grid(world, "boot")
    assert grid.map_ref.map_id.startswith("sim-baseline-")
    assert grid.cells[80 * 160 + 120] == 100  # World x=2, y=0, inside the wall.
    assert grid.cells[80 * 160 + 80] == 0
    assert grid.waypoint_writes
    from senior_localization.navigation_reference import load_navigation_reference

    reference = _WORLD.generate_reference(world, tmp_path / "assets")
    generated = load_navigation_reference(reference)
    assert bytes(generated.grid().cells) == grid.cells
    assert generated.pose_topic == "/amcl_pose"
    assert generated.pose_publisher == "amcl"
    pgm = (reference.parent / "map.pgm").read_bytes()
    assert pgm.startswith(b"P5\n160 160\n255\n")
    assert len(pgm.split(b"\n", 3)[-1]) == grid.width * grid.height
    modified = tmp_path / "baseline.sdf"
    modified.write_bytes(world.read_bytes().replace(b"2 0 0.75", b"3 0 0.75"))
    with pytest.raises(ValueError, match="baseline"):
        baseline_grid(modified, "boot")


def test_map_to_odom_applies_the_same_reference_transform() -> None:
    transform = map_to_odom(Pose2D(5, 3, 1.5707963267948966), Pose2D(2, 1, 0))
    assert transform.x == pytest.approx(6)
    assert transform.y == pytest.approx(1)
    assert transform.yaw == pytest.approx(1.5707963267948966)


def test_route_world_raster_includes_cross_walls_and_preserves_start_corridor() -> None:
    world = Path("ros2/src/senior_sim/worlds/motion_qualification.sdf")
    grid = baseline_grid(world, "boot")

    def cell(x: float, y: float) -> int:
        column = int((x - grid.origin.x) / grid.resolution)
        row = int((y - grid.origin.y) / grid.resolution)
        return grid.cells[row * grid.width + column]

    assert cell(0, 3) == 100
    assert cell(-3.5, 0) == 100
    assert cell(2, 1.2) == 100
    assert all(cell(x, 0) == 0 for x in (0, 0.35, 0.7, 1.05))


def test_scan_projection_accounts_for_robot_and_sensor_mount_and_limits_payload():
    import math

    project_scan = _WORLD.project_scan

    points = project_scan(
        Pose2D(1, 2, math.pi / 2),
        Pose2D(0.2, 0, 0),
        [1.0, float("inf"), float("nan"), 20.0],
        0,
        math.pi / 2,
        0.08,
        20,
    )
    assert points == [{"x": 1.0, "y": 3.2}]
    many = project_scan(Pose2D(0, 0, 0), Pose2D(0, 0, 0), [1.0] * 720, 0, 0.01, 0.08, 20)
    assert len(many) <= 64


async def _startup_owner():
    """Exercise the real launch owner's handshake with only launch events substituted."""
    import ast
    import asyncio
    import json
    import os
    import secrets
    import shutil
    import signal
    import tempfile
    from types import SimpleNamespace as NS

    from senior_navigation.parameters import prepare_navigation_parameters

    path = Path("ros2/src/senior_sim/launch/simulation_motion.launch.py")
    definition = next(
        n
        for n in ast.parse(path.read_text()).body
        if isinstance(n, ast.ClassDef) and n.name == "_NavigationStartup"
    )

    class Event:
        def __init__(self, **kwargs):
            self.__dict__.update(kwargs)

    namespace = dict(
        asyncio=asyncio,
        json=json,
        os=os,
        secrets=secrets,
        shutil=shutil,
        signal=signal,
        tempfile=tempfile,
        Path=Path,
        prepare_navigation_parameters=prepare_navigation_parameters,
        resource_handlers=lambda *_: [],
        RegisterEventHandler=lambda x: x,
        OnProcessStart=Event,
        OnProcessExit=Event,
        OpaqueFunction=Event,
        SignalProcess=Event,
        matches_action=lambda action: action,
    )
    exec(compile(ast.Module(body=[definition], type_ignores=[]), str(path), "exec"), namespace)
    futures, signals = [], []
    context = NS(
        asyncio_loop=asyncio.get_running_loop(),
        add_completion_future=futures.append,
        emit_event_sync=signals.append,
    )
    owner = namespace["_NavigationStartup"](
        context, Path("ros2/src/senior_navigation/config/nav2.yaml")
    )
    a, b, live = object(), object(), object()
    owner._planner = lambda: live
    first, second = owner._bind(a), owner._bind(b)
    first[0].on_start(NS(), context)
    second[0].on_start(NS(), context)
    owner.listen(context)
    await asyncio.gather(*futures)
    return NS(
        owner=owner,
        context=context,
        futures=futures,
        signals=signals,
        first=first,
        second=second,
        live=live,
    )


async def test_simulation_handoff_waits_for_both_owned_groups(tmp_path):
    import asyncio
    from types import SimpleNamespace as NS

    from senior_navigation.parameters import request_live_handoff

    h = await _startup_owner()
    owner, context, futures, signals = h.owner, h.context, h.futures, h.signals
    first, second, live = h.first, h.second, h.live
    a, b = first[-1], second[-1]
    contract = dict(owner.contract)
    task = asyncio.create_task(request_live_handoff(contract))
    try:
        for _ in range(100):
            if len(signals) == 2:
                break
            await asyncio.sleep(0.001)
        assert {s.process_matcher for s in signals} == {a, b}
        assert not task.done() and owner.contract["mode"] == "static"
        assert first[1].on_exit(NS(returncode=0), context) == []
        assert not task.done() and owner.contract["mode"] == "static"
        replacement = second[1].on_exit(NS(returncode=0), context)
        assert replacement[-1].function(context) == [live]
        assert owner.contract["mode"] == "live"
        replacement[0].on_start(NS(), context)
        await task
        assert contract["mode"] == "live"
        assert contract["global_instance_permit"] == owner.binding["global_instance"]
        replacement[1].on_exit(NS(returncode=0), context)
        with pytest.raises(RuntimeError, match="transaction unavailable"):
            await request_live_handoff(contract)
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        owner.groups.clear()
        owner.cleanup(context)
        await asyncio.gather(*futures)


async def test_external_hardware_static_owner_without_handle_cannot_be_taken_over():
    from senior_navigation.parameters import request_live_handoff

    with pytest.raises(RuntimeError, match="external static localization owner"):
        await request_live_handoff(dict(mode="static", run_nonce="run"))


@pytest.mark.parametrize("failure", ["timeout", "disconnect", "cancel"])
async def test_failed_unix_handoff_cannot_spawn_on_late_group_exits(failure):
    import asyncio
    import json
    from types import SimpleNamespace as NS

    h = await _startup_owner()
    owner = h.owner
    original = owner._request
    requests = []

    async def capture(reader, writer):
        requests.append(asyncio.current_task())
        await original(reader, writer)

    # Rebind before connecting; retain the real Unix server/client and production waits.
    owner.server.close()
    await owner.server.wait_closed()
    owner.server = await asyncio.start_unix_server(capture, path=owner.endpoint, limit=8192)
    reader, writer = await asyncio.open_unix_connection(owner.endpoint)
    writer.write(
        (
            json.dumps(dict(command="prepare_live", run_nonce=owner.binding["run_nonce"])) + "\n"
        ).encode()
    )
    await writer.drain()
    try:
        for _ in range(100):
            if len(h.signals) == 2:
                break
            await asyncio.sleep(0.001)
        assert len(h.signals) == 2
        if failure == "timeout":
            response = json.loads(await reader.readline())  # Real production 22 seconds.
            assert response["ok"] is False
        elif failure == "disconnect":
            writer.close()
            await writer.wait_closed()
        else:
            requests[0].cancel()
        if failure != "timeout":
            for _ in range(100):
                if requests[0].done():
                    break
                await asyncio.sleep(0.001)
            assert requests[0].done()
        assert h.first[1].on_exit(NS(returncode=0), h.context) == []
        assert h.second[1].on_exit(NS(returncode=0), h.context) == []
        assert owner.contract["mode"] == "static"
        # This failed transaction is terminal: a socket request cannot reopen it.
        retry_reader, retry_writer = await asyncio.open_unix_connection(owner.endpoint)
        retry_writer.write(
            (
                json.dumps(dict(command="prepare_live", run_nonce=owner.binding["run_nonce"]))
                + "\n"
            ).encode()
        )
        await retry_writer.drain()
        assert json.loads(await retry_reader.readline())["ok"] is False
        retry_writer.close()
        await retry_writer.wait_closed()
    finally:
        writer.close()
        await writer.wait_closed()
        for task in requests:
            if not task.done():
                task.cancel()
        await asyncio.gather(*requests, return_exceptions=True)
        owner.groups.clear()
        owner.cleanup(h.context)
        await asyncio.gather(*h.futures)


@pytest.mark.parametrize("returncode", [0, -9, None])
async def test_run_directory_waits_for_agent_consumer_settlement(returncode):
    import asyncio
    from types import SimpleNamespace as NS

    h = await _startup_owner()
    owner = h.owner
    agent = object()
    # This callback is bound to the actual agent Node in _simulation_nodes.
    binding = owner.bind_consumer(agent)
    try:
        h.first[1].on_exit(NS(returncode=0), h.context)
        h.second[1].on_exit(NS(returncode=0), h.context)
        owner.cleanup(h.context)
        await asyncio.sleep(0.06)
        assert owner.directory.exists()  # adapter/native/local controller still pending
        binding[0].on_exit(NS(returncode=returncode), h.context)
        if returncode == 0:
            await asyncio.gather(*h.futures)
            assert not owner.directory.exists()
        else:
            # An abnormal/unknown root exit cannot prove descendant settlement.
            await asyncio.sleep(0.06)
            assert owner.directory.exists()
    finally:
        for task in h.futures:
            if not task.done():
                task.cancel()
        await asyncio.gather(*h.futures, return_exceptions=True)
        if owner.server is not None:
            owner.server.close()
            await owner.server.wait_closed()
        import shutil

        shutil.rmtree(owner.directory, ignore_errors=True)


async def test_adapter_cleanup_errors_cannot_look_like_normal_consumer_exit():
    """Execute the adapter's actual cleanup block; no ROS/process execution claimed."""
    import ast
    from types import SimpleNamespace as NS

    path = Path("ros2/src/senior_robot/senior_robot/agent_adapter.py")
    block = next(
        node
        for node in ast.walk(ast.parse(path.read_text()))
        if isinstance(node, ast.Try)
        and any(
            isinstance(h.type, ast.Name) and h.type.id == "BaseException" for h in node.handlers
        )
    )
    function = ast.AsyncFunctionDef(
        name="cleanup",
        args=ast.arguments(
            posonlyargs=[],
            args=[ast.arg(arg="mapping_backend"), ast.arg(arg="controller")],
            kwonlyargs=[],
            kw_defaults=[],
            defaults=[],
        ),
        body=[block],
        decorator_list=[],
    )
    namespace = {}
    exec(
        compile(
            ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
            str(path),
            "exec",
        ),
        namespace,
    )
    order = []

    async def native_close():
        order.append("native settled")

    async def controller_close():
        order.append("controller settled")

    await namespace["cleanup"](NS(close=native_close), NS(close=controller_close))
    assert order == ["native settled", "controller settled"]

    class ShutdownError(Exception):
        pass

    async def incomplete():
        raise ShutdownError("ROS shutdown during native cleanup")

    # The real ROS run wrapper catches its ROS shutdown exceptions; our wrapper must
    # convert an incomplete navigation cleanup so that it cannot exit zero that way.
    with pytest.raises(RuntimeError, match="navigation consumer cleanup incomplete"):
        try:
            await namespace["cleanup"](NS(close=incomplete), NS(close=controller_close))
        except ShutdownError:
            pass
    assert order == ["native settled", "controller settled", "controller settled"]


async def test_disconnected_handoff_fences_a_queued_live_launch_action():
    import asyncio
    import json
    from types import SimpleNamespace as NS

    h = await _startup_owner()
    owner = h.owner
    reader, writer = await asyncio.open_unix_connection(owner.endpoint)
    writer.write(
        (
            json.dumps(dict(command="prepare_live", run_nonce=owner.binding["run_nonce"])) + "\n"
        ).encode()
    )
    await writer.drain()
    try:
        for _ in range(100):
            if len(h.signals) == 2:
                break
            await asyncio.sleep(0.001)
        h.first[1].on_exit(NS(returncode=0), h.context)
        replacement = h.second[1].on_exit(NS(returncode=0), h.context)
        writer.close()
        await writer.wait_closed()
        for _ in range(100):
            if owner.pending.done():
                break
            await asyncio.sleep(0.001)
        assert owner.pending.result()["ok"] is False
        assert replacement[-1].function(h.context) == []
        assert not owner.groups
    finally:
        writer.close()
        await writer.wait_closed()
        owner.cleanup(h.context)
        await asyncio.gather(*h.futures)


@pytest.mark.parametrize(
    "shutdown_case", ["slow_sigint", "cancelled_route_wait", "unconfirmed_sigkill"]
)
def test_active_controller_shutdown_settles_before_launch_directory_removal(shutdown_case):
    """Independent R4 probe input: real route, controller and adapter shutdown blocks."""
    import __future__

    import ast
    import asyncio
    import signal
    from contextlib import AsyncExitStack, asynccontextmanager, suppress
    from types import SimpleNamespace as NS

    from senior_navigation.navigation_adapter import RouteService

    controller_path = (
        Path.cwd() / "ros2/src/senior_navigation/senior_navigation/dedicated_controller.py"
    )
    definition = next(
        n for n in ast.parse(controller_path.read_text()).body if isinstance(n, ast.ClassDef)
    )
    processes = {}
    signals = []

    def killpg(pid, number):
        signals.append((pid, number))
        # One ordinary child exits on SIGINT; bt_navigator is still shutting down.
        if pid == 1 or (number == signal.SIGKILL and shutdown_case != "unconfirmed_sigkill"):
            processes[pid].returncode = 0
            processes[pid].finished.set()

    ns = dict(
        asyncio=asyncio,
        AsyncExitStack=AsyncExitStack,
        asynccontextmanager=asynccontextmanager,
        suppress=suppress,
        Path=Path,
        os=NS(killpg=killpg),
        signal=signal,
    )
    exec(
        compile(
            ast.Module(body=[definition], type_ignores=[]),
            str(controller_path),
            "exec",
            flags=__future__.annotations.compiler_flag,
        ),
        ns,
    )
    Controller = ns["DedicatedController"]
    adapter_path = Path.cwd() / "ros2/src/senior_robot/senior_robot/agent_adapter.py"
    tree = ast.parse(adapter_path.read_text())
    serve = next(
        n for n in ast.walk(tree) if isinstance(n, ast.AsyncFunctionDef) and n.name == "serve"
    )
    serve_try = next(n for n in serve.body if isinstance(n, ast.Try))
    cancel_if = next(
        n
        for n in serve_try.body
        if isinstance(n, ast.If) and ast.unparse(n.test).startswith("routes is not None")
    )
    close_try = next(
        n
        for n in serve_try.finalbody
        if isinstance(n, ast.Try)
        and any(isinstance(h.type, ast.Name) and h.type.id == "BaseException" for h in n.handlers)
    )
    function = ast.AsyncFunctionDef(
        name="adapter_shutdown",
        args=ast.arguments(posonlyargs=[], args=[], kwonlyargs=[], kw_defaults=[], defaults=[]),
        body=[cancel_if, close_try],
        decorator_list=[],
    )
    exec(
        compile(
            ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
            str(adapter_path),
            "exec",
        ),
        ns,
    )

    class Process:
        def __init__(self, pid):
            self.pid = pid
            self.returncode = None
            self.finished = asyncio.Event()
            self.waiting = asyncio.Event()
            self.cancelled = False

        async def wait(self):
            self.waiting.set()
            try:
                await self.finished.wait()
            except asyncio.CancelledError:
                self.cancelled = True
                raise
            return self.returncode

    async def shutdown():
        controller = Controller(None, Path("/tmp/unused-review-params"))
        stack = AsyncExitStack()
        for pid in (1, 2):
            processes[pid] = process = Process(pid)
            stack.push_async_callback(controller._stop, process)
        controller._standby = (stack, "/unused-review-topic")
        open_entered, finish_route = asyncio.Event(), asyncio.Event()

        async def run_route(*args, **kwargs):
            open_entered.set()
            await finish_route.wait()
            return False

        routes = RouteService(
            NS(run_route=run_route, active_goal=None),
            qualified=lambda *args: True,
            prepare=controller.open,
        )
        plan = NS(
            execution_id="execution",
            task_epoch=1,
            map_ref=None,
            map_generation="static",
            poses=(),
            action_ids=(),
        )
        route = NS(
            plan=plan,
            cancel_requested=False,
            active_waypoint_index=None,
            started=asyncio.get_running_loop().create_future(),
            completed=[],
            output_exited=False,
            progress=lambda: NS(execution_id="execution"),
        )
        routes._current = route
        route_task = route.task = asyncio.create_task(routes._run(route))
        await open_entered.wait()
        assert controller._in_use and not route_task.done()
        finish_route.set()  # Normal route completion starts the actual AsyncExitStack cleanup.
        await processes[2].waiting.wait()
        assert controller._in_use and processes[2].returncode is None
        print("ACTIVE_ROUTE_WAIT", controller._in_use, route_task.done(), processes[2].returncode)
        if shutdown_case == "cancelled_route_wait":
            route_task.cancel()
            await asyncio.gather(route_task, return_exceptions=True)
            assert processes[2].returncode is None

        async def backend_close():
            pass

        ns.update(
            routes=routes,
            rclpy=NS(ok=lambda: False),
            controller=controller,
            mapping_backend=NS(close=backend_close),
        )
        await ns["adapter_shutdown"]()  # Unmodified adapter 3-second cancellation + close blocks.
        print(
            "ACTUAL_ADAPTER_CLOSE_RETURNED",
            "route_done=",
            route_task.done(),
            "controller_in_use=",
            controller._in_use,
            "navigator_returncode=",
            processes[2].returncode,
        )
        assert processes[2].returncode is not None, (
            "adapter returned with an unsettled active navigator"
        )
        assert not controller._in_use
        await asyncio.gather(route_task, return_exceptions=True)
        # Returning now invokes the same asyncio.run pending-task cancellation as adapter spin.

    failure = None
    try:
        asyncio.run(shutdown())
    except RuntimeError as exc:
        failure = exc
    if shutdown_case == "unconfirmed_sigkill":
        assert failure is not None
        assert "navigation consumer cleanup incomplete" in str(failure)
        assert processes[2].returncode is None
    else:
        assert failure is None
        assert processes[2].returncode is not None
    print(
        "AFTER_RUN",
        "failure=",
        failure,
        "returncode=",
        processes[2].returncode,
        "wait_cancelled=",
        processes[2].cancelled,
        "signals=",
        signals,
    )
    # The initial SIGINT wait times out and is cancelled by wait_for, then the
    # original exact group receives SIGKILL and its exit is actually observed.
    assert processes[2].cancelled
    assert signals == [(2, signal.SIGINT), (2, signal.SIGKILL), (1, signal.SIGINT)]

    async def directory_boundary():
        h = await _startup_owner()
        binding = h.owner.bind_consumer(object())
        h.first[1].on_exit(NS(returncode=0), h.context)
        h.second[1].on_exit(NS(returncode=0), h.context)
        h.owner.cleanup(h.context)
        await asyncio.sleep(0.06)
        assert h.owner.directory.exists()
        # The exit boundary is derived from actual shutdown, never a forced fake code.
        binding[0].on_exit(NS(returncode=0 if failure is None else 1), h.context)
        if failure is None:
            await asyncio.gather(*h.futures)
            assert not h.owner.directory.exists()
        else:
            await asyncio.sleep(0.06)
            assert h.owner.directory.exists()
            import shutil

            shutil.rmtree(h.owner.directory)
        print("DIRECTORY_SETTLEMENT", "failure=", failure, "returncode=", processes[2].returncode)

    asyncio.run(directory_boundary())
