"""Recovery contract exercised with real, non-ROS, non-motion processes."""

import ast
import asyncio
import json
import os
import secrets
import shlex
import shutil
import signal
import subprocess
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace as N

import pytest
from test_docker_start_helper import service_root as endpoint_root  # noqa: F401


@contextmanager
def recovered_process(tmp_path, code):
    from senior_control.runtime.launch_recovery import restart_prefix

    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(Path(__file__).parents[2] / "src")
    environment.pop("SENIOR_LAUNCH_RECOVERED", None)
    environment["SENIOR_STARTUP_LEARNING_CLAIM"] = "unchanged-run"
    with (tmp_path / "recovery.log").open("w+") as log:
        process = subprocess.Popen(
            [*shlex.split(restart_prefix()), sys.executable, "-c", code, str(tmp_path)],
            env=environment,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            yield process
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=8)


def wait_for(path, process):
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        if path.exists():
            return path.read_text()
        assert process.poll() is None, "wrapper exited before child was ready"
        time.sleep(0.02)
    pytest.fail(f"child did not write {path.name}")


def test_sigsegv_recovers_once_then_normal_exit_stays_finished(tmp_path):
    code = """
import json, os, resource, signal, sys, time
from pathlib import Path
resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
root = Path(sys.argv[1])
recovered = os.environ.get('SENIOR_LAUNCH_RECOVERED')
record = [os.getpid(), time.monotonic(), recovered,
          os.environ['SENIOR_STARTUP_LEARNING_CLAIM']]
with (root / 'attempts').open('a') as attempts:
    attempts.write(json.dumps(record) + '\\n')
(root / ('second' if recovered else 'first')).write_text(json.dumps(record))
if not recovered:
    print('original crash diagnostic', flush=True)
    os.kill(os.getpid(), signal.SIGSEGV)
"""
    with recovered_process(tmp_path, code) as process:
        first = json.loads(wait_for(tmp_path / "first", process))
        assert process.wait(timeout=8) == 0
        second = json.loads((tmp_path / "second").read_text())
    assert first[0] != second[0]
    assert second[1] - first[1] >= 2
    assert first[2:] == [None, "unchanged-run"]
    assert second[2:] == ["1", "unchanged-run"]
    log = (tmp_path / "recovery.log").read_text()
    assert "original crash diagnostic" in log
    assert f"pid={first[0]} returncode=-{signal.SIGSEGV}" in log
    assert "attempt=2" in log
    # Waiting beyond the retry delay catches a detached retry owner, too.
    time.sleep(2.1)
    assert len((tmp_path / "attempts").read_text().splitlines()) == 2


def test_signal_during_backoff_cancels_replacement(tmp_path):
    code = """
import os, sys
from pathlib import Path
root = Path(sys.argv[1])
path = root / ('second' if os.environ.get('SENIOR_LAUNCH_RECOVERED') else 'first')
path.write_text(str(os.getpid()))
raise SystemExit(9)
"""
    with recovered_process(tmp_path, code) as process:
        wait_for(tmp_path / "first", process)
        deadline = time.monotonic() + 2
        while "returncode=9" not in (tmp_path / "recovery.log").read_text():
            assert time.monotonic() < deadline
            time.sleep(0.01)
        start = time.monotonic()
        process.send_signal(signal.SIGTERM)
        assert process.wait(timeout=3) == 0
        assert time.monotonic() - start < 1
    time.sleep(2.1)
    assert not (tmp_path / "second").exists()


def test_old_descendant_exits_before_replacement_and_stop_reaps_new_group(tmp_path):
    code = """
import os, subprocess, sys, time
from pathlib import Path
root = Path(sys.argv[1])
if not os.environ.get('SENIOR_LAUNCH_RECOVERED'):
    child = subprocess.Popen([sys.executable, '-c',
        "import signal,sys,time; from pathlib import Path; "
        "signal.signal(signal.SIGINT, lambda *_: "
        "(Path(sys.argv[1]).write_text('exited'), sys.exit(0))); "
        "Path(sys.argv[2]).write_text('ready'); time.sleep(60)",
        str(root / 'old-exit'), str(root / 'old-ready')])
    (root / 'old-pid').write_text(str(child.pid))
    while not (root / 'old-ready').exists(): time.sleep(.01)
    raise SystemExit(7)
assert (root / 'old-exit').exists()
(root / 'second').write_text(str(os.getpid()))
time.sleep(60)
"""
    with recovered_process(tmp_path, code) as process:
        old = int(wait_for(tmp_path / "old-pid", process))
        new = int(wait_for(tmp_path / "second", process))
        with pytest.raises(ProcessLookupError):
            os.kill(old, 0)
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=8) == 0
        with pytest.raises(ProcessLookupError):
            os.kill(new, 0)


def _dedicated_controller_class(root):
    """Exercise the actual spawn/stop methods without importing a ROS context."""
    from contextlib import AsyncExitStack, asynccontextmanager, suppress

    from senior_control.runtime.launch_recovery import owner_command

    path = Path("ros2/src/senior_navigation/senior_navigation/dedicated_controller.py")
    source = next(
        item for item in ast.parse(path.read_text()).body if isinstance(item, ast.ClassDef)
    )
    scope = dict(
        asyncio=asyncio,
        os=os,
        signal=signal,
        secrets=secrets,
        Path=Path,
        AsyncExitStack=AsyncExitStack,
        asynccontextmanager=asynccontextmanager,
        suppress=suppress,
        Transition=N(TRANSITION_CONFIGURE=1, TRANSITION_ACTIVATE=3),
        get_package_prefix=lambda _: str(root),
        owner_command=owner_command,
    )
    module = ast.Module(
        body=[
            ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0),
            source,
        ],
        type_ignores=[],
    )
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), scope)
    return scope["DedicatedController"]


async def _run_dedicated_fixture(root):
    from contextlib import AsyncExitStack

    controller = _dedicated_controller_class(root)(
        N(get_parameter=lambda _: N(value=False)), root / "params.yaml"
    )

    async def transition(*_):
        pass  # This check concerns process ownership, not ROS lifecycle activation.

    controller._transition = transition
    async with AsyncExitStack() as stack:
        for name in ("controller_server", "bt_navigator"):
            await controller._start(stack, "fixture", name, "-r", "cmd_vel:=/private/output")
        await asyncio.sleep(60)


def _check_dedicated_parent_death(root):
    """An isolated Linux reaper proves exit, including reaping orphan zombies."""
    import ctypes

    assert ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) == 0
    code = (
        "import asyncio,sys; from pathlib import Path; "
        "from test_launch_recovery import _run_dedicated_fixture; "
        "asyncio.run(_run_dedicated_fixture(Path(sys.argv[1])))"
    )
    parent = subprocess.Popen([sys.executable, "-c", code, str(root)])
    children = {}

    def birth(pid):
        try:
            return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
        except FileNotFoundError:
            return None

    def stopped():
        return {
            name: path.read_text() if (path := root / f"{name}.stopped").exists() else None
            for name in ("controller_server", "bt_navigator")
        }

    try:
        for name in ("controller_server", "bt_navigator"):
            record = json.loads(wait_for(root / f"{name}.ready", parent))
            pid, group, arguments = record
            children[pid] = birth(pid)
            assert children[pid] is not None, (pid, stopped())
            assert pid == group
            assert arguments == [
                "--ros-args",
                "--params-file",
                str(root / "params.yaml"),
                "-p",
                "use_sim_time:=false",
                "-p",
                "odom_topic:=/wheel/odom",
                "-r",
                "cmd_vel:=/private/output",
            ]
        parent.kill()  # The real owner has no opportunity to run a finally block.
        parent.wait(timeout=3)
        deadline = time.monotonic() + 5
        for pid in tuple(children):
            while time.monotonic() < deadline:
                collected, status = os.waitpid(pid, os.WNOHANG)
                if collected:
                    # waitpid already consumed this lifetime, even on failure.
                    # Never signal it again from finally or mask its real status.
                    del children[pid]
                    detail = {
                        "pid": pid,
                        "raw_status": status,
                        "exit": os.WEXITSTATUS(status) if os.WIFEXITED(status) else None,
                        "signal": os.WTERMSIG(status) if os.WIFSIGNALED(status) else None,
                        "stopped": stopped(),
                    }
                    assert os.WIFEXITED(status) and os.WEXITSTATUS(status) == 0, detail
                    assert not Path(f"/proc/{pid}").exists()
                    break
                time.sleep(0.02)
            assert pid not in children, ("native child survived its owner", pid, stopped())
        for name in ("controller_server", "bt_navigator"):
            # //Compatible: IntEnum.__str__ changed in Python 3.11; the handler gets int.
            assert (root / f"{name}.stopped").read_text() == str(int(signal.SIGTERM))
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait(timeout=3)
        for pid, identity in children.items():
            try:
                collected, _ = os.waitpid(pid, os.WNOHANG)
                if collected:
                    continue
                # Only an unreaped child of this reaper with the same birth
                # identity may receive failure-path cleanup signals.
                if birth(pid) != identity:
                    continue
                try:
                    os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                os.waitpid(pid, 0)
            except ChildProcessError:
                continue  # No longer our child; never signal a reused PID.


@pytest.mark.skipif(sys.platform != "linux", reason="Linux parent-death binding requires prctl")
def test_dedicated_native_children_exit_when_owner_is_killed(tmp_path):
    # Real non-motion executables ignore ROS arguments, but retain their exact
    # argv and install handlers before reporting readiness. No ROS node starts.
    binary = tmp_path / "lib/fixture"
    binary.mkdir(parents=True)
    code = (
        f"#!{sys.executable}\n"
        + """
import json, os, signal, sys, time
from pathlib import Path
root = Path(os.environ['SENIOR_TEST_OWNER_ROOT'])
name = Path(sys.argv[0]).name
def stop(number, _frame):
    (root / (name + '.stopped')).write_text(str(number))
    raise SystemExit(0)
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
pending = root / (name + '.pending')
pending.write_text(json.dumps([os.getpid(), os.getpgrp(), sys.argv[1:]]))
pending.replace(root / (name + '.ready'))
while True: time.sleep(.1)
"""
    )
    for name in ("controller_server", "bt_navigator"):
        path = binary / name
        path.write_text(code)
        path.chmod(0o700)
    environment = dict(os.environ)
    environment["PYTHONPATH"] = os.pathsep.join(
        [str(Path("src").resolve()), str(Path("tests/unit").resolve())]
    )
    environment["SENIOR_TEST_OWNER_ROOT"] = str(tmp_path)
    check = (
        "import sys; from pathlib import Path; "
        "from test_launch_recovery import _check_dedicated_parent_death; "
        "_check_dedicated_parent_death(Path(sys.argv[1]))"
    )
    subprocess.run(
        [sys.executable, "-c", check, str(tmp_path)], env=environment, check=True, timeout=20
    )


@pytest.mark.asyncio
async def test_dedicated_startup_cancel_reaps_native_process(tmp_path):
    from contextlib import AsyncExitStack

    # Resolve a real executable fixture that consumes the unchanged ROS argv
    # without starting a ROS node.
    binary = tmp_path / "lib/fixture"
    binary.mkdir(parents=True)
    executable = binary / "controller_server"
    executable.write_text(f"#!{sys.executable}\nimport time\ntime.sleep(60)\n")
    executable.chmod(0o700)
    controller = _dedicated_controller_class(tmp_path)(
        N(get_parameter=lambda _: N(value=False)), tmp_path / "params.yaml"
    )
    entered = asyncio.Event()
    process = None

    async def transition(_name, _transition, child):
        nonlocal process
        process = child
        entered.set()
        await asyncio.sleep(60)

    controller._transition = transition

    async def start():
        async with AsyncExitStack() as stack:
            await controller._start(stack, "fixture", "controller_server")

    task = asyncio.create_task(start())
    try:
        await asyncio.wait_for(entered.wait(), 5)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, 6)
        assert process is not None and process.returncode is not None
        with pytest.raises(ProcessLookupError):
            os.killpg(process.pid, 0)
        # A bootstrap whose expected spawning parent has already gone must
        # return without ever exec'ing the native command.
        from senior_control.runtime.launch_recovery import owner_command

        command = owner_command([sys.executable, "-c", "raise SystemExit(29)"])
        command[command.index("--owner") + 1] = str(os.getppid())
        rejected = await asyncio.create_subprocess_exec(*command)
        assert await asyncio.wait_for(rejected.wait(), 3) == 0
    finally:
        if not task.done():
            task.cancel()
        if process is not None and process.returncode is None:
            await controller._stop(process)


def test_recovery_handoff_restores_original_estop_revision_without_reset(monkeypatch, tmp_path):
    """Recovery shares the existing explicit revision-matched reset semantics."""
    from senior_control.motion.contracts import MotionEvent, MotionMode
    from senior_control.motion.contracts import MotionEventKind as K
    from senior_control.motion.state_machine import MotionStateMachine

    machine = MotionStateMachine()
    assert machine.restore_software_estop(3, now_steady=10.0)
    assert machine.state.estop_revision == 3
    assert machine.state.mode is MotionMode.ESTOP
    assert not machine.restore_software_estop(2, now_steady=10.0)
    machine.reduce(MotionEvent(K.ESTOP_RESET, "operator", 2, 1), now_steady=10.0)
    assert machine.state.mode is MotionMode.ESTOP
    machine.reduce(MotionEvent(K.ESTOP_RESET, "operator", 3, 2), now_steady=10.0)
    assert machine.state.mode is MotionMode.IDLE
    machine.reduce(MotionEvent(K.SOFTWARE_ESTOP, "operator", 0, 3), now_steady=10.0)
    assert machine.state.estop_revision == 4

    _check_command_guard_core_recovery(monkeypatch, tmp_path)


def production_command_owner(clock):
    """Reuse the existing ROS-free production-method fixture, including real retirement."""
    from test_manual_command_generation import command_class

    from senior_control.agent.manual_control import ManualIngress, MotionRuntimeReceiver
    from senior_control.protocol import MotionRuntimeState
    from senior_control.safety.arbiter import Arbiter

    base = command_class()
    scope = dict(base._publish_barrier.__globals__)
    scope.update(dataclass=dataclass, secrets=secrets, MotionRuntimeState=MotionRuntimeState)
    scope["__name__"] = __name__
    scope["FinalCommand"] = lambda: N(
        manual_linear=0.0, manual_linear_limit=0.0, degraded_linear_limit=0.0
    )
    path = Path("ros2/src/senior_motion_control/senior_motion_control/command_node.py")
    tree = ast.parse(path.read_text())
    source = next(
        item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == "CommandNode"
    )
    names = {
        "submit",
        "manual_request",
        "_close_manual",
        "publish_command",
        "_publish_manual_target",
        "_publish_barrier",
        "_explicit_estop",
        "_on_output_incarnation_changed",
        "_on_motion_runtime",
        "clear_navigation_source",
        "navigation_source_retired",
        "_other_motion_active",
    }
    source.name, source.bases = "RecoveryCommand", []
    source.body = [
        item for item in source.body if isinstance(item, ast.FunctionDef) and item.name in names
    ]
    assert {item.name for item in source.body} == names
    for method in source.body:
        method.decorator_list = []  # DDS metadata is supplied explicitly below.
    retirement = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "_NavigationRetirement"
    )
    module = ast.Module(
        body=[
            ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0),
            retirement,
            source,
        ],
        type_ignores=[],
    )
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), scope)
    node = scope["RecoveryCommand"]()
    node._monotonic_clock = N(monotonic=lambda: clock.now)
    node._owner_boot, node._generation, node._active = "producer", 1, False
    node._manual_ingress = ManualIngress("robot-test", "producer", ttl=0.3)
    node._manual_target = node._navigation_source = None
    node._arbiter = Arbiter("robot-test", 300)
    node._motion_runtime = MotionRuntimeReceiver()
    node._pending_estop_request = node._latched_estop_request = None
    node._pending_estop_reset = node._pending_output_handoff = None
    node._output_incarnation = node._output_handoff_boot = None
    node._latched_estop_revision = 0
    node._output_handoff_missing_estop = False
    node._retirements, node._estop_requests = {}, {}
    node._last_close_source = ("idle", "", 0, clock.now, "", 0)
    node._close_reason = "prepared"
    node.frames = []
    node._final_publisher = N(publish=lambda frame: node.frames.append(frame))
    node.get_parameter = lambda _: N(value="robot-test")
    node.get_logger = lambda: N(warning=lambda _: None)
    node.destroy_subscription = lambda _: None
    node._observe_retirement_core = lambda *_: None  # No DDS feedback transport in this fixture.
    return node


@contextmanager
def native_recovery_codec(tmp_path):
    """Dynamic input through the actual v3 C++ guard header and Python decoder."""
    from senior_robot.final_command_frame import decode_frame

    compiler = os.environ.get("CXX") or shutil.which("c++")
    if not compiler:
        pytest.skip("native C++ compiler unavailable")
    strings = [
        "robot_id",
        "owner_boot_id",
        "session_id",
        "request_id",
        "source_kind",
        "source_id",
        "close_reason",
    ]
    integers = ["generation", "connection_epoch", "sequence", "gesture", "source_epoch"]
    numbers = [
        "steady_deadline",
        "vx",
        "wz",
        "received_steady",
        "manual_linear",
        "manual_linear_limit",
        "degraded_linear_limit",
    ]
    fields = strings + integers + numbers
    declarations = "".join(f"std::string {name};" for name in strings)
    declarations += "".join(f"uint64_t {name};" for name in integers)
    declarations += "".join(f"double {name};" for name in numbers)
    assignments = "".join(f"m.{name}=argv[{index + 1}];" for index, name in enumerate(strings))
    assignments += "".join(
        f"m.{name}=std::stoull(argv[{fields.index(name) + 1}]);" for name in integers
    )
    assignments += "".join(
        f"m.{name}=std::stod(argv[{fields.index(name) + 1}]);" for name in numbers
    )
    source = tmp_path / "recovery_codec.cpp"
    binary = tmp_path / "recovery_codec"
    source.write_text(
        '#include "final_command_frame.hpp"\n#include <iostream>\n'
        "struct Command {" + declarations + "};\n"
        "int main(int argc,char**argv){if(argc!="
        + str(len(fields) + 1)
        + ")return 2;Command m;"
        + assignments
        + "if(!final_command_frame::manual_metadata(m))return 42;"
        'if(m.close_reason!="intent"&&!final_command_frame::barrier(m))return 42;'
        "std::array<uint8_t,24> gid{};gid.fill(17);"
        "auto frame=final_command_frame::encode(1,gid,m);"
        "std::cout.write(reinterpret_cast<const char*>(frame.data()),frame.size());}"
    )
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(Path("ros2/src/senior_motion_control/native").resolve()),
            str(source),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
    )

    def encode(frame):
        result = subprocess.run(
            [str(binary), *(str(getattr(frame, field)) for field in fields)],
            check=True,
            capture_output=True,
            timeout=10,
        )
        kind, gid, decoded = decode_frame(result.stdout)
        assert kind == 1
        return gid, decoded

    yield encode


def _check_command_guard_core_recovery(monkeypatch, tmp_path):
    from test_learning_client import until, wire_owner
    from test_startup_learning_chain import LearningRig

    from senior_control.models import Velocity
    from senior_control.motion.contracts import MotionMode
    from senior_control.protocol import SimpleEnvelope
    from senior_control.runtime import learning_client
    from senior_control.runtime.learning_wire import Attach, Reserve, State
    from senior_control.safety.arbiter import FinalCommandFence
    from senior_control.settings.models import simulation_settings_profile

    rig = LearningRig(monkeypatch, tmp_path)
    try:
        normal = rig.core
        clock = N(now=rig.now)
        node = production_command_owner(clock)
        requests, _ = wire_owner(monkeypatch, rig.run)
        monkeypatch.setattr(
            "senior_robot.vehicle_learning.LearningClient", learning_client.LearningClient
        )
        monkeypatch.setenv("SENIOR_LAUNCH_RECOVERED", "1")
        first = rig.new_core()
        until(first._learning_client, lambda: first._learning_client.record is not None)
        rig.core = first
        rig.step(rig.now + 0.05)
        assert first._learning_client.read_only and not first.startup.active
        assert not rig.run.owner.record.action.consumed
        assert State in requests and Attach not in requests and Reserve not in requests
        # An initial ordinary heartbeat still leaves the explicitly started owner
        # free to run its original one-time learning action.
        rig.core = normal
        node._publish_barrier()
        assert normal.command(node.frames[-1]) and normal.startup.active
        rig.start()
        assert rig.run.owner.record.action.consumed
        clock.now = rig.now
        driving = simulation_settings_profile().driving_defaults.model_dump()
        sequence = 0

        def request(kind, payload=None):
            nonlocal sequence
            sequence += 1
            value = {
                "v": 1,
                "robot_id": "robot-test",
                "session_id": "controller",
                "connection_epoch": 1,
                "controller_generation": 1,
                "request_id": f"op-{sequence}",
                "seq": sequence,
                "expires_at": 100.0,
                "type": kind,
                "payload": payload or {},
            }
            if kind.startswith("safety."):
                value.pop("controller_generation")
            return value

        def manual(kind, payload=None):
            return node.manual_request(
                {
                    "envelope": json.dumps(request(kind, payload)),
                    "driving": driving,
                    "received_steady": clock.now,
                    "deadline_steady": clock.now + 0.3,
                }
            )

        fences = {}
        with native_recovery_codec(tmp_path) as encode:

            def deliver(frame, output):
                gid, decoded = encode(frame)
                fence = fences.setdefault(output.boot, FinalCommandFence())
                fence.set_publisher(gid)
                if not fence.accept(
                    gid,
                    decoded.owner_boot_id,
                    decoded.generation,
                    decoded.sequence,
                    decoded.steady_deadline,
                    Velocity(decoded.vx, decoded.wz),
                    clock.now,
                    session=decoded.session_id,
                    epoch=decoded.connection_epoch,
                ):
                    return False
                return output.command(decoded)

            def runtime(output):
                output.sequence += 1  # Each real tick advances the runtime sample frontier.
                output._publish_runtime(clock.now, "recovery-test")
                value = output.node.runtime[-1]
                endpoint = N(
                    node_name="motion_reporter",
                    node_namespace="/",
                    endpoint_gid=output.boot.encode(),
                )
                node.get_publishers_info_by_topic = lambda _: [endpoint]
                node._on_motion_runtime(
                    N(data=json.dumps(value)), {"publisher_gid": endpoint.endpoint_gid}
                )

            def echo(output):
                runtime(output)
                assert node._motion_runtime.current(clock.now) is None
                frame = node.frames[-1]
                assert frame.source_id == output.boot and frame.vx == frame.wz == 0
                assert deliver(frame, output)
                runtime(output)
                assert node._output_handoff_boot is None

            acquired = manual("control.acquire")
            authority = acquired["authority_id"]
            old_prepared = node.frames[-1]
            assert manual(
                "drive.intent",
                {
                    "authority_id": authority,
                    "gesture": 1,
                    "sequence": 1,
                    "linear": 0.1,
                    "turn": 0,
                    "gear": "low",
                },
            )["ok"]
            old_intent = node.frames[-1]
            replacement = rig.new_core()
            assert not deliver(old_prepared, replacement)
            assert not deliver(old_intent, replacement)
            node._generation += 1
            node._publish_barrier(reason="prepared")  # Old source, but produced after output start.
            assert not deliver(node.frames[-1], replacement)
            echo(replacement)
            assert node._manual_target is None
            assert not manual(
                "drive.intent",
                {
                    "authority_id": authority,
                    "gesture": 1,
                    "sequence": 2,
                    "linear": 0.1,
                    "turn": 0,
                    "gear": "low",
                },
            )["ok"]
            fresh = manual("control.acquire")
            assert fresh["ok"] and fresh["authority_id"] != authority
            assert deliver(node.frames[-1], replacement)
            assert manual(
                "drive.intent",
                {
                    "authority_id": fresh["authority_id"],
                    "gesture": 1,
                    "sequence": 1,
                    "linear": 0.1,
                    "turn": 0,
                    "gear": "low",
                },
            )["ok"]
            assert deliver(node.frames[-1], replacement)
            assert replacement.target.source == "manual"
            # Retire a live old navigation source through the production cleanup,
            # without invoking any controller, RouteService.start, or goal sender.
            node._manual_ingress.revoke(clock.now)
            node._manual_target = None
            assert node._arbiter.begin_navigation("old-goal", clock.now, task_epoch=1)
            node._navigation_source = N(
                execution_id="old-goal",
                task_epoch=1,
                received_steady=clock.now,
                subscription=object(),
            )
            next_output = rig.new_core()
            echo(next_output)
            assert node._navigation_source is None
            assert not node._arbiter.navigation_source_active("old-goal", clock.now)
            for _ in range(3):
                estop = request("safety.estop")
                assert node.submit(SimpleEnvelope.model_validate(estop)).accepted
                assert deliver(node.frames[-1], next_output)
                runtime(next_output)
            assert next_output.machine.state.estop_revision == 3
            original_request = estop["request_id"]
            assert node._pending_estop_request is None and node._latched_estop_request is not None
            restored = rig.new_core()
            assert deliver(node._latched_estop_request, restored)  # Lock first, no causal echo yet.
            assert restored._recovery_pending and restored.machine.state.mode is MotionMode.ESTOP
            echo(restored)
            handoff = node.frames[-1]
            assert (handoff.request_id, handoff.source_epoch, handoff.gesture) == (
                original_request,
                3,
                0,
            )
            assert restored.machine.state.estop_revision == 3
            assert restored.machine.state.mode is MotionMode.ESTOP
            # A normal stop cannot clear the restored emergency request.
            stop = request("drive.stop")
            assert node.submit(SimpleEnvelope.model_validate(stop)).accepted
            assert deliver(node.frames[-1], restored)
            assert restored.machine.state.mode is MotionMode.ESTOP
            assert node._latched_estop_request.request_id == original_request
            assert manual("safety.estop.reset", {"revision": 2})["ok"]
            assert not deliver(node.frames[-1], restored)
            assert restored.machine.state.mode is MotionMode.ESTOP
            assert manual("safety.estop.reset", {"revision": 3})["ok"]
            assert deliver(node.frames[-1], restored)
            runtime(restored)
            assert restored.machine.state.mode is MotionMode.IDLE
            assert node._latched_estop_request is None
            assert restored._learning_client.read_only and not restored.startup.active
            assert Reserve not in requests and Attach not in requests
    finally:
        rig.close()


async def test_launch_endpoint_reclaims_only_same_owner_refused_stale_socket(
    endpoint_root,  # noqa: F811
    monkeypatch,
):
    import socket

    from senior_robot.agent_adapter import _launch_endpoint

    from senior_control.runtime.ipc import _record

    path = endpoint_root / "agent.sock"
    owner = "a" * 32
    monkeypatch.setenv("SENIOR_LAUNCH_SOCKET_OWNER", owner)
    monkeypatch.setenv("SENIOR_LAUNCH_RECOVERED", "1")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as stale:
        stale.bind(str(path))
        _record(path, run_id=owner)
    async with _launch_endpoint(path) as context:
        assert context == owner
        assert not path.exists()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as active:
        active.bind(str(path))
        active.listen()
        _record(path, run_id=owner)
        identity = path.stat().st_ino
        with pytest.raises(ValueError, match="active"):
            async with _launch_endpoint(path):
                pytest.fail("active endpoint must not be yielded")
        assert path.stat().st_ino == identity
    _record(path, run_id="b" * 32)
    with pytest.raises(ValueError, match="unrecognized"):
        async with _launch_endpoint(path):
            pytest.fail("foreign stale endpoint must not be yielded")
    assert path.stat().st_ino == identity
