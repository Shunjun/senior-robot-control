"""Private launch prefix: replace an abnormally exited node after group cleanup.

The original command and its standard streams are inherited unchanged. Each
boundary has one retry owner; private lifecycle groups use public LaunchService
Shutdown, never an outer leaf-prefix loop or an additional launch respawn.
"""

import asyncio
import json
import logging
import os
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from typing import Any

from senior_control.motion.process_resources import child_lifetime

_LOG = logging.getLogger(__name__)
_RETRY_DELAY = 2.0
_MODULE = "senior_control.runtime.launch_recovery"


def restart_prefix() -> str:
    """Use the launch interpreter and sourced Python environment, without PATH lookup."""
    return shlex.join([sys.executable, "-m", __name__, "--"])


def _group_exists(pid: int) -> bool:
    try:
        os.killpg(pid, 0)
    except ProcessLookupError:
        return False
    return True


def _clean_group(process: subprocess.Popen[bytes], stop_signal: int) -> None:
    """Reap the leader and wait for every group member, within the helper deadline."""
    for number, timeout in ((stop_signal, 1.5), (signal.SIGTERM, 1.5), (signal.SIGKILL, 2.0)):
        process.poll()
        try:
            os.killpg(process.pid, number)
        except ProcessLookupError:
            process.wait()
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            if not _group_exists(process.pid):
                process.wait()
                return
            time.sleep(0.02)
    raise RuntimeError(f"old process group {process.pid} still exists; replacement refused")


def run_node(command: list[str]) -> int:
    """Run a long-lived launch command; only nonzero, uncancelled exits retry."""
    if not command:
        raise ValueError("launch recovery requires a node command")
    parent_pid = os.getppid()
    stopping = threading.Event()
    stop_signal: int = signal.SIGINT

    def stop(number: int, _frame: object) -> None:
        nonlocal stop_signal
        stop_signal = number
        stopping.set()

    previous = {number: signal.signal(number, stop) for number in (signal.SIGINT, signal.SIGTERM)}
    process = None
    attempt = 0
    try:
        # Reuse the existing Linux parent-death safeguard: a dead launch owner
        # cancels this prefix and cleans its node instead of leaving a retry loop.
        child_lifetime(parent_pid)
        while not stopping.is_set():
            environment = dict(os.environ)
            # The marker belongs to this owner, not to a recovered ancestor.
            environment.pop("SENIOR_LAUNCH_RECOVERED", None)
            if attempt:
                environment["SENIOR_LAUNCH_RECOVERED"] = "1"
            if stopping.is_set():
                break
            process = subprocess.Popen(command, env=environment, start_new_session=True)
            attempt += 1
            _LOG.info("started pid=%s attempt=%s", process.pid, attempt)
            # A cancellation between the pre-spawn check and Popen must clean
            # the just-created group without handing it to another retry.
            while process.poll() is None and not stopping.wait(0.05):
                pass
            returncode = process.poll()
            if returncode is not None:
                _LOG.info(
                    "exited pid=%s returncode=%s attempt=%s", process.pid, returncode, attempt
                )
            finished = process
            process = None
            _clean_group(finished, stop_signal)
            if stopping.is_set() or returncode == 0:
                return 0
            if stopping.wait(_RETRY_DELAY):
                return 0
        return 0
    finally:
        try:
            if process is not None:
                _clean_group(process, stop_signal)
        finally:
            for number, handler in previous.items():
                signal.signal(number, handler)


def lifecycle_group_command(
    filename: str, arguments: dict[str, str], manager: str, servers: list[str]
) -> list[str]:
    """Private Nav2/localization launch boundary; never a user CLI entry point."""
    packet = {"launch": filename, "arguments": arguments, "manager": manager, "servers": servers}
    return [sys.executable, "-m", _MODULE, "--lifecycle-group", json.dumps(packet)]


def lifecycle_owner_prefix() -> str:
    """Exec the native manager in its own cleanup group, without a retry loop."""
    return shlex.join([sys.executable, "-m", _MODULE, "--owner", "--"])


def _exec_owner(command: list[str]) -> None:
    child_lifetime(os.getppid())
    # //Compatible: keep a launch child that already has its own process group.
    if os.getpgrp() != os.getpid():
        os.setsid()
    # //Compatible: restore optional POSIX ignored signals as subprocess does.
    for name in ("SIGPIPE", "SIGXFZ", "SIGXFSZ"):
        number = getattr(signal, name, None)
        if number is not None:
            signal.signal(number, signal.SIG_DFL)
    os.execvpe(command[0], command, dict(os.environ))


class _LifecycleGroup:
    """One existing Nav2 manager and its launch-owned members, with read-only state probes."""

    def __init__(self, service: Any, packet: dict[str, Any], stop: Any) -> None:
        self.service, self.packet, self.stop = service, packet, stop
        self.servers: set[Any] = set()
        self.managers: set[Any] = set()
        self.wrapped: set[Any] = set()
        self.owner_groups: set[int] = set()
        self.stderr: dict[int, bytes] = {}
        self.failed = self.normal = self.unsafe = False
        self.server_failed = False
        self.failure_generation = 0
        self.inactive_since: float | None = None

    def bind(self, servers: list[Any], managers: list[Any], adapters: list[Any]) -> list[Any]:
        self.servers.update(servers)
        self.managers.update(managers)
        self.wrapped.update([*servers, *adapters])
        return [*servers, *managers, *adapters]

    def setup(self, context: Any) -> list[Any]:
        # Humble LaunchService's default SIGTERM cancels its loop and warns of
        # orphans. This private owner turns both stop signals into normal public
        # Shutdown events and lets every member finish its cleanup first.
        for number in (signal.SIGINT, signal.SIGTERM):
            context.asyncio_loop.add_signal_handler(number, self.stop)
        context.add_completion_future(context.asyncio_loop.create_task(self.monitor()))
        return []

    def started(self, event: Any, context: Any) -> list[Any]:
        if event.action in self.managers:
            self.owner_groups.add(event.pid)
        return []

    def process_io(self, event: Any) -> None:
        if event.action not in self.servers:
            return
        data = self.stderr.get(event.pid, b"") + event.text
        lines = data.split(b"\n")
        self.stderr[event.pid] = lines.pop()[-16384:]
        for line in lines:
            # This is the existing prefix's own actual PID/returncode record,
            # never a vendor warning or a timeout/liveness guess. It only arms
            # a read-only lifecycle check; it does not restart the server here.
            match = re.fullmatch(
                rb"\[launch_recovery\] exited pid=\d+ returncode=(-?\d+) attempt=\d+\r?", line
            )
            if match is not None and int(match[1]) != 0:
                self.server_failed = True
                self.failure_generation += 1
                self.inactive_since = None

    def exited(self, event: Any, context: Any) -> list[Any]:
        from launch.events import Shutdown

        self.stderr.pop(event.pid, None)
        if event.action in self.managers:
            # The exec bootstrap retained the native PID/returncode, with no
            # independent manager respawn owner.
            context.add_completion_future(
                context.asyncio_loop.create_task(self.clear_owner(event.pid))
            )
            if not context.is_shutdown:
                self.failed = event.returncode != 0
                self.normal = event.returncode == 0
                context.emit_event_sync(Shutdown(reason="lifecycle manager exited"))
        elif event.action in self.wrapped:
            if event.returncode != 0:
                # A leaf wrapper exits nonzero only when it could not complete
                # its own launch/cleanup contract. No new group may overlap it.
                self.unsafe = True
                context.emit_event_sync(Shutdown(reason="lifecycle member cleanup failed"))
            elif not context.is_shutdown:
                self.normal = True
                context.emit_event_sync(Shutdown(reason="lifecycle member finished normally"))
        return []

    async def clear_owner(self, pid: int) -> None:
        for number, timeout in ((signal.SIGINT, 1.5), (signal.SIGTERM, 1.5), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(pid, number)
            except ProcessLookupError:
                self.owner_groups.discard(pid)
                return
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                if not _group_exists(pid):
                    self.owner_groups.discard(pid)
                    return
                await asyncio.sleep(0.02)
        self.unsafe = True
        _LOG.error("lifecycle owner group %s still exists; replacement refused", pid)

    async def monitor(self) -> None:
        import rclpy
        from launch.events import Shutdown
        from lifecycle_msgs.msg import State
        from lifecycle_msgs.srv import GetState
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from std_srvs.srv import Trigger

        ros = Context()
        node = executor = None
        try:
            rclpy.init(args=[], context=ros, signal_handler_options=SignalHandlerOptions.NO)
            simulated = self.packet["arguments"].get("use_sim_time", "false") == "true"
            node = Node(
                f"launch_recovery_{os.getpid()}",
                context=ros,
                use_global_arguments=False,
                start_parameter_services=False,
                enable_rosout=False,
                parameter_overrides=[Parameter("use_sim_time", value=simulated)],
            )
            executor = SingleThreadedExecutor(context=ros)
            executor.add_node(node)
            active = node.create_client(Trigger, f"/{self.packet['manager']}/is_active")
            clients = [
                node.create_client(GetState, f"/{name}/get_state")
                for name in self.packet["servers"]
            ]
            reported = False
            while not self.service.context.is_shutdown:
                generation = self.failure_generation
                manager_future = (
                    active.call_async(Trigger.Request()) if active.service_is_ready() else None
                )
                states = [
                    client.call_async(GetState.Request()) if client.service_is_ready() else None
                    for client in clients
                ]
                futures = [item for item in [manager_future, *states] if item is not None]
                deadline = time.monotonic() + 0.5
                while time.monotonic() < deadline and any(not item.done() for item in futures):
                    executor.spin_once(timeout_sec=0)
                    if self.service.context.is_shutdown:
                        return
                    await asyncio.sleep(0.02)
                manager_active = (
                    manager_future.result().success
                    if manager_future is not None
                    and manager_future.done()
                    and manager_future.exception() is None
                    else None
                )
                all_active = manager_active is True and all(
                    item is not None
                    and item.done()
                    and item.exception() is None
                    and item.result().current_state.id == State.PRIMARY_STATE_ACTIVE
                    for item in states
                )
                # A reply requested before the actual crash cannot acknowledge
                # its recovery, even if it arrives after the exit event.
                if generation != self.failure_generation:
                    pass
                elif all_active:
                    if not reported or self.server_failed:
                        _LOG.info("lifecycle group ACTIVE manager=%s", self.packet["manager"])
                    reported = True
                    self.server_failed = False
                    self.inactive_since = None
                elif self.server_failed and manager_active is False:
                    # Humble's native bond reset starts its ten-second reconnect
                    # window after detecting a real crash. Respect its clock;
                    # normal pause/reset without a witnessed crash never retries.
                    now = node.get_clock().now().nanoseconds / 1e9
                    if self.inactive_since is None:
                        # Native Humble's first respawn check runs one second
                        # after bond reset, and only then starts its window.
                        self.inactive_since = now + 1.0
                    elif now - self.inactive_since >= 10.0:
                        self.failed = True
                        self.service.context.emit_event_sync(
                            Shutdown(reason="native lifecycle reconnection window exhausted")
                        )
                for item in futures:
                    if not item.done():
                        item.cancel()
                await asyncio.sleep(0.2)
        except Exception as exc:
            self.unsafe = True
            _LOG.error("lifecycle state probe failed: %s", exc)
            self.service.context.emit_event_sync(Shutdown(reason="lifecycle probe failed"))
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if ros.ok():
                ros.shutdown()


async def _run_lifecycle_group(packet: dict[str, Any]) -> int:
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
    from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
    from launch.events import Shutdown
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    stopped = asyncio.Event()
    service = None

    def stop(*_args: Any) -> None:
        stopped.set()
        if service is not None:
            service.context.emit_event_sync(Shutdown(reason="lifecycle group cancelled"))

    previous = {number: signal.signal(number, stop) for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        child_lifetime(os.getppid())
        while not stopped.is_set():
            service = LaunchService(noninteractive=True)
            owner = _LifecycleGroup(service, packet, stop)
            service.context.extend_globals({"senior_lifecycle_recovery": owner})
            service.include_launch_description(
                LaunchDescription(
                    [
                        RegisterEventHandler(OnProcessStart(on_start=owner.started)),
                        RegisterEventHandler(OnProcessExit(on_exit=owner.exited)),
                        RegisterEventHandler(OnProcessIO(on_stderr=owner.process_io)),
                        OpaqueFunction(function=owner.setup),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(packet["launch"]),
                            launch_arguments=packet["arguments"].items(),
                        ),
                    ]
                )
            )
            result = await service.run_async()
            for number in previous:
                asyncio.get_running_loop().remove_signal_handler(number)
                signal.signal(number, stop)
            service = None
            if owner.unsafe or owner.owner_groups:
                return 1
            if stopped.is_set() or owner.normal or not owner.failed:
                return 0 if stopped.is_set() else result
            try:
                await asyncio.wait_for(stopped.wait(), _RETRY_DELAY)
            except asyncio.TimeoutError:
                continue
        return 0
    finally:
        for number, handler in previous.items():
            asyncio.get_running_loop().remove_signal_handler(number)
            signal.signal(number, handler)


def _main() -> int:
    logging.basicConfig(level=logging.INFO, format="[launch_recovery] %(message)s")
    if len(sys.argv) == 3 and sys.argv[1] == "--lifecycle-group":
        return asyncio.run(_run_lifecycle_group(json.loads(sys.argv[2])))
    if len(sys.argv) > 3 and sys.argv[1:3] == ["--owner", "--"]:
        _exec_owner(sys.argv[3:])
        return 1  # exec never returns on success.
    if len(sys.argv) < 3 or sys.argv[1] != "--":
        _LOG.error("expected private launch prefix followed by -- and original command")
        return 2
    try:
        return run_node(sys.argv[2:])
    except (OSError, ValueError, RuntimeError) as exc:
        _LOG.error("recovery stopped: %s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(_main())
