"""Private launch prefix: replace an abnormally exited node after group cleanup.

The original command and its standard streams are inherited unchanged. Each
boundary has one retry owner; private lifecycle groups use public LaunchService
Shutdown, never an outer leaf-prefix loop or an additional launch respawn.
"""

import asyncio
import json
import logging
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from typing import Any

from vehicle_core.process_resources import child_lifetime

_LOG = logging.getLogger(__name__)
_RETRY_DELAY = 2.0
_MODULE = "vehicle_core.launch_recovery"


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


def lifecycle_group_command(filename: str, arguments: dict[str, str]) -> list[str]:
    """Private Nav2/localization launch boundary; never a user CLI entry point."""
    packet = {"launch": filename, "arguments": arguments, "parent_pid": os.getpid()}
    return [sys.executable, "-m", _MODULE, "--lifecycle-group", json.dumps(packet)]


def lifecycle_owner_prefix() -> str:
    """Exec one launch member in its own cleanup group, without a retry loop."""
    return shlex.join(owner_command([]))


def owner_command(command: list[str]) -> list[str]:
    """Private exec bootstrap, bound to this exact spawning parent; never retries."""
    return [sys.executable, "-m", _MODULE, "--owner", str(os.getpid()), "--", *command]


def _exec_owner(command: list[str], parent_pid: int) -> None:
    # Capture the expected parent in the spawner, before creating this child.
    # Reading getppid here could bind an already orphaned child to PID 1.
    child_lifetime(parent_pid)
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
    """One retry owner for all members of an existing Nav2/localization launch."""

    def __init__(self, stop: Any) -> None:
        self.stop = stop
        self.members: set[Any] = set()
        self.owner_groups: set[int] = set()
        self.failed = self.normal = self.unsafe = False

    def bind(self, members: list[Any]) -> list[Any]:
        self.members.update(members)
        return members

    def setup(self, context: Any) -> list[Any]:
        # Humble's default SIGTERM cancels the loop. Both stop signals instead
        # request public Shutdown and wait for all native members and cleanup.
        for number in (signal.SIGINT, signal.SIGTERM):
            context.asyncio_loop.add_signal_handler(number, self.stop)
        return []

    def started(self, event: Any, context: Any) -> list[Any]:
        if event.action in self.members:
            self.owner_groups.add(event.pid)
        return []

    def exited(self, event: Any, context: Any) -> list[Any]:
        from launch.events import Shutdown

        if event.action not in self.members:
            return []
        # Every member execs its original command in a registered process group;
        # there is no leaf retry loop or separate lifecycle reconnection owner.
        context.add_completion_future(context.asyncio_loop.create_task(self.clear_owner(event.pid)))
        if not context.is_shutdown:
            self.failed = self.failed or event.returncode != 0
            self.normal = self.normal or event.returncode == 0
            _LOG.info("lifecycle member exited pid=%s returncode=%s", event.pid, event.returncode)
            context.emit_event_sync(Shutdown(reason="lifecycle member exited"))
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


async def _run_lifecycle_group(packet: dict[str, Any]) -> int:
    from launch import LaunchDescription, LaunchService
    from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
    from launch.event_handlers import OnProcessExit, OnProcessStart
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
        child_lifetime(packet["parent_pid"])
        while not stopped.is_set():
            service = LaunchService(noninteractive=True)
            owner = _LifecycleGroup(stop)
            service.context.extend_globals({"senior_lifecycle_recovery": owner})
            service.include_launch_description(
                LaunchDescription(
                    [
                        RegisterEventHandler(OnProcessStart(on_start=owner.started)),
                        RegisterEventHandler(OnProcessExit(on_exit=owner.exited)),
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
    if len(sys.argv) > 4 and sys.argv[1] == "--owner" and sys.argv[3] == "--":
        _exec_owner(sys.argv[4:], int(sys.argv[2]))
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
