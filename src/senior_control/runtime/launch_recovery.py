"""Private launch prefix: replace an abnormally exited node after group cleanup.

The original command and its standard streams are inherited unchanged. This
module owns one retry loop; callers must not also enable launch respawn.
"""

import logging
import os
import shlex
import signal
import subprocess
import sys
import threading
import time

from senior_control.motion.process_resources import child_lifetime

_LOG = logging.getLogger(__name__)
_RETRY_DELAY = 2.0


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


def _main() -> int:
    logging.basicConfig(level=logging.INFO, format="[launch_recovery] %(message)s")
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
