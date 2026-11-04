"""Bounded shutdown for the motion-chain ROS entrypoints."""

import signal
import sys
from collections.abc import Callable

import rclpy
from rclpy._rclpy_pybind11 import RCLError  # //Compatible: Humble exposes it here.
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node


def _shutdown_subscription_conversion(exc: RuntimeError) -> bool:
    # //Compatible: Limit the observed Humble pybind error to a closed-context shutdown.
    message = "Unable to convert call argument to Python object (compile in debug mode for details)"
    if rclpy.ok() or str(exc) != message:
        return False
    traceback = exc.__traceback__
    while traceback is not None:
        frame = traceback.tb_frame
        if (
            frame.f_globals.get("__name__") == "rclpy.executors"
            and frame.f_code.co_name == "_take_subscription"
        ):
            return True
        traceback = traceback.tb_next
    return False


def run(
    factory: Callable[[], Node],
    *,
    cli: bool = False,
    spin: Callable[[Node, SingleThreadedExecutor], None] | None = None,
) -> None:
    """Run one node; CLI mode is a single process entrypoint invocation."""
    # The factory owns partial resources until it successfully returns a Node.
    # //Compatible: library callers retain their original signal-handler policy.
    previous_handler = signal.getsignal(signal.SIGINT) if cli else None
    previous_term = signal.getsignal(signal.SIGTERM) if cli else None
    stop_requested = False
    if cli and callable(previous_handler):

        def first_interrupt(signum, frame):
            nonlocal stop_requested
            stop_requested = True
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            if isinstance(sys.exc_info()[1], (KeyboardInterrupt, ExternalShutdownException)):
                return
            if previous_handler is signal.default_int_handler:
                # //Compatible: let Humble's native signal chain close the context
                # without asynchronously interrupting partially built ROS tasks.
                return
            previous_handler(signum, frame)

        # //Compatible: install before Humble chains its native signal handler.
        signal.signal(signal.SIGINT, first_interrupt)
    if cli and previous_term != signal.SIG_IGN:

        def first_termination(signum, frame):
            nonlocal stop_requested
            stop_requested = True
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            if callable(previous_term):
                previous_term(signum, frame)

        signal.signal(signal.SIGTERM, first_termination)
    try:
        if cli:
            # //Compatible: delay Humble's native handlers until construction
            # has completed, so they cannot close a half-built executor context.
            rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        else:
            rclpy.init()
    except BaseException:
        if cli:
            signal.signal(signal.SIGINT, previous_handler)
            signal.signal(signal.SIGTERM, previous_term)
        raise
    node = None
    executor = None
    try:
        if cli and (stop_requested or not rclpy.ok()):
            return
        # //Compatible: Humble's global executor can disappear during context shutdown.
        executor = SingleThreadedExecutor()
        if cli and (stop_requested or not rclpy.ok()):
            return
        node = factory()
        if cli and (stop_requested or not rclpy.ok()):
            return
        if cli:
            rclpy.signals.install_signal_handlers(rclpy.signals.SignalHandlerOptions.ALL)
            if stop_requested or not rclpy.ok():
                return
        if spin is None:
            rclpy.spin(node, executor=executor)
        else:
            # //Compatible: async IPC shares this owned executor and signal lifecycle.
            spin(node, executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise
    except RuntimeError as exc:
        if not _shutdown_subscription_conversion(exc):
            raise
    finally:
        # A launch group may deliver SIGINT both directly and via its parent.
        # Finish destruction once before allowing another Python interrupt.
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        if cli:
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            try:
                if cli:
                    # //Compatible: Humble's native signal thread must finish
                    # before the executor unregisters its signal guard.
                    rclpy.signals.uninstall_signal_handlers()
            finally:
                try:
                    try:
                        if executor is not None:
                            executor.shutdown()
                    finally:
                        if node is not None:
                            node.destroy_node()
                finally:
                    # //Compatible: Humble may already have closed the ROS context.
                    rclpy.try_shutdown()
        finally:
            # In a standalone entrypoint, multiprocessing's owned-child
            # atexit work runs after this function returns. Keep late launch
            # interrupts ignored until the interpreter exits.
            signal.signal(signal.SIGINT, signal.SIG_IGN if cli else previous)
            if cli:
                signal.signal(signal.SIGTERM, previous_term)
