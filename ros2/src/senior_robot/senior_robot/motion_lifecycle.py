"""Bounded shutdown for the motion-chain ROS entrypoints."""

import signal
from collections.abc import Callable
from typing import TypeVar

import rclpy
from rclpy._rclpy_pybind11 import RCLError  # //Compatible: Humble exposes it here.
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor, await_or_execute
from rclpy.node import Node

_MetadataCallback = TypeVar("_MetadataCallback", bound=Callable)


def with_dds_info(callback: _MetadataCallback) -> _MetadataCallback:
    callback._senior_dds_info = True
    return callback


class MotionExecutor(SingleThreadedExecutor):
    # //Compatible: Humble takes message metadata, then drops it before invoking
    # Python callbacks. Preserve it for the explicitly qualified DDS consumers.
    def _take_subscription(self, sub):
        if not getattr(sub.callback, "_senior_dds_info", False):
            return super()._take_subscription(sub)
        with sub.handle:
            return sub.handle.take_message(sub.msg_type, sub.raw)

    async def _execute_subscription(self, sub, sample):
        if not getattr(sub.callback, "_senior_dds_info", False):
            return await super()._execute_subscription(sub, sample)
        if sample is not None:
            message, info = sample
            await await_or_execute(sub.callback, message, info)


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


def _spin_cli(
    node: Node, executor: SingleThreadedExecutor, should_stop: Callable[[], bool]
) -> None:
    executor.add_node(node)
    try:
        while rclpy.ok() and not should_stop():
            executor.spin_once()
    finally:
        executor.remove_node(node)


def run(
    factory: Callable[[], Node],
    *,
    cli: bool = False,
    spin: Callable[[Node, SingleThreadedExecutor, Callable[[], bool]], None] | None = None,
    executor_factory: Callable[[], SingleThreadedExecutor] | None = None,
) -> None:
    """Run one node; CLI mode is a single process entrypoint invocation."""
    # The factory owns partial resources until it successfully returns a Node.
    # //Compatible: library callers retain their original signal-handler policy.
    previous_handler = signal.getsignal(signal.SIGINT) if cli else None
    previous_term = signal.getsignal(signal.SIGTERM) if cli else None
    stop_requested = False
    stop_guard = None
    executor = None
    if cli:

        def request_stop(signum, frame):
            nonlocal stop_requested
            stop_requested = True
            signal.signal(signum, signal.SIG_IGN)
            if stop_guard is not None:
                stop_guard.trigger()

        # Signals request an orderly exit; publishers stay live for source revocation.
        for signum, handler in ((signal.SIGINT, previous_handler), (signal.SIGTERM, previous_term)):
            if handler != signal.SIG_IGN:
                signal.signal(signum, request_stop)
    try:
        if cli:
            # //Compatible: Humble's native handlers close the context before node
            # cleanup. The CLI owns signals until revocation and destruction finish.
            rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        else:
            rclpy.init()
    except BaseException:
        if cli:
            signal.signal(signal.SIGINT, previous_handler)
            signal.signal(signal.SIGTERM, previous_term)
        raise
    node = None
    try:
        if cli and (stop_requested or not rclpy.ok()):
            return
        # //Compatible: Humble's global executor can disappear during context shutdown.
        executor = (executor_factory or SingleThreadedExecutor)()
        if cli and (stop_requested or not rclpy.ok()):
            return
        node = factory()
        if cli and (stop_requested or not rclpy.ok()):
            return
        if cli:
            # A ready callback wakes spin_once without polling timers or closing
            # the ROS context.
            stop_guard = node.create_guard_condition(lambda: None)
        if spin is None:
            if cli:
                _spin_cli(node, executor, lambda: stop_requested)
            else:
                rclpy.spin(node, executor=executor)
        else:
            # //Compatible: async IPC shares this owned executor and signal lifecycle.
            spin(node, executor, lambda: stop_requested)
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
                    # //Compatible: release Humble's registrations before the
                    # executor destroys its signal guard.
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
