"""Bounded shutdown for the motion-chain ROS entrypoints."""

import signal
from collections.abc import Callable

import rclpy
from rclpy._rclpy_pybind11 import RCLError  # //Compatible: Humble exposes it here.
from rclpy.executors import ExternalShutdownException
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


def run(factory: Callable[[], Node], *, cli: bool = False) -> None:
    # //Compatible: library callers retain their original signal-handler policy.
    rclpy.init()
    node = None
    try:
        node = factory()
        rclpy.spin(node)
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
        try:
            try:
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
