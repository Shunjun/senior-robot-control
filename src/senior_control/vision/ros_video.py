"""On-demand ROS camera subscriber with one replaceable frame slot."""

import asyncio
import importlib
import logging
import threading
import time
from typing import Any

from senior_control.vision.synchronizer import ImageFrame

logger = logging.getLogger(__name__)


class RosCameraSource:
    def __init__(self, topic: str) -> None:
        self._topic = topic
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._latest: ImageFrame | None = None
        self._last_stamp = 0

    def _receive(self, message: Any, node: Any) -> None:
        width, height, step = message.width, message.height, message.step
        row = width * 3
        if (
            not 0 < width <= 640
            or not 0 < height <= 480
            or message.encoding not in {"rgb8", "bgr8"}
            or not row <= step <= row + 128
            or len(message.data) != step * height
        ):
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        received_at = time.time()
        age = (node.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= self._last_stamp or not 0 <= age <= 2.0:
            return
        self._last_stamp = stamp
        data = bytes(message.data)
        if step != row:
            data = b"".join(data[start : start + row] for start in range(0, len(data), step))
        frame = ImageFrame(received_at - age, width, height, message.encoding, data)
        with self._lock:
            self._latest = frame

    def _run(self) -> None:
        context = node = executor = None
        try:
            rclpy = importlib.import_module("rclpy")
            context = importlib.import_module("rclpy.context").Context()
            rclpy.init(args=[], context=context)
            node = rclpy.create_node("senior_video_camera", context=context)
            qos_module = importlib.import_module("rclpy.qos")
            qos = qos_module.QoSProfile(
                depth=1,
                reliability=qos_module.ReliabilityPolicy.BEST_EFFORT,
                durability=qos_module.DurabilityPolicy.VOLATILE,
            )
            node.create_subscription(
                importlib.import_module("sensor_msgs.msg").Image,
                self._topic,
                lambda message: self._receive(message, node),
                qos,
            )
            executor = importlib.import_module("rclpy.executors").SingleThreadedExecutor(
                context=context
            )
            executor.add_node(node)
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.1)
        except Exception:
            logger.warning("ROS video unavailable; control remains independent", exc_info=True)
        finally:
            try:
                if executor is not None:
                    executor.shutdown(timeout_sec=1.0)
            finally:
                try:
                    if node is not None:
                        node.destroy_node()
                finally:
                    if context is not None:
                        context.try_shutdown()
            with self._lock:
                self._latest = None

    async def next_frame(self, timeout: float) -> ImageFrame | None:
        if self._thread is None and not self._stop.is_set():
            self._thread = threading.Thread(target=self._run, name="ros-video", daemon=True)
            self._thread.start()
        deadline = asyncio.get_running_loop().time() + max(0, timeout)
        while not self._stop.is_set():
            with self._lock:
                frame, self._latest = self._latest, None
            if frame is not None and 0 <= time.time() - frame.stamp <= 2.0:
                return frame
            remaining = deadline - asyncio.get_running_loop().time()
            if remaining <= 0:
                break
            await asyncio.sleep(min(remaining, 0.025))
        return None

    async def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            await asyncio.to_thread(self._thread.join, 2.0)
            if self._thread.is_alive():
                logger.warning("ROS video subscriber has not finished shutting down")
        with self._lock:
            self._latest = None
