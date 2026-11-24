"""Latest-only serialized image pacing before image_transport compression."""

import time

PREVIEW_FPS = 15.0


class LatestImageGate:
    """Keep image headers/pixels opaque and discard replaced frames before encoding."""

    def __init__(self) -> None:
        self._latest: bytes | None = None
        self.next_at = 0.0

    def offer(self, image: bytes) -> None:
        self._latest = image

    def take(self, now: float) -> bytes | None:
        if self._latest is None or now + 1e-9 < self.next_at:
            return None
        image, self._latest = self._latest, None
        self.next_at = now + 1 / PREVIEW_FPS
        return image


def main() -> None:
    # Importing the pacing policy never requires a ROS installation.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("camera_compression_rate_limit")
    gate = LatestImageGate()
    try:
        source = node.declare_parameter("input_topic", "/camera/color/image_raw").value
        destination = node.declare_parameter("output_topic", "/senior_camera/color/image_raw").value
        qos = QoSProfile(
            depth=1,
            # The vendor raw publishers and image_transport use reliable delivery.
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        publisher = node.create_publisher(Image, destination, 1)
        # Humble supports serialized subscription and publish: no pixel conversion here.
        node.create_subscription(Image, source, gate.offer, qos, raw=True)

        def publish_latest() -> None:
            image = gate.take(time.monotonic())
            if image is not None:
                publisher.publish(image)
            remaining = gate.next_at - time.monotonic()
            delay = remaining if remaining > 0 else 1 / PREVIEW_FPS
            # Reset relative to the last actual publish; never catch up in a burst.
            timer.timer_period_ns = round(max(0.001, delay) * 1e9)
            timer.reset()

        timer = node.create_timer(1 / PREVIEW_FPS, publish_latest)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
