"""ROS adapter for localization. It publishes no velocity.

safety_node remains the only /cmd_vel publisher. Clock and samples are injected
into the adapter; this process does not read a GPS device and does not sleep.
"""

from senior_control.clock import Clock, SystemClock
from senior_control.localization.geo import GeoAlignment, PositionEstimate, QualifiedFix
from senior_control.localization.heading import HeadingObservation, HeadingObserver, MotionSample
from senior_control.localization.relocalization import Relocalizer
from senior_control.mapping.archive import Georeference
from senior_control.models import MapRef


class LocalizationAdapter:
    """In-process adapter. ``published_topics`` stays empty: no /cmd_vel."""

    def __init__(
        self,
        *,
        alignment: GeoAlignment,
        relocalizer: Relocalizer,
        observer: HeadingObserver,
        clock: Clock,
    ) -> None:
        self._alignment = alignment
        self._relocalizer = relocalizer
        self._observer = observer
        self._clock = clock
        self.published_topics: list[str] = []

    def ingest_sample(self, sample: MotionSample) -> HeadingObservation | None:
        return self._observer.observe(sample)

    def ingest_fix(self, fix: QualifiedFix) -> PositionEstimate:
        return self._alignment.to_map(fix)

    def clock(self) -> Clock:
        return self._clock


def create_localization_node() -> type:
    """ROS node type. Imported only when rclpy is installed. Publishes no velocity."""

    from rclpy.node import Node

    class LocalizationNode(Node):
        def __init__(self) -> None:
            super().__init__("localization")
            clock = SystemClock()
            georeference = Georeference("unreferenced")
            alignment = GeoAlignment(
                georeference,
                origin_latitude_deg=None,
                origin_longitude_deg=None,
                origin_altitude_m=None,
                uncertainty_m=(float("inf"), float("inf"), float("inf")),
                clock=clock,
                max_age_s=1.0,
                max_horizontal_std_m=3.0,
            )
            self.adapter = LocalizationAdapter(
                alignment=alignment,
                relocalizer=Relocalizer(
                    map_ref=MapRef("unlocalized", 0),
                    georeference=georeference,
                    alignment=alignment,
                    scan_match=lambda _seed: None,
                    bundle_has_gps=False,
                ),
                observer=HeadingObserver(min_baseline_m=5.0, max_correction_rad=0.35),
                clock=clock,
            )

    return LocalizationNode


def main() -> None:
    import rclpy

    rclpy.init()
    node_type = create_localization_node()
    node = node_type()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
