"""Persist a SLAM pose graph and the separate long-term occupancy layer.

This module does not publish velocity. safety_node remains the only /cmd_vel
publisher. The pose-graph client targets ROS 2 Humble slam_toolbox 2.6.10 and
is not constructed unless that stack is imported.
"""

from pathlib import Path
from typing import Protocol

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.references import (
    AnchorPose,
    AnchorSnapshot,
    GraphNodeMarker,
    GridExtent,
    SlamSnapshot,
    anchor_from_graph_marker,
)
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import OperationResult, OperationStatus, Pose2D

# slam_toolbox/srv/SerializePoseGraph.srv on the 2.6.10 tag.
SERIALIZE_SERVICE = "/slam_toolbox/serialize_map"
SERIALIZE_SUCCESS = 0
# slam_toolbox/srv/DeserializePoseGraph.srv. The 2.6.10 response is empty.
DESERIALIZE_SERVICE = "/slam_toolbox/deserialize_map"
GRAPH_TOPIC = "/slam_toolbox/graph_visualization"
MATCH_UNSET = 0
MATCH_START_AT_FIRST_NODE = 1
MATCH_START_AT_GIVEN_POSE = 2
MATCH_LOCALIZE_AT_POSE = 3


def finished_deserialize_response(future: object) -> object:
    """Accept a finished call only when it produced a response object.

    Humble 2.6.10 DeserializePoseGraph has an empty response message. That
    object is still success. A null result or a set exception is not.
    """

    done = getattr(future, "done", None)
    if not callable(done) or not done():
        raise DomainError(ErrorCode.TIMEOUT, "deserialize did not finish")
    read_exception = getattr(future, "exception", None)
    if callable(read_exception):
        error = read_exception()
        if error is not None:
            if isinstance(error, BaseException):
                raise DomainError(
                    ErrorCode.LOCALIZATION_UNAVAILABLE, "deserialize failed"
                ) from error
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "deserialize failed")
    read_result = getattr(future, "result", None)
    if not callable(read_result):
        raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "deserialize returned nothing")
    try:
        response = read_result()
    except Exception as exc:
        raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "deserialize failed") from exc
    if response is None:
        raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "deserialize returned nothing")
    return response


class PoseGraphBackend(Protocol):
    """Narrow stand-in for the Humble pose-graph services and node poses."""

    def odom_owner(self) -> str:
        """The single odometry frame this map is allowed to use."""

    def painted_cells(self) -> tuple[tuple[int, int], ...]:
        """Cells the localization raster would paint. Not occupancy evidence."""

    def grid(self) -> GridExtent:
        """Current grid extent. Extent changes do not retarget MapRef."""

    async def corrected_anchors(self) -> tuple[AnchorPose, ...]:
        """Corrected keyframe poses. Unavailable when heading was not verified."""

    async def serialize_pose_graph(self, prefix: str) -> int:
        """Write prefix.posegraph and prefix.data. 0 matches RESULT_SUCCESS."""

    async def deserialize_pose_graph(
        self, prefix: str, match_type: int, initial_pose: Pose2D | None
    ) -> object | None:
        """Load that pose graph. None means the call produced no response."""


class SlamAdapter:
    MATCH_START_AT_FIRST_NODE = MATCH_START_AT_FIRST_NODE
    MATCH_START_AT_GIVEN_POSE = MATCH_START_AT_GIVEN_POSE

    def __init__(self, backend: PoseGraphBackend, store: MapReferenceStore) -> None:
        self._backend = backend
        self._store = store

    async def serialize(self, destination: Path) -> SlamSnapshot:
        destination.mkdir(parents=True, exist_ok=True)
        prefix = destination / "posegraph"
        result = await self._backend.serialize_pose_graph(str(prefix))
        if result != SERIALIZE_SUCCESS:
            raise DomainError(ErrorCode.CONFLICT, "pose graph was not serialized")
        snapshot = self._store.snapshot(prefix, tuple(self._backend.painted_cells()))
        snapshot.write(destination)
        return snapshot

    async def load(self, snapshot: SlamSnapshot, initial_pose: Pose2D | None) -> OperationResult:
        operation = OperationResult("slam-load", OperationStatus.FAILED)
        if snapshot.map_ref != self._store.map_ref or snapshot.odom_owner != self._store.odom_owner:
            return operation
        if snapshot.cell_size_m != self._store.cell_size_m or not self._store.is_empty():
            return operation
        match_type = (
            self.MATCH_START_AT_GIVEN_POSE
            if initial_pose is not None
            else self.MATCH_START_AT_FIRST_NODE
        )
        try:
            response = await self._backend.deserialize_pose_graph(
                str(snapshot.pose_graph_prefix), match_type, initial_pose
            )
        except DomainError:
            return operation
        if response is None:
            return operation
        self._store.restore(snapshot)
        return OperationResult("slam-load", OperationStatus.SUCCEEDED)

    async def anchors(self) -> AnchorSnapshot:
        owner = self._backend.odom_owner().strip()
        if owner != self._store.odom_owner:
            raise DomainError(ErrorCode.CONFLICT, "one map has one odometry owner")
        return AnchorSnapshot(
            self._store.map_ref,
            self._store.odom_owner,
            await self._backend.corrected_anchors(),
            self._backend.grid(),
        )


def anchors_from_graph_markers(markers: tuple[GraphNodeMarker, ...]) -> tuple[AnchorPose, ...]:
    """Keep node markers. Yaw-less Humble spheres stay unavailable."""

    anchors: list[AnchorPose] = []
    for marker in markers:
        anchor = anchor_from_graph_marker(marker)
        if anchor is not None:
            anchors.append(anchor)
    return tuple(anchors)


def create_mapping_node() -> type:
    """ROS node type. Imported only when rclpy is installed. Publishes no velocity."""

    from rclpy.node import Node

    class MappingNode(Node):
        def __init__(self) -> None:
            super().__init__("slam_adapter")
            self.declare_parameter("odom_frame", "odom")
            owner = str(self.get_parameter("odom_frame").value).strip()
            if not owner:
                raise ValueError("a map requires one odometry owner")
            self._odom_owner = owner

    return MappingNode


def main() -> None:
    import rclpy
    from geometry_msgs.msg import Pose2D as RosPose2D
    from rclpy.node import Node
    from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph
    from visualization_msgs.msg import Marker, MarkerArray

    class HumblePoseGraph:
        """Service client for slam_toolbox 2.6.10. Not started on this machine."""

        def __init__(self, node: Node, odom_owner: str) -> None:
            self._node = node
            self._odom_owner = odom_owner
            self._markers: tuple[GraphNodeMarker, ...] = ()
            self._serialize = node.create_client(SerializePoseGraph, SERIALIZE_SERVICE)
            self._deserialize = node.create_client(DeserializePoseGraph, DESERIALIZE_SERVICE)
            node.create_subscription(MarkerArray, GRAPH_TOPIC, self._remember_graph, 1)

        def odom_owner(self) -> str:
            return self._odom_owner

        def painted_cells(self) -> tuple[tuple[int, int], ...]:
            # dynamic_map would rasterize stored scans, including a removed box.
            return ()

        def grid(self) -> GridExtent:
            return GridExtent(0, 0, 0.0, 0.0)

        async def corrected_anchors(self) -> tuple[AnchorPose, ...]:
            return anchors_from_graph_markers(self._markers)

        async def serialize_pose_graph(self, prefix: str) -> int:
            if not self._serialize.wait_for_service(timeout_sec=1.0):
                return 255
            request = SerializePoseGraph.Request()
            request.filename = prefix
            future = self._serialize.call_async(request)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            if not future.done() or future.result() is None:
                return 255
            return int(future.result().result)

        async def deserialize_pose_graph(
            self, prefix: str, match_type: int, initial_pose: Pose2D | None
        ) -> object:
            if not self._deserialize.wait_for_service(timeout_sec=1.0):
                raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "deserialize service missing")
            request = DeserializePoseGraph.Request()
            request.filename = prefix
            request.match_type = match_type
            request.initial_pose = RosPose2D()
            if initial_pose is not None:
                request.initial_pose.x = initial_pose.x
                request.initial_pose.y = initial_pose.y
                request.initial_pose.theta = initial_pose.yaw
            future = self._deserialize.call_async(request)
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            return finished_deserialize_response(future)

        def _remember_graph(self, message: MarkerArray) -> None:
            markers: list[GraphNodeMarker] = []
            for marker in message.markers:
                if marker.ns != "slam_toolbox" or marker.action == Marker.DELETEALL:
                    continue
                # 2.6.10 copies x/y and leaves the quaternion at identity.
                markers.append(
                    GraphNodeMarker(
                        marker.ns,
                        int(marker.id),
                        float(marker.pose.position.x),
                        float(marker.pose.position.y),
                        None,
                    )
                )
            self._markers = tuple(markers)

    rclpy.init()
    node_type = create_mapping_node()
    node = node_type()
    node.pose_graph = HumblePoseGraph(node, node._odom_owner)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
