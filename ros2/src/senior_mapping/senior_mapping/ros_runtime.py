"""ROS/process adapter for the serial mapping owner. No database or velocity writes.

Large graph cuts use one private RAM-backed transfer file, not the bounded motion
IPC frame. Only the owner requests the next cut, so neither history nor tasks queue.
"""

import asyncio
import hashlib
import json
import math
import os
import signal
import stat
import tempfile
import uuid
from dataclasses import asdict
from pathlib import Path
from typing import Any

from senior_perception.vision.raw_capture import capture_from_message

from senior_mapping.navigation_bridge import NativeNavigationBridge
from senior_mapping.references import decode_fixed_snapshot
from senior_mapping.shared_scan import LatestScanSlot, decode_qualified_scan
from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.lifecycle_events import ResourceProcesses


class RosMappingBackend:
    def __init__(self, node: Any, *, stopped: Any, reference: Any = None) -> None:
        self.node, self._stopped = node, stopped
        self._services: dict[str, tuple[Any, Any]] = {}
        self._child: asyncio.subprocess.Process | None = None
        self._relay: asyncio.subprocess.Process | None = None
        self._resources = ResourceProcesses("mapping")
        self._boot = ""
        self._root: Path | None = None
        ram = "/dev/shm" if Path("/dev/shm").is_dir() else None
        self._transfer = tempfile.TemporaryDirectory(prefix="senior-map-", dir=ram)
        self._lock = asyncio.Lock()
        self._navigation = NativeNavigationBridge(node, reference, self._call)
        self._scan_subscription = self._scan_timer = None
        self._scan_accepting = False
        self._qualified_gid = None
        self._learning_client = None
        self._scan_slot = None
        self._host_boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self._navigation.host_boot_id = self._host_boot_id
        self._register_qualified_receipt()

    def _prepare_services(self) -> None:
        if self._services:
            return
        try:
            from senior_interfaces.srv import (
                CaptureGraph,
                MappingGate,
                ReadGraph,
                ReadGraphCommit,
                ReadMappingGrid,
                ReadMappingPose,
            )
        except ImportError as exc:
            # //Compatible: older ROS installs have no mapping gate; preserve existing control.
            raise DomainError(
                ErrorCode.UNAVAILABLE, "mapping interfaces need a coordinated rebuild"
            ) from exc

        self._services = {
            "grid": (
                self.node.create_client(ReadMappingGrid, "/senior/slam/read_mapping_grid"),
                ReadMappingGrid,
            ),
            # //Compatible: retained diagnostic service; no production freshness polling.
            "pose": (
                self.node.create_client(ReadMappingPose, "/senior/slam/read_mapping_pose"),
                ReadMappingPose,
            ),
            "graph": (self.node.create_client(ReadGraph, "/senior/slam/read_graph"), ReadGraph),
            "next": (
                self.node.create_client(ReadGraphCommit, "/senior/slam/read_graph_commit"),
                ReadGraphCommit,
            ),
            "capture": (
                self.node.create_client(CaptureGraph, "/senior/slam/capture_graph"),
                CaptureGraph,
            ),
            "gate": (
                self.node.create_client(MappingGate, "/senior/slam/mapping_gate"),
                MappingGate,
            ),
        }

    async def _call(self, method: str, **fields: Any) -> Any:
        client, service = self._services[method]
        deadline = asyncio.get_running_loop().time() + 10
        while not client.service_is_ready():
            if self._child is None or self._child.returncode is not None:
                raise DomainError(ErrorCode.UNAVAILABLE, "SLAM process is not running")
            if asyncio.get_running_loop().time() >= deadline:
                raise DomainError(ErrorCode.TIMEOUT, "SLAM service did not start")
            await asyncio.sleep(0.02)
        request = service.Request()
        for name, value in fields.items():
            setattr(request, name, value)
        future = client.call_async(request)
        try:
            while not future.done():
                if asyncio.get_running_loop().time() >= deadline:
                    raise DomainError(ErrorCode.TIMEOUT, "SLAM service timed out")
                await asyncio.sleep(0.005)
            response = future.result()
            if response is None:
                raise DomainError(ErrorCode.UNAVAILABLE, "SLAM returned no response")
            return response
        finally:
            if not future.done():
                future.cancel()

    async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
        async with self._lock:
            try:
                method = kind.removeprefix("map.backend.")
                if method != "stop":
                    self._prepare_services()
                if method == "status":
                    return {"ok": True, "stopped": bool(self._stopped())}
                if method == "start":
                    return await self._start(fields)
                if method == "stop":
                    await self._stop()
                    return {"ok": True}
                if fields.get("boot_id") != self._boot or not self._boot:
                    raise DomainError(ErrorCode.CONFLICT, "SLAM boot changed")
                if method == "gate":
                    response = await self._call(
                        "gate", boot_id=self._boot, accepting=fields["accepting"]
                    )
                    return {"ok": bool(response.ok), "sequence": response.graph_commit_sequence}
                if method == "grid":
                    if fields.get("purpose") not in ("diagnostic", "migrate_legacy"):
                        raise DomainError(ErrorCode.CONFLICT, "raw raster is localization-only")
                    if fields.get("purpose") == "migrate_legacy":
                        if getattr(self, "_legacy_raster_used", False):
                            raise DomainError(ErrorCode.CONFLICT, "legacy raster already consumed")
                        self._legacy_raster_used = True
                    cut = await self._navigation.raster(fields["sequence"])
                    return (
                        {"ok": True, **cut} if cut.get("pending") else await self._transfer_cut(cut)
                    )
                if method == "reference":
                    if fields.get("active") is False:
                        self._navigation.withdraw()
                    else:
                        self._navigation.authorize_reference(fields)
                    return {"ok": True}
                if method == "install_fixed":
                    cut = await asyncio.to_thread(
                        _read_fixed_transfer, fields["path"], fields["sha256"]
                    )
                    if fields.get("acceptance_floor_steady_ns") != self._navigation._owner_floor_ns:
                        raise DomainError(ErrorCode.CONFLICT, "fixed reference floor changed")
                    ack = self._navigation.install_fixed(cut, graph_cut=self._navigation._cut)
                    if fields.get("coverage_reason") == "coverage_limit":
                        self._navigation.authorize_reference(
                            {**fields, "sequence": cut.watermark.graph_sequence}
                        )
                    return ack
                if method == "next":
                    response = await self._call(
                        "next", boot_id=self._boot, graph_commit_sequence=fields["sequence"]
                    )
                    if response.status == response.PENDING:
                        return {"ok": True, "pending": True}
                    if response.status != response.OK:
                        raise DomainError(ErrorCode.CONFLICT, "SLAM commit history is unavailable")
                    return await self._publish_cut(response)
                if method == "capture":
                    response = await self._call("capture")
                    if response.status != response.OK or response.boot_id != self._boot:
                        raise DomainError(
                            ErrorCode.OPERATION_UNKNOWN, "SLAM capture was not confirmed"
                        )
                    cut = self._graph(response)
                    cut.update(
                        prefix=str(self._root / response.relative_prefix),
                        pair_digest=response.pair_digest,
                    )
                    return await self._transfer_cut(cut)
                raise DomainError(ErrorCode.VALIDATION, "unknown mapping backend request")
            except DomainError as exc:
                return {"ok": False, "error": exc.code.value}
            except (OSError, ValueError, KeyError, TypeError):
                return {"ok": False, "error": ErrorCode.VALIDATION.value}

    async def _start(self, fields: dict[str, Any]) -> dict[str, Any]:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory

        if self._child is not None:
            raise DomainError(ErrorCode.CONFLICT, "old SLAM must exit before starting another")
        if not self._stopped():
            raise DomainError(ErrorCode.CONFLICT, "vehicle must stop before changing maps")
        root = Path(fields["capture_root"])
        if not root.is_absolute() or root.is_symlink() or root.resolve() != root:
            raise DomainError(ErrorCode.VALIDATION, "capture root must be canonical")
        root.mkdir(mode=0o700, parents=True, exist_ok=True)
        if root.stat().st_uid != os.getuid() or root.stat().st_mode & 0o077:
            raise DomainError(ErrorCode.VALIDATION, "capture root must be private")
        executable = (
            Path(get_package_prefix("senior_mapping")) / "lib/senior_mapping/slam_graph_node"
        )
        config = Path(get_package_share_directory("senior_mapping")) / "config/slam.yaml"
        expected_boot = str(uuid.uuid4())
        parameters = {
            "production_boot_id": expected_boot,
            "production_session": True,
            "capture_directory": str(root),
            "bounded_scan_topic": "/safety/raw/scan",
            "use_map_saver": False,
            "enable_interactive_mode": False,
            "transform_publish_period": 0.0,
            "odom_frame": fields.get("odom_frame", "odom"),
            "use_sim_time": bool(self.node.get_parameter("use_sim_time").value),
            "verified_static_lidar_tf_topic": "/senior/slam/static_lidar_tf_receipt",
            "verified_static_lidar_frame": fields.get("lidar_frame", "laser_link"),
            "verified_static_lidar_relay_node": "/slam_static_lidar_tf_receipt",
            "verified_static_lidar_tf_source_topic": "/tf_static",
            "verified_static_lidar_tf_source_node": "/robot_state_publisher",
        }
        if fields.get("prefix"):
            parameters.update(
                production_graph_prefix=fields["prefix"],
                production_graph_digest=fields["pair_digest"],
                production_seed=fields.get("seed", [0.0, 0.0, 0.0]),
            )
        args = [
            str(executable),
            "--ros-args",
            "--params-file",
            str(config),
            "-r",
            "/map:=/senior/slam/unqualified_map",
            "-r",
            "/map_metadata:=/senior/slam/unqualified_metadata",
        ]
        for key, value in parameters.items():
            args.extend(["-p", f"{key}:={json.dumps(value)}"])
        environment = {**os.environ, "OMP_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1"}
        self._child = await asyncio.create_subprocess_exec(
            *args, env=environment, start_new_session=True
        )
        self._resources.started(self._child.pid)
        relay = (
            Path(get_package_prefix("senior_sensor_ingress"))
            / "lib/senior_sensor_ingress/static_tf_receipt_node"
        )
        relay_parameters = {
            "input_topic": "/tf_static",
            "output_topic": "/senior/slam/static_lidar_tf_receipt",
            "base_frame": "base_link",
            "camera_frame": fields.get("lidar_frame", "laser_link"),
            "expected_node_name": "robot_state_publisher",
            "expected_namespace": "/",
            "use_sim_time": parameters["use_sim_time"],
        }
        relay_args = [str(relay), "--ros-args", "-r", "__node:=slam_static_lidar_tf_receipt"]
        for key, value in relay_parameters.items():
            relay_args.extend(["-p", f"{key}:={json.dumps(value)}"])
        try:
            self._relay = await asyncio.create_subprocess_exec(
                *relay_args, env=environment, start_new_session=True
            )
            self._resources.started(self._relay.pid)
        except BaseException:
            await self._stop()
            raise
        self._root = root
        self._legacy_raster_used = False
        try:
            deadline = asyncio.get_running_loop().time() + 10
            while True:
                graph = await self._call("graph")
                if graph.graph_commit_watermark_valid:
                    break
                if asyncio.get_running_loop().time() >= deadline:
                    raise DomainError(ErrorCode.UNAVAILABLE, "SLAM input did not become ready")
                await asyncio.sleep(0.02)
            if (
                graph.boot_id != expected_boot
                or graph.boot_id == self._boot
                or not graph.graph_commit_watermark_valid
            ):
                raise DomainError(ErrorCode.CONFLICT, "SLAM did not start a fresh owned graph")
            self._boot = graph.boot_id
            self._navigation.begin(self._boot, graph.odom_frame)
            self._start_scan_channel()
            return {
                "ok": True,
                "boot_id": self._boot,
                "odom_frame": graph.odom_frame,
                "map_frame": graph.map_frame,
                "loaded": await self._transfer_cut(self._graph(graph)),
                "qualified_scan_path": str(self._scan_slot.path),
                "qualified_scan_nonce": self._scan_slot.nonce,
                "qualified_scan_schema": 1,
                "host_boot_id": self._host_boot_id,
            }
        except BaseException:
            await self._stop()
            raise

    def _register_qualified_receipt(self) -> None:
        # //Compatible: Humble rclpy has no per-message GID. Reuse the node's
        # original authenticated native receipt group, registered before first spin.
        try:
            from senior_interfaces.msg import QualifiedMappingScan
        except ImportError:
            # //Compatible: older installs keep control available, but cannot start mapping.
            return
        from senior_sensor_ingress.native_receipt import subscribe

        self._scan_subscription = subscribe(
            self.node,
            QualifiedMappingScan,
            "/senior/slam/qualified_scan",
            self._on_qualified_scan,
            writer="slam_toolbox",
            qos="reliable",
            invalidate=self._invalidate_qualified,
        )

    def _invalidate_qualified(self) -> None:
        if self._scan_accepting:
            self._navigation.invalidate_current("qualified_receipt_unavailable")

    def _start_scan_channel(self) -> None:
        from senior_perception.learning_client import LearningClient

        from vehicle_core.config import DeviceProfile

        if self._scan_subscription is None:
            raise DomainError(ErrorCode.UNAVAILABLE, "qualified scan interfaces need a rebuild")
        self._scan_profile = DeviceProfile()
        claim = os.environ.get("SENIOR_STARTUP_LEARNING_CLAIM")
        if claim:
            self._learning_client = LearningClient(Path(claim), read_only=True)
        self._scan_slot = LatestScanSlot(
            Path(self._transfer.name) / "latest-scan.json",
            nonce=str(uuid.uuid4()),
            backend_boot=self._boot,
            host_boot_id=self._host_boot_id,
        )
        self._navigation.slot = self._scan_slot
        self._qualified_gid = None
        self._scan_accepting = True
        self._scan_timer = self.node.create_timer(0.05, self._poll_scan_channel)

    def _qualified_owner(self) -> bytes | None:
        endpoints = self.node.get_publishers_info_by_topic("/senior/slam/qualified_scan")
        if len(endpoints) != 1:
            return None
        owner = endpoints[0]
        if (
            owner.node_name != "slam_toolbox"
            or owner.node_namespace != "/"
            or owner.topic_type != "senior_interfaces/msg/QualifiedMappingScan"
        ):
            return None
        gid = bytes(owner.endpoint_gid)
        if self._qualified_gid is not None and self._qualified_gid != gid:
            return None
        return gid

    def _on_qualified_scan(self, message: Any, info: Any) -> None:
        if not self._scan_accepting or message.backend_boot != self._boot:
            return
        owner = self._qualified_owner()
        if (
            owner is None
            or not isinstance(info, dict)
            or info.get("native_receipt_authenticated") is not True
            or bytes(info.get("publisher_gid", b"")) != owner
        ):
            return
        self._qualified_gid = owner
        if message.status != message.QUALIFIED:
            self._navigation.invalidate_current(
                message.reason or "native_invalid", retired=message.status == message.RETIRED
            )
            return
        try:
            view = None if self._learning_client is None else self._learning_client.view
            scan = decode_qualified_scan(
                message, self._scan_profile, learning_view=view, host_boot_id=self._host_boot_id
            )
            self._navigation.observe_scan(scan, message.scan.capture)
        except ValueError:
            self._navigation.invalidate_current("scan_decode_invalid")

    def _poll_scan_channel(self) -> None:
        if not self._scan_accepting:
            return
        if self._learning_client is not None:
            self._learning_client.poll()
        owner = self._qualified_owner()
        if self._qualified_gid is not None and owner is None:
            self._navigation.invalidate_current("qualified_owner_changed", retired=True)
        self._navigation.check_current()
        if self._scan_slot is not None and self._scan_slot.error is not None:
            if self._navigation.reference is not None:
                self._navigation.reference.set_current_coverage(False, "scan_slot_unavailable")

    async def _stop(self) -> None:
        self._scan_accepting = False
        self._navigation.withdraw()
        # The shared receipt group belongs to the node and is closed by its context;
        # the accepting fence above disables only this mapping session's input.
        if self._scan_timer is not None:
            self.node.destroy_timer(self._scan_timer)
            self._scan_timer = None
        if self._learning_client is not None:
            self._learning_client.close()
            self._learning_client = None
        if self._scan_slot is not None:
            await asyncio.to_thread(self._scan_slot.close)
            self._scan_slot = self._navigation.slot = None
        for child in (self._child, self._relay):
            if child is None:
                continue
            if child.returncode is None:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    await asyncio.wait_for(child.wait(), 5)
                except asyncio.TimeoutError:
                    os.killpg(child.pid, signal.SIGKILL)
                    await child.wait()
            self._resources.exited(child.pid)
        self._child = self._relay = None

    @staticmethod
    def _graph(response: Any) -> dict[str, Any]:
        return {
            "boot_id": response.boot_id,
            "sequence": response.graph_commit_sequence,
            "odom_frame": response.odom_frame,
            "map_frame": response.map_frame,
            "anchors": [{"id": f.id, "x": f.x, "y": f.y, "yaw": f.yaw} for f in response.keyframes],
        }

    async def _publish_cut(self, response: Any) -> dict[str, Any]:
        scan = response.scan
        if not response.static_lidar_tf_receipt_valid or response.raw_ray_angle_sign not in (-1, 1):
            raise DomainError(ErrorCode.UNAVAILABLE, "SLAM scan geometry is unavailable")
        cut = self._graph(response)
        cut.update(
            keyframe_id=response.committed_keyframe_id,
            capture=asdict(capture_from_message(scan.capture)),
            stamp=response.sample_stamp.sec + response.sample_stamp.nanosec * 1e-9,
            sensor=[
                response.corrected_sensor_x,
                response.corrected_sensor_y,
                response.corrected_sensor_yaw,
            ],
            angle_min=scan.angle_min * response.raw_ray_angle_sign,
            angle_increment=scan.angle_increment * response.raw_ray_angle_sign,
            odom=[response.original_odom_x, response.original_odom_y, response.original_odom_yaw],
            range_min=scan.range_min,
            range_max=scan.range_max,
            ranges=[float(v) if math.isfinite(v) else None for v in scan.ranges],
        )
        self._navigation.observe_cut(cut)
        return await self._transfer_cut(cut)

    async def _transfer_cut(self, cut: dict[str, Any]) -> dict[str, Any]:
        path = Path(self._transfer.name) / "cut.json"

        def write() -> str:
            payload = json.dumps(cut, allow_nan=False, separators=(",", ":")).encode()
            if len(payload) > 8 * 1024 * 1024:
                raise DomainError(ErrorCode.UNAVAILABLE, "mapping cut exceeds transfer budget")
            path.write_bytes(payload)
            path.chmod(0o600)
            return hashlib.sha256(payload).hexdigest()

        digest = await asyncio.to_thread(write)
        return {"ok": True, "path": str(path), "sha256": digest}

    async def close(self) -> None:
        await self._stop()
        self._transfer.cleanup()
        self._navigation.close()
        for client, _ in self._services.values():
            self.node.destroy_client(client)


def _read_fixed_transfer(path: str, digest: str):
    path = Path(path)
    if not path.is_absolute() or ".." in path.parts:
        raise DomainError(ErrorCode.VALIDATION, "fixed transfer path must be canonical")
    directory_fd = os.open("/", os.O_RDONLY | os.O_DIRECTORY)
    try:
        # Pin each ancestor inode without following symlinks, including races.
        for part in path.parts[1:-1]:
            parent = os.open(
                part, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory_fd
            )
            os.close(directory_fd)
            directory_fd = parent
        directory = os.fstat(directory_fd)
        if directory.st_uid != os.getuid() or directory.st_mode & 0o077:
            raise DomainError(ErrorCode.VALIDATION, "fixed transfer directory must be private")
        fd = os.open(path.name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory_fd)
    finally:
        os.close(directory_fd)
    with os.fdopen(fd, "rb") as stream:
        metadata = os.fstat(stream.fileno())
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != os.getuid()
            or metadata.st_mode & 0o077
            or metadata.st_size > 8 * 1024 * 1024
        ):
            raise DomainError(ErrorCode.VALIDATION, "fixed transfer must be private and bounded")
        payload = stream.read(8 * 1024 * 1024 + 1)
    if len(payload) > 8 * 1024 * 1024 or hashlib.sha256(payload).hexdigest() != digest:
        raise DomainError(ErrorCode.CONFLICT, "fixed transfer digest changed")
    try:
        return decode_fixed_snapshot(payload, transfer=True)
    except (ValueError, TypeError) as exc:
        raise DomainError(ErrorCode.VALIDATION, "fixed transfer is invalid") from exc
