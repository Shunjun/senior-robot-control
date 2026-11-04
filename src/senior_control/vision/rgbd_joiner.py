"""Small, bounded exact-stamp join for provisional native camera messages.

Caller supplies only graph-observed bridge identity; the native egress receipt
must independently authenticate the actual MessageInfo and original bodies.
"""

import sys
from array import array
from dataclasses import dataclass
from numbers import Integral, Real
from types import SimpleNamespace
from typing import Any

from senior_control.vision.independent_motion import _finite, _immutable_header
from senior_control.vision.raw_capture import RawCaptureHeader, capture_from_message


@dataclass(frozen=True)
class JoinResult:
    kind: str
    headers: tuple[RawCaptureHeader, ...] = ()
    bodies: tuple[Any, ...] = ()
    bridges: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class _ImageBody:
    capture: RawCaptureHeader
    width: int
    height: int
    step: int
    encoding: str
    is_bigendian: bool
    data: bytes


@dataclass(frozen=True, slots=True)
class _InfoBody:
    capture: RawCaptureHeader
    width: int
    height: int
    distortion_model: str
    d: tuple[float, ...]
    k: tuple[float, ...]
    r: tuple[float, ...]
    p: tuple[float, ...]
    binning_x: int
    binning_y: int
    roi: tuple[int, ...]
    roi_do_rectify: bool


def _numeric(value: Any) -> bool:
    kind = type(value)
    numpy = sys.modules.get("numpy")
    return kind in (int, float) or (
        numpy is not None and kind in numpy.sctypeDict.values() and numpy.dtype(kind).itemsize <= 8
    )


def _integer(value: Any) -> int:
    if not _numeric(value) or not isinstance(value, Integral):
        raise ValueError("invalid_integer")
    result = int(value)
    if not 0 <= result <= 0xFFFFFFFF:
        raise ValueError("invalid_integer")
    return result


def _vector(value: Any, *, length: int, real: bool, variable: bool = False) -> tuple[Any, ...]:
    kind = type(value)
    numpy = sys.modules.get("numpy")
    if not (
        kind in (tuple, list, array)
        or (numpy is not None and kind is numpy.ndarray and value.ndim == 1)
    ):
        raise ValueError("invalid_vector")
    if not (0 < len(value) <= length if variable else len(value) == length):
        raise ValueError("invalid_vector")
    if real:
        if any(not _numeric(item) or not isinstance(item, Real) for item in value):
            raise ValueError("invalid_vector")
        return tuple(float(item) for item in value)
    return tuple(_integer(item) for item in value)


def _snapshot(stream: int, header: RawCaptureHeader, body: Any) -> _ImageBody | _InfoBody:
    body_type = type(body)
    expected = "BoundedRawImage" if stream < 2 else "BoundedRawCameraInfo"
    module = "_bounded_raw_image" if stream < 2 else "_bounded_raw_camera_info"
    ros_module = sys.modules.get(f"senior_motion_msgs.msg.{module}")
    if body_type is not SimpleNamespace and not (
        ros_module is not None and body_type is getattr(ros_module, expected, None)
    ):
        raise ValueError("unsupported_body")
    capture = body.capture
    capture_type = type(capture)
    if capture_type is RawCaptureHeader:
        original = capture
    elif (
        capture_module := sys.modules.get("senior_motion_msgs.msg._raw_capture_header")
    ) is not None and capture_type is getattr(capture_module, "RawCaptureHeader", None):
        original = capture_from_message(capture)
    else:
        raise ValueError("unsupported_capture")
    if original != header:
        raise ValueError("capture_body_mismatch")
    width, height = _integer(body.width), _integer(body.height)
    if not 0 < width <= 640 or not 0 < height <= 480:
        raise ValueError("image_size")
    if stream < 2:
        step = _integer(body.step)
        if (
            type(body.encoding) is not str
            or body.encoding != ("rgb8" if stream == 0 else "32FC1")
            or type(body.is_bigendian) is not bool
            or step < width * (3 if body.encoding == "rgb8" else 4)
            or step * height > 16_777_216
        ):
            raise ValueError("image_layout")
        data = body.data
        if type(data) not in (bytes, bytearray, memoryview, array):
            raise ValueError("invalid_data")
        if type(data) is array and data.typecode != "B":
            raise ValueError("invalid_data")
        view = memoryview(data)
        if (
            view.ndim != 1
            or not view.c_contiguous
            or view.format != "B"
            or view.nbytes != step * height
        ):
            raise ValueError("invalid_data")
        return _ImageBody(
            header, width, height, step, body.encoding, body.is_bigendian, bytes(view)
        )
    if type(body.distortion_model) is not str or len(body.distortion_model.encode("utf-8")) > 32:
        raise ValueError("invalid_camera_info")
    if type(body.roi_do_rectify) is not bool:
        raise ValueError("invalid_camera_info")
    return _InfoBody(
        header,
        width,
        height,
        body.distortion_model,
        _vector(body.d, length=16, real=True, variable=True),
        _vector(body.k, length=9, real=True),
        _vector(body.r, length=9, real=True),
        _vector(body.p, length=12, real=True),
        _integer(body.binning_x),
        _integer(body.binning_y),
        _vector(body.roi, length=4, real=False),
        body.roi_do_rectify,
    )


class CaptureJoiner:
    def __init__(self, *, capacity: int) -> None:
        if type(capacity) is not int or not 1 <= capacity <= 8:
            raise ValueError("invalid_join_capacity")
        self.capacity = capacity
        self.generation = 1
        self._boot = ""
        self._raw_generation = 0
        self._retired_boots: set[str] = set()
        self._bridge_gids: list[str] = ["", "", "", ""]
        self._identities: list[tuple[str, str, str] | None] = [None, None, None, None]
        self._sequences = [0, 0, 0, 0]
        self._times: list[tuple[int, int, float] | None] = [None, None, None, None]
        self._time_sources: list[tuple[str, str] | None] = [None, None, None, None]
        self._pending: dict[int, dict[int, tuple[RawCaptureHeader, Any, str]]] = {}
        self._faulted = False
        self._closed = False

    def invalidate(self) -> None:
        self.generation += 1
        self._pending.clear()
        self._faulted = True

    def offer(
        self, stream: int, header: RawCaptureHeader, body: Any, *, bridge_gid: str
    ) -> JoinResult:
        if self._closed or type(stream) is not int or not 0 <= stream < 4:
            self.invalidate()
            return JoinResult("invalid")
        if not _immutable_header(header):
            self.invalidate()
            return JoinResult("invalid")
        if header.bridge_gid != bridge_gid:
            return JoinResult("ignored")
        if header.bridge_boot_id in self._retired_boots:
            return JoinResult("ignored")
        if not header.bridge_boot_id or header.capture_generation <= 0 or header.sequence <= 0:
            self.invalidate()
            return JoinResult("invalid")
        if self._boot and self._boot != header.bridge_boot_id:
            prior_gid = self._bridge_gids[stream]
            if not prior_gid or prior_gid == bridge_gid or len(self._retired_boots) >= 64:
                self.invalidate()
                self._closed = True
                return JoinResult("invalid")
            self._retired_boots.add(self._boot)
            self._raw_generation = 0
            self._bridge_gids = ["", "", "", ""]
            self._identities = [None, None, None, None]
            self._sequences = [0, 0, 0, 0]
            # A new bridge boot does not reset the raw sensor's clock. Keep
            # each raw source's frontier until that source identity changes.
            self.invalidate()
        if header.capture_generation < self._raw_generation:
            return JoinResult("ignored")
        generation_changed = header.capture_generation > self._raw_generation
        if generation_changed:
            if self._raw_generation:
                self.invalidate()
            self._raw_generation = header.capture_generation
            self._identities = [None, None, None, None]
            self._sequences = [0, 0, 0, 0]
        self._boot = header.bridge_boot_id
        if self._bridge_gids[stream] and self._bridge_gids[stream] != bridge_gid:
            self.invalidate()
            self._closed = True
            return JoinResult("invalid")
        self._bridge_gids[stream] = bridge_gid
        if header.sequence <= self._sequences[stream]:
            return JoinResult("ignored")
        self._sequences[stream] = header.sequence
        if not header.valid:
            if not self._faulted:
                self.invalidate()
                return JoinResult("invalid")
            return JoinResult("invalid" if generation_changed else "ignored")
        identity = (header.raw_gid, header.source_epoch, header.frame_id)
        if not all(identity) or header.stamp_ns <= 0:
            self.invalidate()
            return JoinResult("invalid")
        if self._identities[stream] is not None and self._identities[stream] != identity:
            self.invalidate()
            self._closed = True
            return JoinResult("invalid")
        source = (header.raw_gid, header.source_epoch)
        previous = self._times[stream] if self._time_sources[stream] == source else None
        if (
            header.received_ros_ns <= 0
            or not _finite(header.received_steady)
            or header.received_steady <= 0
            or (
                previous is not None
                and (
                    header.stamp_ns <= previous[0]
                    or header.received_ros_ns < previous[1]
                    or header.received_steady < previous[2]
                )
            )
        ):
            self.invalidate()
            return JoinResult("invalid")
        self._identities[stream] = identity
        self._times[stream] = (header.stamp_ns, header.received_ros_ns, header.received_steady)
        self._time_sources[stream] = source
        try:
            snapshot = _snapshot(stream, header, body)
        except (AttributeError, TypeError, ValueError, OverflowError, UnicodeError):
            self.invalidate()
            return JoinResult("invalid")
        bucket = self._pending.setdefault(header.stamp_ns, {})
        bucket[stream] = (header, snapshot, bridge_gid)
        while len(self._pending) > self.capacity:
            del self._pending[min(self._pending)]
        if len(bucket) != 4 or header.stamp_ns not in self._pending:
            return JoinResult("invalid" if generation_changed and self._faulted else "pending")
        entries = tuple(bucket[i] for i in range(4))
        for stamp in tuple(self._pending):
            if stamp <= header.stamp_ns:
                del self._pending[stamp]
        self._faulted = False
        return JoinResult(
            "ready",
            tuple(item[0] for item in entries),
            tuple(item[1] for item in entries),
            tuple(item[2] for item in entries),
        )
