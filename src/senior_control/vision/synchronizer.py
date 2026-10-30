"""Image and depth frames, plus the RGB/depth time and size pairing rule."""

import math
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass


class FrameSyncError(ValueError):
    """An RGB/depth pair that must not be fused."""


@dataclass(frozen=True)
class ImageFrame:
    stamp: float
    width: int
    height: int
    encoding: str
    data: bytes

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("image size must be positive")
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(self.encoding)
        if channels is not None and len(self.data) != self.width * self.height * channels:
            raise ValueError("image data does not match size and encoding")


@dataclass(frozen=True)
class DepthFrame:
    """Row-major raw depth. The unit lives in the calibration, not in the frame."""

    stamp: float
    width: int
    height: int
    values: Sequence[float]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("depth size must be positive")
        if len(self.values) != self.width * self.height:
            raise ValueError("depth data does not match size")


@dataclass(frozen=True)
class AlignedRgbdFrame:
    rgb: ImageFrame
    depth: DepthFrame
    skew_s: float

    @property
    def stamp(self) -> float:
        """Geometry is stamped by the depth frame it was measured from."""
        return self.depth.stamp


def align(rgb: ImageFrame, depth: DepthFrame, *, max_skew_s: float) -> AlignedRgbdFrame:
    if not math.isfinite(max_skew_s) or max_skew_s < 0.0:
        raise ValueError("max_skew_s must be finite and non-negative")
    if not math.isfinite(rgb.stamp) or not math.isfinite(depth.stamp):
        raise FrameSyncError("frame stamp is not finite")
    if (rgb.width, rgb.height) != (depth.width, depth.height):
        raise FrameSyncError("rgb and depth size differ; depth is not registered to color")
    skew = abs(rgb.stamp - depth.stamp)
    if skew > max_skew_s:
        raise FrameSyncError(f"rgb/depth skew {skew:.3f}s exceeds {max_skew_s:.3f}s")
    return AlignedRgbdFrame(rgb, depth, skew)


class RgbdSynchronizer:
    """Pairs the nearest frames inside the skew bound. Buffers are bounded."""

    def __init__(self, *, max_skew_s: float, capacity: int = 4) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        if not math.isfinite(max_skew_s) or max_skew_s < 0.0:
            raise ValueError("max_skew_s must be finite and non-negative")
        self._max_skew_s = max_skew_s
        self._rgb: deque[ImageFrame] = deque(maxlen=capacity)
        self._depth: deque[DepthFrame] = deque(maxlen=capacity)
        self.rejected = 0

    def offer_rgb(self, frame: ImageFrame) -> AlignedRgbdFrame | None:
        if self._depth:
            nearest = min(self._depth, key=lambda other: abs(other.stamp - frame.stamp))
            pair = self._try(frame, nearest)
            if pair is not None:
                self._consume(pair)
                return pair
        self._rgb.append(frame)
        return None

    def offer_depth(self, frame: DepthFrame) -> AlignedRgbdFrame | None:
        if self._rgb:
            nearest = min(self._rgb, key=lambda other: abs(other.stamp - frame.stamp))
            pair = self._try(nearest, frame)
            if pair is not None:
                self._consume(pair)
                return pair
        self._depth.append(frame)
        return None

    def _try(self, rgb: ImageFrame, depth: DepthFrame) -> AlignedRgbdFrame | None:
        try:
            return align(rgb, depth, max_skew_s=self._max_skew_s)
        except FrameSyncError:
            self.rejected += 1
            return None

    def _consume(self, pair: AlignedRgbdFrame) -> None:
        """Frames at or before the matched stamps can no longer pair with newer data."""
        self._rgb = deque((f for f in self._rgb if f.stamp > pair.rgb.stamp), self._rgb.maxlen)
        self._depth = deque(
            (f for f in self._depth if f.stamp > pair.depth.stamp), self._depth.maxlen
        )
