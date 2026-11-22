"""Shared immutable image value; no media, sensor policy or diagnostic imports."""

from dataclasses import dataclass


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
