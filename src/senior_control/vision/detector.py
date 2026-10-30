"""Optional object detector and its bounded, non-blocking inference worker.

No detector runtime is imported at module import time. A missing weight file or
a missing runtime yields a detector that is explicitly disabled; it never
returns an empty list that could be read as "nobody is there".
"""

import importlib
import threading
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Protocol

from senior_control.vision.synchronizer import ImageFrame

InferFn = Callable[[ImageFrame], "list[Detection]"]
RuntimeLoader = Callable[[Path], InferFn]


class DetectorDisabled(RuntimeError):
    """``infer`` was called on a detector that has no model behind it."""


@dataclass(frozen=True)
class Detection:
    """One detection in color-image pixels: ``box`` is (x0, y0, x1, y1)."""

    label: str
    confidence: float
    box: tuple[float, float, float, float]
    stamp: float


class Detector(Protocol):
    enabled: bool
    reason: str

    def infer(self, rgb: ImageFrame) -> list[Detection]: ...


class DisabledDetector:
    enabled = False

    def __init__(self, reason: str) -> None:
        self.reason = reason

    def infer(self, rgb: ImageFrame) -> list[Detection]:
        raise DetectorDisabled(self.reason)


class RuntimeDetector:
    enabled = True
    reason = "ready"

    def __init__(self, infer: InferFn) -> None:
        self._infer = infer

    def infer(self, rgb: ImageFrame) -> list[Detection]:
        return self._infer(rgb)


def load_detector(
    weights_path: Path | None,
    *,
    runtime_loader: RuntimeLoader | None = None,
) -> Detector:
    """Return a ready detector, or a disabled one that states why.

    This function never creates a weight file. ``runtime_loader`` is injectable
    so tests and alternative runtimes need no ultralytics install.
    """
    if weights_path is None:
        return DisabledDetector("weights_not_configured")
    if not weights_path.is_file():
        return DisabledDetector("weights_missing")
    loader = runtime_loader or _ultralytics_runtime
    try:
        return RuntimeDetector(loader(weights_path))
    except ImportError:
        return DisabledDetector("runtime_missing")
    except Exception:
        return DisabledDetector("weights_unloadable")


def _ultralytics_runtime(weights_path: Path) -> InferFn:
    """Ultralytics YOLO runtime. Not exercised by unit tests: it needs the optional
    ``vision`` runtime and a trained weight file, neither of which ships here."""
    ultralytics: Any = importlib.import_module("ultralytics")
    numpy: Any = importlib.import_module("numpy")
    model = ultralytics.YOLO(str(weights_path))

    def infer(rgb: ImageFrame) -> list[Detection]:
        channels = 1 if rgb.encoding == "mono8" else 3
        pixels = numpy.frombuffer(rgb.data, dtype=numpy.uint8).reshape(
            rgb.height, rgb.width, channels
        )
        if rgb.encoding == "rgb8":
            pixels = pixels[:, :, ::-1]  # ultralytics reads numpy arrays as BGR
        result = model.predict(pixels, verbose=False)[0]
        names = result.names
        found: list[Detection] = []
        for box, confidence, klass in zip(
            result.boxes.xyxy.tolist(),
            result.boxes.conf.tolist(),
            result.boxes.cls.tolist(),
            strict=True,
        ):
            x0, y0, x1, y1 = (float(value) for value in box)
            found.append(
                Detection(str(names[int(klass)]), float(confidence), (x0, y0, x1, y1), rgb.stamp)
            )
        return found

    return infer


@dataclass(frozen=True)
class DetectionBatch:
    stamp: float
    detections: tuple[Detection, ...]


class InferenceWorker:
    """Runs a detector on its own thread behind a bounded, drop-oldest queue.

    ``submit`` and ``latest`` never wait on inference, so geometry keeps its own
    rate when the detector is slow, failing, or disabled.
    """

    def __init__(self, detector: Detector, *, max_pending: int = 1) -> None:
        if max_pending <= 0:
            raise ValueError("max_pending must be positive")
        self._detector = detector
        self._pending: deque[ImageFrame] = deque()
        self._max_pending = max_pending
        self._condition = threading.Condition()
        self._busy = False
        self._closed = False
        self._thread: threading.Thread | None = None
        self._latest: DetectionBatch | None = None
        self.dropped = 0
        self.failures = 0

    @property
    def detector_status(self) -> tuple[str, str]:
        return ("ready" if self._detector.enabled else "disabled", self._detector.reason)

    def submit(self, frame: ImageFrame) -> bool:
        """Queue a frame for inference. False means it was not accepted at all."""
        if not self._detector.enabled:
            return False
        with self._condition:
            if self._closed:
                return False
            if len(self._pending) >= self._max_pending:
                self._pending.popleft()
                self.dropped += 1
            self._pending.append(frame)
            if self._thread is None:
                self._thread = threading.Thread(
                    target=self._run, name="detector-inference", daemon=True
                )
                self._thread.start()
            self._condition.notify_all()
        return True

    def latest(self) -> DetectionBatch | None:
        with self._condition:
            return self._latest

    def drain(self, timeout_s: float) -> bool:
        with self._condition:
            return self._condition.wait_for(
                lambda: not self._pending and not self._busy, timeout=timeout_s
            )

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._pending.clear()
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=1.0)

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: bool(self._pending) or self._closed)
                if self._closed:
                    return
                frame = self._pending.popleft()
                self._busy = True
            try:
                found = self._detector.infer(frame)
                batch = DetectionBatch(
                    frame.stamp, tuple(replace(item, stamp=frame.stamp) for item in found)
                )
            except Exception:
                batch = None
            with self._condition:
                self._busy = False
                if batch is None:
                    self.failures += 1
                else:
                    self._latest = batch
                self._condition.notify_all()
