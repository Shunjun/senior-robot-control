"""Dedicated reporting thread; ROS control/reception executors never emit logs."""

import threading
import time

from vehicle_core.diagnostics import RuntimeDiagnostics


class DiagnosticPump:
    def __init__(self, diagnostics: RuntimeDiagnostics) -> None:
        self.stop = threading.Event()
        self.thread = threading.Thread(
            target=self._run,
            args=(diagnostics,),
            name="motion-diagnostics",
            daemon=True,
        )
        self.thread.start()

    def _run(self, diagnostics: RuntimeDiagnostics) -> None:
        while not self.stop.wait(0.05):
            diagnostics.pump(time.monotonic())

    def close(self) -> None:
        self.stop.set()
        self.thread.join(timeout=1.0)
