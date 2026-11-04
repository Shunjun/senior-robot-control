"""Local regressions for preflight proof and exact subprocess cleanup."""

import signal
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from candidate_preflight import PreflightError, run_candidate, validate_proof

BOOT = "d417ec41-c421-4d6d-a997-1af64c263fab"
DIGEST = "a" * 64


class PreflightTests(unittest.TestCase):
    def test_requires_matching_receipt_and_complete_graph(self):
        receipt = SimpleNamespace(boot_id=BOOT, attempt_id="attempt", pair_digest=DIGEST)
        graph = SimpleNamespace(
            OK=0,
            status=0,
            boot_id=BOOT,
            map_frame="map",
            odom_frame="odom",
            snapshot_sequence=1,
            sample_stamp=SimpleNamespace(sec=1, nanosec=2),
            keyframes=[SimpleNamespace(id=0, x=0.0, y=1.0, yaw=0.2)],
        )
        args = (receipt, graph, "attempt", DIGEST, "map", "odom")
        self.assertEqual(validate_proof(*args)["keyframes"][0]["yaw"], 0.2)
        for obj, key, value in (
            (receipt, "attempt_id", "old"),
            (receipt, "pair_digest", "b" * 64),
            (graph, "boot_id", "other"),
            (graph, "map_frame", "unexpected"),
            (graph, "keyframes", []),
            (graph.keyframes[0], "yaw", float("nan")),
        ):
            with self.subTest(key=key):
                previous = getattr(obj, key)
                setattr(obj, key, value)
                try:
                    with self.assertRaises(PreflightError):
                        validate_proof(*args)
                finally:
                    setattr(obj, key, previous)

    def test_cancel_escalates_and_reaps_exact_child(self):
        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory) / "ready"
            code = (
                "import pathlib,signal,sys,time;"
                "signal.signal(signal.SIGTERM,signal.SIG_IGN);"
                "pathlib.Path(sys.argv[1]).touch();"
                "time.sleep(30)"
            )
            children = []
            canceled = False

            def observe(process, check):
                nonlocal canceled
                children.append(process)
                deadline = time.monotonic() + 2
                while not ready.exists() and time.monotonic() < deadline:
                    check()
                    time.sleep(0.01)
                self.assertTrue(ready.exists())
                canceled = True
                check()
                self.fail("Cancellation was ignored")

            with self.assertRaisesRegex(PreflightError, "canceled"):
                run_candidate(
                    [sys.executable, "-c", code, str(ready)],
                    observe,
                    lambda: canceled,
                    startup_seconds=3,
                    cleanup_seconds=0.3,
                )
            self.assertEqual(children[0].returncode, -signal.SIGKILL)

    def test_timed_out_observation_reaps_the_candidate(self):
        children = []

        def observe(process, check):
            children.append(process)
            while True:
                check()
                time.sleep(0.01)

        with self.assertRaisesRegex(PreflightError, "timed out"):
            run_candidate(
                [sys.executable, "-c", "import time; time.sleep(30)"],
                observe,
                lambda: False,
                startup_seconds=0.15,
                cleanup_seconds=0.3,
            )
        self.assertIsNotNone(children[0].returncode)

    def test_child_death_during_observation_cannot_return_success(self):
        def observe(process, _check):
            process.wait(timeout=2)
            return {"proof": True}

        with self.assertRaisesRegex(PreflightError, "exited"):
            run_candidate(
                [sys.executable, "-c", "pass"],
                observe,
                lambda: False,
                startup_seconds=3,
                cleanup_seconds=0.3,
            )


if __name__ == "__main__":
    unittest.main()
