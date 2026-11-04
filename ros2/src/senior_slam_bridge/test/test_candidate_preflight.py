"""Local regressions for preflight proof and exact subprocess cleanup."""

import io
import json
import math
import os
import signal
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from types import SimpleNamespace

from candidate_preflight import (
    PreflightError,
    acquire_receipt,
    parse_args,
    read_stationary_scan,
    run_candidate,
    validate_proof,
    validate_stationary_diagnostic,
    validate_stationary_hypotheses,
)

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

    def test_scan_json_checks_layout_and_local_file_bounds(self):
        scan = {
            "seed_x": 0.08,
            "seed_y": -0.04,
            "seed_yaw": 0.06,
            "first_angle": -math.pi,
            "angle_step": 0.05,
            "ranges": [2.0] * 32 + [None],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scan.json"
            path.write_text(json.dumps(scan))
            self.assertEqual(read_stationary_scan(path)["ranges"], [2.0] * 32 + [math.inf])
            link = path.with_name("linked-scan.json")
            link.symlink_to(path)
            with self.assertRaisesRegex(PreflightError, "Invalid stationary scan file"):
                read_stationary_scan(link)
            for invalid in (
                json.dumps({**scan, "seed_yaw": True}),
                json.dumps({**scan, "ranges": [2.0] * 31 + [None]}),
                json.dumps({**scan, "ranges": [2.0] * 2049}),
                json.dumps({**scan, "angle_step": 0.5}),
                json.dumps({**scan, "ranges": [2.0] * 32 + [float("inf")]}),
                json.dumps(scan)[:-1] + ', "seed_x": 0}',
                " " * (64 * 1024 + 1),
            ):
                with self.subTest(example=invalid[:80]):
                    path.write_text(invalid)
                    with self.assertRaisesRegex(PreflightError, "Invalid stationary scan"):
                        read_stationary_scan(path)
            path.unlink()
            os.mkfifo(path)
            with self.assertRaisesRegex(PreflightError, "regular scan file"):
                read_stationary_scan(path)

    def test_stationary_result_requires_complete_finite_values_and_known_refs(self):
        proof = {"keyframes": [{"id": 2}, {"id": 4}]}
        values = dict(
            UNAVAILABLE=0,
            INVALID=1,
            COMPLETE=2,
            SPENT=3,
            status=2,
            primary_result_valid=True,
            second_result_valid=False,
            response=0.0,
            fitted_x=0.08,
            fitted_y=-0.04,
            fitted_yaw=0.06,
            reference_ids=[4, 2],
        )
        result = validate_stationary_diagnostic(SimpleNamespace(**values), proof)
        self.assertEqual(result["response"], 0.0)
        self.assertEqual(result["reference_ids"], [4, 2])
        for patch in (
            {"status": 3},
            {"response": float("nan")},
            {"reference_ids": [2, 5]},
            {"reference_ids": [2, 2]},
        ):
            with self.subTest(patch=patch), self.assertRaises(PreflightError):
                validate_stationary_diagnostic(SimpleNamespace(**(values | patch)), proof)

    def test_comparison_preserves_order_and_reports_one_unavailable_hypothesis(self):
        proof = {"keyframes": [{"id": 4}]}
        primary, second = (0.0, 0.0, 0.0), (20.0, 0.0, 0.0)
        values = dict(
            COMPLETE=2,
            status=2,
            primary_result_valid=False,
            response=0.0,
            fitted_x=0.0,
            fitted_y=0.0,
            fitted_yaw=0.0,
            reference_ids=[],
            second_result_valid=True,
            second_response=0.0,
            second_fitted_x=20.0,
            second_fitted_y=0.0,
            second_fitted_yaw=0.0,
            second_reference_ids=[4],
        )
        result = validate_stationary_hypotheses(SimpleNamespace(**values), proof, primary, second)
        self.assertEqual([item["status"] for item in result], ["unavailable", "complete"])
        self.assertEqual([item["seed_sensor_pose"]["x"] for item in result], [0.0, 20.0])
        self.assertNotIn("response", result[0])
        self.assertEqual(result[1]["response"], 0.0)
        self.assertEqual(result[1]["reference_ids"], [4])
        for patch in ({"second_result_valid": False}, {"reference_ids": [4]}):
            with self.subTest(patch=patch), self.assertRaises(PreflightError):
                validate_stationary_hypotheses(
                    SimpleNamespace(**(values | patch)), proof, primary, second
                )

    def test_comparison_requires_a_scan_and_finite_seed(self):
        base = ["/private/map", DIGEST]
        file_scan = ["--stationary-scan", "/private/scan.json"]
        flag = ["--compare-seed", "20", "0", "0"]
        _, args = parse_args(base + file_scan + flag)
        self.assertEqual(args.compare_seed, [20.0, 0.0, 0.0])
        for options in (flag, [*file_scan, "--compare-seed", "nan", "0", "0"]):
            with self.subTest(options=options), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    parse_args(base + options)
                self.assertEqual(result.exception.code, 2)

    def test_live_mode_requires_explicit_domain_clock_mount_and_seed(self):
        base = ["/private/map", DIGEST]
        valid = [
            "--live-scan",
            "--live-domain",
            "5",
            "--live-clock",
            "sim",
            "--live-lidar-frame",
            "laser_link",
            "--live-seed",
            "0",
            "0",
            "0",
            "--live-upright",
        ]
        _, parsed = parse_args(base + valid)
        self.assertEqual(
            (parsed.live_raw_topic, parsed.live_bounded_topic), ("/scan", "/safety/raw/scan")
        )
        for options in (
            valid[:-1],
            valid[1:],
            [*valid, "--stationary-scan", "/private/scan.json"],
            [*valid, "--live-domain", "233"],
            [*valid, "--live-seed", "nan", "0", "0"],
            [*valid, "--live-raw-topic", "/safety/raw/scan"],
        ):
            with self.subTest(options=options), redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as result:
                    parse_args(base + options)
                self.assertEqual(result.exception.code, 2)

    def test_live_acquisition_bounds_stdout_and_reaps(self):
        command = [sys.executable, "-c", "import sys; sys.stdout.buffer.write(b'receipt')"]
        payload, started = acquire_receipt(command, os.environ, lambda: None)
        self.assertEqual(payload, b"receipt")
        self.assertLessEqual(started, time.monotonic())

        with tempfile.TemporaryDirectory() as directory:
            pid_path = Path(directory) / "pid"
            setup = (
                "import os,pathlib,sys,time;pathlib.Path(sys.argv[1]).write_text(str(os.getpid()));"
            )
            for tail, timeout, error in (
                ("os.write(1,b'x'*65536);time.sleep(30)", 2.0, "byte limit"),
                ("time.sleep(30)", 0.4, "timed out"),
            ):
                with self.subTest(error=error):
                    with self.assertRaisesRegex(PreflightError, error):
                        acquire_receipt(
                            [sys.executable, "-c", setup + tail, str(pid_path)],
                            os.environ,
                            lambda: None,
                            timeout=timeout,
                        )
                    pid = int(pid_path.read_text())
                    with self.assertRaises(ProcessLookupError):
                        os.kill(pid, 0)
                    pid_path.unlink()

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
