"""Real native sender/private seqpacket -> production Python decoder, no ROS needed."""

import importlib.util
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location(
    "frame_codec", ROOT / "ros2/src/senior_robot/senior_robot/final_command_frame.py"
)
codec = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(codec)


class NativeRoundtrip(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.temp.name) / "sender"
        compiler = os.environ.get("CXX") or shutil.which("c++")
        if not compiler:
            raise unittest.SkipTest("native C++ compiler unavailable")
        source = Path(__file__).with_name("final_command_frame_sender.cpp")
        subprocess.run(
            [
                compiler,
                "-std=c++17",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(source.parent.parent / "src"),
                str(source),
                "-o",
                str(cls.binary),
            ],
            check=True,
            capture_output=True,
        )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def send(self, kind):
        parent, child = socket.socketpair(
            socket.AF_UNIX, socket.SOCK_SEQPACKET if sys.platform == "linux" else socket.SOCK_DGRAM
        )
        parent.settimeout(3)
        try:
            result = subprocess.run(
                [str(self.binary), str(child.fileno()), kind],
                pass_fds=(child.fileno(),),
                timeout=3,
                check=False,
            )
            if result.returncode == 42:
                return None
            self.assertEqual(result.returncode, 0)
            return parent.recv(codec.LAYOUT.size + 1)
        finally:
            parent.close()
            child.close()

    def test_intent_exact_original_times_and_source_epoch(self):
        kind, gid, m = codec.decode_frame(self.send("intent"))
        self.assertEqual(
            (kind, gid[0], m.source_kind, m.source_id, m.source_epoch),
            (1, 17, "manual", "authority", 42),
        )
        self.assertEqual(
            (m.received_steady, m.steady_deadline, m.vx, m.wz), (10.25, 10.55, 0.4, 0.2)
        )
        self.assertEqual((m.generation, m.sequence, m.gesture), (5, 9, 1))

    def test_token_retirement_keeps_identity_not_a_stationary_claim(self):
        _, _, m = codec.decode_frame(self.send("release"))
        self.assertEqual(
            (m.request_id, m.gesture, m.sequence, m.close_reason), ("token", 0, 0, "release")
        )
        self.assertEqual((m.source_id, m.source_epoch, m.received_steady), ("authority", 42, 10.25))

    def test_zero_revision_reset_and_qualification_close_are_restricted_events(self):
        _, _, m = codec.decode_frame(self.send("reset"))
        self.assertEqual(
            (m.close_reason, m.gesture, m.source_kind, m.source_id, m.source_epoch),
            ("estop_reset", 0, "idle", "", 0),
        )
        self.assertEqual(
            codec.decode_frame(self.send("qualification_lost"))[2].close_reason,
            "qualification_lost",
        )

    def test_string_boundary_and_native_overflow_rejection(self):
        self.assertEqual(len(codec.decode_frame(self.send("boundary"))[2].source_id), 128)
        self.assertIsNone(self.send("oversize"))

    def test_truncation_wrong_version_and_legacy_do_not_default_fields(self):
        packet = self.send("intent")
        for invalid in [packet[:-1], packet + b"x", packet[:4] + b"\x01" + packet[5:], bytes(597)]:
            with self.assertRaises(ValueError):
                codec.decode_frame(invalid)


if __name__ == "__main__":
    unittest.main()
