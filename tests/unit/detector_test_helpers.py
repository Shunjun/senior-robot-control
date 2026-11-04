"""Reusable module-level helpers for spawned inference tests."""

import time


def slow_detector(frame):
    time.sleep(0.15)
    return [("person", 0.9, (0.0, 0.0, 1.0, 1.0))]


def wait_until(predicate, timeout=5.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    raise AssertionError("timed out waiting for inference")
