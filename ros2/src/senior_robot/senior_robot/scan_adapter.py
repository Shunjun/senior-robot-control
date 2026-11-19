"""//Compatible: historical path forwards the same perception implementation."""

from senior_perception.scan_adapter import ScanAdapter as ScanAdapter

from senior_robot.motion_lifecycle import run


def main():
    """Assemble the shared functional node with the existing bounded ROS lifecycle."""
    run(ScanAdapter, cli=True)
