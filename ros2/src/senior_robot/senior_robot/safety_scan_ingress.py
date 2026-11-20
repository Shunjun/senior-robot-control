"""//Compatible: historical path forwards the same perception implementation."""

from senior_perception.safety_scan_ingress import SafetyScanIngress as SafetyScanIngress

from senior_robot.motion_lifecycle import run


def main():
    """Assemble the shared functional node with the existing bounded ROS lifecycle."""
    run(SafetyScanIngress, cli=True)
