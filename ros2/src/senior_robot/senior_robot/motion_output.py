"""Private nonblocking actuator channel; the native transport owns ROS publication."""

import select
import socket
import struct
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_prefix


class MotionOutput:
    def __init__(self, output_topic="/cmd_vel"):
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        executable = (
            Path(get_package_prefix("senior_motion_msgs"))
            / "lib/senior_motion_msgs/motion_output_guard"
        )
        try:
            self.process = subprocess.Popen(
                [str(executable), str(child.fileno()), output_topic],
                pass_fds=(child.fileno(),),
                close_fds=True,
            )
        except BaseException:
            parent.close()
            raise
        finally:
            child.close()
        parent.setblocking(False)
        self.channel, self.sequence = parent, 0
        self._closed = False

    def publish(self, velocity, deadline, *, computed_steady):
        if self.process.poll() is not None:
            raise ValueError("motion_output_transport_exited")
        self.sequence += 1
        frame = struct.pack(
            "=ddddQ", velocity.vx, velocity.wz, deadline, computed_steady, self.sequence
        )
        try:
            if self.channel.send(frame) != len(frame):
                raise OSError("partial output frame")
        except OSError as exc:
            self.channel.close()  # Native EOF revokes output; no hidden retry queue.
            raise ValueError("motion_output_transport_lost") from exc

    def close(self):
        if self._closed:
            return
        self._closed = True
        # EOF makes the native transport issue zero before this orderly shutdown.
        try:
            self.channel.shutdown(socket.SHUT_WR)
            if select.select([self.channel], [], [], 0.2)[0]:
                self.channel.recv(1)
        except OSError:
            pass
        self.channel.close()
        self.process.terminate()
        try:
            self.process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=1.0)
