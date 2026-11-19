"""Private nonblocking actuator channel; the native transport owns ROS publication."""

import fcntl
import gc
import json
import os
import select
import socket
import stat
import struct
import subprocess
import threading
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix


class MotionOutput:
    def __init__(self, output_topic="/cmd_vel"):
        parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self._init_trace()
        executable = (
            Path(get_package_prefix("senior_motion_msgs"))
            / "lib/senior_motion_msgs/motion_output_guard"
        )
        try:
            self.process = subprocess.Popen(
                [str(executable), str(child.fileno()), output_topic],
                pass_fds=(child.fileno(),) + ((self._trace_fd,) if self._trace_fd >= 0 else ()),
                close_fds=True,
                env={**os.environ, "SENIOR_MOTION_TRACE_FD": str(self._trace_fd)},
            )
        except BaseException:
            parent.close()
            self.finish_trace("spawn_failed")
            raise
        finally:
            child.close()
        parent.setblocking(False)
        self.channel, self.sequence = parent, 0
        self._closed = False

    def _init_trace(self):
        # Temporary same-host diagnostics only; never change the original stderr flags.
        self._trace_fd, self._trace_lost, self._trace_rows = -1, 0, 0
        self._trace_marks, self._trace_gc = [], []
        self._trace_send = None
        self._trace_sent_clock = None
        self._trace_targets = []
        self._trace_last_target = None
        self._trace_state = "disabled"
        self._trace_id = os.environ.get("SENIOR_MOTION_TRACE_ID", "")
        if not self._trace_id:
            return
        fd = -1
        try:
            if len(self._trace_id) > 64 or not all(
                c.isascii() and (c.isalnum() or c in "-_") for c in self._trace_id
            ):
                raise ValueError("invalid trace id")
            self._trace_start = int(os.environ["SENIOR_MOTION_TRACE_START_NS"])
            self._trace_end = int(os.environ["SENIOR_MOTION_TRACE_END_NS"])
            if (
                self._trace_start < 0
                or not 0 < self._trace_end - self._trace_start <= 30_000_000_000
            ):
                raise ValueError("trace window must be positive and at most 30s")
            fd = os.open("/proc/self/fd/2", os.O_WRONLY | os.O_NONBLOCK | os.O_CLOEXEC)
            if not stat.S_ISFIFO(os.fstat(fd).st_mode):
                raise ValueError("stderr is not a FIFO")
            if not fcntl.fcntl(fd, fcntl.F_GETFL) & os.O_NONBLOCK:
                raise ValueError("trace fd is blocking")
            self._trace_size = min(4096, os.fpathconf(fd, "PC_PIPE_BUF"))
            self._trace_fd, fd = fd, -1
            self._trace_state = "armed"
            # Installed only while this finite window is still live; ordinary GC is unchanged.
            self.trace_active()
        except Exception as exc:
            self._trace_state = "unavailable:" + type(exc).__name__ + ":" + str(exc)[:160]
        finally:
            if fd >= 0:
                try:
                    os.close(fd)
                except OSError:
                    pass

    @staticmethod
    def trace_clock():
        return (time.monotonic_ns(), time.thread_time_ns(), time.process_time_ns())

    def trace_active(self):
        if getattr(self, "_trace_fd", -1) < 0:
            return False
        now = time.monotonic_ns()
        if now >= self._trace_end:
            self.finish_trace("window_ended")
            return False
        if now < self._trace_start:
            return False
        if self._trace_gc_event not in gc.callbacks:
            gc.callbacks.append(self._trace_gc_event)
        return True

    def _trace_gc_event(self, phase, info):
        # GC may run on another thread; thread id and all three clocks remain explicit.
        if self._trace_start <= time.monotonic_ns() < self._trace_end:
            if len(self._trace_gc) < 32:
                self._trace_gc.append(
                    (phase, info["generation"], threading.get_ident(), *self.trace_clock())
                )
            else:
                self._trace_lost += 1

    def mark(self, stage):
        if self.trace_active() and len(self._trace_marks) < 64:
            self._trace_marks.append((stage, *self.trace_clock()))

    def target_fact(self, event, **facts):
        # Bounded observations only; flush after the next actual send, never replay commands.
        if self.trace_active():
            if event == "target_installed":
                self._trace_last_target = facts["command"]
            elif event == "target_closed":
                self._trace_last_target = None
            if len(self._trace_targets) < 16:
                self._trace_targets.append((event, facts))
            else:
                self._trace_lost += 1

    def trace_record(self, event, **facts):
        if self.trace_active():
            self._write_trace(event, facts)

    def _write_trace(self, event, facts):
        # A FIFO write <= PIPE_BUF is atomic; full logs are dropped, never retried.
        try:
            if self._trace_rows >= 4095 and event != "trace_end":
                self._trace_lost += 1
                return
            row = dict(
                trace_id=self._trace_id,
                pid=os.getpid(),
                event=event,
                at_ns=time.monotonic_ns(),
                lost=self._trace_lost,
                **facts,
            )
            payload = (json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n").encode()
            if len(payload) > self._trace_size:
                self._trace_lost += 1
                return
            if os.write(self._trace_fd, payload) != len(payload):
                self._trace_lost += 1
            self._trace_rows += 1
        except Exception:
            self._trace_lost += 1

    def trace_status(self):
        self.trace_active()  # Also removes the callback/closes our fd at window end.
        return dict(
            state=getattr(self, "_trace_state", "disabled"),
            trace_id=getattr(self, "_trace_id", ""),
            lost=getattr(self, "_trace_lost", 0),
        )

    def finish_trace(self, reason):
        fd = getattr(self, "_trace_fd", -1)
        if fd < 0:
            return
        try:
            self._trace_lost += len(self._trace_targets)
            self._trace_targets.clear()
            self._write_trace("trace_end", dict(reason=reason, rows=self._trace_rows))
        finally:
            self._trace_fd = -1
            self._trace_state = reason
            if self._trace_gc_event in gc.callbacks:
                gc.callbacks.remove(self._trace_gc_event)
            try:
                os.close(fd)
            except OSError:
                pass

    def publish(self, velocity, deadline, *, computed_steady):
        traced = self.trace_active()
        entered = self.trace_clock() if traced else None
        if self.process.poll() is not None:
            self.trace_record("transport_exited", computed=computed_steady)
            raise ValueError("motion_output_transport_exited")
        self.sequence += 1
        frame = struct.pack(
            "=ddddQ", velocity.vx, velocity.wz, deadline, computed_steady, self.sequence
        )
        send_begin = self.trace_clock() if traced else None
        try:
            if self.channel.send(frame) != len(frame):
                raise OSError("partial output frame")
        except OSError as exc:
            self.channel.close()  # Native EOF revokes output; no hidden retry queue.
            if traced:
                self.trace_record(
                    "transport_send",
                    sequence=self.sequence,
                    computed=computed_steady,
                    deadline=deadline,
                    vx=velocity.vx,
                    wz=velocity.wz,
                    begin=entered,
                    send_begin=send_begin,
                    end=self.trace_clock(),
                    outcome=type(exc).__name__,
                )
            raise ValueError("motion_output_transport_lost") from exc
        if traced:
            ended = self.trace_clock()
            self._trace_sent_clock = ended
            self._trace_send = dict(
                sequence=self.sequence,
                computed=computed_steady,
                computed_ns=round(computed_steady * 1e9),
                deadline=deadline,
                remaining_ms=deadline * 1000 - ended[0] / 1e6,
                since_computed_ms=ended[0] / 1e6 - computed_steady * 1000,
                vx=velocity.vx,
                wz=velocity.wz,
                begin=entered,
                send_begin=send_begin,
                end=ended,
                outcome="sent",
                native_pid=getattr(self.process, "pid", None),
            )

    def flush_send_trace(self, **context):
        # Only a diagnostic fact, never a queued actuator packet or retry.
        value, self._trace_send = getattr(self, "_trace_send", None), None
        targets, self._trace_targets = getattr(self, "_trace_targets", []), []
        for event, facts in targets:
            self.trace_record(event, **facts)
        if value is not None:
            self.trace_record("transport_send", **value, **context)

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
        self.finish_trace("closed")
