"""Authenticated target arbitration and identity retirement; existing rules preserved."""

import math
import time
from typing import Any

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionEvent, MotionMode, MotionTarget
from senior_control.motion.contracts import MotionEventKind as K
from senior_control.motion.manual_target import ManualLinearTarget
from senior_control.motion.state_machine import MotionStateMachine
from senior_robot.final_command_frame import PAYLOAD_FIELDS


class CommandArbitration:
    """Owns arbitration state; context supplies runtime ports."""

    def __init__(self, context):
        self.context = context

    def _event(self, kind, now, source="", revision=0):
        self.event_sequence += 1
        self.machine.reduce(
            MotionEvent(kind, source, revision, self.event_sequence), now_steady=now
        )

    def command(self, msg: Any) -> bool:
        """Called only AFTER the existing authenticated FinalCommand generation fence."""
        now = time.monotonic()
        self.context._trace_target("target_received", msg, now)
        if self.context._closed:
            return False
        reason = str(msg.close_reason)
        if reason not in {
            "prepared",
            "intent",
            "release",
            "expiry",
            "connection_retired",
            "estop",
            "fault",
            "task_revoke",
            "task_done",
            "shutdown",
            "qualification_lost",
            "estop_reset",
            "fault_recover",
        }:
            return False
        if msg.source_kind not in {"manual", "navigation", "idle"}:
            return False
        if reason == "estop" and (not msg.request_id or msg.source_kind == "navigation"):
            return False
        if (
            not all(
                math.isfinite(x) for x in (msg.received_steady, msg.steady_deadline, msg.vx, msg.wz)
            )
            or not 0 <= msg.received_steady <= now
        ):
            return False
        manual_linear = None
        try:
            metadata = ManualLinearTarget(
                msg.manual_linear, msg.manual_linear_limit, msg.degraded_linear_limit
            )
        except (ValueError, AttributeError):
            return False
        if msg.source_kind == "manual" and reason == "intent":
            if msg.vx != metadata.fraction * metadata.full_stick_mps:
                return False
            manual_linear = metadata
        elif metadata != ManualLinearTarget(0, 0, 0):
            return False
        handoff = (
            msg.source_kind == "idle"
            and msg.source_id == self.context.boot
            and (
                (
                    reason == "prepared"
                    and msg.request_id == self.context.boot
                    and msg.source_epoch == 0
                )
                or (reason == "estop" and bool(msg.request_id) and msg.source_epoch > 0)
            )
        )
        # A fresh output cannot identify old intents from timestamps alone.
        # Only this random boot's causal zero echo finishes recovery. An estop
        # may always lock first, but an unrelated estop does not finish handoff.
        if self.context._recovery_pending and not handoff and reason != "estop":
            return False
        if msg.owner_boot_id == self._input_boot and msg.generation < self.output_generation:
            return False
        if msg.owner_boot_id != self._input_boot:
            self._input_boot = msg.owner_boot_id
            self._input_floor = -1
            self._task_retirements.clear()
            self._terminal_released.clear()
        self.output_generation = int(msg.generation)
        if reason != "intent":
            if msg.sequence != 0 or msg.steady_deadline != 0 or msg.vx != 0 or msg.wz != 0:
                return False
            key = (msg.owner_boot_id, int(msg.generation), int(msg.sequence))
            payload = tuple(getattr(msg, name) for name in PAYLOAD_FIELDS)
            if key == self._barrier_key:
                if payload != self._barrier_payload:
                    return False
                if self._barrier_complete:
                    return True
            else:
                self._barrier_key = key
                self._barrier_payload = payload
                self._barrier_complete = False
            accepted = self._apply_close(msg, reason, now)
            if accepted:
                self.context._trace_target("target_closed", msg, now)
                self._barrier_complete = True
                self._remember_task_retirement(msg)
                if handoff:
                    self.context._recovery_pending = False
            return accepted
        if (
            msg.generation <= self._input_floor
            or msg.source_kind == "idle"
            or not msg.source_id
            or not math.isfinite(msg.received_steady)
            or not msg.received_steady <= now < msg.steady_deadline
        ):
            return False
        velocity = Velocity(float(msg.vx), float(msg.wz))
        if not all(math.isfinite(x) for x in (velocity.vx, velocity.wz)):
            return False
        self.context._interrupt_learning("control_takeover", now)
        source_id = f"{msg.source_id}:{msg.source_epoch}"
        if self.target.source_id != source_id:
            if msg.source_kind == "navigation" and self.machine.task_paused:
                self._event(K.TASK_RESUME, now, msg.source_id)
            self._event(
                K.MANUAL_START if msg.source_kind == "manual" else K.TASK_START, now, msg.source_id
            )
        expected = MotionMode.MANUAL if msg.source_kind == "manual" else MotionMode.TASK
        if self.machine.state.mode not in {expected, MotionMode.OBSTACLE_HOLD}:
            self.revoke("state_rejected_target")
            return False
        self.target = MotionTarget(
            velocity, msg.source_kind, source_id, float(msg.steady_deadline), manual_linear
        )
        self.context._trace_target("target_installed", msg, now)
        if msg.source_kind == "navigation":
            self._task_identity = (str(msg.source_id), int(msg.source_epoch))
            # An accepted higher epoch irreversibly supersedes old pause receipts.
            obsolete = [
                key
                for key, proof in self._task_retirements.items()
                if key[1] == msg.source_id
                and proof[0] < msg.source_epoch
                and key not in self._terminal_released
            ]
            for key in obsolete:
                self._task_retirements.pop(key)
                self._terminal_released.discard(key)
        return True

    def _apply_close(self, msg, reason, now):
        if reason == "task_done" and (
            msg.source_kind != "navigation"
            or self.machine.state.task_id not in {None, msg.source_id}
        ):
            return False
        if (
            reason == "prepared"
            and msg.source_kind == "idle"
            and self.target.source == "startup_learning"
        ):
            # Initial owner heartbeat is not an operator takeover of automatic learning.
            return True
        if reason == "estop" or msg.source_kind in {"manual", "navigation"}:
            self.context._interrupt_learning(
                "estop" if reason == "estop" else "control_takeover", now
            )
        self.revoke(reason, now=now)
        if reason == "estop":
            if (
                msg.source_kind == "idle"
                and msg.source_id == self.context.boot
                and msg.source_epoch > 0
            ):
                # Private recovery meaning ONLY for this boot-matched idle
                # estop: source_epoch preserves the original latch revision.
                # request_id remains the operator's original request identity;
                # gesture stays zero, as required by the native v3 guard.
                return self.machine.restore_software_estop(int(msg.source_epoch), now_steady=now)
            self._event(K.SOFTWARE_ESTOP, now, msg.source_id)
        elif reason in {"estop_reset", "fault_recover"}:
            # Only an explicit revision-matched operator request clears a latch.
            self._event(
                K.ESTOP_RESET if reason == "estop_reset" else K.FAULT_RECOVER,
                now,
                msg.source_id,
                int(msg.gesture),
            )
            return self.machine.state.mode not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
        elif msg.source_kind == "manual":
            self._event(
                K.MANUAL_EXPIRED if reason == "expiry" else K.MANUAL_RELEASE, now, msg.source_id
            )
        elif msg.source_kind == "navigation":
            self._event(K.TASK_PAUSE, now, msg.source_id)
        return True

    @staticmethod
    def _task_retirement_key(msg: Any) -> tuple[str, str, int, str]:
        return (msg.owner_boot_id, msg.source_id, int(msg.generation), msg.request_id)

    def _remember_task_retirement(self, msg: Any) -> None:
        if (
            msg.source_kind != "navigation"
            or not msg.request_id
            or msg.source_epoch < 1
            or msg.close_reason
            not in {"task_revoke", "task_done", "qualification_lost", "shutdown", "expiry", "fault"}
        ):
            return
        key = self._task_retirement_key(msg)
        if key in self._task_retirements:
            return
        if len(self._task_retirements) >= 256:
            return  # Output remains revoked; no new identity-release proof is issued.
        epoch = int(msg.source_epoch)
        if self.machine.state.task_id == msg.source_id and self._task_identity is None:
            return  # A retained task with unknown epoch is not an identity-release proof.
        if self._task_identity is not None and self._task_identity[0] == msg.source_id:
            if epoch < self._task_identity[1]:
                return
            epoch = self._task_identity[1]
        self._task_retirements[key] = (epoch, float(msg.received_steady), int(msg.connection_epoch))

    def retirement_epoch(self, msg: Any) -> int | None:
        proof = self._task_retirements.get(self._task_retirement_key(msg))
        return proof[0] if proof is not None else None

    def collect_terminal(self, msg: Any) -> bool:
        """Durable settlement only reclaims metadata; it never releases identity.

        Old barriers cannot recreate forgotten proofs: command generation is
        irreversible, and an identical current barrier returns without minting.
        Unknown collections are idempotent no-ops, never terminal-release ACKs.
        """
        if (
            self.context._closed
            or msg.owner_boot_id != self._input_boot
            or msg.close_reason != "task_settled"
            or msg.source_kind != "navigation"
            or msg.session_id != msg.source_id
            or not msg.source_id
            or not msg.request_id
            or msg.source_epoch < 1
            or msg.generation != 0
            or msg.connection_epoch != msg.source_epoch
            or msg.sequence != 0
            or msg.steady_deadline != 0
            or msg.vx != 0
            or msg.wz != 0
            or msg.gesture != 0
        ):
            return False
        if self.machine.state.task_id == msg.source_id and (
            self._task_identity is None or self._task_identity[1] <= msg.source_epoch
        ):
            return False
        keys = [
            key
            for key, proof in self._task_retirements.items()
            if key[0] == msg.owner_boot_id
            and key[1] == msg.source_id
            and proof[0] <= msg.source_epoch
        ]
        if keys and not any(key in self._terminal_released for key in keys):
            return False
        for key in keys:
            self._task_retirements.pop(key)
            self._terminal_released.discard(key)
        return True

    def release_terminal(self, msg: Any) -> bool:
        """Consume only a proven identity event; never revoke a newer motion owner."""
        if (
            self.context._closed
            or msg.owner_boot_id != self._input_boot
            or msg.close_reason != "task_terminal"
            or msg.source_kind != "navigation"
            or msg.session_id != msg.source_id
            or not msg.source_id
            or not msg.request_id
            or msg.sequence != 0
            or msg.steady_deadline != 0
            or msg.gesture != 0
            or msg.vx != 0
            or msg.wz != 0
        ):
            return False
        key = self._task_retirement_key(msg)
        proof = self._task_retirements.get(key)
        if proof != (msg.source_epoch, msg.received_steady, msg.connection_epoch):
            return False
        if key in self._terminal_released:
            return True
        if self.target.source == "navigation":
            # Even the same epoch must not be released while its navigation source is live.
            return False
        identity = (str(msg.source_id), int(msg.source_epoch))
        if self._task_identity is not None and self._task_identity != identity:
            return False
        if self.machine.state.task_id not in {None, msg.source_id} or (
            self.machine.state.task_id is not None and self._task_identity != identity
        ):
            return False
        revision = self.machine.state.fault_revision
        self._event(K.TASK_RETIRED, time.monotonic(), msg.source_id)
        if self.machine.state.task_id is not None or self.machine.state.fault_revision != revision:
            return False
        self._task_identity = None
        self._terminal_released.add(key)
        return True

    def revoke(self, reason: str, *, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        if reason != "prepared" and not (
            reason == "command_topology_unqualified" and not self.context.startup.motion_started
        ):
            self.context._interrupt_learning(reason, now)
        if reason not in {
            "prepared",
            "release",
            "expiry",
            "connection_retired",
            "task_revoke",
            "qualification_lost",
            "shutdown",
        }:
            self._input_floor = max(self._input_floor, self.output_generation)
        if self.target.source != "idle":
            self._event(
                K.MANUAL_EXPIRED if self.target.source == "manual" else K.TASK_PAUSE,
                now,
                self.target.source_id.rsplit(":", 1)[0],
            )
        self.target = MotionTarget(Velocity(0, 0), "idle", self.context.boot, None)
        self.context._output = Velocity(0, 0)

    def initialize(self):
        self.machine = MotionStateMachine()
        self.target = MotionTarget(Velocity(0, 0), "idle", self.context.boot, None)
        self._task_identity: tuple[str, int] | None = None
        self._task_retirements: dict[tuple[str, str, int, str], tuple[int, float, int]] = {}
        self._terminal_released: set[tuple[str, str, int, str]] = set()
        self._input_boot, self._input_floor = "", -1
        self._barrier_key = self._barrier_payload = None
        self._barrier_complete = False
