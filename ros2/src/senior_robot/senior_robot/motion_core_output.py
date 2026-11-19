"""Production motion composition and control tick scheduling."""

import os
import time
from contextlib import ExitStack
from dataclasses import asdict
from typing import Any
from uuid import uuid4

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionMode
from senior_control.runtime.diagnostics import configure_diagnostics
from senior_robot.motion_sensor_runtime import MotionSensorRuntime
from senior_robot.vehicle_arbitration import CommandArbitration
from senior_robot.vehicle_capabilities import VehicleCapabilities
from senior_robot.vehicle_collision import CollisionPrediction
from senior_robot.vehicle_compat import OwnedState
from senior_robot.vehicle_diagnostics import CoreDiagnostics
from senior_robot.vehicle_learning import LearningRuntime
from senior_robot.vehicle_sensor_access import SensorAccess
from senior_robot.vehicle_speed import SpeedCalculation

_OUTPUT_BUDGET_S = 0.25


class MotionCoreOutput:
    def __init__(self, node: Any, profile: Any):
        self.capabilities = VehicleCapabilities()
        self.collision = CollisionPrediction(profile)
        self.speed = SpeedCalculation(profile)
        self.arbitration = CommandArbitration(self)
        self.learning = LearningRuntime(self)
        self.sensor_access = SensorAccess(self)
        self.diagnostics = None
        self.diagnostics_owner = CoreDiagnostics(self)
        self._resources = ExitStack()
        try:
            self.node, self.profile = node, profile
            self.diagnostics = configure_diagnostics(profile.diagnostics)
            self.sensors = MotionSensorRuntime(profile, self.diagnostics)
            self.boot = uuid4().hex
            self.sequence = self.event_sequence = self.output_generation = 0
            self.arbitration.initialize()
            self._closed = False
            self._launch_recovered = os.environ.get("SENIOR_LAUNCH_RECOVERED") == "1"
            self._recovery_pending = self._launch_recovered
            self._last_tick = None
            self._output = Velocity(0, 0)
            self.diagnostics_owner.initialize()
            self.learning.initialize()
        except BaseException:
            self._resources.close()
            raise

    def _event(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.arbitration._event(*args, **kwargs)

    def command(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.arbitration.command(*args, **kwargs)

    def _apply_close(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.arbitration._apply_close(*args, **kwargs)

    @staticmethod
    def _task_retirement_key(msg):
        return CommandArbitration._task_retirement_key(msg)

    def _remember_task_retirement(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.arbitration._remember_task_retirement(*args, **kwargs)

    def retirement_epoch(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.arbitration.retirement_epoch(*args, **kwargs)

    def collect_terminal(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.arbitration.collect_terminal(*args, **kwargs)

    def release_terminal(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.arbitration.release_terminal(*args, **kwargs)

    def _interrupt_learning(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.learning._interrupt_learning(*args, **kwargs)

    def _learning_observation(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.learning._learning_observation(*args, **kwargs)

    def _remember_learning_failure(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.learning._remember_learning_failure(*args, **kwargs)

    def _scan_key(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.sensor_access._scan_key(*args, **kwargs)

    def _filtered_scan(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.learning._filtered_scan(*args, **kwargs)

    def _startup_target(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.learning._startup_target(*args, **kwargs)

    def revoke(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.arbitration.revoke(*args, **kwargs)

    def _trace_mark(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.diagnostics_owner._trace_mark(*args, **kwargs)

    def _trace_target(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.diagnostics_owner._trace_target(*args, **kwargs)

    def _trace_tick(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.diagnostics_owner._trace_tick(*args, **kwargs)

    def tick(self) -> None:
        now = time.monotonic()
        self.sequence += 1
        self._trace_mark("tick_entry")
        trace = getattr(self.node, "_output_transport", None)
        tracing = trace is not None and trace.trace_active()
        facts = (
            dict(
                previous=(self._output.vx, self._output.wz),
                state=self.machine.state.mode.value,
                target_expired=False,
            )
            if tracing
            else None
        )
        dt = 0.0 if self._last_tick is None else max(0.0, min(now - self._last_tick, 0.1))
        self._last_tick = now
        deadline = self.target.deadline_steady
        if deadline is not None and now >= deadline:
            if tracing:
                facts.update(
                    target_expired=True,
                    expired_source=self.target.source_id,
                    expired_deadline=deadline,
                )
            self.revoke("expiry", now=now)
        try:
            filtered = self._learning_observation(now)
        except (ValueError, TypeError, KeyError) as exc:
            # Learning cannot withdraw manual/navigation or suppress raw scan handling.
            self._remember_learning_failure(now, exc)
            self.startup.hold_motion("learning_scan_unavailable")
            filtered = None
        self._trace_mark("learning_end")
        self._startup_target(now, filtered)
        state = self.machine.state
        output, reason, scan_deadline = Velocity(0, 0), state.reason, None
        if tracing:
            facts["state"] = state.mode.value
            facts["zero_stage"] = (
                "state" if state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT} else None
            )
        if state.mode not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
            requested = self.target.velocity
            if requested != Velocity(0, 0):
                control_filtered = filtered
                if self.node._scans and self.node._scans[-1].predictions:
                    control_filtered = self._filtered_scan(self.node._scans[-1], avoidance=True)
                self._trace_mark("clearance_begin")
                clearance = self.collision.clearance(
                    self.node._scans, now, filtered=control_filtered
                )
                self._trace_mark("clearance_end")
                if tracing:
                    facts["clearance"] = None if clearance is None else asdict(clearance)
                if clearance is None:
                    if tracing:
                        facts["zero_stage"] = "clearance"
                    reason = "scan_temporarily_unavailable"
                else:
                    current = self.sensors.wheel_linear(now)
                    self._trace_mark("limit_begin")
                    output = self.speed.calculate(requested, self._output, current, clearance, dt)
                    self._trace_mark("limit_end")
                    if tracing:
                        facts["zero_stage"] = "limit" if output == Velocity(0, 0) else None
                        facts.update(
                            requested=(requested.vx, requested.wz),
                            measured=current,
                            dt=dt,
                            limited=(output.vx, output.wz),
                        )
                    scan_deadline = clearance.deadline
                    reason = "directional_limit" if output != requested else "target"
            else:
                if tracing:
                    facts["zero_stage"] = "target_zero"
                reason = (
                    self.startup.reason
                    if self.target.source == "startup_learning"
                    else ("standby" if self.target.source == "idle" else "stick_stopped")
                )
        learning_output = self.target.source == "startup_learning"
        if tracing:
            facts["learning_output"] = learning_output
        if learning_output and output != Velocity(0, 0):
            self._trace_mark("prepare_action_begin")
            permitted = self.startup.prepare_action()
            self._trace_mark("prepare_action_end")
            if tracing:
                facts["action_permitted"] = permitted
            if not permitted:
                if tracing:
                    facts["zero_stage"] = "prepare_action"
                output, reason = Velocity(0, 0), "learning_action_reservation_pending"
        if (
            learning_output
            and self.startup.deadline is not None
            and time.monotonic() >= self.startup.deadline
        ):
            if tracing:
                facts["zero_stage"] = "startup_deadline"
            output, reason = Velocity(0, 0), "learning_action_window_finished"
        self._output = output
        candidate = output
        bounds = (
            dict(
                budget=now + _OUTPUT_BUDGET_S,
                target=self.target.deadline_steady,
                scan=scan_deadline,
                startup=self.startup.deadline,
            )
            if tracing
            else None
        )
        # A control-process stall stops the native owner even if a source TTL is longer.
        output_deadline = min(
            now + _OUTPUT_BUDGET_S,
            self.target.deadline_steady or now + _OUTPUT_BUDGET_S,
            scan_deadline or now + _OUTPUT_BUDGET_S,
            self.startup.deadline
            if learning_output and self.startup.deadline is not None
            else now + _OUTPUT_BUDGET_S,
        )
        if learning_output and output != Velocity(0, 0) and time.monotonic() >= output_deadline:
            output = self._output = Velocity(0, 0)
            if tracing:
                facts["zero_stage"] = "pre_submit_deadline"
            reason = "learning_output_expired"
            # Discard this candidate only. The same live owner keeps its grant
            # and original window; the next tick must compute a fresh output.
        self._trace_mark("emit_begin")
        submitted = self.node._emit_motion_core(
            output, reason, output_deadline, computed_steady=now
        )
        self._trace_mark("emit_end")
        if learning_output and output != Velocity(0, 0):
            self.startup.motion_emitted(submitted, deadline=output_deadline)
        else:
            self.startup.output_stopped(submitted)
        self.diagnostics.summary(
            "startup_learning",
            self.learning.summary(),
        )
        self._trace_mark("runtime_begin")
        self._publish_runtime(now, reason)
        self._trace_mark("runtime_end")
        if tracing:
            self._trace_tick(now, bounds, candidate, output, reason, facts)
        self.diagnostics.duration("control_compute", time.monotonic() - now)

    def _publish_runtime(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.diagnostics_owner._publish_runtime(*args, **kwargs)

    def report_status(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.diagnostics_owner.report_status(*args, **kwargs)

    def close(self) -> None:
        self._closed = True
        self._interrupt_learning("shutdown", time.monotonic())
        self._resources.close()

    # //Compatible: legacy entry points expose state held by dedicated owners.
    machine = OwnedState("arbitration", "machine")
    target = OwnedState("arbitration", "target")
    event_sequence = OwnedState("arbitration", "event_sequence")
    output_generation = OwnedState("arbitration", "output_generation")
    _task_identity = OwnedState("arbitration", "_task_identity")
    _task_retirements = OwnedState("arbitration", "_task_retirements")
    _terminal_released = OwnedState("arbitration", "_terminal_released")
    _input_boot = OwnedState("arbitration", "_input_boot")
    _input_floor = OwnedState("arbitration", "_input_floor")
    _barrier_key = OwnedState("arbitration", "_barrier_key")
    _barrier_payload = OwnedState("arbitration", "_barrier_payload")
    _barrier_complete = OwnedState("arbitration", "_barrier_complete")
    startup = OwnedState("learning", "startup")
    self_returns = OwnedState("learning", "self_returns")
    _learning_client = OwnedState("learning", "_learning_client")
    _learning_geometry = OwnedState("learning", "_learning_geometry")
    _retired_learning_owner = OwnedState("learning", "_retired_learning_owner")
    _old_learning_owner = OwnedState("learning", "_old_learning_owner")
    _learning_stage = OwnedState("learning", "_learning_stage")
    _learning_age = OwnedState("learning", "_learning_age")
    _learning_record_phase = OwnedState("learning", "_learning_record_phase")
    _learning_failure = OwnedState("learning", "_learning_failure")
    _learning_attempt = OwnedState("learning", "_learning_attempt")
    _manual_learning_request = OwnedState("learning", "_manual_learning_request")
    _scan_result_key = OwnedState("sensor_access", "_scan_result_key")
    _scan_results = OwnedState("sensor_access", "_scan_results")
    _runtime = OwnedState("diagnostics_owner", "_runtime")
    _diagnostic_pump = OwnedState("diagnostics_owner", "_diagnostic_pump")
    _runtime_type = OwnedState("diagnostics_owner", "_runtime_type")
