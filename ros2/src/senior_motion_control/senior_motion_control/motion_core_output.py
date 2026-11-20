"""Production motion composition and control tick scheduling."""

import os
import time
from contextlib import ExitStack
from dataclasses import asdict
from typing import Any
from uuid import uuid4

from senior_motion_estimation.motion_sensor_runtime import MotionSensorRuntime
from senior_perception.collision import CollisionPrediction
from senior_perception.learning_runtime import LearningControlInput, LearningPorts, LearningRuntime
from senior_perception.scan_access import SensorAccess

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionMode
from senior_motion_control.vehicle_arbitration import ArbitrationPorts, CommandArbitration
from senior_motion_control.vehicle_capabilities import VehicleCapabilities
from senior_motion_control.vehicle_diagnostics import CoreDiagnostics
from senior_motion_control.vehicle_speed import SpeedCalculation
from vehicle_core.diagnostics import configure_diagnostics

_OUTPUT_BUDGET_S = 0.25


class MotionCoreOutput:
    def __init__(self, node: Any, profile: Any):
        self.capabilities = VehicleCapabilities()
        self.collision = CollisionPrediction(profile)
        self.speed = SpeedCalculation(profile)
        self.sensor_access = SensorAccess(self)
        self.diagnostics = None
        self.diagnostics_owner = CoreDiagnostics(self)
        self._resources = ExitStack()
        try:
            self.node, self.profile = node, profile
            self.diagnostics = configure_diagnostics(profile.diagnostics)
            self.sensors = MotionSensorRuntime(profile, self.diagnostics)
            self.boot = uuid4().hex
            self.sequence = 0
            self.arbitration = CommandArbitration(
                ArbitrationPorts(
                    boot=self.boot,
                    closed=lambda: self._closed,
                    recovery_pending=lambda: self._recovery_pending,
                    finish_recovery=self._finish_recovery,
                    trace_target=self._trace_target,
                    interrupt_learning=self._interrupt_learning,
                    learning_started=lambda: self.learning.startup.motion_started,
                    stop_output=self._stop_output,
                )
            )
            self.arbitration.event_sequence = self.arbitration.output_generation = 0
            self.arbitration.initialize()
            self._closed = False
            self._launch_recovered = os.environ.get("SENIOR_LAUNCH_RECOVERED") == "1"
            self._recovery_pending = self._launch_recovered
            self._last_tick = None
            self._output = Velocity(0, 0)
            self.learning = LearningRuntime(
                LearningPorts(
                    get_profile=lambda: self.profile,
                    get_body_frame=lambda: self.node._body,
                    scans=lambda: tuple(self.node._scans),
                    scan_generation=lambda: getattr(self.node, "_scan_generation", None),
                    wheel_linear=lambda now: self.sensors.wheel_linear(now),
                    stationary=lambda now: self.sensors.stationary(now),
                    command_topology_ready=lambda: self.node._command_topology_ready(),
                    trace=lambda mark: self._trace_mark(mark),
                    register_close=self._resources.callback,
                    scan_access=self.sensor_access,
                )
            )
            self.diagnostics_owner.initialize()
            self.learning.initialize(self._learning_control_input())
        except BaseException:
            self._resources.close()
            raise

    def _finish_recovery(self):
        self._recovery_pending = False

    def _stop_output(self):
        self._output = Velocity(0, 0)

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

    def _learning_control_input(self):
        return LearningControlInput(
            output_boot_id=self.boot,
            input_boot_id=self.arbitration._input_boot,
            generation=self.arbitration.output_generation,
            mode=self.arbitration.machine.state.mode,
            task_id=self.arbitration.machine.state.task_id,
            target_source=self.arbitration.target.source,
            target_velocity=self.arbitration.target.velocity,
            launch_recovered=self._launch_recovered,
            recovery_pending=self._recovery_pending,
            sequence=self.sequence,
        )

    def _apply_learning_request(self, request, now):
        if self.arbitration.apply_learning_request(request, now):
            self.learning.applied(request)

    def _interrupt_learning(self, reason, now):
        request = self.learning._interrupt_learning(reason, now, self._learning_control_input())
        self._apply_learning_request(request, now)

    def _learning_observation(self, now):
        return self.learning._learning_observation(now, self._learning_control_input())

    def _remember_learning_failure(self, now, error=None):
        return self.learning._remember_learning_failure(now, self._learning_control_input(), error)

    def _scan_key(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.sensor_access._scan_key(*args, **kwargs)

    def _filtered_scan(self, *args, **kwargs):
        # //Compatible: legacy ROS entry points delegate to the business owner.
        return self.learning._filtered_scan(*args, **kwargs)

    def _startup_target(self, now, observation):
        request = self.learning._startup_target(now, observation, self._learning_control_input())
        self._apply_learning_request(request, now)

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
                state=self.arbitration.machine.state.mode.value,
                target_expired=False,
            )
            if tracing
            else None
        )
        dt = 0.0 if self._last_tick is None else max(0.0, min(now - self._last_tick, 0.1))
        self._last_tick = now
        deadline = self.arbitration.target.deadline_steady
        if deadline is not None and now >= deadline:
            if tracing:
                facts.update(
                    target_expired=True,
                    expired_source=self.arbitration.target.source_id,
                    expired_deadline=deadline,
                )
            self.revoke("expiry", now=now)
        try:
            filtered = self._learning_observation(now)
        except (ValueError, TypeError, KeyError) as exc:
            # Learning cannot withdraw manual/navigation or suppress raw scan handling.
            self._remember_learning_failure(now, exc)
            self.learning.startup.hold_motion("learning_scan_unavailable")
            filtered = None
        self._trace_mark("learning_end")
        self._startup_target(now, filtered)
        state = self.arbitration.machine.state
        output, reason, scan_deadline = Velocity(0, 0), state.reason, None
        if tracing:
            facts["state"] = state.mode.value
            facts["zero_stage"] = (
                "state" if state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT} else None
            )
        if state.mode not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}:
            requested = self.arbitration.target.velocity
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
                    self.learning.startup.reason
                    if self.arbitration.target.source == "startup_learning"
                    else (
                        "standby" if self.arbitration.target.source == "idle" else "stick_stopped"
                    )
                )
        learning_output = self.arbitration.target.source == "startup_learning"
        if tracing:
            facts["learning_output"] = learning_output
        if learning_output and output != Velocity(0, 0):
            self._trace_mark("prepare_action_begin")
            permitted = self.learning.startup.prepare_action()
            self._trace_mark("prepare_action_end")
            if tracing:
                facts["action_permitted"] = permitted
            if not permitted:
                if tracing:
                    facts["zero_stage"] = "prepare_action"
                output, reason = Velocity(0, 0), "learning_action_reservation_pending"
        if (
            learning_output
            and self.learning.startup.deadline is not None
            and time.monotonic() >= self.learning.startup.deadline
        ):
            if tracing:
                facts["zero_stage"] = "startup_deadline"
            output, reason = Velocity(0, 0), "learning_action_window_finished"
        self._output = output
        candidate = output
        bounds = (
            dict(
                budget=now + _OUTPUT_BUDGET_S,
                target=self.arbitration.target.deadline_steady,
                scan=scan_deadline,
                startup=self.learning.startup.deadline,
            )
            if tracing
            else None
        )
        # A control-process stall stops the native owner even if a source TTL is longer.
        output_deadline = min(
            now + _OUTPUT_BUDGET_S,
            self.arbitration.target.deadline_steady or now + _OUTPUT_BUDGET_S,
            scan_deadline or now + _OUTPUT_BUDGET_S,
            self.learning.startup.deadline
            if learning_output and self.learning.startup.deadline is not None
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
            self.learning.startup.motion_emitted(submitted, deadline=output_deadline)
        else:
            self.learning.startup.output_stopped(submitted)
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

    @property
    def machine(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration.machine

    @machine.setter
    def machine(self, value):
        # //Compatible: historical caller delegates to the single arbitration owner.
        self.arbitration.machine = value

    @property
    def target(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration.target

    @target.setter
    def target(self, value):
        # //Compatible: historical caller delegates to the single arbitration owner.
        self.arbitration.target = value

    @property
    def event_sequence(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration.event_sequence

    @event_sequence.setter
    def event_sequence(self, value):
        # //Compatible: historical caller delegates to the single arbitration owner.
        self.arbitration.event_sequence = value

    @property
    def output_generation(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration.output_generation

    @output_generation.setter
    def output_generation(self, value):
        # //Compatible: historical caller delegates to the single arbitration owner.
        self.arbitration.output_generation = value

    @property
    def _task_identity(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration._task_identity

    @property
    def _task_retirements(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration._task_retirements

    @property
    def _terminal_released(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration._terminal_released

    @property
    def _input_boot(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration._input_boot

    @_input_boot.setter
    def _input_boot(self, value):
        # //Compatible: historical caller delegates to the single arbitration owner.
        self.arbitration._input_boot = value

    @property
    def _input_floor(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration._input_floor

    @property
    def _barrier_key(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration._barrier_key

    @property
    def _barrier_payload(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration._barrier_payload

    @property
    def _barrier_complete(self):
        # //Compatible: legacy view; state is owned only by arbitration.
        return self.arbitration._barrier_complete

    @property
    def startup(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning.startup

    @property
    def self_returns(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning.self_returns

    @property
    def _learning_client(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._learning_client

    @property
    def _learning_geometry(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._learning_geometry

    @property
    def _retired_learning_owner(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._retired_learning_owner

    @property
    def _old_learning_owner(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._old_learning_owner

    @property
    def _learning_stage(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._learning_stage

    @property
    def _learning_age(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._learning_age

    @property
    def _learning_record_phase(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._learning_record_phase

    @property
    def _learning_failure(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._learning_failure

    @property
    def _learning_attempt(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._learning_attempt

    @property
    def _manual_learning_request(self):
        # //Compatible: legacy view; state is owned only by learning.
        return self.learning._manual_learning_request

    @property
    def _scan_result_key(self):
        # //Compatible: legacy view; state is owned only by sensor_access.
        return self.sensor_access._scan_result_key

    @property
    def _scan_results(self):
        # //Compatible: legacy view; state is owned only by sensor_access.
        return self.sensor_access._scan_results

    @property
    def _runtime(self):
        # //Compatible: legacy view; state is owned only by diagnostics_owner.
        return self.diagnostics_owner._runtime

    @property
    def _diagnostic_pump(self):
        # //Compatible: legacy view; state is owned only by diagnostics_owner.
        return self.diagnostics_owner._diagnostic_pump

    @property
    def _runtime_type(self):
        # //Compatible: legacy view; state is owned only by diagnostics_owner.
        return self.diagnostics_owner._runtime_type
