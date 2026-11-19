"""Structured control diagnostics and runtime report publication."""

import math
from dataclasses import asdict

from senior_control.motion.contracts import MotionMode
from senior_control.protocol import MotionRuntimeState
from senior_control.sensors.policy import default_policy
from senior_control.vision.raw_capture import capture_from_message

_OUTPUT_BUDGET_S = 0.25


class CoreDiagnostics:
    """Owns diagnostics state; context supplies runtime ports."""

    def __init__(self, context):
        self.context = context

    def _trace_mark(self, stage):
        trace = getattr(self.context.node, "_output_transport", None)
        if trace is not None:
            trace.mark(stage)

    def _trace_target(self, event, msg, now):
        trace = getattr(self.context.node, "_output_transport", None)
        if trace is None or not trace.trace_active():
            return
        try:
            trace.target_fact(
                event,
                boot=self.context.boot,
                core_sequence=self.context.sequence,
                now=now,
                command={
                    name: getattr(msg, name)
                    for name in (
                        "owner_boot_id",
                        "session_id",
                        "request_id",
                        "generation",
                        "connection_epoch",
                        "sequence",
                        "gesture",
                        "source_kind",
                        "source_id",
                        "source_epoch",
                        "received_steady",
                        "steady_deadline",
                        "close_reason",
                        "vx",
                        "wz",
                    )
                },
            )
        except Exception:
            pass  # Diagnostics never change target admission.

    def _trace_tick(self, now, bounds, candidate, output, reason, facts):
        trace = getattr(self.context.node, "_output_transport", None)
        if trace is None or not trace.trace_active():
            return
        try:
            budget = now + _OUTPUT_BUDGET_S
            effective = dict(
                budget=budget,
                target=bounds["target"] or budget,
                scan=bounds["scan"] or budget,
                startup=bounds["startup"]
                if facts["learning_output"] and bounds["startup"] is not None
                else budget,
            )
            deadline = min(effective.values())
            sampled = trace.trace_clock()
            raw = self.context.node._scans[-1] if self.context.node._scans else None
            selected = None
            if raw is not None:
                try:
                    age = default_policy().age(
                        measured_steady=raw.measurement_steady,
                        received_steady=raw.received,
                        now_steady=now,
                    )
                    age_fact = dict(band=age.band, seconds=age.seconds)
                except ValueError as exc:
                    age_fact = dict(error=type(exc).__name__, message=str(exc)[:128])
                m = raw.message
                selected = dict(
                    capture=asdict(capture_from_message(m.raw_capture)),
                    source=m.source_epoch,
                    sequence=m.sequence,
                    measurement=raw.measurement_steady,
                    receipt=raw.received,
                    age=age_fact,
                    scan_deadline=raw.measurement_steady + default_policy().hard_age_s,
                )
            decision = facts
            if facts.get("clearance") is not None:
                # Unobstructed distances may be infinite; encode only their diagnostic copy.
                decision = {
                    **facts,
                    "clearance": {
                        name: value
                        if math.isfinite(value)
                        else ("NaN" if math.isnan(value) else "Inf" if value > 0 else "-Inf")
                        for name, value in facts["clearance"].items()
                    },
                }
            trace.trace_record(
                "core_tick",
                boot=self.context.boot,
                core_sequence=self.context.sequence,
                computed=now,
                computed_ns=round(now * 1e9),
                transport_sequence=trace.sequence,
                target=dict(
                    source=self.context.target.source,
                    id=self.context.target.source_id,
                    deadline=self.context.target.deadline_steady,
                    vx=self.context.target.velocity.vx,
                    wz=self.context.target.velocity.wz,
                ),
                last_installed_command=trace._trace_last_target,
                bounds=bounds,
                effective=effective,
                deadline=deadline,
                argmin=[name for name, value in effective.items() if value == deadline],
                report_clock=sampled,
                remaining_ms_at_report={
                    name: value * 1000 - sampled[0] / 1e6 for name, value in effective.items()
                },
                remaining_ms_at_send=None
                if trace._trace_sent_clock is None
                else {
                    name: value * 1000 - trace._trace_sent_clock[0] / 1e6
                    for name, value in effective.items()
                },
                candidate=(candidate.vx, candidate.wz),
                submitted=(output.vx, output.wz),
                reason=reason,
                decision=decision,
                selected=selected,
            )
            trace.trace_record(
                "core_stages",
                boot=self.context.boot,
                core_sequence=self.context.sequence,
                computed_ns=round(now * 1e9),
                marks=tuple(trace._trace_marks),
            )
            if trace._trace_gc:
                trace.trace_record(
                    "gc",
                    boot=self.context.boot,
                    core_sequence=self.context.sequence,
                    events=tuple(trace._trace_gc),
                )
        except Exception as exc:
            trace.trace_record(
                "core_trace_error",
                boot=self.context.boot,
                core_sequence=self.context.sequence,
                error=type(exc).__name__ + ":" + str(exc)[:128],
            )
        finally:
            trace._trace_marks.clear()
            trace._trace_gc.clear()

    def _publish_runtime(self, now, reason):
        state = self.context.machine.state
        independent = self.context.sensors.independent
        packet = getattr(self.context.sensors, "input_reference", None)
        reference = None if packet is None else packet[0]
        average = None if reference is None else reference.get("average")
        body = None
        if (
            independent is not None
            and average
            and all(value is not None and math.isfinite(value) for value in average.values())
        ):
            body = {
                "average": reference["average"],
                "source": reference["mode"],
                "computed_steady": independent.stamp_steady,
                "valid_until_steady": independent.deadline_steady,
            }
        actions = list(state.allowed_actions)
        if (
            state.task_id == self.context.boot
            and state.mode not in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
            and "acquire" not in actions
        ):
            actions.append("acquire")
        runtime = MotionRuntimeState.model_validate(
            {
                "schema": 1,
                "robot_id": str(self.context.node.get_parameter("robot_id").value),
                "boot_id": self.context.boot,
                "sequence": self.context.sequence,
                "sampled_steady": now,
                "mode": state.mode.value,
                "reason": reason,
                "stationary": bool(self.context.sensors.stationary(now)),
                "healthy": bool(
                    self.context.sensors.latest is not None and self.context.sensors.latest.healthy
                ),
                "evidence_stamp_steady": None,
                "evidence_deadline_steady": None,
                "estop_revision": state.estop_revision,
                "fault_revision": state.fault_revision,
                "confirmed_estop_status": "software_latched"
                if state.mode is MotionMode.ESTOP
                else "clear",
                "allowed_actions": actions,
                "task_id": state.task_id,
                "output_generation": self.context.output_generation,
                "body_reference": body,
                "wheel_linear_mps": self.context.sensors.wheel_linear(now),
            }
        )
        msg = self._runtime_type()
        msg.data = runtime.model_dump_json()
        self._runtime.publish(msg)

    def report_status(self, value):
        if self.context._learning_failure is not None:
            value = {
                **value,
                "values": {**value["values"], "learning_failure": self.context._learning_failure},
            }
        self._runtime.status(value)

    def initialize(self):
        from std_msgs.msg import String

        from senior_control.motion.diagnostic_pump import DiagnosticPump
        from senior_robot.runtime_reports import RuntimeReports

        self._runtime = RuntimeReports(self.context.node)
        self.context._resources.callback(self._runtime.close)
        self._diagnostic_pump = DiagnosticPump(self.context.diagnostics)
        self.context._resources.callback(self._diagnostic_pump.close)
        self._runtime_type = String
