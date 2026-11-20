"""//Compatible: shared timing aliases and the original lazy ROS parameter adapter."""

from typing import TYPE_CHECKING

from vehicle_core.sample_time import SampleTime as SampleTime
from vehicle_core.sensor_policy import Age as Age
from vehicle_core.sensor_policy import SensorClockError as SensorClockError
from vehicle_core.sensor_policy import SensorTiming as SensorTiming
from vehicle_core.sensor_policy import default_policy as default_policy
from vehicle_core.sensor_policy import sensor_current as sensor_current

if TYPE_CHECKING:
    from senior_control.runtime.diagnostics import DiagnosticsSettings


def ros_sensor_parameters(
    diagnostics: "DiagnosticsSettings | None" = None,
) -> dict[str, int | float]:
    from senior_control.runtime.diagnostics import QueueWarningPolicy

    q = QueueWarningPolicy() if diagnostics is None else diagnostics.queue
    p = default_policy()
    return dict(
        queue_warn_count=q.count,
        queue_warn_ratio=float(q.ratio),
        queue_warn_repeat_s=float(q.repeat_s),
        queue_warn_recovery_ratio=float(q.recovery_ratio),
        queue_warn_oldest_wait_ms=float(q.oldest_wait_ms),
        sensor_soft_age_ms=float(p.soft_age_ms),
        sensor_hard_age_ms=float(p.hard_age_ms),
        sensor_diagnostics_period_s=float(
            p.diagnostics_period_s if diagnostics is None else diagnostics.period_s
        ),
    )
