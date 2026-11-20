"""//Compatible: historical entry points forward the same functional implementation."""

from senior_motion_control.motion_lifecycle import MotionExecutor as MotionExecutor
from senior_motion_control.motion_lifecycle import (
    _shutdown_subscription_conversion as _shutdown_subscription_conversion,
)
from senior_motion_control.motion_lifecycle import _spin_cli as _spin_cli
from senior_motion_control.motion_lifecycle import run as run
from senior_motion_control.motion_lifecycle import with_dds_info as with_dds_info
