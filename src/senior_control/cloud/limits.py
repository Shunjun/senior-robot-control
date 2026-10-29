"""Explicit cloud limits. Every bound is a named field, not a hidden constant."""

from dataclasses import dataclass

from senior_control.protocol import MAX_MESSAGE_BYTES


@dataclass(frozen=True)
class CloudLimits:
    login_failures_per_client: int = 5
    login_failures_global: int = 30
    pair_failures_per_client: int = 10
    pair_failures_global: int = 60
    failure_window_seconds: float = 300.0
    tracked_clients: int = 4096
    session_ttl_seconds: float = 8 * 3600.0
    pairing_code_ttl_seconds: float = 600.0
    max_live_pairing_codes: int = 20
    max_body_bytes: int = 4096
    max_ws_message_bytes: int = MAX_MESSAGE_BYTES
    outbox_frames: int = 32
    max_controllers_per_robot: int = 8
