from senior_control.models import Velocity
from senior_control.protocol import DriveFeedbackKey
from senior_control.safety.drive_feedback import OutputFeedback


def key(seq: int) -> DriveFeedbackKey:
    return DriveFeedbackKey(
        robot_id="car-a", session_id="page-a", connection_epoch=2,
        seq=seq, request_id=f"sample-{seq}", gesture=1,
    )


def test_reports_applied_only_after_safety_output() -> None:
    feedback = OutputFeedback()
    request = Velocity(0.2, 0.0)
    limited = Velocity(0.1, 0.0)
    assert feedback.note(key(1), request, 1.5) is None
    assert feedback.note(key(1), request, 1.5) is None
    assert feedback.published(Velocity(0.0, 0.0), 1.35, "evaluation_pending") is None
    applied = feedback.published(limited, 1.4, "limited")
    assert applied is not None
    assert (applied.status, applied.vx, applied.limited) == ("applied", 0.1, True)
    assert feedback.published(limited, 1.41, "limited") is None


def test_separates_blocked_replaced_and_expired() -> None:
    feedback = OutputFeedback()
    requested = Velocity(0.2, 0.0)
    zero = Velocity(0.0, 0.0)
    assert feedback.note(key(1), requested, 1.5) is None
    replaced = feedback.note(key(2), requested, 1.6)
    assert replaced is not None and replaced.status == "superseded"
    assert replaced.request_id == "sample-1"
    blocked = feedback.published(zero, 1.55, "scan_topology_unqualified")
    assert blocked is not None and blocked.status == "blocked"
    assert blocked.reason == "scan_topology_unqualified"
    assert feedback.note(key(3), requested, 1.7) is None
    expired = feedback.published(zero, 1.7, "command_expired")
    assert expired is not None and expired.status == "expired"


async def test_relay_routes_output_only_to_active_lease() -> None:
    import json

    from senior_control.cloud.limits import CloudLimits
    from senior_control.cloud.relay import Hub
    from senior_control.protocol import DriveExecutionFeedback

    hub = Hub(CloudLimits())
    agent = hub.attach_agent("car-a", epoch=2)
    agent.drive_timing = True
    owner = hub.attach_controller("car-a", "owner")
    observer = hub.attach_controller("car-a", "observer")
    assert owner is not None and observer is not None
    # Capability notices precede results.
    await owner.outbox.get()
    await observer.outbox.get()
    owner.lease_session = "page-a"
    result = DriveExecutionFeedback(
        **key(1).model_dump(), status="applied", vx=0.1, wz=0.0, limited=True
    )
    old = result.model_copy(update={"connection_epoch": 1})
    hub.handle_agent_frame(agent, old.model_dump_json(exclude_none=True).encode())
    assert owner.outbox.pending == observer.outbox.pending == 0
    hub.handle_agent_frame(agent, result.model_dump_json(exclude_none=True).encode())
    assert json.loads(await owner.outbox.get())["status"] == "applied"
    assert observer.outbox.pending == 0


def test_timed_sample_sequence_can_round_trip_through_browser_feedback() -> None:
    import pytest

    from senior_control.protocol import VelocityEnvelope, VelocityPayload

    with pytest.raises(ValueError, match="sequence"):
        VelocityEnvelope(
            v=1, type="drive.velocity", robot_id="car-a", session_id="page-a",
            connection_epoch=2, request_id="sample", seq=2**53, expires_at=0.0,
            payload=VelocityPayload(vx=0.0, wz=0.0, gesture=1, sampled_at=100.0),
        )
