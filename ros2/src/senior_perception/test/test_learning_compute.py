"""Direct stream/window boundaries around the actual serial model kernel."""

import math

import pytest
from learning_test_support import TestRun, geometry
from senior_perception.learning_compute import ComputeBatch, LearningKernel
from senior_perception.learning_wire import FrameBatch, MotionObservedThrough, RecordedFrame
from sensor_time_test_support import sample_time
from test_startup_learning_feasibility import BOUNDARY

from senior_control.vision.raw_capture import RawCaptureHeader


def recorded(index, stamp, *, received=None):
    ns = round(stamp * 1e9)
    ranges = [2.0] * 72
    ranges[18] = 0.15
    ranges[35] = math.inf
    return RecordedFrame(
        index=index,
        recorded_steady_ns=round((stamp if received is None else received) * 1e9),
        capture=RawCaptureHeader(
            time=sample_time(stamp_ns=ns, received_ros_ns=ns, received_steady_ns=ns),
            frame_id="laser_link",
            raw_gid="01",
            bridge_gid="02",
            source_epoch="raw",
            bridge_boot_id="boot",
            capture_generation=1,
            sequence=index,
            valid=True,
            reason="",
        ),
        geometry=geometry(BOUNDARY, count=72, step=math.tau / 72, range_max=20.0),
        transform_stamp=stamp,
        ranges=tuple("inf" if value == math.inf else value for value in ranges),
    )


def send(run, frames, through=None):
    for offset in range(0, len(frames), 4):
        part = frames[offset : offset + 4]
        run.service._accept_frames(
            FrameBatch(
                attempt_id=run.owner.record.attempt_id,
                stream_id=run.stream_id,
                frames=tuple(part),
                through_steady_ns=part[-1].recorded_steady_ns
                if through is None
                else round(through * 1e9),
            )
        )


def opened(run):
    session, _ = run.owner.begin_consumer()
    run.owner.reserve_action(session)
    run.pump()
    run.owner.report_action(session, 10_000_000_000, 3_000_000_000, finished=True)
    run.owner.set_phase(session, "validating", "post_motion_validation", "3" * 32)
    run.service.validation_after = 13_000_000_000
    run.service.compute.held = True
    return session


def test_kernel_keeps_complete_raw_identity_and_original_nonfinite_values():
    frames = tuple(recorded(i, 10 + i / 2) for i in range(1, 6))
    decoded = FrameBatch.model_validate_json(
        FrameBatch(
            attempt_id="a" * 32,
            stream_id="b" * 32,
            frames=frames[:4],
            through_steady_ns=11_000_000_000,
        ).model_dump_json()
    )
    assert decoded.frames == frames[:4]
    kernel = LearningKernel()
    for offset in range(0, len(frames), 4):
        result = kernel.update(
            ComputeBatch(
                "a" * 32, "c" * 32, frames[0].geometry, frames[offset : offset + 4], offset == 4
            )
        )
        if offset == 0:
            assert not result.finished and result.points is None and not kernel.model.learned
    # The isolated interior hit cannot connect a separate far wall just because
    # adjacent scan indices also contain valid hits.
    assert result.finished and result.processed == 5 and len(result.points) == 1
    assert [observation.scan.stamp for observation in kernel.observations] == [
        frame.capture.stamp_ns / 1e9 for frame in frames
    ]
    assert all(
        observation.scan.ranges[35] == math.inf and observation.rays[35].kind == "invalid"
        for observation in kernel.observations
    )
    assert all(frame.ranges[35] == "inf" for frame in frames)


def test_cancel_arrives_before_previously_recorded_final_batch_still_saves(tmp_path):
    run = TestRun(tmp_path / "cancel-before-batch")
    try:
        session = opened(run)
        motion = [recorded(i, 10 + i / 10) for i in range(1, 25)]
        validation = [recorded(25 + i, 13.1 + i / 10) for i in range(5)]
        send(run, motion + validation[:4])
        run.rpc(
            MotionObservedThrough(
                stationary=True,
                session=session,
                attempt_id=run.owner.record.attempt_id,
                segment_id="3" * 32,
                through_steady_ns=13_600_000_000,
                validation_after_steady_ns=13_000_000_000,
                first_takeover_ns=13_600_000_000,
                reason="control_takeover",
            )
        )
        assert run.owner.record.capture_summary.state == "open"
        send(run, validation[4:], through=13.7)
        assert run.owner.record.capture_summary.state == "complete"
        action = run.owner.record.action
        run.service.compute.held = False
        for _ in range(10):
            run.pump()
        assert run.owner.active_record.model is not None
        assert run.owner.record.action == action
    finally:
        run.close()


def test_batch_before_cancel_cannot_use_frame_recorded_after_takeover(tmp_path):
    run = TestRun(tmp_path / "batch-before-cancel")
    try:
        session = opened(run)
        motion = [recorded(i, 10 + i / 10) for i in range(1, 25)]
        validation = [recorded(25 + i, 13.1 + i / 10) for i in range(5)]
        validation[-1] = validation[-1].model_copy(update={"recorded_steady_ns": 13_700_000_000})
        run.service.motion_through = 13_400_000_000
        send(run, motion + validation, through=13.9)
        assert run.owner.record.capture_summary.state == "open"
        run.rpc(
            MotionObservedThrough(
                stationary=True,
                session=session,
                attempt_id=run.owner.record.attempt_id,
                segment_id="3" * 32,
                through_steady_ns=13_800_000_000,
                validation_after_steady_ns=13_000_000_000,
                first_takeover_ns=13_600_000_000,
                reason="control_takeover",
            )
        )
        run.pump()
        assert run.owner.record.capture_summary.state == "incomplete"
        assert run.owner.record.phase == "failed" and run.owner.record.model is None
        assert run.owner.record.action.consumed
    finally:
        run.close()


def test_actual_service_fifo_overflow_cannot_publish_a_model(tmp_path):
    run = TestRun(tmp_path / "overflow")
    try:
        opened(run)
        send(run, [recorded(i, 10 + i / 1000) for i in range(1, 130)])
        run.service.compute.held = False
        run.pump()
        assert run.service.gap
        assert run.owner.record.capture_summary.state == "incomplete"
        assert run.owner.record.phase == "failed" and run.owner.active_record is None
        assert run.owner.record.action.consumed
    finally:
        run.close()


def test_actual_tf_metadata_keeps_stream_open_and_kernel_uses_each_transform(tmp_path):
    from senior_control.models import Pose2D
    from senior_control.safety.self_filter import ScanObservation, filter_self

    first = recorded(1, 10.1)
    first = first.model_copy(
        update={
            "geometry": first.geometry.model_copy(
                update={
                    "sensor_to_body": (0.15699, 0.00201, 0.0),
                }
            )
        }
    )
    second = recorded(2, 10.2)
    second = second.model_copy(
        update={
            "geometry": first.geometry.model_copy(
                update={
                    "sensor_to_body": (
                        0.1569900000000001,
                        0.0020100000000000022,
                        -6.001611513070753e-19,
                    ),
                }
            )
        }
    )
    run = TestRun(tmp_path / "actual-tf")
    try:
        send(run, [first, second])
        run.pump()
        assert run.service.accepted_frames == 2 and not run.service.gap
        assert run.owner.record.capture_summary.state == "open"
        kernel = LearningKernel()
        for frame in (first, second):
            result = kernel.update(
                ComputeBatch("a" * 32, "c" * 32, frame.geometry, (frame,), False)
            )
            expected = filter_self(
                ScanObservation(
                    tuple(float(value) for value in frame.ranges),
                    frame.geometry.angle_min,
                    frame.geometry.angle_increment,
                    frame.geometry.range_min,
                    frame.geometry.range_max,
                    frame.capture.stamp_ns / 1e9,
                    "ros",
                    Pose2D(*frame.geometry.sensor_to_body),
                    frame.transform_stamp,
                    frame.geometry.body_frame,
                    frame.capture.stamp_ns / 1e9,
                ),
                kernel.profile,
            )
            assert kernel.observations[-1] == expected
        assert result.processed == 2
        assert kernel.observations[0] != kernel.observations[1]
        assert first.geometry.sensor_to_body != second.geometry.sensor_to_body

        changed = recorded(3, 10.3).model_copy(
            update={
                "geometry": second.geometry.model_copy(
                    update={"angle_increment": second.geometry.angle_increment * 2}
                )
            }
        )
        send(run, [changed])
        run.pump()
        assert run.service.gap and run.owner.record.capture_summary.state == "incomplete"
        with pytest.raises(ValueError, match="learning_geometry_changed"):
            kernel.update(ComputeBatch("a" * 32, "c" * 32, changed.geometry, (changed,), False))
        body_change = second.geometry.model_copy(update={"body_frame": "other_body"})
        with pytest.raises(ValueError, match="learning_geometry_changed"):
            kernel.update(
                ComputeBatch(
                    "a" * 32,
                    "c" * 32,
                    second.geometry,
                    (changed.model_copy(update={"geometry": body_change}),),
                    False,
                )
            )
    finally:
        run.close()
