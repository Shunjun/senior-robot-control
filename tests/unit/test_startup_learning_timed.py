"""The action is a fixed host window; learning compares relative scan positions."""

from dataclasses import replace

from learning_test_support import begin_action, learning_runs, startup_for  # noqa: F401
from sensor_time_test_support import sample_time
from test_startup_learning import side_observation
from test_startup_learning_feasibility import BOUNDARY

from senior_control.safety.self_learning import SelfReturnModel
from senior_control.vision.raw_capture import RawCaptureHeader


def test_fixed_three_seconds_never_waits_for_exact_displacement(tmp_path):
    owner = startup_for(tmp_path / "timed")
    begin_action(owner, 100.0)
    owner.advance(0.0, now=102.9, stationary=False)
    assert owner.velocity == 0.05
    owner.advance(0.0, now=103.0, stationary=False)
    assert owner.velocity == 0 and owner.state == "braking"
    assert owner.started == 100.0 and owner.deadline == 103.0
    owner.advance(0.0, now=1000.0, stationary=True)
    assert owner.velocity == 0 and owner.state != "completed"
    assert owner.session.record.action.consumed


def test_latched_wait_does_not_consume_action_without_final_nonzero_selection(tmp_path):
    owner = startup_for(tmp_path / "waiting")
    owner.state = "moving"
    for t in (10.0, 50.0, 1000.0):
        owner.advance(0.0, now=t, stationary=True)
    assert owner.started is None and not owner.session.record.action.consumed
    assert owner.prepare_action()
    assert owner.session.record.action.consumed


def test_brief_reverse_feedback_does_not_repeat_or_extend_collection(tmp_path):
    owner = startup_for(tmp_path / "turn")
    begin_action(owner, 10.0)
    for i in range(1, 20):
        wheel = -0.02 if i == 8 else 0.05
        owner.advance(wheel, now=10 + i * 0.1, stationary=False)
    assert owner.active and owner.state == "moving"
    assert owner.started == 10.0 and owner.deadline == 13.0


def test_same_run_reconstruction_does_not_replay_consumed_window(tmp_path):
    path = tmp_path / "resume"
    first = startup_for(path)
    begin_action(first, 10.0)
    first.session.close()
    restored = startup_for(path)
    restored.advance(0.0, now=10.5, stationary=True)
    assert restored.velocity == 0 and restored.motion_withdrawn
    assert not restored.prepare_action()


def test_reliable_unmoved_record_can_continue_first_learning(tmp_path):
    path = tmp_path / "unmoved"
    first = startup_for(path)
    first.session.close()
    restored = startup_for(path)
    restored.advance(0.0, now=10.0, stationary=True)
    assert restored.state == "sampling"
    restored.state = "moving"
    assert restored.prepare_action()


def test_real_monotonic_regression_still_stops_action(tmp_path):
    owner = startup_for(tmp_path / "regression")
    begin_action(owner, 10.0)
    owner.advance(0.0, now=9.0, stationary=True)
    assert owner.velocity == 0 and owner.reason == "learning_monotonic_clock_regressed"


def test_original_stamp_and_source_identity_order_and_deduplicate_learning_frames():
    model = SelfReturnModel(BOUNDARY)
    for sequence, _ros in enumerate((1000.0, 900.0, 10000.0, 1.0, 2.0), 1):
        observation = side_observation(0.15)
        observation = replace(
            observation, scan=replace(observation.scan, stamp=10 + sequence * 0.1)
        )
        model.observe(
            observation,
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round((10 + sequence * 0.1) * 1e9),
                    received_ros_ns=round((10 + sequence * 0.1) * 1e9),
                    received_steady_ns=round((10 + sequence * 0.1) * 1e9),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=sequence,
                valid=True,
                reason="",
            ),
            segment_id="segment",
        )
    assert [m / 1e9 for m in model.frame_measurements] == [10.1, 10.2, 10.3, 10.4, 10.5]
    proof = RawCaptureHeader(
        time=sample_time(
            stamp_ns=round((10.3) * 1e9),
            received_ros_ns=round((10.3) * 1e9),
            received_steady_ns=round((10.3) * 1e9),
            clock_generation=1,
        ),
        frame_id="laser",
        raw_gid="01",
        bridge_gid="02",
        source_epoch="producer",
        bridge_boot_id="boot",
        capture_generation=1,
        sequence=3,
        valid=True,
        reason="",
    )
    assert not model.observe(side_observation(0.15), capture=proof, segment_id="segment")
    model.begin_segment("new-segment")
    assert not model.observe(side_observation(0.15), capture=proof, segment_id="segment")


def test_stable_attachment_classifies_without_displacement_or_turn_proof():
    model = SelfReturnModel(BOUNDARY)
    for i in range(12):
        model.observe(
            side_observation(0.15),
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round((10 + i * 0.1) * 1e9),
                    received_ros_ns=round((10 + i * 0.1) * 1e9),
                    received_steady_ns=round((10 + i * 0.1) * 1e9),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=i + 1,
                valid=True,
                reason="",
            ),
            segment_id="small-se2",
        )
    assert model.validate() == 1


def test_completed_result_restores_same_run_without_replaying_action(tmp_path):
    from learning_test_support import geometry

    path = tmp_path / "completed"
    first = startup_for(path)
    first.session.geometry(geometry(BOUNDARY))
    first.session.complete(((0.0, 0.15),))
    first.session.close()
    restored = startup_for(path)
    restored.advance(0.0, now=100.0, stationary=True)
    assert restored.state == "completed" and restored.velocity == 0
    assert restored.session.view.catalog.points == ((0.0, 0.15),)
    assert not restored.prepare_action()


def test_single_interior_echo_is_not_lost_by_external_point_subsampling():
    model = SelfReturnModel(BOUNDARY)
    for i in range(12):
        model.observe(
            side_observation(0.15),
            capture=RawCaptureHeader(
                time=sample_time(
                    stamp_ns=round((10 + i * 0.1) * 1e9),
                    received_ros_ns=round((10 + i * 0.1) * 1e9),
                    received_steady_ns=round((10 + i * 0.1) * 1e9),
                    clock_generation=1,
                ),
                frame_id="laser",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="producer",
                bridge_boot_id="boot",
                capture_generation=1,
                sequence=i + 1,
                valid=True,
                reason="",
            ),
            segment_id="segment",
        )
    assert model.validate() == 1
