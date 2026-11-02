import time

from test_rgbd_coordinator import capture

from senior_control.vision.independent_motion import Profile
from senior_control.vision.rgbd_producer_engine import RgbdProducerEngine
from senior_control.vision.static_tf_binding import Edge, Receipt, StaticBinding


def test_four_streams_produce_compact_candidate_through_real_spawn():
    binding = StaticBinding("base_link", "optical")
    edge = Edge("base_link", "optical", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert binding.observe(
        Receipt("tfboot", "03", 1, 1, 0, 0.0, (edge,), "04"),
        current_gid="03",
        current_relay_gid="04",
    )
    assert binding.admit(1_000_000_000, 1.0)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    engine = RgbdProducerEngine(
        profile,
        binding,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
        generation=1,
        sequence_floor=0,
    )
    try:
        for sequence in (1, 2):
            headers, rgb, depth, rgb_info, depth_info = capture(sequence)
            for stream, body in enumerate((rgb, depth, rgb_info, depth_info)):
                engine.offer(
                    stream,
                    headers[stream],
                    body,
                    bridge_gid="02",
                    now_ros_ns=1_200_000_000,
                    now_steady=1.2,
                )
            deadline = time.monotonic() + 10
            result = None
            while result is None and time.monotonic() < deadline:
                result = engine.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
                time.sleep(0.01)
            assert result is not None
        assert result.reason == "candidate"
        assert not result.hard_error_qualified
        assert result.image_layouts[0][0].encoding == "rgb8"
    finally:
        engine.close()


def test_invalid_capture_status_revokes_then_old_generation_is_ignored():
    from dataclasses import replace

    binding = StaticBinding("base_link", "optical")
    edge = Edge("base_link", "optical", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert binding.observe(
        Receipt("tfboot", "03", 1, 1, 0, 0.0, (edge,), "04"),
        current_gid="03",
        current_relay_gid="04",
    )
    assert binding.admit(1_000_000_000, 1.0)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    engine = RgbdProducerEngine(
        profile,
        binding,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
        generation=1,
        sequence_floor=0,
    )
    try:
        headers, rgb, depth, rgb_info, depth_info = capture(1)
        for stream, body in enumerate((rgb, depth, rgb_info, depth_info)):
            engine.offer(
                stream,
                headers[stream],
                body,
                bridge_gid="02",
                now_ros_ns=1_200_000_000,
                now_steady=1.2,
            )
        invalid = replace(
            headers[0],
            valid=False,
            capture_generation=2,
            sequence=2,
            stamp_ns=0,
            received_ros_ns=0,
            frame_id="",
            raw_gid="",
            source_epoch="",
        )
        engine.offer(0, invalid, None, bridge_gid="02", now_ros_ns=1_200_000_000, now_steady=1.2)
        result = engine.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
        assert result is not None and result.reason == "capture_unavailable"
        generation = engine.coordinator.worker.generation
        engine.offer(0, headers[0], rgb, bridge_gid="02", now_ros_ns=1_200_000_000, now_steady=1.2)
        assert engine.coordinator.worker.generation == generation
    finally:
        engine.close()
