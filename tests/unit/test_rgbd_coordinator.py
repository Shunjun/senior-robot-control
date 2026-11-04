import time
from types import SimpleNamespace as NS

import numpy as np
from test_independent_motion import scene

from senior_control.vision.independent_motion import Profile
from senior_control.vision.raw_capture import RawCaptureHeader
from senior_control.vision.rgbd_coordinator import RgbdCoordinator
from senior_control.vision.static_tf_binding import Edge, Receipt, StaticBinding


def capture(seq, shift=0):
    mono, z = scene(shift)
    h, w = mono.shape
    rgb = np.repeat(mono[:, :, None], 3, axis=2)
    stamp = 1_000_000_000 + seq * 100_000_000
    header = RawCaptureHeader(
        stamp, stamp, 1.0 + seq * 0.1, "optical", "01", "02", "source", "boot", 1, seq, True, ""
    )
    image = NS(
        capture=header,
        width=w,
        height=h,
        step=w * 3,
        encoding="rgb8",
        is_bigendian=False,
        data=rgb.tobytes(),
    )
    depth = NS(
        capture=header,
        width=w,
        height=h,
        step=w * 4,
        encoding="32FC1",
        is_bigendian=False,
        data=z.astype("<f4").tobytes(),
    )
    info = NS(
        capture=header,
        width=w,
        height=h,
        distortion_model="plumb_bob",
        d=(0.0,) * 5,
        k=(250.0, 0.0, 160.0, 0.0, 250.0, 120.0, 0.0, 0.0, 1.0),
        r=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        p=(250.0, 0.0, 160.0, 0.0, 0.0, 250.0, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        binning_x=0,
        binning_y=0,
        roi=(0, 0, 0, 0),
        roi_do_rectify=False,
    )
    return (header,) * 4, image, depth, info, info


def test_candidate_requires_current_tf_and_retained_pair():
    binding = StaticBinding("base_link", "optical")
    edge = Edge("base_link", "optical", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert binding.observe(
        Receipt("tfboot", "03", 1, 1, 0, 0.0, (edge,), "04"),
        current_gid="03",
        current_relay_gid="04",
    )
    assert binding.admit(1_000_000_000, 1.0)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    coordinator = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
    )
    try:
        for seq in (1, 2):
            assert coordinator.offer(
                *capture(seq),
                bridges=("02",) * 4,
                exclusion=bytes(320 * 240),
                now_ros_ns=1_200_000_000,
                now_steady=1.2,
            )
            deadline = time.monotonic() + 10
            result = None
            while result is None and time.monotonic() < deadline:
                result = coordinator.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
                time.sleep(0.01)
            assert result is not None
        assert result.reason == "candidate"
        assert (result.profile_name, result.calibration_epoch, result.registration) == (
            "sim",
            "cal",
            "registered",
        )
        assert result.raw_rgb_encodings == ("rgb8", "rgb8")
        assert result.preprocessing_versions == ("rgb8-mono-v1", "rgb8-mono-v1")
        assert len(result.image_layouts) == 2
        assert result.image_layouts[0][0].step == 960
        assert result.image_layouts[1][1].encoding == "32FC1"
        assert len(result.camera_info) == 2
        assert result.camera_info[0][0].k[0] == 250.0
        assert result.camera_info[1][1].capture == capture(2)[0][3]
        assert result.tf_receipts == (binding.first_receipt, binding.first_receipt)
        assert result.tf_admitted_from == ((1_000_000_000, 1.0),) * 2
        assert not result.hard_error_qualified
    finally:
        coordinator.close()


def test_tf_loss_publishes_explicit_invalid_and_revokes_inflight():
    binding = StaticBinding("base_link", "optical")
    edge = Edge("base_link", "optical", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert binding.observe(
        Receipt("tfboot", "03", 1, 1, 0, 0.0, (edge,), "04"),
        current_gid="03",
        current_relay_gid="04",
    )
    assert binding.admit(1_000_000_000, 1.0)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    coordinator = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
    )
    try:
        assert coordinator.offer(
            *capture(1),
            bridges=("02",) * 4,
            exclusion=bytes(320 * 240),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        binding.revoke()
        result = coordinator.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
        assert result is not None and result.reason == "static_tf_unavailable"
    finally:
        coordinator.close()


def test_invalid_frame_reports_invalid_after_consuming_its_authenticated_headers():
    binding = StaticBinding("base_link", "optical")
    edge = Edge("base_link", "optical", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert binding.observe(
        Receipt("tfboot", "03", 1, 1, 0, 0.0, (edge,), "04"),
        current_gid="03",
        current_relay_gid="04",
    )
    assert binding.admit(1_000_000_000, 1.0)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    coordinator = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
    )
    try:
        headers, image, depth, rgb_info, depth_info = capture(1)
        image.step = 1
        assert not coordinator.offer(
            headers,
            image,
            depth,
            rgb_info,
            depth_info,
            bridges=("02",) * 4,
            exclusion=bytes(320 * 240),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        invalid = coordinator.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
        assert invalid is not None and invalid.reason == "invalid_frame"
        image.step = 960
        assert not coordinator.offer(
            headers,
            image,
            depth,
            rgb_info,
            depth_info,
            bridges=("02",) * 4,
            exclusion=bytes(320 * 240),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
    finally:
        coordinator.close()


def test_evicted_pair_is_explicitly_dropped_without_epoch_reset():
    binding = StaticBinding("base_link", "optical")
    edge = Edge("base_link", "optical", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert binding.observe(
        Receipt("tfboot", "03", 1, 1, 0, 0.0, (edge,), "04"),
        current_gid="03",
        current_relay_gid="04",
    )
    assert binding.admit(1_000_000_000, 1.0)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    coordinator = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
        capacity=1,
    )
    try:
        for seq in (1, 2):
            assert coordinator.offer(
                *capture(seq),
                bridges=("02",) * 4,
                exclusion=bytes(320 * 240),
                now_ros_ns=1_200_000_000,
                now_steady=1.2,
            )
            deadline = time.monotonic() + 10
            result = None
            while result is None and time.monotonic() < deadline:
                result = coordinator.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
                time.sleep(0.01)
            assert result is not None
        assert result.reason == "dependency_unavailable"
        assert coordinator.worker.generation == 1
    finally:
        coordinator.close()


def test_retired_camera_packet_does_not_invalidate_successor_work():
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
    coordinator = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
    )
    try:
        assert coordinator.offer(
            *capture(1),
            bridges=("02",) * 4,
            exclusion=bytes(320 * 240),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        headers, image, depth, rgb_info, depth_info = capture(2)
        successor = replace(headers[0], raw_gid="05", source_epoch="new", capture_generation=2)
        for body in (image, depth, rgb_info, depth_info):
            body.capture = successor
        assert coordinator.offer(
            (successor,) * 4,
            image,
            depth,
            rgb_info,
            depth_info,
            bridges=("02",) * 4,
            exclusion=bytes(320 * 240),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        invalid = coordinator.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
        assert invalid is not None and invalid.reason == "capture_source_changed"
        generation = coordinator.worker.generation
        assert not coordinator.offer(
            *capture(1),
            bridges=("02",) * 4,
            exclusion=bytes(320 * 240),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        assert coordinator.worker.generation == generation
    finally:
        coordinator.close()


def test_clock_rollback_immediately_revokes_inflight_candidate():
    binding = StaticBinding("base_link", "optical")
    edge = Edge("base_link", "optical", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert binding.observe(
        Receipt("tfboot", "03", 1, 1, 0, 0.0, (edge,), "04"),
        current_gid="03",
        current_relay_gid="04",
    )
    assert binding.admit(1_000_000_000, 1.0)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    coordinator = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
    )
    try:
        assert coordinator.offer(
            *capture(1),
            bridges=("02",) * 4,
            exclusion=bytes(320 * 240),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        result = coordinator.poll(now_ros_ns=1_100_000_000, now_steady=1.21)
        assert result is not None and result.reason == "clock_invalid"
    finally:
        coordinator.close()


def test_profile_cannot_change_under_running_worker():
    import pytest

    binding = StaticBinding("base_link", "optical")
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    coordinator = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
    )
    try:
        with pytest.raises(AttributeError):
            coordinator.profile = profile
    finally:
        coordinator.close()


def test_registration_cannot_change_under_running_worker():
    import pytest

    binding = StaticBinding("base_link", "optical")
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    coordinator = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
    )
    try:
        with pytest.raises(AttributeError):
            coordinator.registration = "different"
    finally:
        coordinator.close()


def test_binding_object_cannot_be_swapped_under_running_worker():
    import pytest

    binding = StaticBinding("base_link", "optical")
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    coordinator = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
    )
    try:
        with pytest.raises(AttributeError):
            coordinator.binding = StaticBinding("base_link", "optical")
    finally:
        coordinator.close()


def test_candidate_with_wrong_tf_dependency_is_rejected():
    from senior_control.vision.rgbd_spawn_worker import CompactResult

    binding = StaticBinding("base_link", "optical")
    edge = Edge("base_link", "optical", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert binding.observe(
        Receipt("tfboot", "03", 1, 1, 0, 0.0, (edge,), "04"),
        current_gid="03",
        current_relay_gid="04",
    )
    assert binding.admit(1_000_000_000, 1.0)
    profile = Profile("sim", "fixture", True, "texture", "cal", 0.4, 0.4, 2.0, 0.1, 0.1)
    c = RgbdCoordinator(
        profile,
        binding,
        generation=1,
        sequence_floor=0,
        registration="registered",
        fresh_after_ros_ns=1_000_000_000,
        fresh_after_steady=1.0,
    )
    try:
        first = capture(1)
        second = capture(2)
        assert c.offer(
            *first,
            bridges=("02",) * 4,
            exclusion=bytes(320 * 240),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        deadline = time.monotonic() + 5
        result = None
        while result is None and time.monotonic() < deadline:
            result = c.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None
        assert c.offer(
            *second,
            bridges=("02",) * 4,
            exclusion=bytes(320 * 240),
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        identity = (
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        )
        fake = CompactResult(
            1,
            2,
            "candidate",
            (first[0], second[0]),
            identity,
            2,
            1,
            0.1,
            0.1,
            ("wrong", "wrong"),
            (("wrong",), ("wrong",)),
        )
        c.worker._output.put(fake)
        deadline = time.monotonic() + 5
        result = None
        while result is None and time.monotonic() < deadline:
            result = c.poll(now_ros_ns=1_200_000_000, now_steady=1.2)
            time.sleep(0.01)
        assert result is not None and result.reason == "tf_lifetime_invalid"
    finally:
        c.close()
