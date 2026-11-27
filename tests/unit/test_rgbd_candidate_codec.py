from types import SimpleNamespace as NS

from senior_perception.vision.raw_capture import RawCaptureHeader
from senior_perception.vision.rgbd_candidate_codec import fill_candidate
from senior_perception.vision.rgbd_frame_builder import CameraInfoSnapshot, ImageLayoutSnapshot
from senior_perception.vision.rgbd_spawn_worker import CompactResult
from senior_perception.vision.static_tf_binding import Edge, TransformSnapshot
from sensor_time_test_support import sample_time


def stamp():
    return NS(sec=0, nanosec=0)


def header():
    return NS(stamp=stamp(), received_ros=stamp())


def wire_message():
    result = NS(interval_start=stamp(), interval_end=stamp())
    for prefix in ("first", "second"):
        for stream in ("rgb", "depth", "rgb_info", "depth_info"):
            setattr(result, f"{prefix}_{stream}", header())
        for stream in ("rgb", "depth"):
            setattr(result, f"{prefix}_{stream}_layout", NS(capture=header()))
            setattr(result, f"{prefix}_{stream}_calibration", NS(capture=header()))
        setattr(
            result,
            f"{prefix}_tf",
            NS(transforms=[]),
        )
    return result


def test_candidate_serialization_retains_original_headers_digests_and_actual_tf():
    one = RawCaptureHeader(
        sample_time(stamp_ns=100, received_ros_ns=101, received_steady_ns=round((1.1) * 1e9)),
        "camera",
        "01",
        "02",
        "source",
        "boot",
        1,
        1,
        True,
        "",
    )
    two = RawCaptureHeader(
        sample_time(stamp_ns=200, received_ros_ns=201, received_steady_ns=round((1.2) * 1e9)),
        "camera",
        "01",
        "02",
        "source",
        "boot",
        1,
        2,
        True,
        "",
    )

    def info(h):
        return CameraInfoSnapshot(
            h,
            80,
            80,
            "plumb_bob",
            (0.0,) * 5,
            (1.0,) * 9,
            (1.0,) * 9,
            (1.0,) * 12,
            0,
            0,
            (0, 0, 0, 0),
            False,
        )

    def layout(h, encoding, step):
        return ImageLayoutSnapshot(h, 80, 80, step, encoding, False, 80 * step, b"x" * 32)

    edge = Edge("base_link", "camera", 0, (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    tf = TransformSnapshot((edge,))
    candidate = CompactResult(
        1,
        7,
        "candidate",
        ((one,) * 4, (two,) * 4),
        ((1.0, 0.0, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        50,
        30,
        0.1,
        0.01,
        profile_name="sim",
        calibration_epoch="cal",
        registration="registered",
        raw_rgb_encodings=("rgb8", "rgb8"),
        preprocessing_versions=("rgb8-mono-v1", "rgb8-mono-v1"),
        image_layouts=(
            (layout(one, "rgb8", 240), layout(one, "32FC1", 320)),
            (layout(two, "rgb8", 240), layout(two, "32FC1", 320)),
        ),
        camera_info=((info(one), info(one)), (info(two), info(two))),
        tf_receipts=(tf, tf),
    )
    out = wire_message()
    fill_candidate(
        out,
        candidate,
        producer_gid="05",
        producer_boot="producer",
        owner_generation=1,
        output_sequence=3,
        body_frame="base_link",
        edge_factory=lambda: NS(
            header=NS(stamp=stamp()), transform=NS(translation=NS(), rotation=NS())
        ),
    )
    assert out.valid and not out.hard_error_qualified
    assert out.first_rgb.stamp.nanosec == 100 and out.second_depth.sequence == 2
    assert bytes(out.first_rgb_layout.sha256) == b"x" * 32
    actual = out.first_tf.transforms[0]
    assert actual.header.stamp.sec == 0 and actual.header.frame_id == "base_link"
    assert actual.child_frame_id == "camera"
    assert (
        actual.transform.translation.x,
        actual.transform.translation.y,
        actual.transform.translation.z,
    ) == (1.0, 2.0, 3.0)


def test_candidate_with_missing_original_metadata_is_not_serialized_as_valid():
    import pytest

    message = wire_message()
    result = CompactResult(1, 1, "candidate", (), None, 0, 0, None, None)
    with pytest.raises(ValueError):
        fill_candidate(
            message,
            result,
            producer_gid="05",
            producer_boot="producer",
            owner_generation=1,
            output_sequence=1,
            body_frame="base_link",
            edge_factory=lambda: NS(
                header=NS(stamp=stamp()), transform=NS(translation=NS(), rotation=NS())
            ),
        )
    assert not message.valid


def test_original_source_nanoseconds_survive_python_ros_copy_after_a_long_boot():
    from types import SimpleNamespace

    from senior_perception.vision.rgbd_candidate_codec import _header

    from vehicle_core.sample_time import SampleTime

    exact = 10_000_000_000_000_001
    time = SampleTime(100, 102, exact + 2, exact, 7)
    capture = RawCaptureHeader(time, "camera", "01", "02", "source", "boot", 1, 1, True, "")
    out = SimpleNamespace(stamp=SimpleNamespace(), received_ros=SimpleNamespace())
    _header(out, capture)
    assert out.sample_steady_ns == exact
    assert out.received_steady_ns == exact + 2
    assert out.clock_generation == 7
