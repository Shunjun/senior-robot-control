from types import SimpleNamespace as NS

from senior_control.vision.raw_capture import RawCaptureHeader
from senior_control.vision.rgbd_candidate_codec import fill_candidate
from senior_control.vision.rgbd_frame_builder import CameraInfoSnapshot, ImageLayoutSnapshot
from senior_control.vision.rgbd_spawn_worker import CompactResult
from senior_control.vision.static_tf_binding import Edge, Receipt


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
            NS(
                received_ros=stamp(),
                first_received_ros=stamp(),
                admitted_from_ros=stamp(),
                edges=[],
            ),
        )
    return result


def test_candidate_serialization_retains_original_headers_digests_and_static_receipt():
    one = RawCaptureHeader(100, 101, 1.1, "camera", "01", "02", "source", "boot", 1, 1, True, "")
    two = RawCaptureHeader(200, 201, 1.2, "camera", "01", "02", "source", "boot", 1, 2, True, "")

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
    tf = Receipt("tfboot", "03", 1, 1, 0, 0.0, (edge,), "04")
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
        tf_admitted_from=((50, 0.5), (50, 0.5)),
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
        edge_factory=lambda: NS(stamp=stamp()),
    )
    assert out.valid and not out.hard_error_qualified
    assert out.first_rgb.stamp.nanosec == 100 and out.second_depth.sequence == 2
    assert bytes(out.first_rgb_layout.sha256) == b"x" * 32
    assert out.first_tf.first_received_ros.sec == 0 and out.first_tf.first_sequence == 1
    assert out.first_tf.admitted_from_ros.nanosec == 50
    assert out.first_tf.edges[0].translation == [1.0, 2.0, 3.0]


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
            edge_factory=lambda: NS(stamp=stamp()),
        )
    assert not message.valid
