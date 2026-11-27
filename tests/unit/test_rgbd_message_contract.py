from pathlib import Path

MESSAGES = Path(__file__).resolve().parents[2] / "ros2/src/senior_interfaces/msg"


def test_candidate_retains_eight_headers_and_static_binding():
    schema = (MESSAGES / "RgbdMotionCandidate.msg").read_text()
    for prefix in ("first", "second"):
        for stream in ("rgb", "depth", "rgb_info", "depth_info"):
            assert f"RawCaptureHeader {prefix}_{stream}" in schema
        assert f"tf2_msgs/TFMessage {prefix}_tf" in schema
    assert "bool hard_error_qualified" in schema
    assert "string<=64 preprocessing_version" in schema


def test_candidate_retains_full_calibration_inputs():
    schema = (MESSAGES / "RgbdMotionCandidate.msg").read_text()
    for prefix in ("first", "second"):
        for stream in ("rgb", "depth"):
            assert f"BoundedRawCameraInfo {prefix}_{stream}_calibration" in schema


def test_candidate_retains_all_original_image_layouts():
    schema = (MESSAGES / "RgbdMotionCandidate.msg").read_text()
    for prefix in ("first", "second"):
        for stream in ("rgb", "depth"):
            assert f"RawImageLayout {prefix}_{stream}_layout" in schema
    layout = (MESSAGES / "RawImageLayout.msg").read_text()
    assert "uint32 step" in layout and "bool is_bigendian" in layout
    assert "string<=32 encoding" in layout and "uint32 byte_length" in layout


def test_image_layout_carries_fixed_full_body_digest():
    layout = (MESSAGES / "RawImageLayout.msg").read_text()
    assert "uint8[32] sha256" in layout
