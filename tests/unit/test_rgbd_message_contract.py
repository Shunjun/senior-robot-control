from pathlib import Path

MESSAGES = Path(__file__).resolve().parents[2] / "ros2/src/senior_motion_msgs/msg"


def test_tf_edge_uses_bounded_frame_names_and_fixed_pose():
    schema = (MESSAGES / "StaticTransformEdge.msg").read_text()
    assert "string<=128 parent_frame" in schema
    assert "string<=128 child_frame" in schema
    assert "float64[3] translation" in schema
    assert "float64[4] rotation_xyzw" in schema
    assert "builtin_interfaces/Time stamp" in schema


def test_candidate_retains_eight_headers_and_static_binding():
    schema = (MESSAGES / "RgbdMotionCandidate.msg").read_text()
    for prefix in ("first", "second"):
        for stream in ("rgb", "depth", "rgb_info", "depth_info"):
            assert f"RawCaptureHeader {prefix}_{stream}" in schema
        assert f"BoundedStaticTf {prefix}_tf" in schema
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
