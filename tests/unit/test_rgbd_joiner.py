from dataclasses import replace

from senior_control.vision.raw_capture import RawCaptureHeader
from senior_control.vision.rgbd_joiner import CaptureJoiner


def header(sequence, stamp, *, generation=1, valid=True):
    return RawCaptureHeader(
        stamp,
        stamp,
        stamp / 1e9,
        "camera",
        "01",
        "02",
        "source",
        "boot",
        generation,
        sequence,
        valid,
        "",
    )


def offer(join, stream, capture, body, *, bridge_gid):
    from types import SimpleNamespace as NS

    if type(body) is int:
        if stream < 2:
            encoding = "rgb8" if stream == 0 else "32FC1"
            step = 3 if stream == 0 else 4
            body = NS(
                capture=capture,
                width=1,
                height=1,
                step=step,
                encoding=encoding,
                is_bigendian=False,
                data=bytes([body]) * step,
            )
        else:
            body = NS(
                capture=capture,
                width=1,
                height=1,
                distortion_model="plumb_bob",
                d=[0.0] * 5,
                k=[1.0] * 9,
                r=[1.0] * 9,
                p=[1.0] * 12,
                binning_x=0,
                binning_y=0,
                roi=[0, 0, 0, 0],
                roi_do_rectify=False,
            )
    return join.offer(stream, capture, body, bridge_gid=bridge_gid)


def test_exact_four_stream_join_and_newer_frame_keeps_inflight_generation():
    join = CaptureJoiner(capacity=2)
    for stream in range(3):
        assert (
            offer(join, stream, header(1, 1_000_000_000), stream, bridge_gid="02").kind == "pending"
        )
    ready = offer(join, 3, header(1, 1_000_000_000), 3, bridge_gid="02")
    assert ready.kind == "ready" and len(ready.bodies) == 4
    generation = join.generation
    for stream in range(4):
        newer = offer(join, stream, header(2, 1_100_000_000), stream, bridge_gid="02")
    assert newer.kind == "ready" and join.generation == generation


def test_invalid_generation_revokes_and_old_generation_cannot_restore():
    join = CaptureJoiner(capacity=2)
    for stream in range(4):
        offer(join, stream, header(1, 1_000_000_000), stream, bridge_gid="02")
    invalid = replace(
        header(2, 0, generation=2, valid=False), frame_id="", raw_gid="", source_epoch=""
    )
    assert offer(join, 0, invalid, None, bridge_gid="02").kind == "invalid"
    generation = join.generation
    assert offer(join, 0, header(3, 1_100_000_000), 0, bridge_gid="02").kind == "ignored"
    assert join.generation == generation
    for stream in range(4):
        newer = offer(join, stream, header(3, 1_100_000_000, generation=2), stream, bridge_gid="02")
    assert newer.kind == "ready"


def test_repeated_invalid_status_same_generation_does_not_repeat_revocation():
    join = CaptureJoiner(capacity=2)
    for stream in range(4):
        offer(join, stream, header(1, 1_000_000_000), stream, bridge_gid="02")
    invalid = replace(
        header(2, 0, generation=2, valid=False), frame_id="", raw_gid="", source_epoch=""
    )
    assert offer(join, 0, invalid, None, bridge_gid="02").kind == "invalid"
    generation = join.generation
    assert offer(join, 1, invalid, None, bridge_gid="02").kind == "ignored"
    assert join.generation == generation


def test_valid_first_message_of_new_generation_revokes_but_remains_joinable():
    join = CaptureJoiner(capacity=2)
    for stream in range(4):
        offer(join, stream, header(1, 1_000_000_000), stream, bridge_gid="02")
    first = offer(join, 0, header(2, 1_100_000_000, generation=2), 0, bridge_gid="02")
    assert first.kind == "invalid"
    for stream in range(1, 4):
        result = offer(
            join, stream, header(2, 1_100_000_000, generation=2), stream, bridge_gid="02"
        )
    assert result.kind == "ready" and len(result.bodies) == 4


def test_old_bridge_callback_is_ignored_after_new_bridge_graph_identity():
    join = CaptureJoiner(capacity=2)
    for stream in range(4):
        offer(join, stream, header(1, 1_000_000_000), stream, bridge_gid="02")
    generation = join.generation
    assert offer(join, 0, header(2, 1_100_000_000), 0, bridge_gid="04").kind == "ignored"
    assert join.generation == generation


def test_old_pending_stamp_eviction_does_not_create_a_false_join():
    join = CaptureJoiner(capacity=1)
    assert offer(join, 0, header(1, 1_000_000_000), 0, bridge_gid="02").kind == "pending"
    assert offer(join, 0, header(2, 1_100_000_000), 0, bridge_gid="02").kind == "pending"
    for stream in range(1, 4):
        result = offer(join, stream, header(1, 1_000_000_000), stream, bridge_gid="02")
    assert result.kind != "ready"


def test_unjoined_stream_time_regression_immediately_invalidates():
    for field in ("stamp_ns", "received_ros_ns", "received_steady"):
        join = CaptureJoiner(capacity=4)
        for stream in range(4):
            ready = offer(join, stream, header(1, 1_000_000_000), stream, bridge_gid="02")
        assert ready.kind == "ready"
        before = join.generation
        newer = header(2, 1_200_000_000)
        assert offer(join, 0, newer, 0, bridge_gid="02").kind == "pending"
        values = {
            "stamp_ns": 1_300_000_000,
            "received_ros_ns": 1_300_000_000,
            "received_steady": 1.3,
        }
        values[field] = 1.1 if field == "received_steady" else 1_100_000_000
        older = replace(header(3, 1_300_000_000), **values)
        result = offer(join, 0, older, 0, bridge_gid="02")
        assert result.kind == "invalid", field
        assert join.generation > before
        for stream in range(1, 4):
            result = offer(join, stream, header(2, 1_200_000_000), stream, bridge_gid="02")
        assert result.kind != "ready"


def test_mutating_image_and_info_after_offer_cannot_change_joined_snapshot():
    from array import array
    from types import SimpleNamespace as NS

    h = header(1, 1_000_000_000)
    rgb = NS(
        capture=h,
        width=2,
        height=1,
        step=8,
        encoding="rgb8",
        is_bigendian=False,
        data=array("B", b"abcdefXY"),
    )
    depth = NS(
        capture=h,
        width=2,
        height=1,
        step=8,
        encoding="32FC1",
        is_bigendian=False,
        data=bytearray(8),
    )

    def info():
        return NS(
            capture=h,
            width=2,
            height=1,
            distortion_model="plumb_bob",
            d=[0.0] * 5,
            k=[1.0] * 9,
            r=[1.0] * 9,
            p=[1.0] * 12,
            binning_x=0,
            binning_y=0,
            roi=[0, 0, 0, 0],
            roi_do_rectify=False,
        )

    rgb_info, depth_info = info(), info()
    join = CaptureJoiner(capacity=2)
    for stream, body in enumerate((rgb, depth, rgb_info)):
        assert offer(join, stream, h, body, bridge_gid="02").kind == "pending"
    rgb.data[0] = ord("X")
    depth.data[0] = 9
    rgb_info.k[0] = 7.0
    rgb_info.capture = header(2, 1_100_000_000)
    result = offer(join, 3, h, depth_info, bridge_gid="02")
    assert result.kind == "ready"
    assert result.bodies[0].data == b"abcdefXY"
    assert result.bodies[1].data == bytes(8)
    assert result.bodies[2].k[0] == 1.0
    assert result.bodies[2].capture == h


def test_same_raw_generation_change_cannot_reset_time_frontier():
    join = CaptureJoiner(capacity=2)
    for stream in range(4):
        offer(join, stream, header(1, 1_000_000_000), stream, bridge_gid="02")
    assert offer(join, 0, header(2, 1_200_000_000), 0, bridge_gid="02").kind == "pending"
    invalid = replace(
        header(3, 0, generation=2, valid=False), frame_id="", raw_gid="", source_epoch=""
    )
    assert offer(join, 0, invalid, None, bridge_gid="02").kind == "invalid"
    result = offer(join, 0, header(4, 1_100_000_000, generation=2), 0, bridge_gid="02")
    assert result.kind == "invalid"


def test_info_numpy_vectors_are_copied_to_frozen_numeric_snapshot():
    import pytest

    np = pytest.importorskip("numpy")
    from types import SimpleNamespace as NS

    h = header(1, 1_000_000_000)
    values = np.array([float(i) / 10 for i in range(9)], dtype=np.float64)
    info = NS(
        capture=h,
        width=np.uint32(1),
        height=np.uint32(1),
        distortion_model="plumb_bob",
        d=np.zeros(5),
        k=values,
        r=np.eye(3).reshape(-1),
        p=np.zeros(12),
        binning_x=np.uint32(0),
        binning_y=np.uint32(0),
        roi=np.zeros(4, dtype=np.uint32),
        roi_do_rectify=False,
    )
    join = CaptureJoiner(capacity=2)
    assert offer(join, 2, h, info, bridge_gid="02").kind == "pending"
    values[0] = 4.0
    for stream in (0, 1, 3):
        result = offer(join, stream, h, stream, bridge_gid="02")
    assert result.kind == "ready"
    assert type(result.bodies[2].k) is tuple and result.bodies[2].k[0] == 0.0
    assert result.bodies[2].capture == h
    with pytest.raises((AttributeError, TypeError)):
        result.bodies[2].width = 12


def test_unknown_payload_and_oversized_image_rejected_without_object_hooks():
    from types import SimpleNamespace as NS

    class Trap:
        def __getattribute__(self, name):
            raise AssertionError("untrusted object hook invoked")

    h = header(1, 1_000_000_000)
    join = CaptureJoiner(capacity=2)
    assert offer(join, 0, h, Trap(), bridge_gid="02").kind == "invalid"
    h = header(2, 1_100_000_000)
    giant = NS(
        capture=h,
        width=640,
        height=480,
        step=100_000,
        encoding="rgb8",
        is_bigendian=False,
        data=bytearray(),
    )
    assert offer(join, 0, h, giant, bridge_gid="02").kind == "invalid"


def test_retired_boot_rollback_does_not_revoke_successor():
    join = CaptureJoiner(capacity=2)
    for stream in range(4):
        offer(join, stream, header(1, 1_000_000_000), stream, bridge_gid="02")
    changed = replace(header(1, 1_100_000_000), bridge_boot_id="newboot", bridge_gid="04")
    assert offer(join, 0, changed, 0, bridge_gid="04").kind == "invalid"
    generation = join.generation
    old = header(2, 900_000_000)
    assert offer(join, 0, old, 0, bridge_gid="02").kind == "ignored"
    assert join.generation == generation


def test_snapshot_accepts_supported_byte_buffers_and_preserves_padding():
    from array import array
    from types import SimpleNamespace as NS

    buffers = (
        b"abcdefXY",
        bytearray(b"abcdefXY"),
        memoryview(b"abcdefXY"),
        array("B", b"abcdefXY"),
    )
    for data in buffers:
        h = header(1, 1_000_000_000)
        join = CaptureJoiner(capacity=2)
        image = NS(
            capture=h, width=2, height=1, step=8, encoding="rgb8", is_bigendian=False, data=data
        )
        assert offer(join, 0, h, image, bridge_gid="02").kind == "pending"
        for stream in (1, 2, 3):
            result = offer(join, stream, h, stream, bridge_gid="02")
        assert result.kind == "ready" and result.bodies[0].data == b"abcdefXY"


def test_new_bridge_boot_does_not_reset_same_raw_source_time_frontier():
    join = CaptureJoiner(capacity=4)
    assert offer(join, 0, header(1, 1_200_000_000), 0, bridge_gid="02").kind == "pending"
    first = replace(header(1, 1_100_000_000), bridge_boot_id="newboot", bridge_gid="04")
    assert offer(join, 0, first, 0, bridge_gid="04").kind == "invalid"
    still_old = replace(header(2, 1_150_000_000), bridge_boot_id="newboot", bridge_gid="04")
    assert offer(join, 0, still_old, 0, bridge_gid="04").kind == "invalid"
    fresh = replace(header(3, 1_300_000_000), bridge_boot_id="newboot", bridge_gid="04")
    assert offer(join, 0, fresh, 0, bridge_gid="04").kind == "pending"


def test_new_raw_source_can_establish_own_frontier_after_bridge_restart():
    join = CaptureJoiner(capacity=4)
    assert offer(join, 0, header(1, 1_200_000_000), 0, bridge_gid="02").kind == "pending"
    fresh_source = replace(
        header(1, 1_100_000_000),
        bridge_boot_id="newboot",
        bridge_gid="04",
        raw_gid="03",
        source_epoch="newsource",
    )
    assert offer(join, 0, fresh_source, 0, bridge_gid="04").kind == "invalid"
    later = replace(
        fresh_source,
        sequence=2,
        stamp_ns=1_150_000_000,
        received_ros_ns=1_150_000_000,
        received_steady=1.15,
    )
    assert offer(join, 0, later, 0, bridge_gid="04").kind == "pending"
