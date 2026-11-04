"""Pure depth producer checks use real wire layouts and original source metadata."""

import math
import struct
from dataclasses import replace

import pytest


def inputs():
    from senior_control.vision.depth_runtime import CameraInfoFrame, DepthPacket, OpticalTransform

    packet = DepthPacket(
        stamp_ns=1_000_000_001, received_ros_ns=1_000_000_002, received_steady=10.0,
        source_epoch="camera-epoch", raw_source="camera-gid", sequence=2,
        frame_id="camera_optical_frame", width=3, height=3, step=16,
        encoding="32FC1", big_endian=False,
        data=b"".join(struct.pack("<fff", *row) + b"pad!" for row in (
            (math.nan, 1.0, math.nan), (math.nan, 0.6, math.nan),
            (math.nan, 0.4, math.nan),
        )),
    )
    info = CameraInfoFrame(
        stamp_ns=packet.stamp_ns, received_ros_ns=packet.received_ros_ns,
        received_steady=9.999, frame_id=packet.frame_id, raw_source="info-gid",
        width=3, height=3, distortion_model="plumb_bob", d=(0.0,) * 5,
        k=(2.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0),
        r=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        p=(2.0, 0.0, 1.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        binning_x=0, binning_y=0, roi=(0, 0, 0, 0, False),
    )
    tf = OpticalTransform(
        stamp_ns=0, translation=(0.2316, 0.00224, 0.1282),
        rotation_xyzw=(-0.5, 0.5, -0.5, 0.5), is_static=True,
    )
    return packet, info, tf


def test_depth_producer_preserves_original_identity_and_low_hits():
    from senior_control.vision.depth_runtime import DepthGeometrySettings, produce_depth_evidence

    packet, info, tf = inputs()
    evidence = produce_depth_evidence(packet, info, tf, DepthGeometrySettings(
        min_range_m=0.1, max_range_m=8.0, range_error_m=0.01,
        row_stride=1, column_stride=1, body_frame="base_link",
    ))
    assert evidence.header.stamp_ns == packet.stamp_ns
    assert evidence.header.received_ros_ns == packet.received_ros_ns
    assert evidence.header.received_steady == packet.received_steady
    assert evidence.info == info and evidence.transform == tf
    assert evidence.origin == pytest.approx((0.2316, 0.00224, 0.1282))
    assert len(evidence.samples) == 3
    assert (evidence.samples[-1].u, evidence.samples[-1].v) == (1, 2)
    assert evidence.samples[-1].z == pytest.approx(0.0282)
    assert not evidence.coverage_qualified


@pytest.mark.parametrize("change", ["wrong_frame", "skew", "distortion", "projection", "tf"])
def test_depth_producer_rejects_unqualified_calibration_or_tf(change):
    from senior_control.vision.depth_runtime import DepthGeometrySettings, produce_depth_evidence

    packet, info, tf = inputs()
    if change == "wrong_frame":
        info = replace(info, frame_id="other")
    elif change == "skew":
        info = replace(info, stamp_ns=info.stamp_ns - 1)
    elif change == "distortion":
        info = replace(info, d=(0.0, 0.1, 0.0, 0.0, 0.0))
    elif change == "projection":
        info = replace(info, p=(3.0, *info.p[1:]))
    else:
        tf = replace(tf, rotation_xyzw=(0.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        produce_depth_evidence(packet, info, tf, DepthGeometrySettings(
            min_range_m=0.1, max_range_m=8.0, range_error_m=0.01,
            row_stride=1, column_stride=1, body_frame="base_link",
        ))


def test_buffer_pairs_exact_stamps_across_dropped_frames_and_keeps_receipts():
    from senior_control.vision.depth_runtime import DepthPairBuffer

    packet, info, _ = inputs()
    buffer = DepthPairBuffer(max_age_s=0.5, pair_wait_s=0.1)
    assert buffer.offer_info(replace(info, stamp_ns=900_000_000))
    assert buffer.offer_depth(packet)
    assert buffer.take_ready(now_ros_ns=1_000_000_002, now_steady=10.01) == (None, None)
    assert buffer.offer_info(info)
    pair, failure = buffer.take_ready(now_ros_ns=1_000_000_002, now_steady=10.02)
    assert failure is None and pair == (packet, info)
    assert pair[0].received_steady == 10.0
    assert pair[1].received_steady == 9.999


def test_buffer_future_expiry_and_overflow_are_explicit_failure():
    from senior_control.vision.depth_runtime import DepthPairBuffer

    packet, info, _ = inputs()
    buffer = DepthPairBuffer(max_age_s=0.5, pair_wait_s=0.1)
    assert buffer.offer_depth(packet)
    assert buffer.offer_info(info)
    assert buffer.take_ready(now_ros_ns=packet.stamp_ns - 1, now_steady=10.0) == (None, None)
    pair, failure = buffer.take_ready(now_ros_ns=packet.stamp_ns - 1, now_steady=10.021)
    assert pair is None and failure == "future_expired"
    assert buffer.take_ready(now_ros_ns=packet.stamp_ns, now_steady=10.022) == (None, None)

    for index in range(4):
        assert buffer.offer_depth(replace(packet, stamp_ns=packet.stamp_ns + index + 1))
    assert not buffer.offer_depth(replace(packet, stamp_ns=packet.stamp_ns + 5))
    assert (
        buffer.take_ready(now_ros_ns=packet.stamp_ns + 10, now_steady=10.03)[1] == "depth_overflow"
    )


@pytest.mark.parametrize(
    ("quaternion", "expected"),
    [((0.0, 0.0, 0.0, 1.0), (0.0, -0.25, 1.0)),
     ((1.0, 0.0, 0.0, 0.0), (0.0, 0.25, -1.0))],
)
def test_singular_optical_transforms_project_samples_without_gimbal_lock(quaternion, expected):
    from senior_control.vision.depth_runtime import DepthGeometrySettings, produce_depth_evidence

    packet, info, tf = inputs()
    tf = replace(tf, translation=(0.0, 0.0, 0.0), rotation_xyzw=quaternion)
    evidence = produce_depth_evidence(packet, info, tf, DepthGeometrySettings(
        min_range_m=0.1, max_range_m=8.0, range_error_m=0.01,
        row_stride=1, column_stride=1, body_frame="base_link",
    ))
    first = evidence.samples[0]
    assert (first.u, first.v, first.depth_m) == (1, 0, 1.0)
    assert (first.x, first.y, first.z) == pytest.approx(expected, abs=1e-9)


def test_optical_rotation_matches_quaternion_oracle_near_both_pitch_singularities():
    import math
    import random

    from senior_control.vision.depth import _Rotation, body_extrinsic_from_optical_tf

    def multiply(a, b):
        x, y, z, w = a
        u, v, s, t = b
        return (w*u + x*t + y*s - z*v, w*v - x*s + y*t + z*u,
                w*s + x*v - y*u + z*t, w*t - x*u - y*v - z*s)

    def from_euler(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)

    rng = random.Random(4927)
    point = (0.25, -0.4, 2.0)
    for sign in (-1, 1):
        for epsilon in (0, 1e-12, 1e-10, 1e-8, 1e-6, 1e-4):
            for _ in range(100):
                body = from_euler(rng.uniform(-math.pi, math.pi),
                                  sign * (math.pi / 2 - epsilon),
                                  rng.uniform(-math.pi, math.pi))
                optical = multiply(body, (-0.5, 0.5, -0.5, 0.5))
                inverse = (-optical[0], -optical[1], -optical[2], optical[3])
                expected = multiply(multiply(optical, (*point, 0.0)), inverse)[:3]
                extrinsic = body_extrinsic_from_optical_tf((0, 0, 0), optical)
                actual = _Rotation(extrinsic).rotate((point[2], -point[0], -point[1]))
                assert actual == pytest.approx(expected, abs=1e-12)
