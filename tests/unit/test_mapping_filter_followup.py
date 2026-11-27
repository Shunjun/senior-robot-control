"""Approved mapping consumer changes, independent of ROS and environment startup."""

import json
from dataclasses import replace

import pytest
from senior_mapping.evidence import (
    FixedDelta,
    OccupancyThresholds,
    StableSupport,
    SupportCellRemoval,
)
from senior_mapping.occupancy import FreeSpaceAccumulator
from senior_mapping.references import (
    _support_payload,
    _support_v2,
    decode_fixed_delta,
    encode_fixed_delta,
)
from test_active_map import accepted_fixed_scan, fixed_episode_setup

from vehicle_core.errors import DomainError


def test_5000_point_support_and_cell_removal_roundtrip_without_truncation():
    active, owner, identity, state = fixed_episode_setup()
    scan, cut = accepted_fixed_scan(active, owner, state, 1)
    points = tuple((float(i) + 0.25, 0.25) for i in range(5000))
    cells = tuple((i, 0) for i in range(5000))
    support = StableSupport(
        "wall",
        "kf1",
        1.0,
        points[:2],
        points,
        cells,
        scan.key,
        scan.key,
        identity,
        "geometric",
        "synthetic-admitted",
        "a" * 64,
        1,
        1,
    )
    assert _support_v2(json.loads(json.dumps(_support_payload(support)))) == support
    removal = SupportCellRemoval("wall", 1, cells, "observed_free_space", scan.key, "kf1")
    delta = FixedDelta(
        cut.watermark,
        scan.key,
        1,
        "kf1",
        upsert_supports=(support,),
        support_cell_removals=(removal,),
    )
    assert decode_fixed_delta(encode_fixed_delta(delta)) == delta


def test_mapping_uses_capture_stamp_not_raw_sequence_and_counts_each_frame_once():
    active, owner, identity, state = fixed_episode_setup()
    scan, cut = accepted_fixed_scan(active, owner, state, 1)
    thresholds = OccupancyThresholds(
        1.0, 2, 0.1, 2, 0.1, 0.1, 1.0, 1.0, 2.0, -2.0, -5.0, 5.0, 1.0, 0.5
    )
    accumulator = FreeSpaceAccumulator(thresholds)
    assert accumulator.limits is not None
    accumulator.update(scan, cut.basis)
    cell = next(cell for cell, confidence in accumulator.cells.items() if confidence.count == 1)
    original = accumulator.cells[cell].count
    for index, sequence in enumerate((50, 0), start=1):
        sample = replace(
            scan.capture.time,
            stamp_ns=scan.capture.time.stamp_ns + index * 100_000_000,
            sample_steady_ns=scan.capture.time.sample_steady_ns + index * 100_000_000,
            received_ros_ns=scan.capture.time.received_ros_ns + index * 100_000_000,
            received_steady_ns=scan.capture.time.received_steady_ns + index * 100_000_000,
        )
        capture = replace(scan.capture, time=sample, sequence=sequence)
        key = replace(scan.key, stamp_ns=sample.stamp_ns, sequence=sequence)
        current = replace(
            scan,
            key=key,
            capture=capture,
            valid_until_steady_ns=sample.sample_steady_ns + 500_000_000,
        )
        state["now"] = sample.sample_steady_ns + 1
        active.accept_scan(owner, current, cut.basis)
        accumulator.update(current, cut.basis)
        assert accumulator.cells[cell].count == original + index
        accumulator.update(current, cut.basis)
        assert accumulator.cells[cell].count == original + index
        with pytest.raises(DomainError, match="scan does not belong"):
            active.accept_scan(owner, current, cut.basis)
    successor = replace(
        current,
        key=replace(current.key, capture_generation=2),
        capture=replace(current.capture, capture_generation=2),
    )
    accumulator.update(successor, cut.basis)
    assert accumulator.cells[cell].count == 1


def test_preflight_accepts_long_map_and_odom_frame_names():
    from candidate_preflight import parse_args

    _, args = parse_args(
        [
            "/tmp/native-candidate",
            "a" * 64,
            "--map-frame",
            "map_" + "x" * 200,
            "--odom-frame",
            "odom_" + "x" * 200,
        ]
    )
    assert len(args.map_frame) > 128 and len(args.odom_frame) > 128
