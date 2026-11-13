import io

from tools.scan_matcher_replay import write_scan_records


def envelope(stamp, epoch="source-a", valid=True):
    return {
        "owner": {"valid": valid, "owner_epoch": epoch, "owner_gid": "gid"},
        "body": {
            "valid": valid, "source_epoch": epoch,
            "raw_capture": {"valid": valid, "stamp": {"sec": stamp, "nanosec": 0},
                            "source_epoch": "raw", "raw_gid": "raw-gid"},
            "angle_min": -1., "angle_increment": .1, "range_min": .08,
            "range_max": 20., "ranges": [1., "nan", 2.],
        },
    }


def test_replay_retains_source_order_and_marks_real_source_changes():
    stream = io.StringIO()
    frames = write_scan_records(
        [envelope(3), envelope(2), envelope(4, "source-b")], stream
    )
    lines = stream.getvalue().splitlines()
    assert [frame["stamp_ns"] for frame in frames] == [3_000_000_000, 2_000_000_000, 4_000_000_000]
    assert lines[0].startswith("S 3.0 ")
    assert lines[1].startswith("S 2.0 ")  # Do not sort or hide ROS rollback.
    assert lines[2] == "R"
    assert lines[3].startswith("S 4.0 ")
    assert lines[0].endswith("1.0 nan 2.0")


def test_upstream_invalid_resets_without_fabricating_a_scan_or_epoch():
    stream = io.StringIO()
    frames = write_scan_records([envelope(1), envelope(2, valid=False), envelope(3)], stream)
    assert stream.getvalue().splitlines()[1] == "R"
    assert len(frames) == 2
    assert frames[-1]["producer_epoch"] == "source-a"


def test_body_epoch_disagrees_with_owner_is_rejected_like_node_admission():
    restarted = envelope(2)
    restarted["body"]["source_epoch"] = "unapproved-restart"
    stream = io.StringIO()
    frames = write_scan_records([envelope(1), restarted, envelope(3)], stream)
    assert [frame["stamp_ns"] for frame in frames] == [1_000_000_000, 3_000_000_000]
    assert stream.getvalue().splitlines()[1] == "R"
