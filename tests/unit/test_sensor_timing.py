import math

import pytest

from vehicle_core.sample_time import map_sample
from vehicle_core.sensor_policy import default_policy


@pytest.mark.parametrize(
    "age,band", [(0.299, "normal"), (0.3, "low"), (0.999, "low"), (1, "invalid")]
)
def test_common_boundary(age, band):
    sample = map_sample(
        stamp_ns=20_000_000_000 - round(age * 1e9),
        received_ros_ns=20_000_000_000,
        received_steady_ns=20_000_000_000,
        clock_generation=1,
    )
    state = default_policy().sample_age(sample, now_steady_ns=20_000_000_000)
    assert state.band == band
    assert state.deadline_steady == pytest.approx(21 - age)


def test_original_production_time_includes_transport_and_processing_delay():
    p = default_policy()
    assert p.age(measured_steady=19.4, received_steady=20, now_steady=20.4).band == "invalid"
    assert p.age(measured_steady=19.4, received_steady=20, now_steady=20.2).band == "low"


@pytest.mark.parametrize(
    "field,value",
    [
        ("measured_steady", 21),
        ("measured_steady", -1),
        ("received_steady", 19),
        ("now_steady", 19),
        ("now_steady", math.nan),
        ("measured_steady", math.inf),
    ],
)
def test_invalid_monotonic_clock_has_no_numeric_age(field, value):
    args = dict(measured_steady=20, received_steady=20, now_steady=20)
    args[field] = value
    with pytest.raises(ValueError):
        default_policy().age(**args)


def test_policy_milliseconds_keep_one_second_hard_age_projection():
    policy = default_policy()
    assert (policy.soft_age_ms, policy.hard_age_ms, policy.hard_age_s) == (300, 1000, 1)
    with pytest.raises(TypeError):
        type(policy)(soft_age_s=0.3, hard_age_ms=1000, diagnostics_period_s=5)


def test_later_consumption_preserves_original_deadline():
    p = default_policy()
    a = p.age(measured_steady=20, received_steady=20.1, now_steady=20.2)
    b = p.age(measured_steady=20, received_steady=20.1, now_steady=20.9)
    assert a.deadline_steady == b.deadline_steady == 21
    assert b.seconds == 0.9


def test_standard_stamp_mapping_retains_transport_and_queue_age():
    from vehicle_core.sample_time import map_sample

    sample = map_sample(
        stamp_ns=99_800_000_000,
        received_ros_ns=100_000_000_000,
        received_steady_ns=20_000_000_000,
        clock_generation=1,
    )
    assert sample.sample_steady_ns == 19_800_000_000
    age = default_policy().sample_age(sample, now_steady_ns=20_150_000_000)
    assert age.seconds == 0.35
    assert age.band == "low"
    assert age.deadline_steady == 20.8
    assert default_policy().sample_age(sample, now_steady_ns=20_800_000_000).band == "invalid"


def test_cpp_standard_mapping_and_age_use_same_integer_contract(tmp_path):
    """Standalone arithmetic only; generated messages and ROS remain unverified."""
    import shutil
    import subprocess
    from pathlib import Path

    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable")
    source = tmp_path / "sample_time.cpp"
    source.write_text(r"""
#include "senior_sensor_ingress/sample_time.hpp"
#include "senior_sensor_ingress/sensor_timing.hpp"
#include <cassert>
using namespace senior_sensor_ingress::time;
int main() {
  const auto sample = map_sample(99'800'000'000,100'000'000'000,20'000'000'000,1);
  assert(sample.sample_steady_ns == 19'800'000'000);
  sensor_timing::Policy policy(300,1000,5);
  assert(policy.age_ns(sample,20'150'000'000) == 350'000'000);
  assert(policy.age_ns(sample,20'100'000'000) == policy.soft_ns);
  assert(policy.age_ns(sample,20'800'000'000) == policy.hard_ns);
  const auto later_age = policy.age_ns(sample,20'700'000'000);
  assert(later_age == 900'000'000);
  const auto deadline = static_cast<int64_t>(sample.sample_steady_ns) + policy.hard_ns;
  assert(deadline == 20'800'000'000);
  assert(20'150'000'000 + policy.hard_ns - 350'000'000 == deadline);
  assert(20'700'000'000 + policy.hard_ns - later_age == deadline);
}
""")
    executable = tmp_path / "sample_time"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I" + str(Path("ros2/src/senior_sensor_ingress/include").resolve()),
            str(source),
            "-o",
            str(executable),
        ],
        check=True,
    )
    subprocess.run([str(executable)], check=True)


def test_fixture_shared_endpoints_and_cached_repeats_keep_original_deadline():
    """Adjacent laser intervals share raw endpoints despite new delta sequences."""
    import json
    from collections import defaultdict
    from pathlib import Path

    from vehicle_core.sample_time import SampleTime

    paths = [
        Path("tests/fixtures") / name
        for name in (
            "motion_reference_backlog.json",
            "navigation_startup_reference.json",
            "reference_clock_frontier.json",
            "reference_nanosecond_boundary.json",
        )
    ]
    records = defaultdict(list)
    shared = defaultdict(list)

    def collect(node, path):
        if isinstance(node, list):
            for item in node:
                collect(item, path)
        elif isinstance(node, dict):
            samples = [(row["source"], row["sample"]) for row in node.get("sources", [])]
            samples.extend(("pose", pose) for pose in node.get("poses", []))
            for source, sample in samples:
                for role in ("time", "first_time", "last_time"):
                    if role not in sample:
                        continue
                    record = sample[role]
                    # These retained epochs include raw publisher GID and ingress lifecycle.
                    # A derived interval sequence is not part of raw endpoint identity.
                    key = source, sample["epoch"], record["clock_generation"], record["stamp_ns"]
                    records[key].append(record)
                    if source == "laser":
                        shared[path.name, record["stamp_ns"]].append((role, record))
            for item in node.values():
                collect(item, path)

    for path in paths:
        collect(json.loads(path.read_text()), path)

    assert sum(len(copies) for copies in records.values()) == 255
    policy = default_policy()
    for copies in records.values():
        now = max(record["received_steady_ns"] for record in copies) + 50_000_000
        deadlines = []
        for record in copies:
            assert record == copies[0]
            deadlines.append(
                policy.sample_age(SampleTime(**record), now_steady_ns=now).deadline_steady
            )
        assert all(deadline == deadlines[0] for deadline in deadlines)

    for key, (original_steady, copies) in {
        ("motion_reference_backlog.json", 1_300_000_000): (125_716_720_020_824, 3),
        ("reference_clock_frontier.json", 6_100_000_000): (129_861_541_005_342, 2),
    }.items():
        endpoints = shared[key]
        assert len(endpoints) == copies
        assert {role for role, _ in endpoints} == {"first_time", "last_time"}
        assert all(record["sample_steady_ns"] == original_steady for _, record in endpoints)
