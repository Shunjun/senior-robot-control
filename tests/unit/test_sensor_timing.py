import math

import pytest

from senior_control.sensors.policy import default_policy
from senior_control.sensors.sample_time import map_sample


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
    from senior_control.sensors.sample_time import map_sample

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


def test_integer_stamp_endpoints_and_local_clock_generations():
    from senior_control.sensors.sample_time import ClockTracker, interval_ns, map_sample

    epoch = 1_790_000_000_000_000_000
    first = map_sample(
        stamp_ns=epoch,
        received_ros_ns=epoch + 2,
        received_steady_ns=20_000_000_000,
        clock_generation=1,
    )
    last = map_sample(
        stamp_ns=epoch + 1,
        received_ros_ns=epoch + 2,
        received_steady_ns=20_000_000_000,
        clock_generation=1,
    )
    assert interval_ns(first, last).end_ns - interval_ns(first, last).start_ns == 1
    assert last.sample_steady_ns - first.sample_steady_ns == 1
    assert interval_ns(first, first).start_ns == interval_ns(first, first).end_ns
    with pytest.raises(ValueError, match="reversed"):
        interval_ns(last, first)
    tracker = ClockTracker()
    assert tracker.observe(ros_ns=100, steady_ns=100).generation == 1
    assert not tracker.observe(ros_ns=100, steady_ns=100).discontinuity
    assert not tracker.observe(ros_ns=200, steady_ns=101).discontinuity
    rollback = tracker.observe(ros_ns=199, steady_ns=102)
    assert rollback.discontinuity and rollback.generation == 2
    assert tracker.observe(ros_ns=200, steady_ns=101).generation == 3
    changed = map_sample(
        stamp_ns=epoch + 1,
        received_ros_ns=epoch + 2,
        received_steady_ns=20_000_000_000,
        clock_generation=2,
    )
    with pytest.raises(ValueError, match="generation"):
        interval_ns(first, changed)


@pytest.mark.parametrize(
    "field,value",
    [
        ("stamp_ns", -1),
        ("stamp_ns", 101),
        ("stamp_ns", 1 << 63),
        ("stamp_ns", 0.0),
        ("stamp_ns", True),
        ("received_ros_ns", -1),
        ("received_ros_ns", 1 << 63),
        ("received_steady_ns", -1),
        ("received_steady_ns", 1 << 64),
        ("received_steady_ns", 0),
        ("clock_generation", -1),
        ("clock_generation", 1 << 64),
    ],
)
def test_mapping_rejects_invalid_integer_coordinates(field, value):
    from senior_control.sensors.sample_time import map_sample

    fields = dict(stamp_ns=99, received_ros_ns=100, received_steady_ns=20, clock_generation=1)
    fields[field] = value
    with pytest.raises(ValueError):
        map_sample(**fields)


def test_zero_stamp_and_full_width_integer_mapping():
    from sensor_time_test_support import capture_fields, sample_time

    zero = sample_time(stamp_ns=0, received_ros_ns=0, received_steady_ns=0)
    assert zero.sample_steady_ns == 0
    maximum = sample_time(
        stamp_ns=(1 << 63) - 1,
        received_ros_ns=(1 << 63) - 1,
        received_steady_ns=(1 << 64) - 1,
    )
    assert maximum.sample_steady_ns == (1 << 64) - 1
    fields = capture_fields(time=zero)
    assert set(fields) == {
        "stamp",
        "received_ros",
        "received_steady_ns",
        "sample_steady_ns",
        "clock_generation",
    }
    assert fields["stamp"].sec == fields["stamp"].nanosec == 0
    with pytest.raises(ValueError):
        default_policy().sample_age(maximum, now_steady_ns=(1 << 64) - 2)


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
#include "senior_motion_msgs/sample_time.hpp"
#include "senior_motion_msgs/sensor_timing.hpp"
#include <cassert>
#include <limits>
using namespace senior_motion_msgs::time;
template<class F> void invalid(F run) {
  bool rejected = false;
  try { run(); } catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
}
int main() {
  const auto sample = map_sample(99'800'000'000,100'000'000'000,20'000'000'000,1);
  assert(sample.sample_steady_ns == 19'800'000'000);
  sensor_timing::Policy policy(300,1000,5);
  assert(policy.age_ns(sample,20'150'000'000) == 350'000'000);
  assert(policy.age_ns(sample,20'100'000'000) == policy.soft_ns);
  assert(policy.age_ns(sample,20'800'000'000) == policy.hard_ns);
  constexpr int64_t epoch = 1'790'000'000'000'000'000;
  const auto first = map_sample(epoch,epoch+2,20'000'000'000,1);
  const auto last = map_sample(epoch+1,epoch+2,20'000'000'000,1);
  const auto interval = interval_ns(first,last);
  assert(interval.end_ns - interval.start_ns == 1);
  assert(last.sample_steady_ns - first.sample_steady_ns == 1);
  assert(interval_ns(first,first).start_ns == epoch);
  assert(map_sample(0,0,0,1).sample_steady_ns == 0);
  constexpr auto max_ros = std::numeric_limits<int64_t>::max();
  constexpr auto max_steady = std::numeric_limits<uint64_t>::max();
  assert(map_sample(max_ros,max_ros,max_steady,1).sample_steady_ns == max_steady);
  invalid([] {map_sample(-1,0,10,1);});
  invalid([] {map_sample(0,-1,10,1);});
  invalid([] {map_sample(1,0,10,1);});
  invalid([] {map_sample(0,100,99,1);});
  invalid([&] {interval_ns(last,first);});
  invalid([&] {interval_ns(first,map_sample(epoch+1,epoch+2,20'000'000'000,2));});
  invalid([&] {policy.age_ns(sample,19'999'999'999);});
  ClockTracker clock;
  assert(clock.observe(100,100).generation == 1);
  assert(!clock.observe(100,100).discontinuity);
  assert(!clock.observe(200,101).discontinuity);
  const auto rollback = clock.observe(199,102);
  assert(rollback.discontinuity && rollback.generation == 2);
  assert(clock.observe(200,101).generation == 3);
  invalid([&] {clock.observe(-1,102);});
  assert(clock.observe(200,101).generation == 3);
  assert(monotonic_ns() > 0);
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
            "-I" + str(Path("ros2/src/senior_motion_msgs/include").resolve()),
            str(source),
            "-o",
            str(executable),
        ],
        check=True,
    )
    subprocess.run([str(executable)], check=True)
