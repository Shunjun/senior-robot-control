"""Compile the exact policy boundary used by native callbacks; no ROS build claim."""

import shutil
import subprocess
from pathlib import Path

import pytest


def test_native_checked_age_input_matrix_does_not_throw(tmp_path):
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable")
    source = tmp_path / "checked_age.cpp"
    source.write_text(r"""
#include "senior_motion_msgs/sensor_timing.hpp"
#include <cassert>
#include <limits>
#include <type_traits>

int main() {
  sensor_timing::Policy policy(.3, 1., 5.);
  static_assert(noexcept(policy.try_age_ns(1, 2, 3., 4, 5.)));
  sensor_timing::AgeSummary summary;
  // Same call shape as the ROS-header adapter. Invalid input has no numeric age.
  auto checked = [&](int64_t stamp, int64_t received, double receipt_steady,
                     int64_t now, double steady) {
    const auto age = policy.try_age_ns(stamp, received, receipt_steady, now, steady);
    if (!age) summary.reject_invalid_clock();
    return age;
  };
  assert(!checked(100, 120, 5., 110, 5.));  // receipt is in the future, stamp isn't
  assert(!checked(100, 120, 5., 99, 5.));   // ROS rollback
  assert(!checked(100, 120, 5., 130, 4.));  // steady rollback
  assert(!checked(-1, 120, 5., 130, 5.));
  assert(!checked(100, 99, 5., 130, 5.));
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  assert(!checked(100, 120, nan, 130, 5.));
  assert(!checked(100, 120, 5., 130, nan));
  assert(!checked(100, 120, inf, 130, inf));
  assert(!checked(100, 120, 0., 130, inf));
  const double largest = std::numeric_limits<double>::max();
  assert(!checked(100, 120, -largest, 130, largest)); // subtraction overflow
  assert(!checked(100, 120, 0., 130, largest));      // unrepresentable ns delta
  assert(!checked(100, 120, 0., 130, 9e9+1.));      // same existing numeric range
  assert(summary.invalid_clock == 12 && summary.count == 0);
  // No new absolute steady epoch restriction, only the existing elapsed range.
  assert(*checked(100, 120, largest, 130, largest) == 30);
  assert(*checked(100, 120, 5., 130, 5.2) == 200000020);
  assert(*checked(0, 0, 0., 0, 9e9) == 9000000000000000000LL);
  const auto maximum = std::numeric_limits<int64_t>::max();
  assert(*checked(0, maximum, 0., maximum, 1.) == maximum); // saturating sum
  for (const auto age_ns : {299999999LL, 300000000LL, 999999999LL, 1000000000LL}) {
    const auto age = checked(0, 0, 5., age_ns, 5.);
    assert(age && *age == age_ns);
    summary.observe(*age, policy);
  }
  assert(summary.count == 4 && summary.over_soft == 3 && summary.over_hard == 1);
  // Constructor/programmer API remains throwing; callback API does not swallow
  // unrelated exceptions or change invalid policy configuration into a default.
  bool threw = false;
  try { policy.age_ns(100, 120, 5., 110, 5.); }
  catch (const std::invalid_argument &) { threw = true; }
  assert(threw);
  summary.invalid_clock = std::numeric_limits<uint64_t>::max();
  summary.reject_invalid_clock();
  assert(summary.invalid_clock == std::numeric_limits<uint64_t>::max());
}
""")
    executable = tmp_path / "checked_age"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-fsanitize=undefined,float-cast-overflow",
            "-fno-sanitize-recover=all",
            "-I" + str(Path("ros2/src/senior_motion_msgs/include").resolve()),
            str(source),
            "-o",
            str(executable),
        ],
        check=True,
    )
    subprocess.run([str(executable)], check=True)
