"""Compile the exact policy boundary used by native callbacks; no ROS build claim."""

import shutil
import subprocess
from pathlib import Path

import pytest


def test_native_checked_age_input_matrix_does_not_throw(tmp_path: Path) -> None:
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable")
    assert compiler is not None
    source = tmp_path / "checked_age.cpp"
    source.write_text(r"""
#include "senior_motion_msgs/sensor_timing.hpp"
#include <cassert>
#include <limits>
#include <type_traits>

int main() {
  sensor_timing::Policy policy(300.,1000.,5.);
  static_assert(noexcept(policy.try_age_ns(1,2.,3.)));
  sensor_timing::AgeSummary summary;
  auto checked=[&](uint64_t measured,double receipt,double now) {
    auto age=policy.try_age_ns(measured,receipt,now);
    if(!age)summary.reject_invalid_clock();
    return age;
  };
  assert(!checked(6'000'000'000ULL,5.,5.));
  assert(!checked(1,5.,4.));
  assert(!checked(1,-1.,5.));
  const double nan=std::numeric_limits<double>::quiet_NaN();
  const double inf=std::numeric_limits<double>::infinity();
  assert(!checked(1,nan,5.));assert(!checked(1,5.,nan));
  assert(!checked(1,inf,inf));assert(!checked(1,0.,inf));
  assert(!checked(1,0.,std::numeric_limits<double>::max()));
  assert(!checked(1,0.,9e9+1.));
  assert(summary.invalid_clock==9 && summary.count==0);
  assert(*checked(5'000'000'000ULL,5.1,5.2)==200'000'000);
  assert(*checked(0,0.,9e9)==9'000'000'000'000'000'000LL);
  for(const auto ns:{299999999LL,300000000LL,999999999LL,1000000000LL}) {
    auto age=checked(0,0.,static_cast<double>(ns)/1e9);
    assert(age && *age==ns);summary.observe(*age,policy);
  }
  assert(summary.count==4 && summary.over_soft==3 && summary.over_hard==1);
  bool threw=false;
  try{policy.age_ns(6'000'000'000ULL,5.,5.);}catch(const std::invalid_argument&){threw=true;}
  assert(threw);
  summary.invalid_clock=std::numeric_limits<uint64_t>::max();summary.reject_invalid_clock();
  assert(summary.invalid_clock==std::numeric_limits<uint64_t>::max());
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
