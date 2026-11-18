"""Compile the shared arithmetic helper only; this is not a ROS node build."""

import shutil
import subprocess
from pathlib import Path

import pytest


def test_cpp_age_queue_boundaries_and_transient_crossing(tmp_path: Path) -> None:
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable")
    assert compiler is not None
    source = tmp_path / "policy.cpp"
    source.write_text(r"""
#include "senior_motion_msgs/sensor_timing.hpp"
#include "senior_motion_msgs/sample_time.hpp"
#include <cassert>
#include <limits>
int main() {
  using namespace senior_motion_msgs::time;
  const auto original = map_sample(99800000000LL, 100000000000LL, 20000000000ULL, 1);
  assert(original.sample_steady_ns == 19800000000ULL);
  assert(20150000000ULL - original.sample_steady_ns == 350000000ULL);
  const auto next = map_sample(99800000001LL, 100000000000LL, 20000000000ULL, 1);
  assert(interval_ns(original, next).end_ns - interval_ns(original, next).start_ns == 1);
  assert(map_sample(0, 0, 0, 1).stamp_ns == 0);
  ClockTracker clock;
  assert(clock.observe(100, 200).generation == 1);
  const auto rollback = clock.observe(99, 201);
  assert(rollback.discontinuity && rollback.generation == 2);
  bool changed = false;
  try { interval_ns(original, map_sample(99800000001LL,100000000000LL,20000000000ULL,2)); }
  catch (const std::invalid_argument &) { changed = true; }
  assert(changed);

  sensor_timing::Policy p(300.,1000.,5.);
  assert(p.age_ns(4800000000ULL,5.,5.7)==900000000);
  bool bad=false; try {sensor_timing::Policy q(0.,std::numeric_limits<double>::quiet_NaN(),5.);}
    catch(const std::invalid_argument &) {bad=true;} assert(bad);
  bad=false; try {p.age_ns(1200000000ULL,1.2,1.1);}
    catch(const std::invalid_argument &) {bad=true;} assert(bad);
  sensor_timing::AgeSummary ages;
  ages.observe(299999999,p); ages.observe(300000000,p); ages.observe(1000000000,p);
  assert(ages.count==3 && ages.over_soft==2 && ages.over_hard==1);
  sensor_timing::QueuePolicy q(32,.75,5.,.5,300.);
  assert(q.threshold(4)==3 && q.threshold(16)==12 && q.threshold(1024)==32);
  sensor_timing::QueueMonitor m;
  m.note(4,4,0.,10.,q); // burst drains before the reporter
  assert(m.observe(0,4,0.,10.1,q)==1 && m.warning_pending==4);
  assert(m.observe(0,4,0.,14.9,q)==0);
  assert(m.observe(0,4,0.,15.1,q)==-1);
  assert(m.observe(1,1,301.,20.1,q)==1);
}
""")
    executable = tmp_path / "policy"
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
