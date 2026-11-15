"""Compile the shared arithmetic helper only; this is not a ROS node build."""

import shutil
import subprocess
from pathlib import Path

import pytest


def test_cpp_age_queue_boundaries_and_transient_crossing(tmp_path):
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("C++ compiler unavailable")
    source = tmp_path / "policy.cpp"
    source.write_text(r"""
#include "senior_motion_msgs/sensor_timing.hpp"
#include <cassert>
#include <limits>
int main() {
  sensor_timing::Policy p(.3,1.,5.);
  assert(p.age_ns(1000000000,1200000000,5.,1400000000,5.7)==900000000);
  bool bad=false; try {sensor_timing::Policy q(0.,std::numeric_limits<double>::quiet_NaN(),5.);}
    catch(const std::invalid_argument &) {bad=true;} assert(bad);
  bad=false; try {p.age_ns(1000000000,1200000000,5.,1100000000,5.);}
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
