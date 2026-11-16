"""Compile real C++ timing and patched publication bodies without starting ROS."""

import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from types import ModuleType
from xml.etree import ElementTree

import pytest
from senior_robot import navigation_overlay

from tools.prepare_nav2_overlay import BUNDLE, prepare


def test_installed_overlay_requires_exact_package_versions_and_contract(tmp_path, monkeypatch):
    import sys

    module = ModuleType("ament_index_python.packages")
    module.get_package_share_directory = lambda name: str(tmp_path / name)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", module)
    for name in navigation_overlay.ABI_PACKAGES:
        share = tmp_path / name
        share.mkdir()
        (share / "package.xml").write_text("<package><version>1.1.20</version></package>")
    marker = tmp_path / "nav2_controller/navigation-contract.json"
    with pytest.raises(RuntimeError, match="not installed"):
        navigation_overlay.require_navigation_overlay()
    marker.write_text(
        json.dumps(
            {
                "commit": navigation_overlay.NAV2_COMMIT,
                "version": "1.1.20",
                "contract": navigation_overlay.CONTRACT,
            }
        )
    )
    (tmp_path / "nav2_mppi_controller/navigation-contract.json").write_text(marker.read_text())
    navigation_overlay.require_navigation_overlay()
    (tmp_path / "nav2_core/package.xml").write_text("<package><version>1.1.19</version></package>")
    with pytest.raises(RuntimeError, match="ABI mismatch: nav2_core"):
        navigation_overlay.require_navigation_overlay()


@pytest.fixture
def patched_source(tmp_path):
    source = os.environ.get("NAV2_SOURCE_CACHE")
    if not source:
        pytest.skip("requires verified official source cache: NAV2_SOURCE_CACHE")
    return prepare(Path(source), tmp_path / "overlay")


def test_locked_official_source_patch_and_all_time_boundaries(patched_source, tmp_path):
    source = (patched_source / "src/controller_server.cpp").read_text()
    assert source.count("navigation_producer_ =") == 1
    assert source.count("last_valid_cmd_time_ = control_clock_.now();") == 2
    assert "(control_clock_.now() - last_valid_cmd_time_)" in source
    assert "rclcpp::Rate" not in source
    compute = source.split("void ControllerServer::computeAndPublishVelocity()", 1)[1].split(
        "void ControllerServer::updateGlobalPath",
        1,
    )[0]
    assert compute.index("monotonic_ns()") < compute.index("computeVelocityCommands(")
    assert "publishVelocity(cmd_vel_2d, produced);" in compute
    assert "cmd_vel_2d.header.stamp = now();" in compute  # ROS stamped data retains its domain.
    assert "RCL_STEADY_TIME" in (patched_source / "plugins/simple_progress_checker.cpp").read_text()
    assert ElementTree.parse(patched_source / "package.xml").findtext("version") == "1.1.20"
    # A single-byte source change fails before a destination is created.
    bad = tmp_path / "bad-source"
    shutil.copytree(Path(os.environ["NAV2_SOURCE_CACHE"]), bad)
    (bad / "nav2_controller/src/controller_server.cpp").write_text("wrong version")
    with pytest.raises(ValueError, match="source checksum mismatch"):
        prepare(bad, tmp_path / "must-not-exist")
    assert not (tmp_path / "must-not-exist").exists()
    assert (
        json.loads((BUNDLE / "manifest.json").read_text())["commit"]
        == navigation_overlay.NAV2_COMMIT
    )


def test_actual_patched_cpp_nonzero_and_zero_preserve_production_time(patched_source, tmp_path):
    compiler = shutil.which("clang++") or shutil.which("g++")
    if not compiler:
        pytest.skip("C++ compiler unavailable")
    source = (patched_source / "src/controller_server.cpp").read_text()
    bodies = source.split("void ControllerServer::publishVelocity(", 1)[1].split(
        "bool ControllerServer::isGoalReached",
        1,
    )[0]
    code = (
        r"""
#include <memory>
#include <string>
#include <vector>
#include <cassert>
#include <iostream>
#include "senior_motion_msgs/navigation_timing.hpp"
namespace geometry_msgs::msg {
struct Vector { double x=0, y=0, z=0; };
struct Twist { Vector linear, angular; };
struct TwistStamped {
  Twist twist;
  struct {std::string frame_id; double stamp;} header;
};
}
namespace senior_motion_msgs::msg {
struct NavigationSample {
  std::string clock_domain, run_id;
  uint64_t sequence=0, produced_steady_ns=0;
  double vx=0, wz=0;
};
}
struct Publisher {
  std::vector<senior_motion_msgs::msg::NavigationSample> messages;
  bool is_activated() {return true;}
  int get_subscription_count() {return 1;}
  void publish(std::unique_ptr<senior_motion_msgs::msg::NavigationSample> msg) {
    messages.push_back(*msg);
  }
};
struct Costmap { std::string getBaseFrameID() {return "base_link";} };
namespace nav2_controller {
struct ControllerServer {
  std::unique_ptr<senior_motion_msgs::navigation::Producer> navigation_producer_;
  std::shared_ptr<Publisher> vel_publisher_=std::make_shared<Publisher>();
  std::shared_ptr<Costmap> costmap_ros_=std::make_shared<Costmap>();
  double wall=100;
  double now() {return wall;}
  void publishVelocity(const geometry_msgs::msg::TwistStamped &, uint64_t);
  void publishZeroVelocity();
};
void ControllerServer::publishVelocity(
"""
        + bodies
        + r"""
}
int main() {
  using senior_motion_msgs::navigation::Producer;
  nav2_controller::ControllerServer server;
  server.navigation_producer_=std::make_unique<Producer>("same-boot", std::string(32, 'a'));
  geometry_msgs::msg::TwistStamped v;
  v.twist.linear.x=-8; v.twist.angular.z=4;
  const auto produced=senior_motion_msgs::navigation::monotonic_ns();
  server.wall=-100000;
  server.publishVelocity(v, produced);
  server.wall=100000;
  server.publishZeroVelocity();
  const auto & samples=server.vel_publisher_->messages;
  assert(samples.size()==2 && samples[0].sequence==1 && samples[1].sequence==2);
  assert(samples[0].produced_steady_ns==produced);
  assert(samples[1].produced_steady_ns>=produced);
  assert(samples[0].vx==-8 && samples[0].wz==4);
  assert(samples[1].vx==0 && samples[1].wz==0);
  assert(samples[0].clock_domain=="same-boot" && samples[0].run_id==std::string(32,'a'));
  bool rejected=false;
  try {Producer invalid("same-boot", "wrong-run");} catch (const std::invalid_argument &) {
    rejected=true;
  }
  assert(rejected);
  std::cout << "patched nonzero/zero " << produced << "\n";
}
"""
    )
    cpp = tmp_path / "producer.cpp"
    cpp.write_text(code)
    binary = tmp_path / "producer"
    subprocess.run(
        [
            compiler,
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pthread",
            "-I",
            str(Path("ros2/src/senior_motion_msgs/include").resolve()),
            str(cpp),
            "-o",
            str(binary),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    # Linux Python monotonic and the producer share CLOCK_MONOTONIC. macOS
    # Python uses mach_absolute_time, so its offline C++ probe compares POSIX time.
    clock_ns = (
        time.monotonic_ns
        if sys.platform == "linux"
        else (lambda: time.clock_gettime_ns(time.CLOCK_MONOTONIC))
    )
    before = clock_ns()
    result = subprocess.run([str(binary)], check=True, capture_output=True, text=True)
    after = clock_ns()
    produced = int(result.stdout.split()[-1])
    assert before <= produced <= after
    assert "nonzero/zero" in result.stdout
