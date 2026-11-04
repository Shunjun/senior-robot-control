#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <senior_motion_msgs/msg/scan_motion.hpp>
#include <senior_motion_msgs/msg/fenced_filtered_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "senior_scan_matcher/scan_matcher.hpp"

namespace
{
enum class Mode {closed_graph, live_rcl_error, live_other_error};
Mode mode;
bool observed_graph_rcl_error = false;
}

namespace rclcpp
{
template<typename NodeT>
void test_spin(std::shared_ptr<NodeT> node)
{
  if (mode == Mode::closed_graph) {
    rclcpp::shutdown();
    try {
      node->get_publishers_info_by_topic("/shutdown_graph_test");
    } catch (const rclcpp::exceptions::RCLError &) {
      observed_graph_rcl_error = true;
      throw;
    }
  } else if (mode == Mode::live_rcl_error) {
    RCL_SET_ERROR_MSG("live error");
    auto error = rclcpp::exceptions::RCLError(RCL_RET_ERROR, rcl_get_error_state(), "live error");
    rcl_reset_error();
    throw error;
  } else {
    throw std::runtime_error("live error");
  }
}
}

#define spin test_spin
#define main scan_motion_entrypoint
#include "../src/scan_motion_node.cpp"
#undef main
#undef spin

namespace
{
int entrypoint()
{
  std::vector<std::string> args = {
    "scan_motion_test", "--ros-args",
    "-p", "transform_static:=true", "-p", "scan_max_age:=0.8",
    "-p", "max_linear_speed:=0.5", "-p", "max_yaw_rate:=0.8",
    "-p", "max_interval:=0.4", "-p", "range_limit:=8.0", "-p", "min_response:=0.3",
    "-p", "max_residual:=0.09", "-p", "min_inlier_fraction:=0.55", "-p", "min_spread:=0.04",
    "-p", "min_translation_observability:=0.05", "-p", "min_yaw_observability:=0.01",
    "-p", "max_variance:=0.05", "-p", "reanchor_distance:=0.12", "-p", "reanchor_yaw:=0.1",
    "-p", "min_rays:=100"};
  std::vector<char *> argv;
  for (auto & arg : args) {argv.push_back(arg.data());}
  argv.push_back(nullptr);
  return scan_motion_entrypoint(static_cast<int>(args.size()), argv.data());
}
}

TEST(ScanMotionShutdown, ClosedContextGraphQueryExitsCleanly)
{
  mode = Mode::closed_graph;
  EXPECT_NO_THROW(EXPECT_EQ(entrypoint(), 0));
  EXPECT_TRUE(observed_graph_rcl_error);
  if (rclcpp::ok()) {rclcpp::shutdown();}
}

TEST(ScanMotionShutdown, LiveErrorsPropagate)
{
  mode = Mode::live_rcl_error;
  EXPECT_THROW(entrypoint(), rclcpp::exceptions::RCLError);
  if (rclcpp::ok()) {rclcpp::shutdown();}
  mode = Mode::live_other_error;
  EXPECT_THROW(entrypoint(), std::runtime_error);
  if (rclcpp::ok()) {rclcpp::shutdown();}
}
