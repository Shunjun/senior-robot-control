#include <memory>
#include <stdexcept>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "senior_sensor_ingress/static_tf_receipt.hpp"

namespace {
enum class Mode {closed_graph, live_rcl_error, live_other_error};
Mode mode;
}
namespace rclcpp {
template<typename NodeT>
void test_spin(std::shared_ptr<NodeT> node)
{
  if (mode == Mode::closed_graph) {
    rclcpp::shutdown();
    node->get_publishers_info_by_topic("/shutdown_graph_test");
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
#define main static_tf_entrypoint
#include "../src/static_tf_main.cpp"
#undef main
#undef spin

TEST(StaticTfShutdown, ClosedContextGraphQueryExitsCleanly)
{
  mode = Mode::closed_graph;
  char name[] = "static_tf_test";
  char * argv[] = {name, nullptr};
  EXPECT_NO_THROW(EXPECT_EQ(static_tf_entrypoint(1, argv), 0));
  if (rclcpp::ok()) rclcpp::shutdown();
}

TEST(StaticTfShutdown, LiveErrorsPropagate)
{
  char name[] = "static_tf_test";
  char * argv[] = {name, nullptr};
  mode = Mode::live_rcl_error;
  EXPECT_THROW(static_tf_entrypoint(1, argv), rclcpp::exceptions::RCLError);
  if (rclcpp::ok()) rclcpp::shutdown();
  mode = Mode::live_other_error;
  EXPECT_THROW(static_tf_entrypoint(1, argv), std::runtime_error);
  if (rclcpp::ok()) rclcpp::shutdown();
}
