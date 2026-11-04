#include "scan_receipt_admission.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "rclcpp/serialization.hpp"

namespace
{
using namespace std::chrono_literals;
namespace admission = senior_slam_bridge::scan_receipt_admission;
using admission::Message;

Message::ConstSharedPtr acquire(const rclcpp::Node::SharedPtr & node)
{
  const auto raw = node->declare_parameter<std::string>("raw_topic", "/scan");
  const auto bounded = node->declare_parameter<std::string>("bounded_topic", "/safety/raw/scan");
  const auto frame = node->declare_parameter<std::string>("expected_frame", "laser_link");
  if (raw.empty() || raw.front() != '/' || bounded.empty() || bounded.front() != '/' ||
    raw == bounded || frame.size() > 128 || !admission::text_ok(frame))
  {
    throw std::invalid_argument("expected distinct absolute topics and an exact scan frame");
  }

  const auto deadline = std::chrono::steady_clock::now() + 8s;
  std::optional<admission::ScanOwner> owner;
  std::int64_t ready_ros = 0;
  std::int64_t last_ros = 0;
  double ready_steady = 0;
  Message::ConstSharedPtr baseline, acquired;
  bool retired = false;

  const auto check_owner = [&]() {
      const auto current = admission::unique_scan_owner(*node, bounded, raw);
      return owner && current && *current == *owner;
    };
  const auto subscription = node->create_subscription<Message>(
    bounded, rclcpp::SensorDataQoS().keep_last(1),
    [&](Message::ConstSharedPtr scan, const rclcpp::MessageInfo & info) {
      if (!owner || retired || acquired) {
        return;
      }
      const auto now_ros = node->now().nanoseconds();
      if (now_ros < last_ros || !check_owner()) {
        retired = true;
        return;
      }
      last_ros = now_ros;
      const auto & receipt = scan->capture;
      const auto actual = admission::gid_hex(info.get_rmw_message_info().publisher_gid.data);
      if (actual != owner->bridge_gid || receipt.bridge_gid != owner->bridge_gid) {
        return;
      }
      if (baseline && (!receipt.valid || !admission::same_source(baseline->capture, receipt))) {
        retired = true;
        return;
      }
      const auto stamp = admission::time_ns(receipt.stamp);
      if (!receipt.valid || !owner->matches(receipt) || stamp <= ready_ros ||
        admission::time_ns(receipt.received_ros) <= ready_ros ||
        !(receipt.received_steady > ready_steady) || receipt.sequence == 0 ||
        receipt.capture_generation == 0 || receipt.frame_id != frame ||
        !admission::text_ok(receipt.bridge_boot_id) || !admission::text_ok(receipt.source_epoch) ||
        !admission::fresh(receipt, now_ros, admission::steady_seconds()) ||
        !admission::layout_ok(*scan))
      {
        return;
      }
      if (!baseline) {
        // The relay announces a source transition before its first valid scan.
        // Pin the resulting epoch and require a later scan in that exact epoch.
        baseline = std::move(scan);
      } else if (receipt.sequence > baseline->capture.sequence &&
        stamp > admission::time_ns(baseline->capture.stamp))
      {
        acquired = std::move(scan);
      }
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline && !retired) {
    executor.spin_once(20ms);
    const auto now_ros = node->now().nanoseconds();
    if (owner && (now_ros < last_ros || !check_owner())) {
      break;
    }
    last_ros = now_ros;
    if (acquired) {
      if (std::chrono::steady_clock::now() < deadline && rclcpp::ok() &&
        admission::fresh(acquired->capture, now_ros, admission::steady_seconds()))
      {
        return acquired;
      }
      break;
    }
    if (!owner && now_ros > 0) {
      owner = admission::unique_scan_owner(*node, bounded, raw);
      if (owner) {
        ready_ros = now_ros;
        ready_steady = admission::steady_seconds();
        std::cerr << "Scan receipt armed " << ready_ros << '\n';
      }
    }
  }
  return nullptr;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (setenv("RCUTILS_LOGGING_USE_STDOUT", "0", 1) != 0) {
      throw std::runtime_error("Could not reserve stdout for the scan receipt");
    }
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<rclcpp::Node>("acquire_scan_receipt");
    const auto scan = acquire(node);
    if (!scan) {
      rclcpp::shutdown();
      std::cerr << "No fresh scan receipt from the pinned source\n";
      return 1;
    }
    // Stdout is exactly one ROS CDR message. The supervising reader owns the
    // process/pipe deadline. This receipt attests acquisition only.
    rclcpp::Serialization<Message> serialization;
    rclcpp::SerializedMessage bytes;
    serialization.serialize_message(scan.get(), &bytes);
    rclcpp::shutdown();
    std::cout.write(
      reinterpret_cast<const char *>(bytes.get_rcl_serialized_message().buffer), bytes.size());
    std::cout.flush();
    return std::cout ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "Scan receipt acquisition failed: " << error.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
