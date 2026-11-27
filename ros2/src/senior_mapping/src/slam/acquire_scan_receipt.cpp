#include <senior_sensor_ingress/publisher_topology.hpp>
#include <senior_sensor_ingress/ros_wiring.hpp>
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
namespace admission = senior_mapping::scan_receipt_admission;
using admission::Message;

Message::ConstSharedPtr acquire(const rclcpp::Node::SharedPtr & node)
{
  const auto raw = node->declare_parameter<std::string>("raw_topic", "/scan");
  const auto bounded = node->declare_parameter<std::string>("bounded_topic", senior_sensor_ingress::wiring::safety_raw_scan);
  const auto frame = node->declare_parameter<std::string>("expected_frame", "laser_link");
  if (raw.empty() || raw.front() != '/' || bounded.empty() || bounded.front() != '/' ||
    raw == bounded || !admission::text_ok(frame))
  {
    throw std::invalid_argument("expected distinct absolute topics and an exact scan frame");
  }

  const auto deadline = std::chrono::steady_clock::now() + 8s;
  Message::ConstSharedPtr acquired;
  const auto subscription = node->create_subscription<Message>(
    bounded, rclcpp::SensorDataQoS().keep_last(1),
    [&](Message::ConstSharedPtr scan) {
      if (acquired) return;
      const auto publishers = node->get_publishers_info_by_topic(bounded);
      if (!senior_sensor_ingress::publisher_matches(publishers,
          senior_sensor_ingress::wiring::node_scan_receipt,
          senior_sensor_ingress::wiring::ns_scan_receipt,
          "senior_interfaces/msg/BoundedRawLaserScan")) return;
      const auto & capture = scan->capture;
      if (!capture.valid || capture.frame_id != frame || capture.bridge_boot_id.empty() ||
          capture.capture_generation == 0 || !admission::sample_valid(capture) ||
          !admission::layout_ok(*scan)) return;
      acquired = std::move(scan);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(20ms);
    if (acquired) return acquired;
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
      std::cerr << "No structurally valid scan received before acquisition timeout\n";
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
