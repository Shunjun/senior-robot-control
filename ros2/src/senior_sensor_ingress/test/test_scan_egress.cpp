#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <senior_interfaces/msg/bounded_filtered_scan.hpp>
#include <senior_interfaces/msg/bounded_raw_laser_scan.hpp>
#include <senior_interfaces/msg/bounded_safety_scan.hpp>
#include <senior_interfaces/msg/fenced_filtered_scan.hpp>
#include <senior_interfaces/msg/fenced_safety_scan.hpp>

#include "senior_sensor_ingress/scan_receipt.hpp"
#include "senior_sensor_ingress/raw_receipt.hpp"

using namespace std::chrono_literals;
namespace {
bool wait_until(const std::function<bool()> & predicate)
{
  const auto end = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

TEST(ScanEgress, RawSafetyAndFilteredBodiesRequireUnchangedSourceBeams)
{
  rclcpp::init(0, nullptr);
  auto bridge = std::make_shared<senior_sensor_ingress::ScanReceipt>();
  auto fixture = std::make_shared<rclcpp::Node>("scan_egress_fixture");
  using Scan = sensor_msgs::msg::LaserScan;
  using Raw = senior_interfaces::msg::BoundedRawLaserScan;
  using Safety = senior_interfaces::msg::BoundedSafetyScan;
  using Filtered = senior_interfaces::msg::BoundedFilteredScan;
  using SafetyOut = senior_interfaces::msg::FencedSafetyScan;
  using FilteredOut = senior_interfaces::msg::FencedFilteredScan;
  std::mutex mutex;
  std::vector<Raw> raw;
  std::vector<SafetyOut> safety;
  std::vector<FilteredOut> filtered;
  auto source = fixture->create_publisher<Scan>("/scan", rclcpp::SensorDataQoS());
  auto safety_owner = fixture->create_publisher<Safety>("/safety/scan_candidate", 10);
  auto filter_owner = fixture->create_publisher<Filtered>("/perception/scan_candidate", 10);
  auto raw_sub = fixture->create_subscription<Raw>(
    "/safety/raw/scan", rclcpp::SensorDataQoS(), [&](Raw::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); raw.push_back(*value);
    });
  auto safety_sub = fixture->create_subscription<SafetyOut>(
    "/safety/bounded_scan", 10, [&](SafetyOut::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); safety.push_back(*value);
    });
  auto filter_sub = fixture->create_subscription<FilteredOut>(
    "/perception/scan_filtered", rclcpp::SensorDataQoS(), [&](FilteredOut::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); filtered.push_back(*value);
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bridge); executor.add_node(fixture);
  std::thread thread([&] {executor.spin();});
  auto cleanup = [&](int *) {
    executor.cancel();
    if (thread.joinable()) thread.join();
    rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_until([&] {
    return source->get_subscription_count() && safety_owner->get_subscription_count() &&
           filter_owner->get_subscription_count();
  }));
  Scan scan;
  scan.header.stamp = fixture->now(); scan.header.frame_id = "laser";
  scan.angle_min = -1; scan.angle_max = 1; scan.angle_increment = 1;
  scan.range_min = 0.1; scan.range_max = 10; scan.ranges = {1, 2, 3};
  source->publish(scan);
  Raw original;
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : raw) if (value.capture.valid) {original = value; return true;}
    return false;
  }));
  Safety safe;
  safe.raw_capture = original.capture;
  safe.stamp = original.capture.stamp; safe.frame_id = original.capture.frame_id;
  safe.raw_source = original.capture.raw_gid; safe.source_epoch = "safety-owner";
  safe.sequence = 1; safe.valid = true;
  safe.ingress_steady = (static_cast<double>(original.capture.received_steady_ns) / 1e9);
  safe.ingress_ros = static_cast<double>(rclcpp::Time(original.capture.received_ros).nanoseconds()) / 1e9;
  safe.angle_min = original.angle_min; safe.angle_increment = original.angle_increment;
  safe.range_min = original.range_min; safe.range_max = original.range_max;
  safe.ranges = original.ranges;
  Filtered filt;
  filt.raw_capture = original.capture; filt.source_epoch = "filter-owner";
  filt.sequence = 1; filt.valid = true;
  filt.angle_min = original.angle_min; filt.angle_max = original.angle_max;
  filt.angle_increment = original.angle_increment; filt.time_increment = original.time_increment;
  filt.scan_time = original.scan_time; filt.range_min = original.range_min; filt.range_max = original.range_max;
  filt.ranges = original.ranges;
  filt.ranges[1] = std::numeric_limits<float>::quiet_NaN();
  safety_owner->publish(safe); filter_owner->publish(filt);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return !safety.empty() && safety.back().owner.valid &&
           !filtered.empty() && filtered.back().owner.valid;
  }));
  safe.sequence = 2; safe.ranges[0] = 9;
  filt.sequence = 2; filt.ranges[2] = 9;
  safety_owner->publish(safe); filter_owner->publish(filt);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return !safety.empty() && !safety.back().owner.valid &&
           !filtered.empty() && !filtered.back().owner.valid;
  }));
  EXPECT_EQ(safety.back().owner.reason, "scan_body_mismatch");
  EXPECT_EQ(filtered.back().owner.reason, "scan_body_mismatch");
  (void)raw_sub; (void)safety_sub; (void)filter_sub;
}

}  // namespace
