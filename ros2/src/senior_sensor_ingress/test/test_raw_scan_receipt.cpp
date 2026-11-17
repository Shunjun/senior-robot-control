#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl/time.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <senior_motion_msgs/msg/bounded_raw_laser_scan.hpp>

#include "senior_sensor_ingress/raw_receipt.hpp"
#include "senior_sensor_ingress/scan_receipt.hpp"

using namespace std::chrono_literals;

namespace {
bool wait_until(const std::function<bool()> & predicate,
                std::chrono::milliseconds timeout = 5s)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

TEST(RawScanReceipt, RealMessageIdentityAndRetiredPublisherAfterSuccessor)
{
  rclcpp::init(0, nullptr);
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false, resume = false;
  std::atomic<bool> arm{false};
  auto bridge = std::make_shared<senior_sensor_ingress::ScanReceipt>(
    rclcpp::NodeOptions(), [&](size_t stream, const rmw_gid_t &) {
      if (stream != 6 || !arm.exchange(false)) return;
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      condition.notify_all();
      condition.wait_for(lock, 5s, [&] {return resume;});
    }, true);
  auto fixture = std::make_shared<rclcpp::Node>("raw_scan_fixture");
  using Scan = sensor_msgs::msg::LaserScan;
  using Bounded = senior_motion_msgs::msg::BoundedRawLaserScan;
  std::vector<Bounded> received;
  auto output = fixture->create_subscription<Bounded>(
    "/safety/raw/scan", rclcpp::SensorDataQoS(), [&](Bounded::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*msg);
    });
  auto old = fixture->create_publisher<Scan>("/scan", rclcpp::SensorDataQoS());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(fixture);
  std::thread thread([&] {executor.spin();});
  auto cleanup = [&](int *) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      resume = true;
      condition.notify_all();
    }
    executor.cancel();
    if (thread.joinable()) thread.join();
    rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_until([&] {return old->get_subscription_count() > 0;}));
  Scan scan;
  scan.header.stamp = fixture->now() - rclcpp::Duration::from_seconds(.2);
  scan.header.frame_id = "laser";
  scan.angle_min = -1; scan.angle_max = 1; scan.angle_increment = 1;
  scan.range_min = 0.1; scan.range_max = 10;
  scan.ranges = {1, 2, 3};
  const auto old_gid = senior_sensor_ingress::gid_hex(old->get_gid());
  const auto output_graph = bridge->get_publishers_info_by_topic("/safety/raw/scan");
  ASSERT_EQ(output_graph.size(), 1u);
  const auto bridge_gid = senior_sensor_ingress::gid_hex(output_graph[0].endpoint_gid());
  old->publish(scan);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & msg : received) {
      if (msg.capture.valid && msg.capture.raw_gid == old_gid &&
          msg.capture.stamp == scan.header.stamp && (static_cast<double>(msg.capture.received_steady_ns) / 1e9) > 0 &&
          msg.capture.bridge_gid == bridge_gid && msg.ranges.size() == 3 &&
          msg.ranges[0] == 1 && msg.ranges[1] == 2 && msg.ranges[2] == 3) {
        EXPECT_EQ(msg.capture.received_steady_ns - msg.capture.sample_steady_ns,
          static_cast<uint64_t>(rclcpp::Time(msg.capture.received_ros).nanoseconds() -
                                rclcpp::Time(msg.capture.stamp).nanoseconds()));
        EXPECT_GE(msg.capture.received_steady_ns - msg.capture.sample_steady_ns, 200000000ULL);
        EXPECT_GT(msg.capture.clock_generation, 0u);
        return true;
      }
    }
    return false;
  }));
  arm = true;
  old->publish(scan);
  ASSERT_TRUE(wait_until([&] {std::lock_guard<std::mutex> lock(mutex); return entered;}));
  old.reset();
  auto current = fixture->create_publisher<Scan>("/scan", rclcpp::SensorDataQoS());
  const auto new_gid = senior_sensor_ingress::gid_hex(current->get_gid());
  ASSERT_TRUE(wait_until([&] {
    const auto graph = bridge->get_publishers_info_by_topic("/scan");
    return graph.size() == 1 && senior_sensor_ingress::gid_hex(graph[0].endpoint_gid()) == new_gid;
  }));
  scan.header.stamp = fixture->now();
  current->publish(scan);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & msg : received) {
      if (msg.capture.valid && msg.capture.raw_gid == new_gid) return true;
    }
    return false;
  }, 1s));
  {
    std::lock_guard<std::mutex> lock(mutex);
    resume = true;
    condition.notify_all();
  }
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_FALSE(received.empty());
    EXPECT_TRUE(received.back().capture.valid);
    EXPECT_EQ(received.back().capture.raw_gid, new_gid);
  }
  scan.time_increment = 0.001;
  scan.header.stamp = fixture->now();
  current->publish(scan);
  EXPECT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & msg : received) {
      if (!msg.capture.valid && msg.capture.reason == "raw_scan_invalid") return true;
    }
    return false;
  }));
  for (int i = 0; i < 3; ++i) current->publish(scan);
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(mutex);
    size_t failures = 0;
    for (const auto & msg : received) {
      if (!msg.capture.valid && msg.capture.reason == "raw_scan_invalid") ++failures;
    }
    EXPECT_EQ(failures, 1u);
  }
  scan.time_increment = 0;
  scan.header.stamp = fixture->now();
  current->publish(scan);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return !received.empty() && received.back().capture.valid &&
           received.back().capture.raw_gid == new_gid &&
           received.back().capture.stamp == scan.header.stamp;
  }));
  scan.ranges.assign(2049, 1.0f);
  scan.header.stamp = fixture->now();
  current->publish(scan);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return !received.empty() && !received.back().capture.valid &&
           received.back().capture.reason == "raw_scan_invalid";
  }));
  (void)output;
}

TEST(RawScanReceipt, RollbackDuringQualificationCannotRelabelOldCallback)
{
  rclcpp::init(0, nullptr);
  std::atomic<bool> arm{false};
  rclcpp::Clock::SharedPtr clock;
  rclcpp::NodeOptions options;
  options.append_parameter_override("use_sim_time", true);
  auto bridge = std::make_shared<senior_sensor_ingress::ScanReceipt>(
    options, [&](size_t stream, const rmw_gid_t &) {
      if (stream == 6 && arm.exchange(false)) {
        std::lock_guard<std::mutex> clock_lock(clock->get_clock_mutex());
        EXPECT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 90000000000LL), RCL_RET_OK);
      }
    });
  clock = bridge->get_clock();
  {
    std::lock_guard<std::mutex> clock_lock(clock->get_clock_mutex());
    ASSERT_EQ(rcl_enable_ros_time_override(clock->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(rcl_set_ros_time_override(clock->get_clock_handle(), 100000000000LL), RCL_RET_OK);
  }
  auto fixture = std::make_shared<rclcpp::Node>("scan_clock_qualification_fixture");
  using Scan = sensor_msgs::msg::LaserScan;
  using Bounded = senior_motion_msgs::msg::BoundedRawLaserScan;
  std::mutex mutex;
  std::vector<Bounded> received;
  auto publisher = fixture->create_publisher<Scan>("/scan", rclcpp::SensorDataQoS());
  auto subscription = fixture->create_subscription<Bounded>(
    "/safety/raw/scan", rclcpp::SensorDataQoS(), [&](Bounded::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*message);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge); executor.add_node(fixture);
  std::thread thread([&] {executor.spin();});
  auto cleanup = [&](int *) {
    executor.cancel(); if (thread.joinable()) thread.join(); rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_until([&] {return publisher->get_subscription_count() > 0;}));
  Scan scan;
  scan.header.frame_id = "laser";
  scan.angle_min = -1; scan.angle_max = 1; scan.angle_increment = 1;
  scan.range_min = .1; scan.range_max = 10; scan.ranges = {1, 2, 3};
  scan.header.stamp = rclcpp::Time(99900000000LL);
  publisher->publish(scan);
  uint64_t original_generation = 0;
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.capture.valid) {
      original_generation = item.capture.clock_generation;
      return true;
    }
    return false;
  }));
  // Entry is in ROS100; the existing qualification hook changes it to ROS90.
  scan.header.stamp = rclcpp::Time(99990000000LL);
  const auto old_stamp = scan.header.stamp;
  arm = true;
  publisher->publish(scan);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.capture.reason == "clock_rollback") return true;
    return false;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) EXPECT_FALSE(item.capture.valid && item.capture.stamp == old_stamp);
  }
  // The rejected callback must not poison the new segment's stamp frontier.
  scan.header.stamp = rclcpp::Time(89990000000LL);
  publisher->publish(scan);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.capture.valid && item.capture.stamp == scan.header.stamp) {
      EXPECT_GT(item.capture.clock_generation, original_generation);
      EXPECT_EQ(rclcpp::Time(item.capture.received_ros).nanoseconds(), 90000000000LL);
      EXPECT_EQ(item.capture.received_steady_ns - item.capture.sample_steady_ns, 10000000ULL);
      return true;
    }
    return false;
  }));
  (void)subscription;
}

}  // namespace
