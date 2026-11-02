#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <senior_motion_msgs/msg/bounded_raw_image.hpp>
#include <senior_motion_msgs/msg/bounded_raw_camera_info.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "senior_sensor_ingress/raw_receipt.hpp"

using namespace std::chrono_literals;

namespace {
bool wait_until(const std::function<bool()> & predicate, std::chrono::seconds timeout = 5s)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

TEST(RawReceipt, RealCallbackRejectsQueuedOldPublisherAfterGraphReplacement)
{
  rclcpp::init(0, nullptr);
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false, resume = false;
  std::atomic<bool> arm{false};
  std::string incoming_gid;
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("rgb_topic", "/raw_binding_test/rgb"),
    rclcpp::Parameter("rgb_output", "/raw_binding_test/bounded_rgb"),
  });
  auto bridge = std::make_shared<senior_sensor_ingress::RawReceipt>(
    options, [&](size_t stream, const rmw_gid_t & gid) {
      if (stream != 0 || !arm.exchange(false)) return;
      std::unique_lock<std::mutex> lock(mutex);
      incoming_gid = senior_sensor_ingress::gid_hex(gid);
      entered = true;
      condition.notify_all();
      condition.wait_for(lock, 5s, [&] {return resume;});
    });
  auto fixture = std::make_shared<rclcpp::Node>("raw_binding_fixture");
  using Image = sensor_msgs::msg::Image;
  using Bounded = senior_motion_msgs::msg::BoundedRawImage;
  std::vector<Bounded> received;
  auto output = fixture->create_subscription<Bounded>(
    "/raw_binding_test/bounded_rgb", rclcpp::SensorDataQoS(),
    [&](Bounded::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*message);
    });
  auto old_publisher = fixture->create_publisher<Image>(
    "/raw_binding_test/rgb", rclcpp::SensorDataQoS());
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
  ASSERT_TRUE(wait_until([&] {
    return bridge->get_publishers_info_by_topic("/raw_binding_test/rgb").size() == 1 &&
           old_publisher->get_subscription_count() > 0;
  }));
  Image image;
  image.header.stamp = fixture->now();
  image.header.frame_id = "optical";
  image.width = image.height = 2;
  image.step = 6;
  image.encoding = "rgb8";
  image.data.assign(12, 7);
  const auto expected_stamp = image.header.stamp;
  const auto old_gid = senior_sensor_ingress::gid_hex(old_publisher->get_gid());
  const auto bridge_graph = bridge->get_publishers_info_by_topic("/raw_binding_test/bounded_rgb");
  ASSERT_EQ(bridge_graph.size(), 1u);
  const auto bridge_gid = senior_sensor_ingress::gid_hex(bridge_graph[0].endpoint_gid());
  old_publisher->publish(image);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) {
      if (item.capture.valid && item.capture.raw_gid == old_gid) return true;
    }
    return false;
  }));
  size_t valid_before = 0;
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) {
      if (item.capture.valid) {
        ++valid_before;
        EXPECT_EQ(item.capture.stamp, expected_stamp);
        EXPECT_GT(item.capture.received_steady, 0.0);
        EXPECT_EQ(item.capture.bridge_gid, bridge_gid);
        EXPECT_FALSE(item.capture.bridge_boot_id.empty());
        EXPECT_GT(item.capture.capture_generation, 0u);
      }
    }
  }
  auto malformed = image;
  malformed.header.stamp = fixture->now();
  malformed.step = 5;
  old_publisher->publish(malformed);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) {
      if (!item.capture.valid && item.capture.reason == "raw_image_invalid" &&
          item.capture.stamp == malformed.header.stamp && item.capture.raw_gid == old_gid &&
          item.capture.received_steady > 0 && item.capture.received_ros.sec > 0) return true;
    }
    return false;
  }));
  arm = true;
  old_publisher->publish(image);
  ASSERT_TRUE(wait_until([&] {std::lock_guard<std::mutex> lock(mutex); return entered;}));
  old_publisher.reset();
  auto new_publisher = fixture->create_publisher<Image>(
    "/raw_binding_test/rgb", rclcpp::SensorDataQoS());
  const auto new_gid = senior_sensor_ingress::gid_hex(new_publisher->get_gid());
  const bool replaced = wait_until([&] {
    const auto graph = bridge->get_publishers_info_by_topic("/raw_binding_test/rgb");
    return graph.size() == 1 && senior_sensor_ingress::gid_hex(graph[0].endpoint_gid()) == new_gid;
  });
  {
    std::lock_guard<std::mutex> lock(mutex);
    resume = true;
    condition.notify_all();
  }
  ASSERT_TRUE(replaced);
  EXPECT_EQ(incoming_gid, old_gid);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) {
      if (!item.capture.valid && item.capture.reason == "raw_gid_mismatch" &&
          item.capture.raw_gid == old_gid && item.capture.stamp == image.header.stamp &&
          item.capture.received_steady > 0) return true;
    }
    return false;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex);
    size_t count = 0;
    for (const auto & item : received) if (item.capture.valid) ++count;
    EXPECT_EQ(count, valid_before);  // old callback was not relabeled as new publisher
  }
  ASSERT_TRUE(wait_until([&] {return new_publisher->get_subscription_count() > 0;}));
  image.header.stamp = fixture->now();
  new_publisher->publish(image);
  EXPECT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) {
      if (item.capture.valid && item.capture.raw_gid == new_gid) return true;
    }
    return false;
  }));
  using Info = sensor_msgs::msg::CameraInfo;
  using BoundedInfo = senior_motion_msgs::msg::BoundedRawCameraInfo;
  std::vector<Bounded> depths;
  std::vector<BoundedInfo> rgb_infos, depth_infos;
  auto depth_output = fixture->create_subscription<Bounded>(
    "/safety/raw/depth", rclcpp::SensorDataQoS(), [&](Bounded::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); depths.push_back(*value);
    });
  auto rgb_info_output = fixture->create_subscription<BoundedInfo>(
    "/safety/raw/rgb_info", rclcpp::SensorDataQoS(), [&](BoundedInfo::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); rgb_infos.push_back(*value);
    });
  auto depth_info_output = fixture->create_subscription<BoundedInfo>(
    "/safety/raw/depth_info", rclcpp::SensorDataQoS(), [&](BoundedInfo::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); depth_infos.push_back(*value);
    });
  auto depth_publisher = fixture->create_publisher<Image>(
    "/camera/depth/image_raw", rclcpp::SensorDataQoS());
  auto rgb_info_publisher = fixture->create_publisher<Info>(
    "/camera/color/camera_info", rclcpp::SensorDataQoS());
  auto depth_info_publisher = fixture->create_publisher<Info>(
    "/camera/depth/camera_info", rclcpp::SensorDataQoS());
  ASSERT_TRUE(wait_until([&] {
    return depth_publisher->get_subscription_count() &&
           rgb_info_publisher->get_subscription_count() &&
           depth_info_publisher->get_subscription_count();
  }));
  Image depth;
  depth.header.frame_id = "optical";
  depth.width = depth.height = 2;
  depth.encoding = "16UC1";
  depth.step = 4;
  depth.is_bigendian = 1;
  depth.data = {0, 100, 0, 101, 0, 102, 0, 103};
  Info info;
  info.header.frame_id = "optical";
  info.width = info.height = 2;
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.k = {2, 0, 1, 0, 2, 1, 0, 0, 1};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {2, 0, 1, 0, 0, 2, 1, 0, 0, 0, 1, 0};
  bool qualified_bundle = false;
  for (int attempt = 0; attempt < 10 && !qualified_bundle; ++attempt) {
    image.header.stamp = depth.header.stamp = info.header.stamp = fixture->now();
    const auto stamp = image.header.stamp;
    new_publisher->publish(image);
    depth_publisher->publish(depth);
    rgb_info_publisher->publish(info);
    depth_info_publisher->publish(info);
    qualified_bundle = wait_until([&] {
      std::lock_guard<std::mutex> lock(mutex);
      for (const auto & rgb : received) {
        if (!rgb.capture.valid || rgb.capture.stamp != stamp) continue;
        for (const auto & dep : depths) {
          if (!dep.capture.valid || dep.capture.stamp != stamp ||
              dep.capture.capture_generation != rgb.capture.capture_generation ||
              dep.capture.bridge_boot_id != rgb.capture.bridge_boot_id) continue;
          for (const auto & ci : rgb_infos) {
            if (!ci.capture.valid || ci.capture.stamp != stamp ||
                ci.capture.capture_generation != rgb.capture.capture_generation ||
                ci.capture.bridge_boot_id != rgb.capture.bridge_boot_id) continue;
            for (const auto & di : depth_infos) {
              if (!di.capture.valid || di.capture.stamp != stamp ||
                  di.capture.capture_generation != rgb.capture.capture_generation ||
                  di.capture.bridge_boot_id != rgb.capture.bridge_boot_id) continue;
              EXPECT_EQ(rgb.capture.raw_gid, new_gid);
              EXPECT_EQ(dep.capture.raw_gid, senior_sensor_ingress::gid_hex(depth_publisher->get_gid()));
              EXPECT_EQ(ci.capture.raw_gid, senior_sensor_ingress::gid_hex(rgb_info_publisher->get_gid()));
              EXPECT_EQ(di.capture.raw_gid, senior_sensor_ingress::gid_hex(depth_info_publisher->get_gid()));
              EXPECT_TRUE(dep.is_bigendian);
              EXPECT_EQ(dep.data.size(), depth.data.size());
              EXPECT_TRUE(dep.data.size() == depth.data.size() &&
                          std::equal(dep.data.begin(), dep.data.end(), depth.data.begin()));
              EXPECT_GT(rgb.capture.received_steady, 0.0);
              EXPECT_GT(dep.capture.received_steady, 0.0);
              EXPECT_GT(ci.capture.received_steady, 0.0);
              EXPECT_GT(di.capture.received_steady, 0.0);
              EXPECT_EQ(ci.k, info.k);
              EXPECT_EQ(di.r, info.r);
              return true;
            }
          }
        }
      }
      return false;
    }, 1s);
  }
  ASSERT_TRUE(qualified_bundle);
  auto malformed_info = info;
  malformed_info.header.stamp = fixture->now();
  malformed_info.d.assign(17, 0.0);
  rgb_info_publisher->publish(malformed_info);
  EXPECT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : rgb_infos) {
      if (!value.capture.valid && value.capture.reason == "raw_camera_info_invalid" &&
          value.capture.stamp == malformed_info.header.stamp &&
          value.capture.raw_gid == senior_sensor_ingress::gid_hex(rgb_info_publisher->get_gid()) &&
          value.capture.received_steady > 0) return true;
    }
    return false;
  }));
  (void)output;
  (void)depth_output;
  (void)rgb_info_output;
  (void)depth_info_output;
}
TEST(RawReceipt, RetiredRealRawCallbackCannotRevokeSuccessor)
{
  rclcpp::init(0, nullptr);
  std::mutex mutex;
  std::condition_variable condition;
  bool entered = false, resume = false;
  std::atomic<bool> arm{false};
  auto bridge = std::make_shared<senior_sensor_ingress::RawReceipt>(
    rclcpp::NodeOptions(), [&](size_t stream, const rmw_gid_t &) {
      if (stream != 0 || !arm.exchange(false)) return;
      std::unique_lock<std::mutex> lock(mutex);
      entered = true;
      condition.notify_all();
      condition.wait_for(lock, 5s, [&] {return resume;});
    }, true);
  auto fixture = std::make_shared<rclcpp::Node>("raw_retired_fixture");
  using Image = sensor_msgs::msg::Image;
  using Bounded = senior_motion_msgs::msg::BoundedRawImage;
  std::vector<Bounded> received;
  auto output = fixture->create_subscription<Bounded>(
    "/safety/raw/rgb", rclcpp::SensorDataQoS(), [&](Bounded::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex);
      received.push_back(*value);
    });
  auto old = fixture->create_publisher<Image>("/camera/color/image_raw", rclcpp::SensorDataQoS());
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
  Image image;
  image.header.stamp = fixture->now();
  image.header.frame_id = "optical";
  image.width = image.height = 2;
  image.encoding = "rgb8";
  image.step = 6;
  image.data.assign(12, 1);
  const auto old_gid = senior_sensor_ingress::gid_hex(old->get_gid());
  old->publish(image);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) {
      if (value.capture.valid && value.capture.raw_gid == old_gid) return true;
    }
    return false;
  }));
  arm = true;
  old->publish(image);
  ASSERT_TRUE(wait_until([&] {std::lock_guard<std::mutex> lock(mutex); return entered;}));
  old.reset();
  auto current = fixture->create_publisher<Image>(
    "/camera/color/image_raw", rclcpp::SensorDataQoS());
  const auto new_gid = senior_sensor_ingress::gid_hex(current->get_gid());
  ASSERT_TRUE(wait_until([&] {
    const auto graph = bridge->get_publishers_info_by_topic("/camera/color/image_raw");
    return graph.size() == 1 && senior_sensor_ingress::gid_hex(graph[0].endpoint_gid()) == new_gid;
  }));
  image.header.stamp = fixture->now();
  current->publish(image);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) {
      if (value.capture.valid && value.capture.raw_gid == new_gid) return true;
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
  (void)output;
}
}  // namespace
