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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <senior_motion_msgs/msg/bounded_depth_evidence.hpp>
#include <senior_motion_msgs/msg/bounded_raw_camera_info.hpp>
#include <senior_motion_msgs/msg/bounded_raw_image.hpp>
#include <senior_motion_msgs/msg/fenced_depth_evidence.hpp>

#include "senior_sensor_ingress/raw_receipt.hpp"

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

TEST(DepthEgress, RealSourcesPreserveHeadersAndRejectCurrentHeaderMutation)
{
  rclcpp::init(0, nullptr);
  std::mutex pause_mutex;
  std::condition_variable pause_condition;
  bool entered = false, resume = false;
  std::atomic<bool> arm{false};
  auto bridge = std::make_shared<senior_sensor_ingress::RawReceipt>(
    rclcpp::NodeOptions(), [&](size_t stream, const rmw_gid_t &) {
      if (stream != 4 || !arm.exchange(false)) return;
      std::unique_lock<std::mutex> lock(pause_mutex);
      entered = true;
      pause_condition.notify_all();
      pause_condition.wait_for(lock, 5s, [&] {return resume;});
    }, true);
  auto fixture = std::make_shared<rclcpp::Node>("depth_egress_fixture");
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  using RawImage = senior_motion_msgs::msg::BoundedRawImage;
  using RawInfo = senior_motion_msgs::msg::BoundedRawCameraInfo;
  using Depth = senior_motion_msgs::msg::BoundedDepthEvidence;
  using Fenced = senior_motion_msgs::msg::FencedDepthEvidence;
  std::mutex mutex;
  std::vector<RawImage> images;
  std::vector<RawInfo> infos;
  std::vector<Fenced> results;
  auto depth_input = fixture->create_publisher<Image>("/camera/depth/image_raw", rclcpp::SensorDataQoS());
  auto info_input = fixture->create_publisher<Info>("/camera/depth/camera_info", rclcpp::SensorDataQoS());
  auto owner = fixture->create_publisher<Depth>("/safety/depth_candidate", rclcpp::SensorDataQoS());
  auto images_sub = fixture->create_subscription<RawImage>(
    "/safety/raw/depth", rclcpp::SensorDataQoS(), [&](RawImage::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex); images.push_back(*msg);
    });
  auto infos_sub = fixture->create_subscription<RawInfo>(
    "/safety/raw/depth_info", rclcpp::SensorDataQoS(), [&](RawInfo::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex); infos.push_back(*msg);
    });
  auto fenced_sub = fixture->create_subscription<Fenced>(
    "/safety/bounded_depth", rclcpp::SensorDataQoS(), [&](Fenced::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex); results.push_back(*msg);
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bridge);
  executor.add_node(fixture);
  std::thread thread([&] {executor.spin();});
  auto cleanup = [&](int *) {
    {
      std::lock_guard<std::mutex> lock(pause_mutex);
      resume = true;
      pause_condition.notify_all();
    }
    executor.cancel();
    if (thread.joinable()) thread.join();
    rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_until([&] {
    return depth_input->get_subscription_count() && info_input->get_subscription_count() &&
           owner->get_subscription_count();
  }));
  Image image;
  image.header.frame_id = "optical";
  image.width = image.height = 2;
  image.encoding = "16UC1";
  image.step = 4;
  image.data.assign(8, 2);
  Info info;
  info.header.frame_id = "optical";
  info.width = info.height = 2;
  info.distortion_model = "plumb_bob";
  Depth input;
  bool paired = false;
  for (int i = 0; i < 12 && !paired; ++i) {
    image.header.stamp = info.header.stamp = fixture->now();
    depth_input->publish(image);
    info_input->publish(info);
    paired = wait_until([&] {
      std::lock_guard<std::mutex> lock(mutex);
      for (const auto & a : images) for (const auto & b : infos) {
        if (a.capture.valid && b.capture.valid && a.capture.stamp == image.header.stamp &&
            b.capture.stamp == image.header.stamp &&
            a.capture.bridge_boot_id == b.capture.bridge_boot_id &&
            a.capture.capture_generation == b.capture.capture_generation) {
          input.depth_capture = a.capture;
          input.camera_info_capture = b.capture;
          return true;
        }
      }
      return false;
    }, 500ms);
  }
  ASSERT_TRUE(paired);
  input.stamp = input.depth_capture.stamp;
  input.camera_info_stamp = input.camera_info_capture.stamp;
  input.received_ros = input.depth_capture.received_ros;
  input.camera_info_received_ros = input.camera_info_capture.received_ros;
  input.received_steady = input.depth_capture.received_steady;
  input.camera_info_received_steady = input.camera_info_capture.received_steady;
  input.frame_id = input.depth_capture.frame_id;
  input.raw_source = input.depth_capture.raw_gid;
  input.camera_info_source = input.camera_info_capture.raw_gid;
  input.source_epoch = "fixture-owner";
  input.sequence = 1;
  input.valid = true;
  owner->publish(input);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : results) {
      if (value.owner.valid && value.body.depth_capture == input.depth_capture) return true;
    }
    return false;
  }));
  {
    std::lock_guard<std::mutex> lock(mutex);
    const auto & result = results.back();
    EXPECT_EQ(result.owner.owner_gid, senior_sensor_ingress::gid_hex(owner->get_gid()));
    EXPECT_EQ(result.body.camera_info_capture, input.camera_info_capture);
    EXPECT_EQ(result.body.received_steady, input.received_steady);
  }
  const auto original_capture = input.depth_capture;
  input.sequence = 2;
  input.depth_capture.received_steady += 0.001;
  owner->publish(input);
  EXPECT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : results) {
      if (!value.owner.valid && value.owner.reason == "capture_mismatch") return true;
    }
    return false;
  }));
  // The actual old DDS callback is paused before its MessageInfo graph check.
  // A new publisher is qualified before that old callback is released.
  input.depth_capture = original_capture;
  input.sequence = 3;
  owner->publish(input);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : results) {
      if (value.owner.valid && value.owner.owner_sequence == 3) return true;
    }
    return false;
  }));
  arm = true;
  input.valid = false;
  input.sequence = 4;
  owner->publish(input);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(pause_mutex);
    return entered;
  }));
  const auto old_gid = senior_sensor_ingress::gid_hex(owner->get_gid());
  owner.reset();
  auto successor = fixture->create_publisher<Depth>(
    "/safety/depth_candidate", rclcpp::SensorDataQoS());
  const auto new_gid = senior_sensor_ingress::gid_hex(successor->get_gid());
  ASSERT_TRUE(wait_until([&] {
    const auto graph = bridge->get_publishers_info_by_topic("/safety/depth_candidate");
    return graph.size() == 1 && senior_sensor_ingress::gid_hex(graph[0].endpoint_gid()) == new_gid;
  }));
  input.valid = true;
  input.source_epoch = "successor-owner";
  input.sequence = 1;
  successor->publish(input);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : results) {
      if (value.owner.valid && value.owner.owner_gid == new_gid) return true;
    }
    return false;
  }, 1s));
  {
    std::lock_guard<std::mutex> lock(pause_mutex);
    resume = true;
    pause_condition.notify_all();
  }
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_FALSE(results.empty());
    EXPECT_TRUE(results.back().owner.valid);
    EXPECT_EQ(results.back().owner.owner_gid, new_gid);
    EXPECT_NE(old_gid, new_gid);
  }
  // A delayed invalid body carrying an already-retired raw generation must not
  // revoke a newer valid body from the same current owner.
  const auto prior_input = input;
  image.header.stamp = fixture->now();
  image.encoding = "unsupported";
  depth_input->publish(image);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : images) {
      if (!value.capture.valid && value.capture.reason == "raw_image_invalid" &&
          value.capture.capture_generation > prior_input.depth_capture.capture_generation) return true;
    }
    return false;
  }));
  image.encoding = "16UC1";
  bool new_pair = false;
  for (int i = 0; i < 8 && !new_pair; ++i) {
    image.header.stamp = info.header.stamp = fixture->now();
    depth_input->publish(image);
    info_input->publish(info);
    new_pair = wait_until([&] {
      std::lock_guard<std::mutex> lock(mutex);
      for (const auto & a : images) for (const auto & b : infos) {
        if (a.capture.valid && b.capture.valid && a.capture.stamp == image.header.stamp &&
            b.capture.stamp == image.header.stamp &&
            a.capture.capture_generation == b.capture.capture_generation &&
            a.capture.capture_generation > prior_input.depth_capture.capture_generation) {
          input.depth_capture = a.capture;
          input.camera_info_capture = b.capture;
          return true;
        }
      }
      return false;
    }, 500ms);
  }
  ASSERT_TRUE(new_pair);
  input.stamp = input.depth_capture.stamp;
  input.camera_info_stamp = input.camera_info_capture.stamp;
  input.received_ros = input.depth_capture.received_ros;
  input.camera_info_received_ros = input.camera_info_capture.received_ros;
  input.received_steady = input.depth_capture.received_steady;
  input.camera_info_received_steady = input.camera_info_capture.received_steady;
  input.sequence = 2;
  successor->publish(input);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : results) {
      if (value.owner.valid && value.owner.owner_gid == new_gid &&
          value.body.depth_capture == input.depth_capture) return true;
    }
    return false;
  }));
  auto stale_invalid = prior_input;
  stale_invalid.valid = false;
  stale_invalid.sequence = 3;
  stale_invalid.reason = "stale_failure";
  successor->publish(stale_invalid);
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_FALSE(results.empty());
    EXPECT_TRUE(results.back().owner.valid);
    EXPECT_EQ(results.back().body.depth_capture, input.depth_capture);
  }
  (void)images_sub;
  (void)infos_sub;
  (void)fenced_sub;
}
}  // namespace
