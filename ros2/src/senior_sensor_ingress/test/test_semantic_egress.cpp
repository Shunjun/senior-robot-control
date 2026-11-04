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
#include <senior_motion_msgs/msg/bounded_semantic_evidence.hpp>
#include <senior_motion_msgs/msg/fenced_depth_evidence.hpp>
#include <senior_motion_msgs/msg/fenced_semantic_evidence.hpp>

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

TEST(SemanticEgress, RealFourStreamDepthReceiptAndBodyMutation)
{
  rclcpp::init(0, nullptr);
  std::mutex pause_mutex;
  std::condition_variable pause_condition;
  bool entered = false, resume = false;
  std::atomic<bool> arm{false};
  auto bridge = std::make_shared<senior_sensor_ingress::RawReceipt>(
    rclcpp::NodeOptions(), [&](size_t stream, const rmw_gid_t &) {
      if (stream != 5 || !arm.exchange(false)) return;
      std::unique_lock<std::mutex> lock(pause_mutex);
      entered = true;
      pause_condition.notify_all();
      pause_condition.wait_for(lock, 5s, [&] {return resume;});
    }, true);
  auto fixture = std::make_shared<rclcpp::Node>("semantic_egress_fixture");
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  using RawImage = senior_motion_msgs::msg::BoundedRawImage;
  using RawInfo = senior_motion_msgs::msg::BoundedRawCameraInfo;
  using Depth = senior_motion_msgs::msg::BoundedDepthEvidence;
  using DepthOutput = senior_motion_msgs::msg::FencedDepthEvidence;
  using Semantic = senior_motion_msgs::msg::BoundedSemanticEvidence;
  using SemanticOutput = senior_motion_msgs::msg::FencedSemanticEvidence;
  std::mutex mutex;
  std::vector<RawImage> rgb, depth;
  std::vector<RawInfo> rgb_info, depth_info;
  std::vector<DepthOutput> depths;
  std::vector<SemanticOutput> semantics;
  auto rgb_input = fixture->create_publisher<Image>("/camera/color/image_raw", rclcpp::SensorDataQoS());
  auto depth_input = fixture->create_publisher<Image>("/camera/depth/image_raw", rclcpp::SensorDataQoS());
  auto rgb_info_input = fixture->create_publisher<Info>("/camera/color/camera_info", rclcpp::SensorDataQoS());
  auto depth_info_input = fixture->create_publisher<Info>("/camera/depth/camera_info", rclcpp::SensorDataQoS());
  auto depth_owner = fixture->create_publisher<Depth>("/safety/depth_candidate", rclcpp::SensorDataQoS());
  auto semantic_owner = fixture->create_publisher<Semantic>(
    "/safety/semantic_candidate_unfenced", rclcpp::SensorDataQoS());
  auto rgb_sub = fixture->create_subscription<RawImage>(
    "/safety/raw/rgb", rclcpp::SensorDataQoS(), [&](RawImage::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); rgb.push_back(*value);
    });
  auto depth_sub = fixture->create_subscription<RawImage>(
    "/safety/raw/depth", rclcpp::SensorDataQoS(), [&](RawImage::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); depth.push_back(*value);
    });
  auto rgb_info_sub = fixture->create_subscription<RawInfo>(
    "/safety/raw/rgb_info", rclcpp::SensorDataQoS(), [&](RawInfo::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); rgb_info.push_back(*value);
    });
  auto depth_info_sub = fixture->create_subscription<RawInfo>(
    "/safety/raw/depth_info", rclcpp::SensorDataQoS(), [&](RawInfo::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); depth_info.push_back(*value);
    });
  auto depth_out = fixture->create_subscription<DepthOutput>(
    "/safety/bounded_depth", rclcpp::SensorDataQoS(), [&](DepthOutput::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); depths.push_back(*value);
    });
  auto semantic_out = fixture->create_subscription<SemanticOutput>(
    "/safety/semantic_candidates", rclcpp::SensorDataQoS(), [&](SemanticOutput::ConstSharedPtr value) {
      std::lock_guard<std::mutex> lock(mutex); semantics.push_back(*value);
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
    return rgb_input->get_subscription_count() && depth_input->get_subscription_count() &&
           rgb_info_input->get_subscription_count() && depth_info_input->get_subscription_count() &&
           depth_owner->get_subscription_count() && semantic_owner->get_subscription_count();
  }));
  Image color, dep;
  color.header.frame_id = dep.header.frame_id = "optical";
  color.width = color.height = dep.width = dep.height = 2;
  color.encoding = "rgb8"; color.step = 6; color.data.assign(12, 1);
  dep.encoding = "16UC1"; dep.step = 4; dep.data.assign(8, 1);
  Info info;
  info.header.frame_id = "optical"; info.width = info.height = 2; info.distortion_model = "plumb_bob";
  Semantic candidate;
  bool paired = false;
  for (int i = 0; i < 16 && !paired; ++i) {
    color.header.stamp = dep.header.stamp = info.header.stamp = fixture->now();
    rgb_input->publish(color); depth_input->publish(dep);
    rgb_info_input->publish(info); depth_info_input->publish(info);
    paired = wait_until([&] {
      std::lock_guard<std::mutex> lock(mutex);
      for (const auto & a : rgb) for (const auto & b : depth)
      for (const auto & c : rgb_info) for (const auto & d : depth_info) {
        if (!a.capture.valid || !b.capture.valid || !c.capture.valid || !d.capture.valid ||
            a.capture.stamp != color.header.stamp || b.capture.stamp != color.header.stamp ||
            c.capture.stamp != color.header.stamp || d.capture.stamp != color.header.stamp ||
            a.capture.capture_generation != b.capture.capture_generation ||
            a.capture.capture_generation != c.capture.capture_generation ||
            a.capture.capture_generation != d.capture.capture_generation) continue;
        candidate.rgb_capture = a.capture; candidate.rgb_camera_info_capture = c.capture;
        candidate.depth.depth_capture = b.capture; candidate.depth.camera_info_capture = d.capture;
        return true;
      }
      return false;
    }, 500ms);
  }
  ASSERT_TRUE(paired);
  auto & body = candidate.depth;
  body.stamp = body.depth_capture.stamp; body.camera_info_stamp = body.camera_info_capture.stamp;
  body.received_ros = body.depth_capture.received_ros;
  body.camera_info_received_ros = body.camera_info_capture.received_ros;
  body.received_steady = body.depth_capture.received_steady;
  body.camera_info_received_steady = body.camera_info_capture.received_steady;
  body.frame_id = body.depth_capture.frame_id;
  body.raw_source = body.depth_capture.raw_gid;
  body.camera_info_source = body.camera_info_capture.raw_gid;
  body.source_epoch = "depth-owner"; body.sequence = 1; body.valid = true;
  depth_owner->publish(body);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : depths) {
      if (value.owner.valid && value.body == body) {candidate.depth_owner = value.owner; return true;}
    }
    return false;
  }));
  candidate.rgb_stamp = candidate.rgb_capture.stamp;
  candidate.rgb_received_ros = candidate.rgb_capture.received_ros;
  candidate.rgb_received_steady = candidate.rgb_capture.received_steady;
  candidate.rgb_camera_info_stamp = candidate.rgb_camera_info_capture.stamp;
  candidate.rgb_camera_info_received_ros = candidate.rgb_camera_info_capture.received_ros;
  candidate.rgb_camera_info_received_steady = candidate.rgb_camera_info_capture.received_steady;
  candidate.rgb_frame_id = candidate.rgb_capture.frame_id;
  candidate.rgb_source = candidate.rgb_capture.raw_gid;
  candidate.rgb_camera_info_source = candidate.rgb_camera_info_capture.raw_gid;
  candidate.source_epoch = "semantic-owner"; candidate.sequence = 1; candidate.valid = true;
  semantic_owner->publish(candidate);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : semantics) {
      if (value.owner.valid && value.body == candidate) return true;
    }
    return false;
  }));
  const auto prior_depth_owner = candidate.depth_owner;
  body.sequence = 2;
  body.valid = false;
  body.reason = "depth_failure";
  depth_owner->publish(body);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : semantics) {
      if (!value.owner.valid && value.owner.upstream_invalid &&
          value.owner.reason == "depth_invalid") return true;
    }
    return false;
  }));
  body.sequence = 3;
  body.valid = true;
  body.reason.clear();
  depth_owner->publish(body);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : depths) {
      if (value.owner.valid && value.body == body) {candidate.depth_owner = value.owner; return true;}
    }
    return false;
  }));
  const auto recovered_depth_owner = candidate.depth_owner;
  candidate.depth_owner = prior_depth_owner;
  candidate.sequence = 2;
  semantic_owner->publish(candidate);
  std::this_thread::sleep_for(100ms);
  {
    std::lock_guard<std::mutex> lock(mutex);
    ASSERT_FALSE(semantics.empty());
    EXPECT_FALSE(semantics.back().owner.valid);
  }
  candidate.depth_owner = recovered_depth_owner;
  candidate.sequence = 3;
  semantic_owner->publish(candidate);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : semantics) {
      if (value.owner.valid && value.body == candidate) return true;
    }
    return false;
  }));
  candidate.sequence = 4;
  const auto original_depth = candidate.depth;
  candidate.depth.range_error_m += 0.001;
  semantic_owner->publish(candidate);
  EXPECT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : semantics) {
      if (!value.owner.valid && value.owner.reason == "depth_body_mismatch") return true;
    }
    return false;
  }));
  candidate.depth = original_depth;
  candidate.sequence = 5;
  semantic_owner->publish(candidate);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : semantics) {
      if (value.owner.valid && value.owner.owner_sequence == 5) return true;
    }
    return false;
  }));
  arm = true;
  candidate.valid = false;
  candidate.sequence = 6;
  semantic_owner->publish(candidate);
  ASSERT_TRUE(wait_until([&] {std::lock_guard<std::mutex> lock(pause_mutex); return entered;}));
  const auto old_gid = senior_sensor_ingress::gid_hex(semantic_owner->get_gid());
  semantic_owner.reset();
  auto successor = fixture->create_publisher<Semantic>(
    "/safety/semantic_candidate_unfenced", rclcpp::SensorDataQoS());
  const auto new_gid = senior_sensor_ingress::gid_hex(successor->get_gid());
  ASSERT_TRUE(wait_until([&] {
    const auto graph = bridge->get_publishers_info_by_topic("/safety/semantic_candidate_unfenced");
    return graph.size() == 1 && senior_sensor_ingress::gid_hex(graph[0].endpoint_gid()) == new_gid;
  }));
  candidate.valid = true;
  candidate.source_epoch = "semantic-successor";
  candidate.sequence = 1;
  successor->publish(candidate);
  ASSERT_TRUE(wait_until([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : semantics) {
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
    ASSERT_FALSE(semantics.empty());
    EXPECT_TRUE(semantics.back().owner.valid);
    EXPECT_EQ(semantics.back().owner.owner_gid, new_gid);
    EXPECT_NE(old_gid, new_gid);
  }
  (void)rgb_sub; (void)depth_sub; (void)rgb_info_sub; (void)depth_info_sub;
  (void)depth_out; (void)semantic_out;
}
}  // namespace
