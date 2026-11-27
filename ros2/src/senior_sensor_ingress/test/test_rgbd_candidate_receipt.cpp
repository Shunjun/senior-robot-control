#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include <openssl/sha.h>
#include <rclcpp/rclcpp.hpp>
#include <senior_interfaces/msg/bounded_raw_camera_info.hpp>
#include <senior_interfaces/msg/bounded_raw_image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <senior_interfaces/msg/fenced_rgbd_candidate.hpp>
#include <senior_interfaces/msg/rgbd_motion_candidate.hpp>
#include "senior_sensor_ingress/rgbd_candidate_receipt.hpp"

namespace senior_sensor_ingress {
class RgbdCandidateReceiptTestPeer {
public:
  static bool accept_clock(RgbdCandidateReceipt & node, int64_t ros, uint64_t steady) {
    return node.clocks_ok(ros, steady);
  }
};
}
using namespace std::chrono_literals;
namespace {
rclcpp::NodeOptions clock_test_options()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("sensor_soft_age_ms", 300.0);
  options.append_parameter_override("sensor_hard_age_ms", 1000.0);
  options.append_parameter_override("sensor_diagnostics_period_s", 5.0);
  options.append_parameter_override("queue_warn_count", 32);
  options.append_parameter_override("queue_warn_ratio", 0.75);
  options.append_parameter_override("queue_warn_repeat_s", 5.0);
  options.append_parameter_override("queue_warn_recovery_ratio", 0.5);
  options.append_parameter_override("queue_warn_oldest_wait_ms", 300.0);
  return options;
}
bool wait_for(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 5s)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < end) {
    if (predicate()) return true;
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}
std::string gid(const rmw_gid_t & value)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (auto byte : value.data) out << std::setw(2) << unsigned(byte);
  return out.str();
}
double steady_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

TEST(RgbdCandidateReceipt, RealOwnersBindBothPixelBodiesAndRejectTampering)
{
  using Image = senior_interfaces::msg::BoundedRawImage;
  using Info = senior_interfaces::msg::BoundedRawCameraInfo;
  using Static = tf2_msgs::msg::TFMessage;
  using Candidate = senior_interfaces::msg::RgbdMotionCandidate;
  using Fenced = senior_interfaces::msg::FencedRgbdCandidate;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("topic_prefix", "/d2a_egress_test"),
                               rclcpp::Parameter("camera_frame", "camera")});
  options.append_parameter_override("sensor_soft_age_ms",300.0);
  options.append_parameter_override("sensor_hard_age_ms",1000.0);
  options.append_parameter_override("sensor_diagnostics_period_s",5.0);
  options.append_parameter_override("queue_warn_count",32);
  options.append_parameter_override("queue_warn_ratio",0.75);
  options.append_parameter_override("queue_warn_repeat_s",5.0);
  options.append_parameter_override("queue_warn_recovery_ratio",0.5);
  options.append_parameter_override("queue_warn_oldest_wait_ms",300.0);
  auto receipt = std::make_shared<senior_sensor_ingress::RgbdCandidateReceipt>(options);
  auto raw = std::make_shared<rclcpp::Node>("raw_receipt");
  auto tf = std::make_shared<rclcpp::Node>("robot_state_publisher");
  auto worker = std::make_shared<rclcpp::Node>("rgbd_candidate_producer");
  auto observer = std::make_shared<rclcpp::Node>("rgbd_test_observer");
  auto qos = rclcpp::SensorDataQoS().keep_last(4);
  auto rgb_pub = raw->create_publisher<Image>("/d2a_egress_test/rgb", qos);
  auto depth_pub = raw->create_publisher<Image>("/d2a_egress_test/depth", qos);
  auto rgb_info_pub = raw->create_publisher<Info>("/d2a_egress_test/rgb_info", qos);
  auto depth_info_pub = raw->create_publisher<Info>("/d2a_egress_test/depth_info", qos);
  auto tf_pub = tf->create_publisher<Static>("/tf_static", rclcpp::QoS(1).reliable().transient_local());
  auto candidate_pub = worker->create_publisher<Candidate>("/d2a_egress_test/candidate", qos);
  std::mutex mutex;
  std::vector<Fenced> received;
  auto sub = observer->create_subscription<Fenced>("/d2a_egress_test/fenced", qos,
    [&](Fenced::ConstSharedPtr message) {std::lock_guard<std::mutex> lock(mutex); received.push_back(*message);});
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(receipt); executor.add_node(raw); executor.add_node(tf);
  executor.add_node(worker); executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {executor.cancel(); if (spin.joinable()) spin.join(); rclcpp::shutdown();};
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for([&] {return rgb_pub->get_subscription_count() && depth_pub->get_subscription_count() &&
    rgb_info_pub->get_subscription_count() && depth_info_pub->get_subscription_count() &&
    tf_pub->get_subscription_count() && candidate_pub->get_subscription_count();}));
  std::this_thread::sleep_for(80ms);  // allow native graph admission before original captures
  const auto now_ros = observer->now();
  const double steady = steady_now();
  Static calibration;
  geometry_msgs::msg::TransformStamped edge;
  edge.header.frame_id = "base_link";
  edge.child_frame_id = "camera";
  edge.transform.rotation.w = 1.0;
  calibration.transforms.push_back(edge);
  tf_pub->publish(calibration);
  Candidate candidate;
  candidate.valid = true;
  candidate.hard_error_qualified = false;
  candidate.producer_boot_id = "producer";
  candidate.producer_gid = gid(candidate_pub->get_gid());
  candidate.owner_generation = candidate.worker_generation = 1;
  candidate.profile_id = "sim";
  candidate.calibration_id = "cal";
  candidate.registration_id = "registered";
  candidate.preprocessing_version = "rgb8-mono-v1";
  candidate.raw_rgb_encoding = "rgb8";
  candidate.body_frame = "base_link";
  candidate.camera_transform[0] = candidate.camera_transform[5] = candidate.camera_transform[10] = candidate.camera_transform[15] = 1;
  candidate.matched = 30;
  candidate.inliers = 20;
  candidate.median_reprojection_px = .1;
  candidate.median_depth_residual_m = .01;
  candidate.first_tf = candidate.second_tf = calibration;
  std::array<Image, 2> saved_rgb, saved_depth;
  std::array<Info, 2> saved_rgb_info, saved_depth_info;
  auto publish_pair = [&](double shift) {
  for (int iteration = 1; iteration <= 2; ++iteration) {
    Image rgb, depth;
    Info rgb_info, depth_info;
    auto init = [&](auto & body, const std::string & bridge_gid, const std::string & epoch) {
      auto & h = body.capture;
      h.valid = true;
      h.bridge_gid = bridge_gid;
      h.raw_gid = "10";
      h.source_epoch = epoch;
      h.bridge_boot_id = "rawboot";
      h.capture_generation = 1;
      h.sequence = iteration;
      h.frame_id = "camera";
      h.stamp = static_cast<builtin_interfaces::msg::Time>(now_ros - rclcpp::Duration::from_seconds((3-iteration)*.01 + shift));
      h.received_ros = h.stamp;
      h.received_steady_ns = static_cast<uint64_t>((steady - ((3-iteration)*.01 + shift)) * 1e9);
      h.sample_steady_ns = h.received_steady_ns;
      h.clock_generation = 1;
      body.width = body.height = 80;
    };
    init(rgb, gid(rgb_pub->get_gid()), "rgb");
    init(depth, gid(depth_pub->get_gid()), "depth");
    init(rgb_info, gid(rgb_info_pub->get_gid()), "rgbinfo");
    init(depth_info, gid(depth_info_pub->get_gid()), "depthinfo");
    // //Compatible: Humble BoundedVector needs explicit count and byte types.
    rgb.step = 240; rgb.encoding = "rgb8"; rgb.data.assign(size_t(80*240), uint8_t(42));
    depth.step = 320; depth.encoding = "32FC1"; depth.data.assign(size_t(80*320), uint8_t(0));
    auto layout = [](const Image & image) {
      senior_interfaces::msg::RawImageLayout value;
      value.capture = image.capture;
      value.width = image.width; value.height = image.height; value.step = image.step;
      value.encoding = image.encoding; value.is_bigendian = image.is_bigendian;
      value.byte_length = image.data.size();
      SHA256(&image.data[0], image.data.size(), value.sha256.data());
      return value;
    };
    if (iteration == 1) {
      candidate.first_rgb = rgb.capture; candidate.first_depth = depth.capture;
      candidate.first_rgb_info = rgb_info.capture; candidate.first_depth_info = depth_info.capture;
      candidate.first_rgb_layout = layout(rgb); candidate.first_depth_layout = layout(depth);
      candidate.first_rgb_calibration = rgb_info; candidate.first_depth_calibration = depth_info;
      candidate.interval_start = rgb.capture.stamp;
    } else {
      candidate.second_rgb = rgb.capture; candidate.second_depth = depth.capture;
      candidate.second_rgb_info = rgb_info.capture; candidate.second_depth_info = depth_info.capture;
      candidate.second_rgb_layout = layout(rgb); candidate.second_depth_layout = layout(depth);
      candidate.second_rgb_calibration = rgb_info; candidate.second_depth_calibration = depth_info;
      candidate.interval_end = rgb.capture.stamp;
    }
    saved_rgb[iteration-1] = rgb; saved_depth[iteration-1] = depth;
    saved_rgb_info[iteration-1] = rgb_info; saved_depth_info[iteration-1] = depth_info;
    rgb_pub->publish(rgb); depth_pub->publish(depth);
    rgb_info_pub->publish(rgb_info); depth_info_pub->publish(depth_info);
  }
  std::this_thread::sleep_for(50ms);
  };
  publish_pair(.04);
  candidate.sequence = 1;
  candidate_pub->publish(candidate);
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (value.owner.valid) return true;
    return false;
  }, 300ms));
  // A new acquisition with a mismatched pixel digest is rejected; sequence alone
  // never makes the previous pair a new observation.
  publish_pair(0.0);
  candidate.sequence = 0;
  candidate.second_rgb_layout.sha256[0] ^= 1;
  candidate_pub->publish(candidate);
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received)
      if (!value.owner.valid && value.owner.owner_sequence == 0 &&
          value.owner.reason == "candidate_invalid") return true;
    return false;
  }, 300ms));
}
} // namespace

TEST(RgbdCandidateReceipt, ClockRollbackRevokesWindowThenAcceptsNewSegment)
{
  using Peer = senior_sensor_ingress::RgbdCandidateReceiptTestPeer;
  rclcpp::init(0, nullptr);
  auto cleanup = [](int *) {rclcpp::shutdown();};
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  senior_sensor_ingress::RgbdCandidateReceipt receipt(clock_test_options());
  EXPECT_TRUE(Peer::accept_clock(receipt, 0, 1.0));
  EXPECT_TRUE(Peer::accept_clock(receipt, 100, 2.0));
  EXPECT_FALSE(Peer::accept_clock(receipt, 99, 3.0));
  EXPECT_TRUE(Peer::accept_clock(receipt, 101, 4.0));
}
