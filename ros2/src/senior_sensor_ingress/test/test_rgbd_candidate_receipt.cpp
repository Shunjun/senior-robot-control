#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <gtest/gtest.h>
#include <openssl/sha.h>
#include <rclcpp/rclcpp.hpp>
#include <senior_motion_msgs/msg/bounded_raw_camera_info.hpp>
#include <senior_motion_msgs/msg/bounded_raw_image.hpp>
#include <senior_motion_msgs/msg/bounded_static_tf.hpp>
#include <senior_motion_msgs/msg/fenced_rgbd_candidate.hpp>
#include <senior_motion_msgs/msg/rgbd_motion_candidate.hpp>
#include "senior_sensor_ingress/rgbd_candidate_receipt.hpp"

namespace senior_sensor_ingress {
class RgbdCandidateReceiptTestPeer {
public:
  static bool pending_tf(RgbdCandidateReceipt & node, const std::string & gid) {
    std::lock_guard<std::mutex> lock(node.mutex_);
    return std::any_of(node.pending_tf_.begin(), node.pending_tf_.end(),
      [&](const auto & entry) {return entry.first == gid;});
  }
  static bool admitted_tf(RgbdCandidateReceipt & node, const std::string & gid) {
    std::lock_guard<std::mutex> lock(node.mutex_);
    return node.tf_ready_ && node.tf_gid_ == gid;
  }
  static bool accept_clock(RgbdCandidateReceipt & node, int64_t ros, double steady) {
    return node.clocks_ok(ros, steady);
  }
  static void fill_producer_retirement(RgbdCandidateReceipt & node) {
    std::lock_guard<std::mutex> lock(node.mutex_);
    for (int i = 0; i < 64; ++i) node.retired_producer_gids_.insert("retired-" + std::to_string(i));
  }
};
}

using namespace std::chrono_literals;
namespace {
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
  using Image = senior_motion_msgs::msg::BoundedRawImage;
  using Info = senior_motion_msgs::msg::BoundedRawCameraInfo;
  using Static = senior_motion_msgs::msg::BoundedStaticTf;
  using Candidate = senior_motion_msgs::msg::RgbdMotionCandidate;
  using Fenced = senior_motion_msgs::msg::FencedRgbdCandidate;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("topic_prefix", "/d2a_egress_test"),
                               rclcpp::Parameter("camera_frame", "camera")});
  auto receipt = std::make_shared<senior_sensor_ingress::RgbdCandidateReceipt>(options);
  auto raw = std::make_shared<rclcpp::Node>("raw_receipt");
  auto tf = std::make_shared<rclcpp::Node>("static_tf_receipt");
  auto worker = std::make_shared<rclcpp::Node>("rgbd_candidate_producer");
  auto observer = std::make_shared<rclcpp::Node>("rgbd_test_observer");
  auto qos = rclcpp::SensorDataQoS().keep_last(4);
  auto rgb_pub = raw->create_publisher<Image>("/d2a_egress_test/rgb", qos);
  auto depth_pub = raw->create_publisher<Image>("/d2a_egress_test/depth", qos);
  auto rgb_info_pub = raw->create_publisher<Info>("/d2a_egress_test/rgb_info", qos);
  auto depth_info_pub = raw->create_publisher<Info>("/d2a_egress_test/depth_info", qos);
  auto tf_pub = tf->create_publisher<Static>("/d2a_egress_test/tf", rclcpp::QoS(1).reliable().transient_local());
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
  calibration.valid = calibration.admitted = true;
  calibration.bridge_boot_id = "tfboot";
  calibration.bridge_gid = gid(tf_pub->get_gid());
  calibration.raw_gid = "01";
  calibration.generation = calibration.sequence = calibration.first_sequence = 1;
  calibration.received_ros = calibration.first_received_ros = static_cast<builtin_interfaces::msg::Time>(now_ros - rclcpp::Duration::from_seconds(.15));
  calibration.received_steady = calibration.first_received_steady = steady - .15;
  calibration.admitted_from_ros = static_cast<builtin_interfaces::msg::Time>(now_ros - rclcpp::Duration::from_seconds(.1));
  calibration.admitted_from_steady = steady - .1;
  senior_motion_msgs::msg::StaticTransformEdge edge;
  edge.parent_frame = "base_link";
  edge.child_frame = "camera";
  edge.rotation_xyzw[3] = 1;
  calibration.edges.push_back(edge);
  Static awaiting_clock = calibration;
  awaiting_clock.admitted = false;
  awaiting_clock.admitted_from_ros = builtin_interfaces::msg::Time();
  awaiting_clock.admitted_from_steady = 0.0;
  tf_pub->publish(awaiting_clock);
  std::this_thread::sleep_for(30ms);
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
      h.stamp = static_cast<builtin_interfaces::msg::Time>(now_ros - rclcpp::Duration::from_seconds((3-iteration)*.01));
      h.received_ros = h.stamp;
      h.received_steady = steady - (3-iteration)*.01;
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
      senior_motion_msgs::msg::RawImageLayout value;
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
  candidate.sequence = 1;
  candidate_pub->publish(candidate);
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (value.owner.valid) return true;
    return false;
  }, 300ms));
  size_t before_older_admission;
  {std::lock_guard<std::mutex> lock(mutex); before_older_admission = received.size();}
  tf_pub->publish(awaiting_clock);
  std::this_thread::sleep_for(60ms);
  {std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(received.size(), before_older_admission);}
  candidate.sequence = 2;
  candidate.second_rgb_layout.sha256[0] ^= 1;
  candidate_pub->publish(candidate);
  EXPECT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (!value.owner.valid && value.owner.owner_sequence == 2) return true;
    return false;
  }, 300ms));
  Image invalid_raw;
  invalid_raw.capture.valid = false;
  invalid_raw.capture.bridge_gid = gid(rgb_pub->get_gid());
  invalid_raw.capture.bridge_boot_id = "rawboot";
  invalid_raw.capture.capture_generation = 2;
  invalid_raw.capture.sequence = 3;
  rgb_pub->publish(invalid_raw);
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (!value.owner.valid && value.owner.reason == "raw_invalid") return true;
    return false;
  }, 300ms));
  candidate.second_rgb_layout.sha256[0] ^= 1;
  for (int i=0; i<2; ++i) {
    const auto seq = uint64_t(i+4);
    saved_rgb[i].capture.sequence = saved_depth[i].capture.sequence = seq;
    saved_rgb_info[i].capture.sequence = saved_depth_info[i].capture.sequence = seq;
    if (i == 0) {
      candidate.first_rgb = saved_rgb[i].capture; candidate.first_depth = saved_depth[i].capture;
      candidate.first_rgb_info = saved_rgb_info[i].capture; candidate.first_depth_info = saved_depth_info[i].capture;
      candidate.first_rgb_layout.capture = saved_rgb[i].capture;
      candidate.first_depth_layout.capture = saved_depth[i].capture;
      candidate.first_rgb_calibration = saved_rgb_info[i]; candidate.first_depth_calibration = saved_depth_info[i];
    } else {
      candidate.second_rgb = saved_rgb[i].capture; candidate.second_depth = saved_depth[i].capture;
      candidate.second_rgb_info = saved_rgb_info[i].capture; candidate.second_depth_info = saved_depth_info[i].capture;
      candidate.second_rgb_layout.capture = saved_rgb[i].capture;
      candidate.second_depth_layout.capture = saved_depth[i].capture;
      candidate.second_rgb_calibration = saved_rgb_info[i]; candidate.second_depth_calibration = saved_depth_info[i];
    }
    rgb_pub->publish(saved_rgb[i]); depth_pub->publish(saved_depth[i]);
    rgb_info_pub->publish(saved_rgb_info[i]); depth_info_pub->publish(saved_depth_info[i]);
  }
  std::this_thread::sleep_for(30ms);
  ASSERT_LT(steady_now() - saved_rgb[0].capture.received_steady, .3);
  candidate.sequence = 3;
  candidate_pub->publish(candidate);
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (!value.owner.valid && value.owner.owner_sequence == 3) return true;
    return false;
  }, 300ms));
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) EXPECT_FALSE(value.owner.valid && value.owner.owner_sequence == 3);
  }
  Static successor = calibration;
  successor.generation = successor.sequence = successor.first_sequence = 2;
  successor.edges[0].translation[0] = .1;
  tf_pub->publish(successor);
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & value : received) if (!value.owner.valid && value.owner.reason == "tf_changed") return true;
    return false;
  }, 300ms));
  size_t before_old;
  {std::lock_guard<std::mutex> lock(mutex); before_old = received.size();}
  Static old_invalid = calibration;
  old_invalid.valid = false;
  tf_pub->publish(old_invalid);
  std::this_thread::sleep_for(120ms);
  {std::lock_guard<std::mutex> lock(mutex); EXPECT_EQ(received.size(), before_old);}
}
}  // namespace

TEST(RgbdCandidateReceipt, ProducerBootChangeOnSameGidRevokesImmediately)
{
  using Candidate = senior_motion_msgs::msg::RgbdMotionCandidate;
  using Fenced = senior_motion_msgs::msg::FencedRgbdCandidate;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("topic_prefix", "/d2a_boot_test")});
  auto receipt = std::make_shared<senior_sensor_ingress::RgbdCandidateReceipt>(options);
  auto worker = std::make_shared<rclcpp::Node>("rgbd_candidate_producer");
  auto observer = std::make_shared<rclcpp::Node>("d2a_boot_observer");
  auto qos = rclcpp::SensorDataQoS().keep_last(4);
  auto publisher = worker->create_publisher<Candidate>("/d2a_boot_test/candidate", qos);
  std::mutex mutex;
  std::vector<Fenced> received;
  auto sub = observer->create_subscription<Fenced>("/d2a_boot_test/fenced", qos,
    [&](Fenced::ConstSharedPtr value) {std::lock_guard<std::mutex> lock(mutex); received.push_back(*value);});
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(receipt); executor.add_node(worker); executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {executor.cancel(); if (spin.joinable()) spin.join(); rclcpp::shutdown();};
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for([&] {return publisher->get_subscription_count() > 0;}));
  std::this_thread::sleep_for(80ms);
  Candidate candidate;
  candidate.valid = false;
  candidate.producer_gid = gid(publisher->get_gid());
  candidate.producer_boot_id = "first-boot";
  candidate.owner_generation = candidate.worker_generation = candidate.sequence = 1;
  publisher->publish(candidate);
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.owner.owner_sequence == 1) return true;
    return false;
  }));
  candidate.producer_boot_id = "new-boot";
  candidate.sequence = 2;
  publisher->publish(candidate);
  EXPECT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received)
      if (!item.owner.valid && item.owner.reason == "producer_boot_invalid") return true;
    return false;
  }, 300ms));
}

TEST(RgbdCandidateReceipt, NewBridgeBootCanRequalifyAfterObservedGraphLoss)
{
  using Image = senior_motion_msgs::msg::BoundedRawImage;
  using Fenced = senior_motion_msgs::msg::FencedRgbdCandidate;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("topic_prefix", "/d2a_raw_restart")});
  auto receipt = std::make_shared<senior_sensor_ingress::RgbdCandidateReceipt>(options);
  auto raw = std::make_shared<rclcpp::Node>("raw_receipt");
  auto observer = std::make_shared<rclcpp::Node>("d2a_restart_observer");
  auto qos = rclcpp::SensorDataQoS().keep_last(4);
  auto publisher = raw->create_publisher<Image>("/d2a_raw_restart/rgb", qos);
  std::mutex mutex;
  std::vector<Fenced> received;
  auto sub = observer->create_subscription<Fenced>("/d2a_raw_restart/fenced", qos,
    [&](Fenced::ConstSharedPtr value) {std::lock_guard<std::mutex> lock(mutex); received.push_back(*value);});
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(receipt); executor.add_node(raw); executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {executor.cancel(); if (spin.joinable()) spin.join(); rclcpp::shutdown();};
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for([&] {return publisher->get_subscription_count() > 0;}));
  Image body;
  body.capture.valid = true;
  body.capture.bridge_boot_id = "oldboot";
  body.capture.bridge_gid = gid(publisher->get_gid());
  body.capture.raw_gid = "10";
  body.capture.source_epoch = "source-old";
  body.capture.frame_id = "camera";
  body.capture.capture_generation = body.capture.sequence = 1;
  body.capture.stamp = body.capture.received_ros = static_cast<builtin_interfaces::msg::Time>(observer->now());
  body.capture.received_steady = steady_now();
  body.width = body.height = 1; body.step = 3; body.encoding = "rgb8";
  body.data = {'a','b','c'};
  publisher->publish(body);
  std::this_thread::sleep_for(70ms);
  publisher.reset();
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.owner.reason == "raw_owner_lost") return true;
    return false;
  }));
  publisher = raw->create_publisher<Image>("/d2a_raw_restart/rgb", qos);
  ASSERT_TRUE(wait_for([&] {return publisher->get_subscription_count() > 0;}));
  body.capture.bridge_boot_id = "newboot";
  body.capture.bridge_gid = gid(publisher->get_gid());
  body.capture.source_epoch = "source-new";
  body.capture.sequence = 1;
  body.capture.stamp = body.capture.received_ros = static_cast<builtin_interfaces::msg::Time>(observer->now());
  body.capture.received_steady = steady_now();
  publisher->publish(body);
  std::this_thread::sleep_for(50ms);
  Image invalid;
  invalid.capture = body.capture;
  invalid.capture.valid = false;
  invalid.capture.capture_generation = 2;
  invalid.capture.sequence = 2;
  publisher->publish(invalid);
  EXPECT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & item : received) if (item.owner.reason == "raw_invalid") return true;
    return false;
  }, 300ms));
}

TEST(RgbdCandidateReceipt, SameCurrentHeaderWithDifferentPixelsRevokes)
{
  using Image = senior_motion_msgs::msg::BoundedRawImage;
  using Fenced = senior_motion_msgs::msg::FencedRgbdCandidate;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("topic_prefix", "/d2a_duplicate_raw")});
  auto receipt = std::make_shared<senior_sensor_ingress::RgbdCandidateReceipt>(options);
  auto raw = std::make_shared<rclcpp::Node>("raw_receipt");
  auto observer = std::make_shared<rclcpp::Node>("d2a_duplicate_observer");
  auto qos = rclcpp::SensorDataQoS().keep_last(4);
  auto publisher = raw->create_publisher<Image>("/d2a_duplicate_raw/rgb", qos);
  std::mutex mutex;
  std::vector<Fenced> received;
  auto sub = observer->create_subscription<Fenced>("/d2a_duplicate_raw/fenced", qos,
    [&](Fenced::ConstSharedPtr value) {std::lock_guard<std::mutex> lock(mutex); received.push_back(*value);});
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(receipt); executor.add_node(raw); executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {executor.cancel(); if (spin.joinable()) spin.join(); rclcpp::shutdown();};
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for([&] {return publisher->get_subscription_count() > 0;}));
  Image body;
  body.capture.valid = true;
  body.capture.bridge_boot_id = "boot";
  body.capture.bridge_gid = gid(publisher->get_gid());
  body.capture.raw_gid = "10";
  body.capture.source_epoch = "source";
  body.capture.frame_id = "camera";
  body.capture.capture_generation = body.capture.sequence = 1;
  body.capture.stamp = body.capture.received_ros = static_cast<builtin_interfaces::msg::Time>(observer->now());
  body.capture.received_steady = steady_now();
  body.width = body.height = 1; body.step = 3; body.encoding = "rgb8";
  body.data = {'a','b','c'};
  publisher->publish(body);
  std::this_thread::sleep_for(50ms);
  body.data[0] = 'x';
  publisher->publish(body);
  auto invalid_count = [&] {
    std::lock_guard<std::mutex> lock(mutex);
    return std::count_if(received.begin(), received.end(), [](const auto & item) {
      return item.owner.reason == "raw_image_invalid";
    });
  };
  ASSERT_TRUE(wait_for([&] {return invalid_count() >= 1;}, 300ms));
  body.data[0] = 'a';
  publisher->publish(body);
  EXPECT_TRUE(wait_for([&] {return invalid_count() >= 2;}, 300ms));
  ++body.capture.sequence;
  publisher->publish(body);
  std::this_thread::sleep_for(50ms);
  body.data[0] = 'x';
  publisher->publish(body);
  EXPECT_TRUE(wait_for([&] {return invalid_count() >= 3;}, 300ms));
}

TEST(RgbdCandidateReceipt, InvalidTfGenerationRejectsDelayedOlderValid)
{
  using Static = senior_motion_msgs::msg::BoundedStaticTf;
  using Fenced = senior_motion_msgs::msg::FencedRgbdCandidate;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("topic_prefix", "/d2a_tf_fence"),
    rclcpp::Parameter("camera_frame", "camera")});
  auto receipt = std::make_shared<senior_sensor_ingress::RgbdCandidateReceipt>(options);
  auto source = std::make_shared<rclcpp::Node>("static_tf_receipt");
  auto observer = std::make_shared<rclcpp::Node>("d2a_tf_fence_observer");
  auto pub = source->create_publisher<Static>("/d2a_tf_fence/tf", rclcpp::QoS(1).reliable().transient_local());
  std::mutex mutex;
  std::vector<Fenced> received;
  auto sub = observer->create_subscription<Fenced>("/d2a_tf_fence/fenced", rclcpp::SensorDataQoS(),
    [&](Fenced::ConstSharedPtr value) {std::lock_guard<std::mutex> lock(mutex); received.push_back(*value);});
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(receipt); executor.add_node(source); executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {executor.cancel(); if (spin.joinable()) spin.join(); rclcpp::shutdown();};
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for([&] {return pub->get_subscription_count() > 0;}));
  std::this_thread::sleep_for(80ms);
  Static body;
  body.valid = body.admitted = true;
  body.bridge_boot_id = "tfboot"; body.bridge_gid = gid(pub->get_gid()); body.raw_gid = "01";
  body.generation = body.sequence = body.first_sequence = 1;
  body.received_ros = body.first_received_ros = body.admitted_from_ros =
    static_cast<builtin_interfaces::msg::Time>(observer->now());
  body.received_steady = body.first_received_steady = body.admitted_from_steady = steady_now();
  senior_motion_msgs::msg::StaticTransformEdge edge;
  edge.parent_frame = "base_link"; edge.child_frame = "camera"; edge.rotation_xyzw[3] = 1;
  body.edges.push_back(edge);
  pub->publish(body);
  std::this_thread::sleep_for(80ms);
  auto invalid = body; invalid.valid = false; invalid.generation = 3;
  pub->publish(invalid);
  auto count_reason = [&](const std::string & reason) {
    std::lock_guard<std::mutex> lock(mutex);
    return std::count_if(received.begin(), received.end(), [&](const auto & item) {return item.owner.reason == reason;});
  };
  ASSERT_TRUE(wait_for([&] {return count_reason("tf_invalid") > 0;}, 300ms));
  auto delayed = body; delayed.generation = 2;
  pub->publish(delayed);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(count_reason("tf_changed"), 0);
  auto successor = body; successor.generation = 4;
  pub->publish(successor);
  EXPECT_TRUE(wait_for([&] {return count_reason("tf_changed") > 0;}, 300ms));
}

TEST(RgbdCandidateReceipt, RetirementExhaustionRevokesActualProducerOnGraphLoss)
{
  using Candidate = senior_motion_msgs::msg::RgbdMotionCandidate;
  using Fenced = senior_motion_msgs::msg::FencedRgbdCandidate;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("topic_prefix", "/d2a_cap")});
  auto receipt = std::make_shared<senior_sensor_ingress::RgbdCandidateReceipt>(options);
  auto worker = std::make_shared<rclcpp::Node>("rgbd_candidate_producer");
  auto observer = std::make_shared<rclcpp::Node>("d2a_cap_observer");
  auto pub = worker->create_publisher<Candidate>("/d2a_cap/candidate", rclcpp::SensorDataQoS());
  std::mutex mutex;
  std::vector<Fenced> received;
  auto sub = observer->create_subscription<Fenced>("/d2a_cap/fenced", rclcpp::SensorDataQoS(),
    [&](Fenced::ConstSharedPtr value) {std::lock_guard<std::mutex> lock(mutex); received.push_back(*value);});
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(receipt); executor.add_node(worker); executor.add_node(observer);
  std::thread spin([&] {executor.spin();});
  auto cleanup = [&](int *) {executor.cancel(); if (spin.joinable()) spin.join(); rclcpp::shutdown();};
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  ASSERT_TRUE(wait_for([&] {return pub->get_subscription_count() > 0;}));
  std::this_thread::sleep_for(120ms);
  Candidate malformed;
  malformed.producer_gid = gid(pub->get_gid());
  pub->publish(malformed);
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return std::any_of(received.begin(), received.end(), [](const auto & item) {return item.owner.reason == "producer_invalid";});
  }, 300ms));
  senior_sensor_ingress::RgbdCandidateReceiptTestPeer::fill_producer_retirement(*receipt);
  pub.reset();
  ASSERT_TRUE(wait_for([&] {
    std::lock_guard<std::mutex> lock(mutex);
    return std::any_of(received.begin(), received.end(), [](const auto & item) {
      return !item.owner.valid && item.owner.owner_lost && item.owner.reason == "retirement_exhausted";
    });
  }, 1s));
}


TEST(RgbdCandidateReceipt, ReconsidersSingleTransientStatusWhenOwnerBecomesUnique)
{
  using Static = senior_motion_msgs::msg::BoundedStaticTf;
  using Peer = senior_sensor_ingress::RgbdCandidateReceiptTestPeer;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("topic_prefix", "/d2a_tf_pending")});
  auto receipt = std::make_shared<senior_sensor_ingress::RgbdCandidateReceipt>(options);
  auto source = std::make_shared<rclcpp::Node>("static_tf_receipt");
  const std::string topic = "/d2a_tf_pending/tf";
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto old = source->create_publisher<Static>(topic, qos);
  auto replacement = source->create_publisher<Static>(topic, qos);
  rclcpp::executors::SingleThreadedExecutor executor;
  std::thread spin;
  auto cleanup = [&](int *) {
    executor.cancel();
    if (spin.joinable()) spin.join();
    rclcpp::shutdown();
  };
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  executor.add_node(receipt);
  executor.add_node(source);
  spin = std::thread([&] {executor.spin();});
  ASSERT_TRUE(wait_for([&] {
    return old->get_subscription_count() > 0 && replacement->get_subscription_count() > 0 &&
      receipt->get_publishers_info_by_topic(topic).size() == 2;
  }));

  Static sample;
  sample.valid = sample.admitted = true;
  sample.bridge_boot_id = "new-relay";
  sample.bridge_gid = gid(replacement->get_gid());
  sample.raw_gid = "01";
  sample.generation = sample.sequence = sample.first_sequence = 1;
  sample.received_ros = sample.first_received_ros = sample.admitted_from_ros =
    static_cast<builtin_interfaces::msg::Time>(source->now());
  sample.received_steady = sample.first_received_steady = sample.admitted_from_steady = steady_now();
  senior_motion_msgs::msg::StaticTransformEdge edge;
  edge.parent_frame = "base_link";
  edge.child_frame = "camera_color_optical_frame";
  edge.rotation_xyzw[3] = 1;
  sample.edges.push_back(edge);
  // Publish only once; the real callback must capture the actual DDS writer GID.
  replacement->publish(sample);
  ASSERT_TRUE(wait_for([&] {return Peer::pending_tf(*receipt, sample.bridge_gid);}));
  EXPECT_FALSE(Peer::admitted_tf(*receipt, sample.bridge_gid));

  old.reset();
  EXPECT_TRUE(wait_for([&] {return Peer::admitted_tf(*receipt, sample.bridge_gid);}));
}

TEST(RgbdCandidateReceipt, ClockRollbackRequiresRestart)
{
  using Peer = senior_sensor_ingress::RgbdCandidateReceiptTestPeer;
  rclcpp::init(0, nullptr);
  auto cleanup = [](int *) {rclcpp::shutdown();};
  int sentinel = 0;
  std::unique_ptr<int, decltype(cleanup)> guard(&sentinel, cleanup);
  senior_sensor_ingress::RgbdCandidateReceipt receipt;
  EXPECT_FALSE(Peer::accept_clock(receipt, 0, 1.0));
  EXPECT_TRUE(Peer::accept_clock(receipt, 100, 2.0));
  EXPECT_FALSE(Peer::accept_clock(receipt, 99, 3.0));
  EXPECT_FALSE(Peer::accept_clock(receipt, 101, 4.0));
}
