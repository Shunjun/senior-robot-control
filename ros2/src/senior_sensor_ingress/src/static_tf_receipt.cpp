#include "senior_sensor_ingress/static_tf_receipt.hpp"

#include <cmath>
#include <map>
#include <set>

namespace senior_sensor_ingress {
namespace {
bool valid_frame(const std::string & value)
{
  if (value.empty() || value.size() > 128) return false;
  for (const unsigned char byte : value) if (byte < 33 || byte > 126) return false;
  return true;
}
}  // namespace

bool validate_static_tf(const tf2_msgs::msg::TFMessage & message,
                        const std::string & base, const std::string & camera)
{
  if (!valid_frame(base) || !valid_frame(camera) || base == camera ||
      message.transforms.empty() || message.transforms.size() > 32) return false;
  std::map<std::string, std::string> parents;
  size_t bytes = 0;
  for (const auto & edge : message.transforms) {
    const auto & q = edge.transform.rotation;
    const auto & t = edge.transform.translation;
    if (!valid_frame(edge.header.frame_id) || !valid_frame(edge.child_frame_id) ||
        edge.header.frame_id == edge.child_frame_id || edge.header.stamp.sec < 0 ||
        edge.header.stamp.nanosec >= 1000000000u ||
        !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
        !std::isfinite(q.w) || !std::isfinite(t.x) || !std::isfinite(t.y) ||
        !std::isfinite(t.z) || std::abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 1e-6 ||
        !parents.emplace(edge.child_frame_id, edge.header.frame_id).second) return false;
    bytes += edge.header.frame_id.size() + edge.child_frame_id.size() + 72;
    if (bytes > 16384) return false;
  }
  for (const auto & entry : parents) {
    std::set<std::string> seen;
    auto node = entry.first;
    while (parents.count(node)) {
      if (!seen.insert(node).second) return false;
      node = parents.at(node);
    }
  }
  auto node = camera;
  while (node != base && parents.count(node)) node = parents.at(node);
  return node == base;
}
}  // namespace senior_sensor_ingress

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace senior_sensor_ingress {
namespace {
using namespace std::chrono_literals;
std::string hex_gid(const uint8_t * data, size_t length)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (size_t i = 0; i < length; ++i) out << std::setw(2) << unsigned(data[i]);
  return out.str();
}
double steady_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string make_boot()
{
  std::random_device random;
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) out << std::setw(8) << random();
  return out.str();
}
}  // namespace

StaticTfReceipt::StaticTfReceipt(const rclcpp::NodeOptions & options,
                                 BeforeVerify before_verify, bool test_reentrant,
                                 std::function<void()> before_inspect)
: Node("static_tf_receipt", options), before_verify_(std::move(before_verify)),
  before_inspect_(std::move(before_inspect)), boot_id_(make_boot())
{
  input_topic_ = declare_parameter<std::string>("input_topic", "/tf_static");
  output_topic_ = declare_parameter<std::string>("output_topic", "/safety/static_tf_receipt");
  base_ = declare_parameter<std::string>("base_frame", "base_link");
  camera_ = declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
  expected_node_ = declare_parameter<std::string>("expected_node_name", "robot_state_publisher");
  expected_namespace_ = declare_parameter<std::string>("expected_namespace", "/");
  if (!valid_frame(input_topic_) || !valid_frame(output_topic_) || input_topic_ == output_topic_ ||
      !valid_frame(base_) || !valid_frame(camera_) || base_ == camera_ ||
      !valid_frame(expected_node_) || !valid_frame(expected_namespace_)) {
    throw std::invalid_argument("invalid static TF receipt configuration");
  }
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  publisher_ = create_publisher<Output>(output_topic_, qos);
  rclcpp::SubscriptionOptions subscription_options;
  if (test_reentrant) {
    test_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    subscription_options.callback_group = test_callback_group_;
  }
  subscription_ = create_subscription<Input>(input_topic_, qos,
    [this](Input::ConstSharedPtr value, const rclcpp::MessageInfo & info) {
      on_message(*value, info);
    }, subscription_options);
  timer_ = create_wall_timer(20ms, [this] {inspect();});
}

std::string StaticTfReceipt::output_gid() const
{
  const auto gid = publisher_->get_gid();
  return hex_gid(gid.data, RMW_GID_STORAGE_SIZE);
}

bool StaticTfReceipt::owner_matches(const std::string & gid)
{
  const auto graph = get_publishers_info_by_topic(input_topic_);
  return graph.size() == 1 && graph[0].node_name() == expected_node_ &&
    graph[0].node_namespace() == expected_namespace_ &&
    hex_gid(graph[0].endpoint_gid().data(), RMW_GID_STORAGE_SIZE) == gid;
}

void StaticTfReceipt::invalidate(const std::string & reason, bool retire_source)
{
  if (have_body_ || active_) {
    Output invalid = first_;
    invalid.bridge_boot_id = boot_id_;
    invalid.bridge_gid = output_gid();
    invalid.raw_gid = raw_gid_;
    invalid.generation = generation_;
    invalid.sequence = sequence_;
    invalid.valid = false;
    invalid.admitted = false;
    invalid.reason = reason;
    publisher_->publish(invalid);
  }
  if (retire_source && !raw_gid_.empty()) {
    if (retired_.size() >= 64) failed_ = true;
    else retired_.insert(raw_gid_);
    raw_gid_.clear();
    active_ = false;
  }
  have_body_ = false;
  ++generation_;
}

bool StaticTfReceipt::update_clock(int64_t ros_ns, double steady)
{
  if (ros_ns < 0 || !std::isfinite(steady) || steady <= 0 ||
      (last_steady_ > 0 && (steady < last_steady_ || ros_ns < last_ros_ns_))) {
    invalidate("clock_invalid", false);
    clock_ready_ = false;
    last_ros_ns_ = ros_ns;
    last_steady_ = steady;
    return false;
  }
  if (last_ros_ns_ > 0 && ros_ns > last_ros_ns_) clock_ready_ = true;
  last_ros_ns_ = ros_ns;
  last_steady_ = steady;
  return true;
}

void StaticTfReceipt::on_message(const Input & message, const rclcpp::MessageInfo & info)
{
  const int64_t received_ros = get_clock()->now().nanoseconds();
  const double received_steady = steady_now();
  const auto gid = info.get_rmw_message_info().publisher_gid;
  const auto actual = hex_gid(gid.data, RMW_GID_STORAGE_SIZE);
  if (before_verify_) before_verify_(gid);
  std::lock_guard<std::mutex> lock(mutex_);
  if (failed_ || retired_.count(actual)) return;
  const bool nonzero = std::any_of(std::begin(gid.data), std::end(gid.data),
                                   [](uint8_t value) {return value != 0;});
  // A delayed callback from a never-admitted writer must not retire a
  // currently unique, graph-proven successor or be read as a clock rollback.
  if (active_ && actual != raw_gid_ && owner_matches(raw_gid_)) return;
  if (!nonzero || !owner_matches(actual)) {
    invalidate("source_mismatch", true);
    return;
  }
  // The callback's original receipt can precede a later timer sample. Compare
  // clock progress at qualification time while retaining the original receipt.
  if (!update_clock(get_clock()->now().nanoseconds(), steady_now())) return;
  if (active_ && raw_gid_ != actual) invalidate("source_changed", true);
  if (failed_) return;
  if (!active_) {
    raw_gid_ = actual;
    active_ = true;
    ++generation_;
    sequence_ = 0;
  }
  if (!validate_static_tf(message, base_, camera_)) {
    invalidate("invalid_static_tf", false);
    return;
  }
  const bool changed = have_body_ && message != body_;
  if (changed) invalidate("static_tf_changed", false);
  Output output;
  output.bridge_boot_id = boot_id_;
  output.bridge_gid = output_gid();
  output.raw_gid = actual;
  output.generation = generation_;
  output.sequence = ++sequence_;
  output.received_ros = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(received_ros));
  output.received_steady = received_steady;
  output.valid = true;
  for (const auto & edge : message.transforms) {
    senior_motion_msgs::msg::StaticTransformEdge bounded;
    bounded.parent_frame = edge.header.frame_id;
    bounded.child_frame = edge.child_frame_id;
    bounded.stamp = edge.header.stamp;
    bounded.translation = {edge.transform.translation.x, edge.transform.translation.y,
                           edge.transform.translation.z};
    bounded.rotation_xyzw = {edge.transform.rotation.x, edge.transform.rotation.y,
                             edge.transform.rotation.z, edge.transform.rotation.w};
    output.edges.push_back(bounded);
  }
  if (!owner_matches(actual)) {
    invalidate("source_mismatch", true);
    return;
  }
  if (!update_clock(get_clock()->now().nanoseconds(), steady_now())) return;
  if (!have_body_) {
    output.first_sequence = output.sequence;
    output.first_received_ros = output.received_ros;
    output.first_received_steady = output.received_steady;
    body_ = message;
    first_ = output;
    have_body_ = true;
  } else {
    output.first_sequence = first_.first_sequence;
    output.first_received_ros = first_.first_received_ros;
    output.first_received_steady = first_.first_received_steady;
  }
  if (clock_ready_ && !first_.admitted) {
    first_.admitted = true;
    first_.admitted_from_ros = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(last_ros_ns_));
    first_.admitted_from_steady = last_steady_;
  }
  output.admitted = first_.admitted;
  output.admitted_from_ros = first_.admitted_from_ros;
  output.admitted_from_steady = first_.admitted_from_steady;
  publisher_->publish(output);
}

void StaticTfReceipt::inspect()
{
  if (before_inspect_) before_inspect_();
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ros_ns = get_clock()->now().nanoseconds();
  const auto steady = steady_now();
  if (failed_ || !update_clock(ros_ns, steady)) return;
  if (active_ && !owner_matches(raw_gid_)) {
    invalidate("source_lost", true);
    return;
  }
  if (have_body_ && clock_ready_ && !first_.admitted) {
    first_.admitted = true;
    first_.admitted_from_ros = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(ros_ns));
    first_.admitted_from_steady = steady;
    publisher_->publish(first_);
  }
}
}  // namespace senior_sensor_ingress
