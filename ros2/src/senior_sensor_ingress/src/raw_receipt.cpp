#include "senior_sensor_ingress/raw_receipt.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace senior_sensor_ingress {
namespace {
using namespace std::chrono_literals;
constexpr uint64_t max_pixels = 2073600;
constexpr uint64_t max_bytes = 16777216;
constexpr uint32_t max_dimension = 4096;

std::string bytes_hex(const uint8_t * data, size_t length)
{
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (size_t i = 0; i < length; ++i) output << std::setw(2) << unsigned(data[i]);
  return output.str();
}

bool text_ok(const std::string & value, size_t max_length)
{
  return !value.empty() && value.size() <= max_length &&
    std::all_of(value.begin(), value.end(), [](unsigned char c) {return c >= 33 && c <= 126;});
}

bool stamp_ok(const builtin_interfaces::msg::Time & value)
{
  return value.sec >= 0 && value.nanosec < 1000000000u &&
         (value.sec > 0 || value.nanosec > 0);
}

double steady_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool dimensions_ok(uint32_t width, uint32_t height)
{
  return width > 0 && height > 0 && width <= max_dimension && height <= max_dimension &&
         uint64_t(width) * height <= max_pixels;
}

bool image_layout_ok(const sensor_msgs::msg::Image & image, size_t index)
{
  uint32_t channels = 0;
  if (index == 0 && (image.encoding == "rgb8" || image.encoding == "bgr8")) channels = 3;
  if (index == 0 && image.encoding == "mono8") channels = 1;
  if (index == 1 && image.encoding == "16UC1") channels = 2;
  if (index == 1 && image.encoding == "32FC1") channels = 4;
  return dimensions_ok(image.width, image.height) && channels > 0 &&
    image.step >= uint64_t(image.width) * channels &&
    uint64_t(image.step) * image.height <= max_bytes &&
    image.data.size() == uint64_t(image.step) * image.height;
}
}  // namespace

std::string gid_hex(const rmw_gid_t & gid)
{
  return bytes_hex(gid.data, RMW_GID_STORAGE_SIZE);
}

std::string gid_hex(const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid)
{
  return bytes_hex(gid.data(), gid.size());
}

RawReceipt::RawReceipt(const rclcpp::NodeOptions & options, BeforeVerify before_verify,
                       bool test_reentrant_callbacks)
: Node("raw_receipt", options), before_verify_(std::move(before_verify))
{
  input_topics_ = {
    declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw"),
    declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw"),
    declare_parameter<std::string>("rgb_info_topic", "/camera/color/camera_info"),
    declare_parameter<std::string>("depth_info_topic", "/camera/depth/camera_info")};
  const std::array<std::string, 4> outputs = {
    declare_parameter<std::string>("rgb_output", "/safety/raw/rgb"),
    declare_parameter<std::string>("depth_output", "/safety/raw/depth"),
    declare_parameter<std::string>("rgb_info_output", "/safety/raw/rgb_info"),
    declare_parameter<std::string>("depth_info_output", "/safety/raw/depth_info")};
  std::set<std::string> topics;
  for (const auto & topic : input_topics_) {
    if (!text_ok(topic, 128) || !topics.insert(topic).second)
      throw std::invalid_argument("invalid raw input topic configuration");
  }
  for (const auto & topic : outputs) {
    if (!text_ok(topic, 128) || !topics.insert(topic).second)
      throw std::invalid_argument("invalid raw output topic configuration");
  }
  depth_input_topic_ = declare_parameter<std::string>("depth_candidate_topic", "/safety/depth_candidate");
  const auto depth_output = declare_parameter<std::string>("depth_fenced_topic", "/safety/bounded_depth");
  if (!text_ok(depth_input_topic_, 128) || !topics.insert(depth_input_topic_).second ||
      !text_ok(depth_output, 128) || !topics.insert(depth_output).second) {
    throw std::invalid_argument("invalid depth egress topic configuration");
  }
  semantic_input_topic_ = declare_parameter<std::string>(
    "semantic_candidate_topic", "/safety/semantic_candidate_unfenced");
  const auto semantic_output = declare_parameter<std::string>(
    "semantic_fenced_topic", "/safety/semantic_candidates");
  if (!text_ok(semantic_input_topic_, 128) || !topics.insert(semantic_input_topic_).second ||
      !text_ok(semantic_output, 128) || !topics.insert(semantic_output).second) {
    throw std::invalid_argument("invalid semantic egress topic configuration");
  }
  boot_id_ = new_epoch();
  for (auto & source : sources_) source.epoch = new_epoch();
  const auto qos = rclcpp::SensorDataQoS().keep_last(4);
  depth_fenced_pub_ = create_publisher<senior_motion_msgs::msg::FencedDepthEvidence>(depth_output, qos);
  rclcpp::SubscriptionOptions depth_options;
  if (test_reentrant_callbacks) {
    depth_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    depth_options.callback_group = depth_callback_group_;
  }
  depth_body_sub_ = create_subscription<senior_motion_msgs::msg::BoundedDepthEvidence>(
    depth_input_topic_, qos,
    [this](senior_motion_msgs::msg::BoundedDepthEvidence::ConstSharedPtr message,
           const rclcpp::MessageInfo & info) {on_depth(*message, info);}, depth_options);
  semantic_fenced_pub_ = create_publisher<senior_motion_msgs::msg::FencedSemanticEvidence>(
    semantic_output, qos);
  semantic_body_sub_ = create_subscription<senior_motion_msgs::msg::BoundedSemanticEvidence>(
    semantic_input_topic_, qos,
    [this](senior_motion_msgs::msg::BoundedSemanticEvidence::ConstSharedPtr message,
           const rclcpp::MessageInfo & info) {on_semantic(*message, info);}, depth_options);
  rgb_publisher_ = create_publisher<RawImage>(outputs[0], qos);
  depth_publisher_ = create_publisher<RawImage>(outputs[1], qos);
  rgb_info_publisher_ = create_publisher<RawInfo>(outputs[2], qos);
  depth_info_publisher_ = create_publisher<RawInfo>(outputs[3], qos);
  rgb_subscription_ = create_subscription<Image>(
    input_topics_[0], qos, [this](Image::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
      on_image(0, *message, info);
    }, depth_options);
  depth_subscription_ = create_subscription<Image>(
    input_topics_[1], qos, [this](Image::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
      on_image(1, *message, info);
    }, depth_options);
  rgb_info_subscription_ = create_subscription<Info>(
    input_topics_[2], qos, [this](Info::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
      on_info(2, *message, info);
    }, depth_options);
  depth_info_subscription_ = create_subscription<Info>(
    input_topics_[3], qos, [this](Info::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
      on_info(3, *message, info);
    }, depth_options);
  timer_ = create_wall_timer(20ms, [this] {inspect_topology();});
}

std::string RawReceipt::new_epoch()
{
  std::random_device random;
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned i = 0; i < 4; ++i) output << std::setw(8) << uint32_t(random());
  output << '-' << ++counter_;
  return output.str();
}

std::string RawReceipt::output_gid(size_t index) const
{
  if (index == 0) return gid_hex(rgb_publisher_->get_gid());
  if (index == 1) return gid_hex(depth_publisher_->get_gid());
  if (index == 2) return gid_hex(rgb_info_publisher_->get_gid());
  return gid_hex(depth_info_publisher_->get_gid());
}

RawReceipt::Header RawReceipt::header(size_t index, const std::string & reason)
{
  Header result;
  auto & source = sources_[index];
  if (source.sequence == std::numeric_limits<uint64_t>::max()) {
    failed_ = true;
    return result;
  }
  result.source_epoch = source.epoch;
  result.bridge_boot_id = boot_id_;
  result.capture_generation = capture_generation_;
  result.sequence = ++source.sequence;
  result.bridge_gid = output_gid(index);
  result.raw_gid = source.raw_gid;
  result.reason = reason;
  return result;
}

void RawReceipt::invalidate_all(const std::string & reason, size_t trigger,
                                const std_msgs::msg::Header * original,
                                int64_t received_ros_ns, double received_steady,
                                const std::string & raw_gid)
{
  if (failed_ || capture_generation_ == std::numeric_limits<uint64_t>::max()) {
    failed_ = true;
    return;
  }
  ++capture_generation_;
  if (!ledger_.advance(boot_id_, capture_generation_)) {
    failed_ = true;
    return;
  }
  for (size_t index = 0; index < sources_.size(); ++index) {
    auto value = header(index, reason);
    if (failed_) return;
    if (index == trigger && original != nullptr) {
      value.stamp = original->stamp;
      if (text_ok(original->frame_id, 128)) value.frame_id = original->frame_id;
      if (received_ros_ns > 0) value.received_ros = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(received_ros_ns));
      value.received_steady = received_steady;
      value.raw_gid = raw_gid;
    }
    if (index == 0 || index == 1) {
      RawImage output;
      output.capture = value;
      (index == 0 ? rgb_publisher_ : depth_publisher_)->publish(output);
    } else {
      RawInfo output;
      output.capture = value;
      (index == 2 ? rgb_info_publisher_ : depth_info_publisher_)->publish(output);
    }
  }
}

bool RawReceipt::clock_ok(size_t trigger, const std_msgs::msg::Header * original,
                          int64_t received_ros_ns, double received_steady,
                          const std::string & raw_gid)
{
  const int64_t current = get_clock()->now().nanoseconds();
  if (current <= 0 || (last_ros_ns_ > 0 && current < last_ros_ns_)) {
    if (!clock_bad_) {
      invalidate_all(current <= 0 ? "clock_unqualified" : "clock_rollback", trigger, original,
                     received_ros_ns, received_steady, raw_gid);
      for (auto & source : sources_) {
        source.active = false;
        source.raw_gid.clear();
        source.epoch = new_epoch();
        source.sequence = 0;
      }
    }
    clock_bad_ = true;
    last_ros_ns_ = current;
    return false;
  }
  clock_bad_ = false;
  last_ros_ns_ = current;
  return !failed_;
}

bool RawReceipt::qualify_source(size_t index, const rmw_gid_t & incoming,
                                const std_msgs::msg::Header & source_header,
                                int64_t received_ros_ns, double received_steady)
{
  const auto graph = get_publishers_info_by_topic(input_topics_[index]);
  const std::string actual = gid_hex(incoming);
  auto & source = sources_[index];
  if (source.retired_gids.count(actual) != 0) return false;
  const bool nonzero = std::any_of(std::begin(incoming.data), std::end(incoming.data),
                                   [](uint8_t value) {return value != 0;});
  if (!nonzero || graph.size() != 1 || gid_hex(graph[0].endpoint_gid()) != actual) {
    if (!retire_gid(source.retired_gids, source.raw_gid) ||
        !retire_gid(source.retired_gids, actual)) return false;
    invalidate_all("raw_gid_mismatch", index, &source_header, received_ros_ns,
                   received_steady, actual);
    sources_[index].active = false;
    sources_[index].raw_gid.clear();
    sources_[index].epoch = new_epoch();
    sources_[index].sequence = 0;
    return false;
  }
  if (!source.active || source.raw_gid != actual) {
    if (!retire_gid(source.retired_gids, source.raw_gid)) return false;
    invalidate_all("raw_source_changed", index, &source_header, received_ros_ns,
                   received_steady, actual);
    source.active = true;
    source.raw_gid = actual;
    source.epoch = new_epoch();
    source.sequence = 0;
    source.invalid = false;
  }
  return !failed_;
}

void RawReceipt::inspect_topology()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!clock_ok()) return;
  inspect_depth_owner();
  inspect_semantic_owner();
  std::array<bool, 4> changed{};
  bool any = false;
  for (size_t index = 0; index < sources_.size(); ++index) {
    if (!sources_[index].active) continue;
    const auto graph = get_publishers_info_by_topic(input_topics_[index]);
    changed[index] = graph.size() != 1 || gid_hex(graph[0].endpoint_gid()) != sources_[index].raw_gid;
    any = any || changed[index];
  }
  if (!any) return;
  invalidate_all("raw_topology_changed");
  for (size_t index = 0; index < sources_.size(); ++index) {
    if (!changed[index]) continue;
    if (!retire_gid(sources_[index].retired_gids, sources_[index].raw_gid)) return;
    sources_[index].active = false;
    sources_[index].raw_gid.clear();
    sources_[index].epoch = new_epoch();
    sources_[index].sequence = 0;
  }
}

void RawReceipt::on_image(size_t index, const Image & message, const rclcpp::MessageInfo & info)
{
  const int64_t received_ros_ns = get_clock()->now().nanoseconds();
  const double received_steady = steady_now();
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  if (before_verify_) before_verify_(index, incoming);
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!clock_ok(index, &message.header, received_ros_ns, received_steady, gid_hex(incoming)) ||
      !qualify_source(index, incoming, message.header, received_ros_ns,
                                     received_steady)) return;
  if (received_ros_ns <= 0 || !stamp_ok(message.header.stamp) ||
      !text_ok(message.header.frame_id, 128) || !image_layout_ok(message, index)) {
    if (!sources_[index].invalid) {
      invalidate_all("raw_image_invalid", index, &message.header, received_ros_ns,
                     received_steady, gid_hex(incoming));
      sources_[index].invalid = true;
    }
    return;
  }
  RawImage output;
  output.width = message.width;
  output.height = message.height;
  output.step = message.step;
  output.encoding = message.encoding;
  output.is_bigendian = message.is_bigendian != 0;
  output.data.assign(message.data.begin(), message.data.end());
  if (!qualify_source(index, incoming, message.header, received_ros_ns, received_steady)) return;
  output.capture = header(index, "");
  if (failed_) return;
  output.capture.valid = true;
  output.capture.stamp = message.header.stamp;
  output.capture.frame_id = message.header.frame_id;
  output.capture.received_ros = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(received_ros_ns));
  output.capture.received_steady = received_steady;
  sources_[index].invalid = false;
  if (!ledger_.observe(index, output.capture, steady_now())) return;
  (index == 0 ? rgb_publisher_ : depth_publisher_)->publish(output);
}

void RawReceipt::on_info(size_t index, const Info & message, const rclcpp::MessageInfo & info)
{
  const int64_t received_ros_ns = get_clock()->now().nanoseconds();
  const double received_steady = steady_now();
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  if (before_verify_) before_verify_(index, incoming);
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!clock_ok(index, &message.header, received_ros_ns, received_steady, gid_hex(incoming)) ||
      !qualify_source(index, incoming, message.header, received_ros_ns,
                                     received_steady)) return;
  if (received_ros_ns <= 0 || !stamp_ok(message.header.stamp) ||
      !text_ok(message.header.frame_id, 128) || !dimensions_ok(message.width, message.height) ||
      !text_ok(message.distortion_model, 32) || message.d.size() > 16) {
    if (!sources_[index].invalid) {
      invalidate_all("raw_camera_info_invalid", index, &message.header, received_ros_ns,
                     received_steady, gid_hex(incoming));
      sources_[index].invalid = true;
    }
    return;
  }
  RawInfo output;
  output.width = message.width;
  output.height = message.height;
  output.distortion_model = message.distortion_model;
  output.d.assign(message.d.begin(), message.d.end());
  output.k = message.k;
  output.r = message.r;
  output.p = message.p;
  output.binning_x = message.binning_x;
  output.binning_y = message.binning_y;
  output.roi = {message.roi.x_offset, message.roi.y_offset, message.roi.width, message.roi.height};
  output.roi_do_rectify = message.roi.do_rectify;
  if (!qualify_source(index, incoming, message.header, received_ros_ns, received_steady)) return;
  output.capture = header(index, "");
  if (failed_) return;
  output.capture.valid = true;
  output.capture.stamp = message.header.stamp;
  output.capture.frame_id = message.header.frame_id;
  output.capture.received_ros = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(received_ros_ns));
  output.capture.received_steady = received_steady;
  sources_[index].invalid = false;
  if (!ledger_.observe(index, output.capture, steady_now())) return;
  (index == 2 ? rgb_info_publisher_ : depth_info_publisher_)->publish(output);
}
bool RawReceipt::retire_gid(std::set<std::string> & retired, const std::string & gid)
{
  if (!gid.empty()) retired.insert(gid);
  if (retired.size() >= 256) {
    failed_ = true;
    return false;
  }
  return true;
}

void RawReceipt::invalidate_depth(
  const std::string & reason, bool owner_lost,
  const senior_motion_msgs::msg::BoundedDepthEvidence * body)
{
  if (failed_ || (depth_invalid_ && !owner_lost)) return;
  if (depth_relay_sequence_ == std::numeric_limits<uint64_t>::max()) {
    failed_ = true;
    return;
  }
  senior_motion_msgs::msg::FencedDepthEvidence output;
  if (body != nullptr) output.body = *body;
  clear_depth_history();
  if (semantic_active_) invalidate_semantic("depth_invalid", false, true);
  auto & header = output.owner;
  header.relay_gid = gid_hex(depth_fenced_pub_->get_gid());
  header.relay_boot = boot_id_;
  header.relay_sequence = ++depth_relay_sequence_;
  header.owner_gid = depth_owner_gid_;
  header.owner_epoch = depth_owner_epoch_;
  header.owner_generation = depth_owner_generation_;
  header.owner_sequence = owner_lost ? 0 : depth_owner_sequence_;
  header.owner_lost = owner_lost;
  header.reason = reason;
  depth_invalid_ = true;
  depth_fenced_pub_->publish(output);
}

void RawReceipt::inspect_depth_owner()
{
  if (!depth_active_ || failed_) return;
  const auto graph = get_publishers_info_by_topic(depth_input_topic_);
  if (graph.size() == 1 && gid_hex(graph[0].endpoint_gid()) == depth_owner_gid_) return;
  if (!retire_gid(depth_retired_gids_, depth_owner_gid_)) return;
  if (depth_owner_generation_ == std::numeric_limits<uint64_t>::max()) {
    failed_ = true;
    return;
  }
  ++depth_owner_generation_;
  invalidate_depth("owner_lost", true);
  depth_active_ = false;
}

void RawReceipt::on_depth(
  const senior_motion_msgs::msg::BoundedDepthEvidence & message,
  const rclcpp::MessageInfo & info)
{
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  const auto actual = gid_hex(incoming);
  if (before_verify_) before_verify_(4, incoming);
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (failed_ || depth_retired_gids_.count(actual) != 0) return;
  const auto graph = get_publishers_info_by_topic(depth_input_topic_);
  if (graph.size() != 1 || gid_hex(graph[0].endpoint_gid()) != actual) {
    inspect_depth_owner();
    return;
  }
  if (!text_ok(message.source_epoch, 64) || message.sequence == 0 ||
      depth_retired_epochs_.count(message.source_epoch) != 0) return;
  const auto original_now = steady_now();
  if ((message.depth_capture.sequence != 0 &&
       ledger_.match(1, message.depth_capture, original_now) == CaptureLedger::Match::retired) ||
      (message.camera_info_capture.sequence != 0 &&
       ledger_.match(3, message.camera_info_capture, original_now) == CaptureLedger::Match::retired)) {
    return;
  }
  const bool changed = !depth_active_ || actual != depth_owner_gid_ ||
                       message.source_epoch != depth_owner_epoch_;
  if (changed) {
    clear_depth_history();
    depth_history_retired_sequence_ = 0;
    if (semantic_active_) invalidate_semantic("depth_owner_changed", false, true);
    if (depth_active_ && actual != depth_owner_gid_ &&
        !retire_gid(depth_retired_gids_, depth_owner_gid_)) return;
    if (!depth_owner_epoch_.empty() && message.source_epoch != depth_owner_epoch_) {
      depth_retired_epochs_.insert(depth_owner_epoch_);
      if (depth_retired_epochs_.size() >= 256) {failed_ = true; return;}
    }
    if (depth_owner_generation_ == std::numeric_limits<uint64_t>::max()) {
      failed_ = true;
      return;
    }
    ++depth_owner_generation_;
    depth_owner_gid_ = actual;
    depth_owner_epoch_ = message.source_epoch;
    depth_owner_sequence_ = 0;
    depth_active_ = true;
    depth_invalid_ = false;
  }
  if (message.sequence <= depth_owner_sequence_) return;
  depth_owner_sequence_ = message.sequence;
  if (!message.valid) {
    invalidate_depth(message.reason.empty() ? "depth_invalid" : message.reason, false, &message);
    return;
  }
  const auto now = steady_now();
  const auto & d = message.depth_capture;
  const auto & c = message.camera_info_capture;
  const bool shape = ledger_.match(1, d, now) == CaptureLedger::Match::exact &&
    ledger_.match(3, c, now) == CaptureLedger::Match::exact && d.valid && c.valid &&
    d.bridge_boot_id == c.bridge_boot_id && d.capture_generation == c.capture_generation &&
    d.stamp == c.stamp && d.stamp == message.stamp && c.stamp == message.camera_info_stamp &&
    d.received_ros == message.received_ros && c.received_ros == message.camera_info_received_ros &&
    d.received_steady == message.received_steady &&
    c.received_steady == message.camera_info_received_steady &&
    d.frame_id == message.frame_id && c.frame_id == message.frame_id &&
    d.raw_gid == message.raw_source && c.raw_gid == message.camera_info_source;
  if (!shape) {
    invalidate_depth("capture_mismatch", false, &message);
    return;
  }
  if (depth_relay_sequence_ == std::numeric_limits<uint64_t>::max()) {
    failed_ = true;
    return;
  }
  senior_motion_msgs::msg::FencedDepthEvidence output;
  output.body = message;
  auto & header = output.owner;
  header.relay_gid = gid_hex(depth_fenced_pub_->get_gid());
  header.relay_boot = boot_id_;
  header.relay_sequence = ++depth_relay_sequence_;
  header.owner_gid = actual;
  header.owner_epoch = depth_owner_epoch_;
  header.owner_generation = depth_owner_generation_;
  header.owner_sequence = message.sequence;
  header.valid = true;
  depth_invalid_ = false;
  depth_history_.push_back(output);
  while (depth_history_.size() > 16) {
    depth_history_retired_sequence_ = depth_history_.front().owner.relay_sequence;
    depth_history_.pop_front();
  }
  depth_fenced_pub_->publish(output);
}
void RawReceipt::clear_depth_history()
{
  while (!depth_history_.empty()) {
    depth_history_retired_sequence_ = depth_history_.front().owner.relay_sequence;
    depth_history_.pop_front();
  }
}

void RawReceipt::invalidate_semantic(
  const std::string & reason, bool owner_lost, bool upstream_invalid,
  const senior_motion_msgs::msg::BoundedSemanticEvidence * body)
{
  if (failed_ || (semantic_invalid_ && !owner_lost)) return;
  if (semantic_relay_sequence_ == std::numeric_limits<uint64_t>::max()) {
    failed_ = true;
    return;
  }
  senior_motion_msgs::msg::FencedSemanticEvidence output;
  if (body != nullptr) output.body = *body;
  auto & header = output.owner;
  header.relay_gid = gid_hex(semantic_fenced_pub_->get_gid());
  header.relay_boot = boot_id_;
  header.relay_sequence = ++semantic_relay_sequence_;
  header.owner_gid = semantic_owner_gid_;
  header.owner_epoch = semantic_owner_epoch_;
  header.owner_generation = semantic_owner_generation_;
  header.owner_sequence = owner_lost ? 0 : semantic_owner_sequence_;
  header.owner_lost = owner_lost;
  header.upstream_invalid = upstream_invalid;
  header.reason = reason;
  semantic_invalid_ = true;
  semantic_fenced_pub_->publish(output);
}

void RawReceipt::inspect_semantic_owner()
{
  if (!semantic_active_ || failed_) return;
  const auto graph = get_publishers_info_by_topic(semantic_input_topic_);
  if (graph.size() == 1 && gid_hex(graph[0].endpoint_gid()) == semantic_owner_gid_) return;
  if (!retire_gid(semantic_retired_gids_, semantic_owner_gid_)) return;
  if (semantic_owner_generation_ == std::numeric_limits<uint64_t>::max()) {
    failed_ = true;
    return;
  }
  ++semantic_owner_generation_;
  invalidate_semantic("owner_lost", true);
  semantic_active_ = false;
}

void RawReceipt::on_semantic(
  const senior_motion_msgs::msg::BoundedSemanticEvidence & message,
  const rclcpp::MessageInfo & info)
{
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  const auto actual = gid_hex(incoming);
  if (before_verify_) before_verify_(5, incoming);
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (failed_ || semantic_retired_gids_.count(actual) != 0) return;
  const auto graph = get_publishers_info_by_topic(semantic_input_topic_);
  if (graph.size() != 1 || gid_hex(graph[0].endpoint_gid()) != actual) {
    inspect_semantic_owner();
    return;
  }
  if (!text_ok(message.source_epoch, 64) || message.sequence == 0 ||
      semantic_retired_epochs_.count(message.source_epoch) != 0) return;
  const auto now = steady_now();
  const std::array<Header, 4> captures = {message.rgb_capture, message.depth.depth_capture,
    message.rgb_camera_info_capture, message.depth.camera_info_capture};
  for (size_t index = 0; index < captures.size(); ++index) {
    if (captures[index].sequence != 0 &&
        ledger_.match(index, captures[index], now) == CaptureLedger::Match::retired) return;
  }
  if (message.valid && message.depth_owner.owner_generation < depth_owner_generation_ &&
      (depth_retired_gids_.count(message.depth_owner.owner_gid) != 0 ||
       depth_retired_epochs_.count(message.depth_owner.owner_epoch) != 0)) return;
  while (!depth_history_.empty() &&
         (now < depth_history_.front().body.depth_capture.received_steady ||
          now - depth_history_.front().body.depth_capture.received_steady > 1.0)) {
    depth_history_retired_sequence_ = depth_history_.front().owner.relay_sequence;
    depth_history_.pop_front();
  }
  if (message.valid && message.depth_owner.owner_generation == depth_owner_generation_ &&
      message.depth_owner.relay_sequence > 0 &&
      message.depth_owner.relay_sequence <= depth_history_retired_sequence_) return;
  const bool changed = !semantic_active_ || actual != semantic_owner_gid_ ||
                       message.source_epoch != semantic_owner_epoch_;
  if (changed) {
    if (semantic_active_ && actual != semantic_owner_gid_ &&
        !retire_gid(semantic_retired_gids_, semantic_owner_gid_)) return;
    if (!semantic_owner_epoch_.empty() && message.source_epoch != semantic_owner_epoch_) {
      semantic_retired_epochs_.insert(semantic_owner_epoch_);
      if (semantic_retired_epochs_.size() >= 256) {failed_ = true; return;}
    }
    if (semantic_owner_generation_ == std::numeric_limits<uint64_t>::max()) {
      failed_ = true;
      return;
    }
    ++semantic_owner_generation_;
    semantic_owner_gid_ = actual;
    semantic_owner_epoch_ = message.source_epoch;
    semantic_owner_sequence_ = 0;
    semantic_active_ = true;
    semantic_invalid_ = false;
  }
  if (message.sequence <= semantic_owner_sequence_) return;
  semantic_owner_sequence_ = message.sequence;
  if (!message.valid) {
    invalidate_semantic(message.reason.empty() ? "semantic_invalid" : message.reason,
                        false, false, &message);
    return;
  }
  const auto & rgb = message.rgb_capture;
  const auto & rgb_info = message.rgb_camera_info_capture;
  bool capture_ok = true;
  for (size_t index = 0; index < captures.size(); ++index) {
    capture_ok = capture_ok && captures[index].valid &&
      ledger_.match(index, captures[index], now) == CaptureLedger::Match::exact &&
      captures[index].bridge_boot_id == rgb.bridge_boot_id &&
      captures[index].capture_generation == rgb.capture_generation &&
      captures[index].stamp == rgb.stamp;
  }
  capture_ok = capture_ok && rgb.stamp == message.rgb_stamp &&
    rgb_info.stamp == message.rgb_camera_info_stamp &&
    rgb.received_ros == message.rgb_received_ros &&
    rgb_info.received_ros == message.rgb_camera_info_received_ros &&
    rgb.received_steady == message.rgb_received_steady &&
    rgb_info.received_steady == message.rgb_camera_info_received_steady &&
    rgb.frame_id == message.rgb_frame_id && rgb_info.frame_id == message.rgb_frame_id &&
    rgb.raw_gid == message.rgb_source && rgb_info.raw_gid == message.rgb_camera_info_source;
  if (!capture_ok) {
    invalidate_semantic("capture_mismatch", false, false, &message);
    return;
  }
  bool depth_exact = false;
  if (depth_active_ && !depth_invalid_ &&
      message.depth_owner.owner_generation == depth_owner_generation_) {
    for (const auto & issued : depth_history_) {
      if (issued.owner == message.depth_owner && issued.body == message.depth) {
        depth_exact = true;
        break;
      }
    }
  }
  if (!depth_exact) {
    invalidate_semantic("depth_body_mismatch", false, false, &message);
    return;
  }
  if (semantic_relay_sequence_ == std::numeric_limits<uint64_t>::max()) {
    failed_ = true;
    return;
  }
  senior_motion_msgs::msg::FencedSemanticEvidence output;
  output.body = message;
  auto & header = output.owner;
  header.relay_gid = gid_hex(semantic_fenced_pub_->get_gid());
  header.relay_boot = boot_id_;
  header.relay_sequence = ++semantic_relay_sequence_;
  header.owner_gid = actual;
  header.owner_epoch = semantic_owner_epoch_;
  header.owner_generation = semantic_owner_generation_;
  header.owner_sequence = message.sequence;
  header.valid = true;
  semantic_invalid_ = false;
  semantic_fenced_pub_->publish(output);
}
}  // namespace senior_sensor_ingress
