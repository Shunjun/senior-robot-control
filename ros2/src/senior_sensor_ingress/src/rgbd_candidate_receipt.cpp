#include "senior_sensor_ingress/rgbd_candidate_receipt.hpp"
#include "senior_sensor_ingress/static_tf_receipt.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <tf2_msgs/msg/tf_message.hpp>

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
std::string actual_gid(const rclcpp::MessageInfo & info)
{
  const auto value = info.get_rmw_message_info().publisher_gid;
  return hex_gid(value.data, RMW_GID_STORAGE_SIZE);
}
std::string new_boot()
{
  std::random_device random;
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (int i = 0; i < 4; ++i) out << std::setw(8) << random();
  return out.str();
}
double steady_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
int64_t nanos(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u) return -1;
  return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
}
bool text_ok(const std::string & value, size_t maximum)
{
  return !value.empty() && value.size() <= maximum &&
    std::all_of(value.begin(), value.end(), [](unsigned char c) {return c >= 33 && c <= 126;});
}
bool valid_gid(const std::string & value)
{
  if (value.size() != 2 * RMW_GID_STORAGE_SIZE) return false;
  bool any = false;
  for (const char c : value) {
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
    any = any || c != '0';
  }
  return any;
}
bool rigid(const std::array<double, 16> & m)
{
  for (const double v : m) if (!std::isfinite(v)) return false;
  if (std::abs(m[12]) > 1e-8 || std::abs(m[13]) > 1e-8 || std::abs(m[14]) > 1e-8 ||
      std::abs(m[15]-1) > 1e-8) return false;
  for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) {
    double dot = 0;
    for (int k=0; k<3; ++k) dot += m[k*4+i] * m[k*4+j];
    if (!std::isfinite(dot) || std::abs(dot - (i==j ? 1.0 : 0.0)) > 1e-6) return false;
  }
  const double det = m[0]*(m[5]*m[10]-m[6]*m[9]) - m[1]*(m[4]*m[10]-m[6]*m[8]) +
                     m[2]*(m[4]*m[9]-m[5]*m[8]);
  return std::isfinite(det) && std::abs(det-1) <= 1e-6;
}
}  // namespace

RgbdCandidateReceipt::RgbdCandidateReceipt(const rclcpp::NodeOptions & options)
: Node("rgbd_candidate_receipt", options), relay_boot_(new_boot())
{
  const auto prefix = declare_parameter<std::string>("topic_prefix", "/rgbd_diagnostic");
  if (!text_ok(prefix, 100)) throw std::invalid_argument("invalid RGB-D receipt prefix");
  raw_topics_ = {prefix+"/rgb", prefix+"/depth", prefix+"/rgb_info", prefix+"/depth_info"};
  tf_topic_ = prefix+"/tf";
  candidate_topic_ = prefix+"/candidate";
  output_topic_ = prefix+"/fenced";
  base_ = declare_parameter<std::string>("base_frame", "base_link");
  camera_ = declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
  if (!text_ok(base_, 128) || !text_ok(camera_, 128) || base_ == camera_)
    throw std::invalid_argument("invalid RGB-D frame configuration");
  const auto qos = rclcpp::SensorDataQoS().keep_last(4);
  publisher_ = create_publisher<Fenced>(output_topic_, qos);
  for (size_t i=0; i<2; ++i) {
    images_[i] = create_subscription<Image>(raw_topics_[i], qos,
      [this, i](Image::ConstSharedPtr body, const rclcpp::MessageInfo & info) {image(i, *body, info);});
    infos_[i] = create_subscription<Info>(raw_topics_[i+2], qos,
      [this, i](Info::ConstSharedPtr body, const rclcpp::MessageInfo & info) {camera_info(i, *body, info);});
  }
  tf_sub_ = create_subscription<Static>(tf_topic_, rclcpp::QoS(1).reliable().transient_local(),
    [this](Static::ConstSharedPtr body, const rclcpp::MessageInfo & info) {tf(*body, actual_gid(info));});
  candidate_sub_ = create_subscription<Candidate>(candidate_topic_, qos,
    [this](Candidate::ConstSharedPtr body, const rclcpp::MessageInfo & info) {candidate(*body, info);});
  timer_ = create_wall_timer(20ms, [this] {inspect();});
}

std::string RgbdCandidateReceipt::output_gid() const
{
  const auto value = publisher_->get_gid();
  return hex_gid(value.data, RMW_GID_STORAGE_SIZE);
}
bool RgbdCandidateReceipt::graph_matches(const std::string & topic, const std::string & node,
                                         const std::string & gid)
{
  const auto graph = get_publishers_info_by_topic(topic);
  return graph.size() == 1 && graph[0].node_name() == node && graph[0].node_namespace() == "/" &&
    hex_gid(graph[0].endpoint_gid().data(), RMW_GID_STORAGE_SIZE) == gid;
}
void RgbdCandidateReceipt::invalidate(const std::string & reason, bool owner_lost)
{
  Fenced output;
  output.owner.relay_gid = output_gid();
  output.owner.relay_boot = relay_boot_;
  output.owner.relay_sequence = ++relay_sequence_;
  output.owner.owner_gid = producer_gid_;
  output.owner.owner_epoch = producer_boot_;
  output.owner.owner_generation = producer_generation_;
  output.owner.owner_sequence = producer_sequence_;
  output.owner.valid = false;
  output.owner.owner_lost = owner_lost;
  output.owner.reason = reason;
  output.body = last_candidate_;
  publisher_->publish(output);
}
bool RgbdCandidateReceipt::clocks_ok(int64_t ros_ns, double steady)
{
  if (failed_) return false;
  if (last_ros_ > 0 && (ros_ns < last_ros_ || steady < last_steady_)) {
    // A new time epoch requires restarting the diagnostic chain and its ledgers.
    failed_ = true;
    ledger_.clear(); pending_tf_.clear(); tf_ready_ = false; producer_ready_ = false;
    invalidate("clock_invalid", true);
    return false;
  }
  if (ros_ns <= 0 || !std::isfinite(steady) || steady <= 0) return false;
  last_ros_ = ros_ns; last_steady_ = steady;
  return true;
}

bool RgbdCandidateReceipt::accept_raw(size_t stream, const Header & h, const std::string & actual)
{
  auto & source = sources_[stream];
  if (source.retired_bridge.count(actual) || source.retired_raw.count(h.raw_gid) ||
      retired_boots_.count(h.bridge_boot_id)) return false;
  if (!valid_gid(actual) || h.bridge_gid != actual || !graph_matches(raw_topics_[stream], "raw_receipt", actual)) {
    ledger_.clear(); invalidate("raw_owner_invalid", true); return false;
  }
  if (!text_ok(h.bridge_boot_id, 64) || h.capture_generation == 0 || h.sequence == 0) {
    ledger_.clear(); invalidate("raw_header_invalid"); return false;
  }
  bool boot_replaced = false;
  if (!raw_boot_.empty() && h.bridge_boot_id != raw_boot_) {
    const auto & prior_bridge = source.bridge_gid.empty() ? source.last_bridge_gid : source.bridge_gid;
    if (prior_bridge.empty() || prior_bridge == actual || retired_boots_.size() >= 64) {
      ledger_.clear(); invalidate("raw_boot_invalid"); failed_ = true; return false;
    }
    retired_boots_.insert(raw_boot_);
    for (auto & item : sources_) {
      if (!item.bridge_gid.empty()) {
        if (item.retired_bridge.size() >= 64) {ledger_.clear(); invalidate("retirement_exhausted", true); failed_ = true; return false;}
        item.last_bridge_gid = item.bridge_gid;
        item.last_bridge_boot = raw_boot_;
        item.retired_bridge.insert(item.bridge_gid);
      }
      item.bridge_gid.clear(); item.sequence = 0; item.has_last_header = false;
    }
    raw_boot_ = h.bridge_boot_id;
    boot_replaced = true;
    raw_generation_ = 0;
    ledger_.clear(); invalidate("raw_restarted");
  }
  if (!boot_replaced && !raw_boot_.empty() && h.bridge_boot_id == raw_boot_ && source.bridge_gid.empty() &&
      !source.last_bridge_gid.empty() && source.last_bridge_boot == raw_boot_ &&
      source.last_bridge_gid != actual) {
    ledger_.clear(); invalidate("raw_boot_invalid"); failed_ = true; return false;
  }
  if (h.capture_generation < raw_generation_) return false;
  if (h.capture_generation > raw_generation_) {
    if (raw_generation_) {ledger_.clear(); invalidate("raw_generation_changed");}
    raw_generation_ = h.capture_generation;
    for (auto & item : sources_) {item.sequence = 0; item.epoch.clear(); item.has_last_header = false;}
  }
  raw_boot_ = h.bridge_boot_id;
  if (!source.bridge_gid.empty() && source.bridge_gid != actual) {
    ledger_.clear(); invalidate("raw_owner_changed"); failed_ = true; return false;
  }
  if (source.sequence && h.sequence < source.sequence) return false;
  if (source.sequence && h.sequence == source.sequence &&
      (!source.has_last_header || h != source.last_header)) {
    ledger_.clear(); invalidate("raw_header_conflict"); return false;
  }
  if (!h.valid) {
    source.bridge_gid = actual;
    source.sequence = h.sequence;
    source.last_header = h; source.has_last_header = true;
    ledger_.clear(); invalidate("raw_invalid"); return false;
  }
  if (!text_ok(h.source_epoch, 64) || !text_ok(h.frame_id, 128) || !text_ok(h.raw_gid, 128)) {
    source.bridge_gid = actual;
    source.sequence = h.sequence;
    source.last_header = h; source.has_last_header = true;
    ledger_.clear(); invalidate("raw_header_invalid"); return false;
  }
  if (!source.raw_gid.empty() && source.raw_gid != h.raw_gid) {
    if (source.retired_raw.size() >= 64) {ledger_.clear(); invalidate("retirement_exhausted", true); failed_ = true; return false;}
    source.retired_raw.insert(source.raw_gid);
    ledger_.clear(); invalidate("raw_source_changed");
  } else if (!source.epoch.empty() && source.epoch != h.source_epoch && source.sequence != 0) {
    ledger_.clear(); invalidate("raw_epoch_changed"); failed_ = true; return false;
  }
  source.bridge_gid = actual; source.raw_gid = h.raw_gid; source.epoch = h.source_epoch;
  source.sequence = h.sequence;
  source.last_header = h; source.has_last_header = true;
  return true;
}
void RgbdCandidateReceipt::image(size_t stream, const Image & body, const rclcpp::MessageInfo & info)
{
  const auto actual = actual_gid(info);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clocks_ok(get_clock()->now().nanoseconds(), steady_now()) || !accept_raw(stream, body.capture, actual)) return;
  if (!ledger_.remember_image(stream, body) || !graph_matches(raw_topics_[stream], "raw_receipt", actual)) {
    ledger_.clear(); invalidate("raw_image_invalid");
  }
}
void RgbdCandidateReceipt::camera_info(size_t stream, const Info & body, const rclcpp::MessageInfo & info)
{
  const auto actual = actual_gid(info);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clocks_ok(get_clock()->now().nanoseconds(), steady_now()) || !accept_raw(stream+2, body.capture, actual)) return;
  if (!ledger_.remember_info(stream, body) || !graph_matches(raw_topics_[stream+2], "raw_receipt", actual)) {
    ledger_.clear(); invalidate("raw_info_invalid");
  }
}

void RgbdCandidateReceipt::tf(const Static & body, const std::string & actual)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool clock_ready = clocks_ok(get_clock()->now().nanoseconds(), steady_now());
  if (failed_ || retired_tf_gids_.count(actual) || !valid_gid(actual) ||
      body.bridge_gid != actual) return;
  // A transient-local status can arrive before graph discovery catches up.
  auto prior = std::find_if(pending_tf_.begin(), pending_tf_.end(),
    [&](const auto & entry) {return entry.first == actual;});
  if (prior != pending_tf_.end()) {
    const auto & previous = prior->second;
    if (body.generation < previous.generation ||
        (body.generation == previous.generation &&
        (body.sequence < previous.sequence ||
        (body.sequence == previous.sequence && !previous.valid && body.valid)))) return;
    pending_tf_.erase(prior);
  }
  if (pending_tf_.size() == 4) pending_tf_.pop_front();
  pending_tf_.emplace_back(actual, body);
  if (clock_ready) retry_tf();
}

void RgbdCandidateReceipt::retry_tf()
{
  for (auto pending = pending_tf_.begin(); pending != pending_tf_.end();) {
    if (retired_tf_gids_.count(pending->first)) {
      pending = pending_tf_.erase(pending);
      continue;
    }
    if (graph_matches(tf_topic_, "static_tf_receipt", pending->first)) {
      accept_tf(pending->second, pending->first);
      pending_tf_.erase(pending);
      return;
    }
    ++pending;
  }
}

void RgbdCandidateReceipt::accept_tf(const Static & body, const std::string & actual)
{
  if (!tf_gid_.empty() && tf_gid_ != actual) {
    if (retired_tf_gids_.size() >= 64) {ledger_.clear(); invalidate("retirement_exhausted", true); failed_ = true; return;}
    retired_tf_gids_.insert(tf_gid_); tf_ready_ = false; ledger_.clear(); invalidate("tf_owner_changed");
    tf_generation_ = 0; tf_boot_.clear(); tf_raw_gid_.clear(); tf_needs_new_generation_ = false;
  }
  tf_gid_ = actual;
  if (retired_tf_raw_gids_.count(body.raw_gid) || body.generation < tf_generation_) return;
  if (!tf_boot_.empty() && body.bridge_boot_id != tf_boot_) {
    tf_ready_ = false; ledger_.clear(); invalidate("tf_boot_invalid"); failed_ = true; return;
  }
  if (tf_needs_new_generation_ && body.generation <= tf_generation_) return;
  if (body.valid && !body.admitted && tf_ready_ && body.generation == tf_generation_) {
    Static previous = body;
    previous.sequence = body.first_sequence;
    previous.received_ros = body.first_received_ros;
    previous.received_steady = body.first_received_steady;
    previous.admitted = true;
    previous.admitted_from_ros = tf_first_.admitted_from_ros;
    previous.admitted_from_steady = tf_first_.admitted_from_steady;
    if (previous == tf_first_) return;
  }
  if (!body.valid || !body.admitted || !text_ok(body.bridge_boot_id, 64) ||
      !text_ok(body.raw_gid, 128) || body.generation == 0 || body.first_sequence == 0 ||
      body.sequence < body.first_sequence || body.edges.empty() || body.edges.size() > 32 ||
      nanos(body.first_received_ros) < 0 || nanos(body.admitted_from_ros) <= 0 ||
      !std::isfinite(body.first_received_steady) || !std::isfinite(body.admitted_from_steady) ||
      body.first_received_steady < 0 || body.admitted_from_steady < body.first_received_steady ||
      nanos(body.admitted_from_ros) < nanos(body.first_received_ros)) {
    if (!body.valid && text_ok(body.bridge_boot_id, 64) && body.generation > tf_generation_) {
      tf_generation_ = body.generation;
      tf_boot_ = body.bridge_boot_id;
      if (tf_raw_gid_.empty() && text_ok(body.raw_gid, 128)) tf_raw_gid_ = body.raw_gid;
    }
    tf_ready_ = false; tf_needs_new_generation_ = true;
    ledger_.clear(); invalidate("tf_invalid"); return;
  }
  tf2_msgs::msg::TFMessage tree;
  for (const auto & edge : body.edges) {
    geometry_msgs::msg::TransformStamped value;
    value.header.frame_id = edge.parent_frame; value.child_frame_id = edge.child_frame;
    value.header.stamp = edge.stamp;
    value.transform.translation.x = edge.translation[0]; value.transform.translation.y = edge.translation[1];
    value.transform.translation.z = edge.translation[2];
    value.transform.rotation.x = edge.rotation_xyzw[0]; value.transform.rotation.y = edge.rotation_xyzw[1];
    value.transform.rotation.z = edge.rotation_xyzw[2]; value.transform.rotation.w = edge.rotation_xyzw[3];
    tree.transforms.push_back(value);
  }
  if (!validate_static_tf(tree, base_, camera_)) {
    tf_ready_ = false; ledger_.clear(); invalidate("tf_invalid"); return;
  }
  Static first = body;
  first.sequence = body.first_sequence;
  first.received_ros = body.first_received_ros;
  first.received_steady = body.first_received_steady;
  if (tf_generation_ && body.generation == tf_generation_ && tf_first_ != first) {
    tf_ready_ = false; tf_needs_new_generation_ = true;
    ledger_.clear(); invalidate("tf_invalid"); return;
  }
  if (tf_generation_ && body.generation > tf_generation_) {
    if (!tf_raw_gid_.empty() && tf_raw_gid_ != body.raw_gid) {
      if (retired_tf_raw_gids_.size() >= 64) {ledger_.clear(); invalidate("retirement_exhausted", true); failed_ = true; return;}
      retired_tf_raw_gids_.insert(tf_raw_gid_);
    }
    ledger_.clear(); invalidate("tf_changed");
  }
  tf_first_ = first; tf_generation_ = body.generation;
  tf_boot_ = body.bridge_boot_id; tf_raw_gid_ = body.raw_gid;
  tf_needs_new_generation_ = false; tf_ready_ = true;
}

bool RgbdCandidateReceipt::originals_current(const Candidate & body, int64_t ros_ns, double steady)
{
  const std::array<const Header *, 4> first = {&body.first_rgb, &body.first_depth,
    &body.first_rgb_info, &body.first_depth_info};
  const std::array<const Header *, 4> second = {&body.second_rgb, &body.second_depth,
    &body.second_rgb_info, &body.second_depth_info};
  const int64_t first_stamp = nanos(first[0]->stamp), second_stamp = nanos(second[0]->stamp);
  if (first_stamp <= 0 || second_stamp <= first_stamp || second_stamp-first_stamp > 200000000LL ||
      nanos(body.interval_start) != first_stamp || nanos(body.interval_end) != second_stamp) return false;
  const int64_t tf_admission = nanos(tf_first_.admitted_from_ros);
  for (size_t i=0; i<4; ++i) {
    const auto & current = sources_[i];
    if (current.bridge_gid.empty() || !graph_matches(raw_topics_[i], "raw_receipt", current.bridge_gid)) return false;
    for (const auto * h : {first[i], second[i]}) {
      const auto stamp = nanos(h->stamp), received = nanos(h->received_ros);
      if (!h->valid || h->bridge_gid != current.bridge_gid || h->raw_gid != current.raw_gid ||
          h->source_epoch != current.epoch || h->bridge_boot_id != raw_boot_ ||
          h->capture_generation != raw_generation_ || h->frame_id != camera_ || h->sequence == 0 ||
          stamp <= producer_floor_ros_ || received <= producer_floor_ros_ ||
          h->received_steady <= producer_floor_steady_ || stamp < tf_admission || received < tf_admission ||
          h->received_steady < tf_first_.admitted_from_steady ||
          stamp > ros_ns || received > ros_ns || ros_ns-stamp > 300000000LL ||
          ros_ns-received > 300000000LL || !std::isfinite(h->received_steady) ||
          h->received_steady > steady || steady-h->received_steady > .3) return false;
    }
    if (nanos(first[i]->stamp) != first_stamp || nanos(second[i]->stamp) != second_stamp ||
        second[i]->sequence <= first[i]->sequence) return false;
  }
  if (body.first_rgb_layout.capture != body.first_rgb || body.first_depth_layout.capture != body.first_depth ||
      body.second_rgb_layout.capture != body.second_rgb || body.second_depth_layout.capture != body.second_depth ||
      body.first_rgb_calibration.capture != body.first_rgb_info ||
      body.first_depth_calibration.capture != body.first_depth_info ||
      body.second_rgb_calibration.capture != body.second_rgb_info ||
      body.second_depth_calibration.capture != body.second_depth_info) return false;
  return ledger_.match_image(0, body.first_rgb_layout) && ledger_.match_image(1, body.first_depth_layout) &&
    ledger_.match_image(0, body.second_rgb_layout) && ledger_.match_image(1, body.second_depth_layout) &&
    ledger_.match_info(0, body.first_rgb_calibration) && ledger_.match_info(1, body.first_depth_calibration) &&
    ledger_.match_info(0, body.second_rgb_calibration) && ledger_.match_info(1, body.second_depth_calibration);
}

void RgbdCandidateReceipt::candidate(const Candidate & body, const rclcpp::MessageInfo & info)
{
  const auto actual = actual_gid(info);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ros_ns = get_clock()->now().nanoseconds();
  const auto steady = steady_now();
  if (!clocks_ok(ros_ns, steady) || retired_producer_gids_.count(actual)) return;
  if (!producer_ready_ || actual != producer_gid_ || body.producer_gid != actual ||
      !graph_matches(candidate_topic_, "rgbd_candidate_producer", actual)) return;
  if (!text_ok(body.producer_boot_id, 64) || body.owner_generation == 0 || body.worker_generation == 0 ||
      body.sequence == 0) {
    last_candidate_ = body; invalidate("producer_invalid"); return;
  }
  if (producer_boot_.empty()) {producer_boot_ = body.producer_boot_id; producer_generation_ = body.owner_generation;}
  if (body.producer_boot_id != producer_boot_) {
    last_candidate_ = body;
    invalidate("producer_boot_invalid", true);
    if (retired_producer_gids_.size() >= 64) {ledger_.clear(); invalidate("retirement_exhausted", true); failed_ = true; return;}
    retired_producer_gids_.insert(actual);
    producer_ready_ = false;
    producer_gid_.clear(); producer_boot_.clear(); producer_generation_ = producer_sequence_ = 0;
    return;
  }
  if (body.owner_generation < producer_generation_ || body.sequence <= producer_sequence_) return;
  if (body.owner_generation > producer_generation_) {
    producer_generation_ = body.owner_generation;
    producer_floor_ros_ = ros_ns; producer_floor_steady_ = steady;
    invalidate("producer_generation_changed");
  }
  producer_sequence_ = body.sequence;
  last_candidate_ = body;
  if (!body.valid || body.hard_error_qualified || !tf_ready_ ||
      body.first_tf != tf_first_ || body.second_tf != tf_first_ ||
      !graph_matches(tf_topic_, "static_tf_receipt", tf_gid_) ||
      !text_ok(body.profile_id, 128) || !text_ok(body.calibration_id, 128) ||
      !text_ok(body.registration_id, 128) || body.body_frame != base_ ||
      body.raw_rgb_encoding != "rgb8" || body.preprocessing_version != "rgb8-mono-v1" ||
      body.first_rgb_layout.encoding != "rgb8" || body.second_rgb_layout.encoding != "rgb8" ||
      body.first_depth_layout.encoding != "32FC1" || body.second_depth_layout.encoding != "32FC1" ||
      body.matched > 500 || body.inliers > body.matched || !rigid(body.camera_transform) ||
      !std::isfinite(body.median_reprojection_px) || body.median_reprojection_px < 0 ||
      !std::isfinite(body.median_depth_residual_m) || body.median_depth_residual_m < 0 ||
      !originals_current(body, ros_ns, steady)) {
    invalidate("candidate_invalid");
    return;
  }
  Fenced output;
  output.owner.relay_gid = output_gid(); output.owner.relay_boot = relay_boot_;
  output.owner.owner_gid = actual;
  output.owner.owner_epoch = producer_boot_; output.owner.owner_generation = producer_generation_;
  output.owner.owner_sequence = producer_sequence_; output.owner.valid = true;
  output.body = body;

  // Body matching and copying can outlive the capture deadline or graph owners.
  for (size_t stream = 0; stream < sources_.size(); ++stream) {
    if (!graph_matches(raw_topics_[stream], "raw_receipt", sources_[stream].bridge_gid)) {
      invalidate("raw_owner_lost", true);
      return;
    }
  }
  if (!graph_matches(candidate_topic_, "rgbd_candidate_producer", actual) ||
      !graph_matches(tf_topic_, "static_tf_receipt", tf_gid_)) {
    invalidate("candidate_owner_lost", true);
    return;
  }
  const auto final_ros = get_clock()->now().nanoseconds();
  const auto final_steady = steady_now();
  if (!clocks_ok(final_ros, final_steady)) return;
  for (const auto * header : {&body.first_rgb, &body.first_depth, &body.first_rgb_info,
      &body.first_depth_info, &body.second_rgb, &body.second_depth, &body.second_rgb_info,
      &body.second_depth_info}) {
    if (final_ros - nanos(header->stamp) > 300000000LL ||
        final_ros - nanos(header->received_ros) > 300000000LL ||
        final_steady - header->received_steady > .3) {
      invalidate("candidate_expired");
      return;
    }
  }
  output.owner.relay_sequence = ++relay_sequence_;
  publisher_->publish(output);
}

void RgbdCandidateReceipt::inspect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ros_ns = get_clock()->now().nanoseconds();
  const auto steady = steady_now();
  if (!clocks_ok(ros_ns, steady)) return;
  for (size_t i=0; i<4; ++i) {
    auto & source = sources_[i];
    if (!source.bridge_gid.empty() && !graph_matches(raw_topics_[i], "raw_receipt", source.bridge_gid)) {
      if (source.retired_bridge.size() >= 64) {ledger_.clear(); invalidate("retirement_exhausted", true); failed_ = true; return;}
      source.retired_bridge.insert(source.bridge_gid);
      source.last_bridge_gid = source.bridge_gid;
      source.last_bridge_boot = raw_boot_;
      ledger_.clear(); invalidate("raw_owner_lost", true);
      source.bridge_gid.clear(); source.sequence = 0; source.has_last_header = false;
    }
  }
  if (tf_ready_ && !graph_matches(tf_topic_, "static_tf_receipt", tf_gid_)) {
    if (retired_tf_gids_.size() >= 64) {ledger_.clear(); invalidate("retirement_exhausted", true); failed_ = true; return;}
    retired_tf_gids_.insert(tf_gid_);
    tf_ready_ = false; ledger_.clear(); invalidate("tf_owner_lost", true); tf_gid_.clear();
    tf_generation_ = 0; tf_boot_.clear(); tf_raw_gid_.clear(); tf_needs_new_generation_ = false;
  }
  retry_tf();
  const auto graph = get_publishers_info_by_topic(candidate_topic_);
  const bool unique = graph.size() == 1 && graph[0].node_name() == "rgbd_candidate_producer" &&
                      graph[0].node_namespace() == "/";
  const std::string current = unique ? hex_gid(graph[0].endpoint_gid().data(), RMW_GID_STORAGE_SIZE) : "";
  if (producer_ready_ && current != producer_gid_) {
    if (retired_producer_gids_.size() >= 64) {ledger_.clear(); invalidate("retirement_exhausted", true); failed_ = true; return;}
    retired_producer_gids_.insert(producer_gid_);
    producer_ready_ = false; invalidate("producer_lost", true);
    producer_gid_.clear(); producer_boot_.clear(); producer_generation_ = producer_sequence_ = 0;
  }
  if (!producer_ready_ && valid_gid(current) && !retired_producer_gids_.count(current)) {
    producer_gid_ = current; producer_floor_ros_ = ros_ns; producer_floor_steady_ = steady;
    producer_ready_ = true;
  }
}
}  // namespace senior_sensor_ingress
