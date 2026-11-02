#include "senior_sensor_ingress/scan_receipt.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "senior_sensor_ingress/raw_receipt.hpp"

namespace senior_sensor_ingress {
namespace {
using namespace std::chrono_literals;
constexpr size_t max_rays = 2048;

bool text_ok(const std::string & value, size_t maximum)
{
  return !value.empty() && value.size() <= maximum &&
    std::all_of(value.begin(), value.end(), [](unsigned char c) {return c >= 33 && c <= 126;});
}

bool stamp_ok(const builtin_interfaces::msg::Time & value)
{
  return value.sec >= 0 && value.nanosec < 1000000000u && (value.sec > 0 || value.nanosec > 0);
}

double steady_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool layout_ok(const sensor_msgs::msg::LaserScan & scan)
{
  return !scan.ranges.empty() && scan.ranges.size() <= max_rays &&
    scan.intensities.size() <= max_rays && std::isfinite(scan.angle_min) &&
    std::isfinite(scan.angle_max) && std::isfinite(scan.angle_increment) &&
    scan.angle_increment > 0 && std::isfinite(scan.time_increment) && scan.time_increment == 0 &&
    std::isfinite(scan.scan_time) && scan.scan_time >= 0 && std::isfinite(scan.range_min) &&
    std::isfinite(scan.range_max) && scan.range_min >= 0 && scan.range_max > scan.range_min;
}
}  // namespace

ScanReceipt::ScanReceipt(const rclcpp::NodeOptions & options, BeforeVerify before_verify,
                         bool test_reentrant_callbacks)
: Node("scan_receipt", options), before_verify_(std::move(before_verify))
{
  input_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  const auto output = declare_parameter<std::string>("raw_scan_topic", "/safety/raw/scan");
  if (!text_ok(input_topic_, 128) || !text_ok(output, 128) || input_topic_ == output) {
    throw std::invalid_argument("invalid scan receipt topics");
  }
  safety_owner_.topic = declare_parameter<std::string>("safety_candidate_topic", "/safety/scan_candidate");
  filter_owner_.topic = declare_parameter<std::string>("filter_candidate_topic", "/perception/scan_candidate");
  const auto safety_output = declare_parameter<std::string>("safety_output_topic", "/safety/bounded_scan");
  const auto filter_output = declare_parameter<std::string>("filter_output_topic", "/perception/scan_filtered");
  std::set<std::string> topics{input_topic_, output, safety_owner_.topic, filter_owner_.topic,
                               safety_output, filter_output};
  if (topics.size() != 6 || !text_ok(safety_owner_.topic, 128) ||
      !text_ok(filter_owner_.topic, 128) || !text_ok(safety_output, 128) ||
      !text_ok(filter_output, 128)) throw std::invalid_argument("invalid scan egress topics");
  ledger_ = ScanLedger(128, declare_parameter<double>("scan_ledger_max_age_s", 3.0));
  boot_ = new_epoch();
  epoch_ = new_epoch();
  const auto qos = rclcpp::SensorDataQoS().keep_last(4);
  publisher_ = create_publisher<Bounded>(output, qos);
  safety_publisher_ = create_publisher<senior_motion_msgs::msg::FencedSafetyScan>(safety_output, 10);
  filter_publisher_ = create_publisher<senior_motion_msgs::msg::FencedFilteredScan>(filter_output, qos);
  rclcpp::SubscriptionOptions options_for_scan;
  if (test_reentrant_callbacks) {
    test_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options_for_scan.callback_group = test_group_;
  }
  safety_subscription_ = create_subscription<senior_motion_msgs::msg::BoundedSafetyScan>(
    safety_owner_.topic, 10,
    [this](senior_motion_msgs::msg::BoundedSafetyScan::ConstSharedPtr message,
           const rclcpp::MessageInfo & info) {on_safety(*message, info);}, options_for_scan);
  filter_subscription_ = create_subscription<senior_motion_msgs::msg::BoundedFilteredScan>(
    filter_owner_.topic, qos,
    [this](senior_motion_msgs::msg::BoundedFilteredScan::ConstSharedPtr message,
           const rclcpp::MessageInfo & info) {on_filtered(*message, info);}, options_for_scan);
  subscription_ = create_subscription<Scan>(
    input_topic_, qos, [this](Scan::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
      on_scan(*message, info);
    }, options_for_scan);
  timer_ = create_wall_timer(20ms, [this] {inspect_topology();});
}

std::string ScanReceipt::new_epoch()
{
  std::random_device random;
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned i = 0; i < 4; ++i) output << std::setw(8) << uint32_t(random());
  output << '-' << ++counter_;
  return output.str();
}

bool ScanReceipt::retire(const std::string & gid)
{
  if (!gid.empty()) retired_gids_.insert(gid);
  if (retired_gids_.size() >= 256) {failed_ = true; return false;}
  return true;
}

void ScanReceipt::invalidate(const std::string & reason, const Scan * source,
                             int64_t received_ros, double received_steady,
                             const std::string & raw_gid)
{
  if (failed_ || generation_ == std::numeric_limits<uint64_t>::max() ||
      sequence_ == std::numeric_limits<uint64_t>::max()) {failed_ = true; return;}
  ++generation_;
  if (!ledger_.advance(boot_, generation_)) {failed_ = true; return;}
  if (safety_owner_.active) invalidate_owner(safety_owner_, 0, reason, false, true);
  if (filter_owner_.active) invalidate_owner(filter_owner_, 1, reason, false, true);
  Bounded output;
  auto & header = output.capture;
  header.bridge_boot_id = boot_;
  header.bridge_gid = gid_hex(publisher_->get_gid());
  header.capture_generation = generation_;
  header.source_epoch = epoch_;
  header.sequence = ++sequence_;
  header.raw_gid = raw_gid.empty() ? raw_gid_ : raw_gid;
  header.reason = reason;
  if (source != nullptr) {
    header.stamp = source->header.stamp;
    if (text_ok(source->header.frame_id, 128)) header.frame_id = source->header.frame_id;
    if (received_ros > 0) {
      header.received_ros = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(received_ros));
    }
    header.received_steady = received_steady;
  }
  publisher_->publish(output);
}

bool ScanReceipt::clock_ok()
{
  const int64_t now = get_clock()->now().nanoseconds();
  if (now <= 0 || (last_ros_ns_ > 0 && now < last_ros_ns_)) {
    if (!clock_bad_) {
      invalidate(now <= 0 ? "clock_unqualified" : "clock_rollback");
      active_ = false;
      raw_gid_.clear();
      epoch_ = new_epoch();
      sequence_ = 0;
    }
    clock_bad_ = true;
    last_ros_ns_ = now;
    return false;
  }
  clock_bad_ = false;
  last_ros_ns_ = now;
  return !failed_;
}

void ScanReceipt::inspect_topology()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clock_ok()) return;
  inspect_owner(safety_owner_, 0);
  inspect_owner(filter_owner_, 1);
  if (!active_) return;
  const auto graph = get_publishers_info_by_topic(input_topic_);
  if (graph.size() == 1 && gid_hex(graph[0].endpoint_gid()) == raw_gid_) return;
  if (!retire(raw_gid_)) return;
  invalidate("raw_topology_changed");
  active_ = false;
  raw_gid_.clear();
  epoch_ = new_epoch();
  sequence_ = 0;
  invalid_ = false;
}

void ScanReceipt::on_scan(const Scan & message, const rclcpp::MessageInfo & info)
{
  const int64_t received_ros = get_clock()->now().nanoseconds();
  const double received_steady = steady_now();
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  const auto actual = gid_hex(incoming);
  if (before_verify_) before_verify_(6, incoming);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clock_ok() || retired_gids_.count(actual) != 0) return;
  const auto graph = get_publishers_info_by_topic(input_topic_);
  const bool nonzero = std::any_of(std::begin(incoming.data), std::end(incoming.data),
                                   [](uint8_t value) {return value != 0;});
  if (!nonzero || graph.size() != 1 || gid_hex(graph[0].endpoint_gid()) != actual) {
    if (active_ && graph.size() == 1 && gid_hex(graph[0].endpoint_gid()) == raw_gid_) {
      retire(actual);
      return;
    }
    if (!retire(raw_gid_) || !retire(actual)) return;
    invalidate("raw_gid_mismatch", &message, received_ros, received_steady, actual);
    active_ = false;
    raw_gid_.clear();
    epoch_ = new_epoch();
    sequence_ = 0;
    invalid_ = false;
    return;
  }
  if (!active_ || raw_gid_ != actual) {
    if (!retire(raw_gid_)) return;
    invalidate("raw_source_changed", &message, received_ros, received_steady, actual);
    active_ = true;
    raw_gid_ = actual;
    epoch_ = new_epoch();
    sequence_ = 0;
    invalid_ = false;
  }
  if (received_ros <= 0 || !stamp_ok(message.header.stamp) ||
      !text_ok(message.header.frame_id, 128) || !layout_ok(message)) {
    if (!invalid_) {
      invalidate("raw_scan_invalid", &message, received_ros, received_steady, actual);
      invalid_ = true;
    }
    return;
  }
  if (sequence_ == std::numeric_limits<uint64_t>::max()) {failed_ = true; return;}
  Bounded output;
  output.angle_min = message.angle_min;
  output.angle_max = message.angle_max;
  output.angle_increment = message.angle_increment;
  output.time_increment = message.time_increment;
  output.scan_time = message.scan_time;
  output.range_min = message.range_min;
  output.range_max = message.range_max;
  output.ranges.assign(message.ranges.begin(), message.ranges.end());
  output.intensities.assign(message.intensities.begin(), message.intensities.end());
  auto & header = output.capture;
  header.stamp = message.header.stamp;
  header.frame_id = message.header.frame_id;
  header.received_ros = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(received_ros));
  header.received_steady = received_steady;
  header.raw_gid = actual;
  header.bridge_gid = gid_hex(publisher_->get_gid());
  header.source_epoch = epoch_;
  header.bridge_boot_id = boot_;
  header.capture_generation = generation_;
  header.sequence = ++sequence_;
  header.valid = true;
  invalid_ = false;
  if (!ledger_.observe(output, steady_now())) return;
  publisher_->publish(output);
}
bool ScanReceipt::qualify_owner(Owner & owner, const rmw_gid_t & incoming,
                                const std::string & epoch, uint64_t sequence)
{
  const auto actual = gid_hex(incoming);
  if (failed_ || owner.retired_gids.count(actual) || owner.retired_epochs.count(epoch)) return false;
  const auto graph = get_publishers_info_by_topic(owner.topic);
  if (graph.size() != 1 || gid_hex(graph[0].endpoint_gid()) != actual) {
    inspect_owner(owner, &owner == &safety_owner_ ? 0 : 1);
    return false;
  }
  if (!text_ok(epoch, 64) || sequence == 0) return false;
  const bool changed = !owner.active || owner.gid != actual || owner.epoch != epoch;
  if (changed) {
    if (owner.active && owner.gid != actual) owner.retired_gids.insert(owner.gid);
    if (!owner.epoch.empty() && owner.epoch != epoch) owner.retired_epochs.insert(owner.epoch);
    if (owner.retired_gids.size() >= 256 || owner.retired_epochs.size() >= 256 ||
        owner.generation == std::numeric_limits<uint64_t>::max()) {
      failed_ = true;
      return false;
    }
    ++owner.generation;
    owner.gid = actual;
    owner.epoch = epoch;
    owner.sequence = 0;
    owner.active = true;
    owner.invalid = false;
  }
  if (sequence <= owner.sequence) return false;
  owner.sequence = sequence;
  return true;
}

void ScanReceipt::inspect_owner(Owner & owner, size_t index)
{
  if (!owner.active || failed_) return;
  const auto graph = get_publishers_info_by_topic(owner.topic);
  if (graph.size() == 1 && gid_hex(graph[0].endpoint_gid()) == owner.gid) return;
  owner.retired_gids.insert(owner.gid);
  if (owner.retired_gids.size() >= 256 ||
      owner.generation == std::numeric_limits<uint64_t>::max()) {failed_ = true; return;}
  ++owner.generation;
  invalidate_owner(owner, index, "owner_lost", true);
  owner.active = false;
}

void ScanReceipt::invalidate_owner(
  Owner & owner, size_t index, const std::string & reason, bool owner_lost,
  bool upstream_invalid, const senior_motion_msgs::msg::BoundedSafetyScan * safety,
  const senior_motion_msgs::msg::BoundedFilteredScan * filtered)
{
  if (failed_ || (owner.invalid && !owner_lost)) return;
  if (owner.relay_sequence == std::numeric_limits<uint64_t>::max()) {failed_ = true; return;}
  senior_motion_msgs::msg::EgressHeader header;
  header.relay_gid = index == 0 ? gid_hex(safety_publisher_->get_gid()) :
                                  gid_hex(filter_publisher_->get_gid());
  header.relay_boot = boot_;
  header.relay_sequence = ++owner.relay_sequence;
  header.owner_gid = owner.gid;
  header.owner_epoch = owner.epoch;
  header.owner_generation = owner.generation;
  header.owner_sequence = owner_lost ? 0 : owner.sequence;
  header.owner_lost = owner_lost;
  header.upstream_invalid = upstream_invalid;
  header.reason = reason.substr(0, 64);
  owner.invalid = true;
  if (index == 0) {
    senior_motion_msgs::msg::FencedSafetyScan output;
    output.owner = header;
    if (safety != nullptr) output.body = *safety;
    safety_publisher_->publish(output);
  } else {
    senior_motion_msgs::msg::FencedFilteredScan output;
    output.owner = header;
    if (filtered != nullptr) output.body = *filtered;
    filter_publisher_->publish(output);
  }
}

void ScanReceipt::on_safety(const senior_motion_msgs::msg::BoundedSafetyScan & message,
                            const rclcpp::MessageInfo & info)
{
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  if (before_verify_) before_verify_(7, incoming);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = steady_now();
  if (message.raw_capture.sequence &&
      ledger_.match(message.raw_capture, now) == ScanLedger::Match::retired) return;
  if (!qualify_owner(safety_owner_, incoming, message.source_epoch, message.sequence)) return;
  if (!message.valid) {
    invalidate_owner(safety_owner_, 0, message.reason.empty() ? "scan_invalid" : message.reason,
                     false, false, &message);
    return;
  }
  const auto * source = ledger_.find(message.raw_capture, now);
  const auto & c = message.raw_capture;
  const auto same = [](float a, float b) {return std::memcmp(&a, &b, sizeof(float)) == 0;};
  const bool exact = source != nullptr && ledger_.same_ranges(c, message.ranges, now) &&
    message.stamp == c.stamp && message.frame_id == c.frame_id && message.raw_source == c.raw_gid &&
    message.ingress_steady == c.received_steady &&
    message.ingress_ros == static_cast<double>(rclcpp::Time(c.received_ros).nanoseconds()) / 1e9 &&
    same(message.angle_min, source->angle_min) &&
    same(message.angle_increment, source->angle_increment) &&
    same(message.range_min, source->range_min) && same(message.range_max, source->range_max);
  if (!exact) {
    invalidate_owner(safety_owner_, 0, "scan_body_mismatch", false, false, &message);
    return;
  }
  if (safety_owner_.relay_sequence == std::numeric_limits<uint64_t>::max()) {failed_ = true; return;}
  senior_motion_msgs::msg::FencedSafetyScan output;
  output.body = message;
  auto & h = output.owner;
  h.relay_gid = gid_hex(safety_publisher_->get_gid()); h.relay_boot = boot_;
  h.relay_sequence = ++safety_owner_.relay_sequence;
  h.owner_gid = safety_owner_.gid; h.owner_epoch = safety_owner_.epoch;
  h.owner_generation = safety_owner_.generation; h.owner_sequence = safety_owner_.sequence;
  h.valid = true;
  safety_owner_.invalid = false;
  safety_publisher_->publish(output);
}

void ScanReceipt::on_filtered(const senior_motion_msgs::msg::BoundedFilteredScan & message,
                              const rclcpp::MessageInfo & info)
{
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  if (before_verify_) before_verify_(8, incoming);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = steady_now();
  if (message.raw_capture.sequence &&
      ledger_.match(message.raw_capture, now) == ScanLedger::Match::retired) return;
  if (!qualify_owner(filter_owner_, incoming, message.source_epoch, message.sequence)) return;
  if (!message.valid) {
    invalidate_owner(filter_owner_, 1, message.reason.empty() ? "filter_invalid" : message.reason,
                     false, false, nullptr, &message);
    return;
  }
  const auto * source = ledger_.find(message.raw_capture, now);
  const auto same = [](float a, float b) {return std::memcmp(&a, &b, sizeof(float)) == 0;};
  const bool exact = source != nullptr && ledger_.filtered_ranges(message.raw_capture, message.ranges, now) &&
    same(message.angle_min, source->angle_min) && same(message.angle_max, source->angle_max) &&
    same(message.angle_increment, source->angle_increment) &&
    same(message.time_increment, source->time_increment) && same(message.scan_time, source->scan_time) &&
    same(message.range_min, source->range_min) && same(message.range_max, source->range_max);
  if (!exact) {
    invalidate_owner(filter_owner_, 1, "scan_body_mismatch", false, false, nullptr, &message);
    return;
  }
  if (filter_owner_.relay_sequence == std::numeric_limits<uint64_t>::max()) {failed_ = true; return;}
  senior_motion_msgs::msg::FencedFilteredScan output;
  output.body = message;
  auto & h = output.owner;
  h.relay_gid = gid_hex(filter_publisher_->get_gid()); h.relay_boot = boot_;
  h.relay_sequence = ++filter_owner_.relay_sequence;
  h.owner_gid = filter_owner_.gid; h.owner_epoch = filter_owner_.epoch;
  h.owner_generation = filter_owner_.generation; h.owner_sequence = filter_owner_.sequence;
  h.valid = true;
  filter_owner_.invalid = false;
  filter_publisher_->publish(output);
}
}  // namespace senior_sensor_ingress
