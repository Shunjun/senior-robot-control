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
#include "senior_sensor_ingress/scan_layout.hpp"

namespace senior_sensor_ingress {
namespace {
using namespace std::chrono_literals;

bool text_ok(const std::string & value, size_t maximum)
{
  (void)maximum;  //Compatible: callers retain the old helper signature.
  return !value.empty();
}

bool stamp_ok(const builtin_interfaces::msg::Time & value)
{
  return value.sec >= 0 && value.nanosec < 1000000000u;
}

double steady_now()
{
  return static_cast<double>(senior_sensor_ingress::time::monotonic_ns()) / 1e9;
}

bool layout_ok(const sensor_msgs::msg::LaserScan & scan)
{
  return scan_layout_ok(scan);
}
}  // namespace

ScanReceipt::ScanReceipt(const rclcpp::NodeOptions & options, BeforeVerify before_verify,
                         bool test_reentrant_callbacks)
: Node("scan_receipt", options), before_verify_(std::move(before_verify))
{
  input_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  expected_node_name_ = declare_parameter<std::string>("expected_node_name", "");
  expected_namespace_ = declare_parameter<std::string>("expected_namespace", "");
  const auto output = declare_parameter<std::string>("raw_scan_topic", "/safety/raw/scan");
  if (!text_ok(input_topic_, 128) || !text_ok(output, 128) || input_topic_ == output) {
    throw std::invalid_argument("invalid scan receipt topics");
  }
  safety_owner_.topic = declare_parameter<std::string>("safety_candidate_topic", "/safety/scan_candidate");
  filter_owner_.topic = declare_parameter<std::string>("filter_candidate_topic", "/perception/scan_candidate");
  safety_owner_.expected_name = declare_parameter<std::string>("safety_publisher_name", "");
  safety_owner_.expected_namespace = declare_parameter<std::string>("safety_publisher_namespace", "");
  filter_owner_.expected_name = declare_parameter<std::string>("filter_publisher_name", "");
  filter_owner_.expected_namespace = declare_parameter<std::string>("filter_publisher_namespace", "");
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
  safety_publisher_ = create_publisher<senior_interfaces::msg::FencedSafetyScan>(safety_output, 10);
  filter_publisher_ = create_publisher<senior_interfaces::msg::FencedFilteredScan>(filter_output, qos);
  rclcpp::SubscriptionOptions options_for_scan;
  if (test_reentrant_callbacks) {
    test_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    options_for_scan.callback_group = test_group_;
  }
  safety_subscription_ = create_subscription<senior_interfaces::msg::BoundedSafetyScan>(
    safety_owner_.topic, 10,
    [this](senior_interfaces::msg::BoundedSafetyScan::ConstSharedPtr message,
           const rclcpp::MessageInfo & info) {on_safety(*message, info);}, options_for_scan);
  filter_subscription_ = create_subscription<senior_interfaces::msg::BoundedFilteredScan>(
    filter_owner_.topic, qos,
    [this](senior_interfaces::msg::BoundedFilteredScan::ConstSharedPtr message,
           const rclcpp::MessageInfo & info) {on_filtered(*message, info);}, options_for_scan);
  subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    input_topic_, qos, [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
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

void ScanReceipt::invalidate(const std::string & reason, const Scan * source,
                             int64_t received_ros, uint64_t received_steady_ns,
                             const std::string & raw_gid)
{
  if (failed_ || generation_ == std::numeric_limits<uint64_t>::max()) {failed_ = true; return;}
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
    header.received_steady_ns = received_steady_ns;
  }
  publisher_->publish(output);
}

bool ScanReceipt::clock_ok(int64_t ros_ns, uint64_t steady_ns)
{
  senior_sensor_ingress::time::ClockObservation observed;
  try {
    observed = clock_.observe(ros_ns, steady_ns);
  } catch (const std::invalid_argument & error) {
    invalidate(error.what());
    return false;
  }
  clock_generation_ = observed.generation;
  if (observed.discontinuity) {
    if (generation_ == std::numeric_limits<uint64_t>::max()) {failed_ = true; return false;}
    ++generation_;
    if (!ledger_.advance(boot_, generation_)) {failed_ = true; return false;}
    invalidate("clock_rollback");
    epoch_ = new_epoch();
  }
  return !failed_;
}

std::optional<ReceiptClock> ScanReceipt::begin_receipt()
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto ros = get_clock()->now().nanoseconds();
  const auto steady = senior_sensor_ingress::time::monotonic_ns();
  if (!clock_ok(ros, steady)) return std::nullopt;
  return ReceiptClock{ros, steady, clock_generation_};
}

// Caller holds the owner lock. A changed segment invalidates the old callback;
// its coordinates are never retaken or mapped into the replacement segment.
bool ScanReceipt::current_receipt(const ReceiptClock & receipt)
{
  return clock_ok(get_clock()->now().nanoseconds(), senior_sensor_ingress::time::monotonic_ns()) &&
    receipt.generation == clock_generation_;
}

void ScanReceipt::inspect_topology()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clock_ok(get_clock()->now().nanoseconds(),
                senior_sensor_ingress::time::monotonic_ns())) return;
  inspect_owner(safety_owner_, 0);
  inspect_owner(filter_owner_, 1);
  const auto graph = get_publishers_info_by_topic(input_topic_);
  if (graph.size() != 1) {
    active_ = false;
    if (!invalid_) {invalidate("raw_topology_unavailable"); invalid_ = true;}
  }
}

void ScanReceipt::on_scan(const sensor_msgs::msg::LaserScan & message, const rclcpp::MessageInfo & info)
{
  const auto receipt = begin_receipt();
  if (!receipt) return;
  const int64_t received_ros = receipt->ros_ns;
  const uint64_t received_steady_ns = receipt->steady_ns;
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  const auto actual = gid_hex(incoming);
  if (before_verify_) before_verify_(6, incoming);
  std::lock_guard<std::mutex> lock(mutex_);
  if (!current_receipt(*receipt)) return;
  const auto graph = get_publishers_info_by_topic(input_topic_);
  if (graph.size() != 1 || graph.front().topic_type() != "sensor_msgs/msg/LaserScan" ||
      (!expected_node_name_.empty() && graph.front().node_name() != expected_node_name_) ||
      (!expected_namespace_.empty() && graph.front().node_namespace() != expected_namespace_)) {
    active_ = false;
    if (!invalid_) {invalidate("raw_topology_unavailable", &message, received_ros,
                               received_steady_ns, actual); invalid_ = true;}
    return;
  }
  // Actual MessageInfo identity is diagnostic; graph lookup never replaces it.
  raw_gid_ = actual;
  active_ = true;
  std::optional<senior_sensor_ingress::time::SampleTime> sample;
  try {
    sample = time::map_sample(stamp_ns(message.header.stamp), received_ros,
                              received_steady_ns, receipt->generation);
  } catch (const std::invalid_argument & error) {
    invalidate(error.what(), &message, received_ros, received_steady_ns, actual);
    return;
  }
  if (!sample) return;
  if (received_ros < 0 || !stamp_ok(message.header.stamp) ||
      !text_ok(message.header.frame_id, 128) || !layout_ok(message)) {
    if (!invalid_) {
      invalidate("raw_scan_invalid", &message, received_ros, received_steady_ns, actual);
      invalid_ = true;
    }
    return;
  }
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
  set_sample(header, *sample);
  header.stamp = message.header.stamp;
  header.frame_id = message.header.frame_id;
  header.received_ros = static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(received_ros));
  header.received_steady_ns = received_steady_ns;
  header.raw_gid = actual;
  header.bridge_gid = gid_hex(publisher_->get_gid());
  header.source_epoch = epoch_;
  header.bridge_boot_id = boot_;
  header.capture_generation = generation_;
  header.sequence = ++sequence_;
  header.valid = true;
  invalid_ = false;
  // The history budget controls derived lookups, never raw publication.
  ledger_.observe(output, steady_now());
  if (!current_receipt(*receipt)) return;
  publisher_->publish(output);
}
bool ScanReceipt::qualify_owner(Owner & owner, const rmw_gid_t & incoming,
                                const std::string & epoch, uint64_t sequence)
{
  if (failed_) return false;
  const auto graph = get_publishers_info_by_topic(owner.topic);
  const auto type = &owner == &safety_owner_ ?
    "senior_interfaces/msg/BoundedSafetyScan" : "senior_interfaces/msg/BoundedFilteredScan";
  if (graph.size() != 1 || graph.front().topic_type() != type ||
      (!owner.expected_name.empty() && graph.front().node_name() != owner.expected_name) ||
      (!owner.expected_namespace.empty() && graph.front().node_namespace() != owner.expected_namespace)) {
    invalidate_owner(owner, &owner == &safety_owner_ ? 0 : 1, "owner_unavailable", true);
    owner.active = false;
    return false;
  }
  owner.gid = gid_hex(incoming);
  owner.epoch = epoch;
  owner.sequence = sequence;  // Metadata, not a delivery frontier.
  owner.generation = generation_;
  owner.active = true;
  return true;
}

void ScanReceipt::inspect_owner(Owner & owner, size_t index)
{
  if (!owner.active || failed_) return;
  if (get_publishers_info_by_topic(owner.topic).size() == 1) return;
  invalidate_owner(owner, index, "owner_unavailable", true);
  owner.active = false;
}

void ScanReceipt::invalidate_owner(
  Owner & owner, size_t index, const std::string & reason, bool owner_lost,
  bool upstream_invalid, const senior_interfaces::msg::BoundedSafetyScan * safety,
  const senior_interfaces::msg::BoundedFilteredScan * filtered)
{
  if (failed_ || (owner.invalid && !owner_lost)) return;
  senior_interfaces::msg::EgressHeader header;
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
    senior_interfaces::msg::FencedSafetyScan output;
    output.owner = header;
    if (safety != nullptr) output.body = *safety;
    safety_publisher_->publish(output);
  } else {
    senior_interfaces::msg::FencedFilteredScan output;
    output.owner = header;
    if (filtered != nullptr) output.body = *filtered;
    filter_publisher_->publish(output);
  }
}

void ScanReceipt::on_safety(const senior_interfaces::msg::BoundedSafetyScan & message,
                            const rclcpp::MessageInfo & info)
{
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  if (before_verify_) before_verify_(7, incoming);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = steady_now();
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
    message.stamp == c.stamp && message.frame_id == c.frame_id &&
    message.ingress_steady == steady_seconds(c.received_steady_ns) &&
    message.ingress_ros == static_cast<double>(rclcpp::Time(c.received_ros).nanoseconds()) / 1e9 &&
    same(message.angle_min, source->angle_min) &&
    same(message.angle_increment, source->angle_increment) &&
    same(message.range_min, source->range_min) && same(message.range_max, source->range_max);
  if (!exact) {
    invalidate_owner(safety_owner_, 0, "scan_body_mismatch", false, false, &message);
    return;
  }
  senior_interfaces::msg::FencedSafetyScan output;
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

void ScanReceipt::on_filtered(const senior_interfaces::msg::BoundedFilteredScan & message,
                              const rclcpp::MessageInfo & info)
{
  const auto incoming = info.get_rmw_message_info().publisher_gid;
  if (before_verify_) before_verify_(8, incoming);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = steady_now();
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
  senior_interfaces::msg::FencedFilteredScan output;
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
