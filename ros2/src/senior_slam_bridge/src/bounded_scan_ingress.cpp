#include "bounded_scan_ingress.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <utility>

namespace senior_slam_bridge
{
namespace
{
using namespace std::chrono_literals;
using Receipt = BoundedScanIngress::Receipt;
using Message = senior_motion_msgs::msg::BoundedRawLaserScan;
constexpr std::size_t pending_limit = 8;
constexpr double max_age_seconds = 0.5;
constexpr std::int64_t max_age_ns = 500000000;

template<typename Bytes>
std::string gid_hex(const Bytes & bytes)
{
  constexpr char hex[] = "0123456789abcdef";
  if (std::all_of(std::begin(bytes), std::end(bytes), [](auto byte) {return byte == 0;})) {
    return {};
  }
  std::string value;
  for (const auto byte : bytes) {
    value += hex[byte >> 4];
    value += hex[byte & 0x0f];
  }
  return value;
}

std::int64_t time_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000) {
    return 0;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}

bool fresh(const Receipt & receipt, std::int64_t now_ros, double now_steady)
{
  const auto stamp = time_ns(receipt.stamp);
  const auto received = time_ns(receipt.received_ros);
  return stamp > 0 && received >= stamp && now_ros >= received &&
         now_ros - stamp <= max_age_ns && std::isfinite(receipt.received_steady) &&
         receipt.received_steady > 0 && receipt.received_steady <= now_steady &&
         now_steady - receipt.received_steady <= max_age_seconds;
}

double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool same_source(const Receipt & lhs, const Receipt & rhs)
{
  return lhs.bridge_gid == rhs.bridge_gid && lhs.raw_gid == rhs.raw_gid &&
         lhs.bridge_boot_id == rhs.bridge_boot_id && lhs.source_epoch == rhs.source_epoch &&
         lhs.capture_generation == rhs.capture_generation && lhs.frame_id == rhs.frame_id;
}

bool text_ok(const std::string & value)
{
  return !value.empty() &&
         std::all_of(value.begin(), value.end(), [](unsigned char byte) {
           return byte >= 33 && byte <= 126;
         });
}

bool layout_ok(const Message & scan)
{
  // Reject layouts that disagree with the fixed-angle Karto sensor metadata.
  return scan.ranges.size() >= 2 && std::isfinite(scan.angle_min) &&
         std::isfinite(scan.angle_max) && std::isfinite(scan.angle_increment) &&
         scan.angle_increment > 0 && scan.angle_max > scan.angle_min &&
         std::abs((scan.angle_max - scan.angle_min) / scan.angle_increment -
         static_cast<double>(scan.ranges.size() - 1)) <= 0.01 &&
         scan.time_increment == 0 && std::isfinite(scan.scan_time) && scan.scan_time >= 0 &&
         std::isfinite(scan.range_min) && scan.range_min >= 0 &&
         std::isfinite(scan.range_max) && scan.range_max > scan.range_min &&
         std::none_of(scan.ranges.begin(), scan.ranges.end(), [](float range) {
           return std::isnan(range) || range < 0;
         });
}

bool same_geometry(const Message & lhs, const Message & rhs)
{
  return lhs.angle_min == rhs.angle_min && lhs.angle_max == rhs.angle_max &&
         lhs.angle_increment == rhs.angle_increment && lhs.range_min == rhs.range_min &&
         lhs.range_max == rhs.range_max && lhs.ranges.size() == rhs.ranges.size();
}
}  // namespace

BoundedScanIngress::BoundedScanIngress(
  rclcpp::Node & node, tf2_ros::Buffer & tf, std::string bounded_topic,
  std::string raw_topic, std::string odom_frame, std::string base_frame, Submit submit)
: node_(node), tf_(tf), bounded_topic_(std::move(bounded_topic)),
  raw_topic_(std::move(raw_topic)), odom_frame_(std::move(odom_frame)),
  base_frame_(std::move(base_frame)), submit_(std::move(submit))
{
  subscription_ = node_.create_subscription<Message>(
    bounded_topic_, rclcpp::SensorDataQoS().keep_last(pending_limit),
    [this](Message::ConstSharedPtr scan, const rclcpp::MessageInfo & info) {
      offer(std::move(scan), info);
    });
  timer_ = node_.create_wall_timer(20ms, [this] {poll();});
}

bool BoundedScanIngress::observe_clock(std::int64_t now)
{
  if (closed_) {
    return false;
  }
  if (now < last_ros_ || (now == 0 && source_)) {
    close();
    return false;
  }
  last_ros_ = now;
  return now > 0;
}

void BoundedScanIngress::close()
{
  closed_ = true;
  pending_.clear();
}

bool BoundedScanIngress::current_owner(const Receipt & receipt)
{
  const auto bridge = node_.get_publishers_info_by_topic(bounded_topic_);
  const auto raw = node_.get_publishers_info_by_topic(raw_topic_);
  return bridge.size() == 1 && raw.size() == 1 &&
         bridge.front().node_name() == "scan_receipt" &&
         bridge.front().node_namespace() == "/" &&
         bridge.front().topic_type() == "senior_motion_msgs/msg/BoundedRawLaserScan" &&
         raw.front().topic_type() == "sensor_msgs/msg/LaserScan" &&
         gid_hex(bridge.front().endpoint_gid()) == receipt.bridge_gid &&
         gid_hex(raw.front().endpoint_gid()) == receipt.raw_gid;
}

bool BoundedScanIngress::source_active()
{
  if (!observe_clock(node_.now().nanoseconds()) || !source_) {
    return false;
  }
  if (!current_owner(source_->capture)) {
    close();
    return false;
  }
  return true;
}

void BoundedScanIngress::offer(Message::ConstSharedPtr scan, const rclcpp::MessageInfo & info)
{
  const auto now_ros = node_.now().nanoseconds();
  if (!observe_clock(now_ros)) {
    return;
  }
  if (source_ && !current_owner(source_->capture)) {
    close();
    return;
  }
  const auto & receipt = scan->capture;
  const auto actual = gid_hex(info.get_rmw_message_info().publisher_gid.data);
  if (actual.empty() || actual != receipt.bridge_gid) {
    return;
  }
  if (source_) {
    const auto & previous = source_->capture;
    if (actual != previous.bridge_gid) {
      // A new DDS writer can deliver a message before it appears in the graph.
      return;
    }
    if (previous.bridge_boot_id == receipt.bridge_boot_id &&
      (receipt.capture_generation < previous.capture_generation ||
      (receipt.capture_generation == previous.capture_generation &&
      receipt.source_epoch == previous.source_epoch && receipt.sequence <= last_sequence_)))
    {
      return;
    }
    if (!same_source(previous, receipt)) {
      // An authoritative bridge transition cannot silently change an active map source.
      close();
      return;
    }
  }
  if (receipt.sequence <= last_sequence_ || receipt.capture_generation == 0) {
    return;
  }
  if (!receipt.valid) {
    if (source_) {
      close();
    }
    return;
  }
  if (!current_owner(receipt) || !text_ok(receipt.bridge_boot_id) ||
    !text_ok(receipt.source_epoch) || !text_ok(receipt.frame_id) ||
    !fresh(receipt, now_ros, steady_seconds()) || !layout_ok(*scan))
  {
    return;
  }
  const auto stamp = time_ns(receipt.stamp);
  if (source_ && !same_geometry(*source_, *scan)) {
    close();
    return;
  }
  // Sequence is the identity. The stamp is only a TF/temporal admission fence.
  last_sequence_ = receipt.sequence;
  if (stamp <= last_stamp_) {
    return;
  }
  last_stamp_ = stamp;
  if (!source_) {
    source_ = scan;
  }
  if (pending_.size() < pending_limit) {
    pending_.push_back(std::move(scan));
  }
}

void BoundedScanIngress::poll()
{
  if (!source_active()) {
    return;
  }
  const auto now_ros = node_.now().nanoseconds();
  const auto now_steady = steady_seconds();
  while (!pending_.empty() && !fresh(pending_.front()->capture, now_ros, now_steady)) {
    pending_.pop_front();
  }
  if (pending_.empty()) {
    return;
  }
  const auto input = pending_.front();
  const auto & receipt = input->capture;
  const auto stamp = rclcpp::Time(receipt.stamp, node_.get_clock()->get_clock_type());
  if (!tf_.canTransform(odom_frame_, base_frame_, stamp) ||
    !tf_.canTransform(base_frame_, receipt.frame_id, stamp))
  {
    return;
  }
  if (!source_active() ||
    !fresh(receipt, node_.now().nanoseconds(), steady_seconds()))
  {
    return;
  }
  pending_.pop_front();
  auto scan = std::make_shared<Scan>();
  scan->header.stamp = receipt.stamp;
  scan->header.frame_id = receipt.frame_id;
  scan->angle_min = input->angle_min;
  scan->angle_max = input->angle_max;
  scan->angle_increment = input->angle_increment;
  scan->time_increment = input->time_increment;
  scan->scan_time = input->scan_time;
  scan->range_min = input->range_min;
  scan->range_max = input->range_max;
  scan->ranges.assign(input->ranges.begin(), input->ranges.end());
  scan->intensities.assign(input->intensities.begin(), input->intensities.end());
  submit_(std::move(scan), input);
}
}  // namespace senior_slam_bridge
