#ifndef SENIOR_SLAM_BRIDGE__SCAN_RECEIPT_ADMISSION_HPP_
#define SENIOR_SLAM_BRIDGE__SCAN_RECEIPT_ADMISSION_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "senior_motion_msgs/msg/bounded_raw_laser_scan.hpp"

namespace senior_slam_bridge::scan_receipt_admission
{
using Receipt = senior_motion_msgs::msg::RawCaptureHeader;
using Message = senior_motion_msgs::msg::BoundedRawLaserScan;

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

inline std::int64_t time_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000) {
    return 0;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}

inline bool fresh(const Receipt & receipt, std::int64_t now_ros, double now_steady)
{
  const auto stamp = time_ns(receipt.stamp);
  const auto received = time_ns(receipt.received_ros);
  return stamp > 0 && received >= stamp && now_ros >= received &&
         now_ros - stamp <= max_age_ns && std::isfinite(receipt.received_steady) &&
         receipt.received_steady > 0 && receipt.received_steady <= now_steady &&
         now_steady - receipt.received_steady <= max_age_seconds;
}

inline double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline bool same_source(const Receipt & lhs, const Receipt & rhs)
{
  return lhs.bridge_gid == rhs.bridge_gid && lhs.raw_gid == rhs.raw_gid &&
         lhs.bridge_boot_id == rhs.bridge_boot_id && lhs.source_epoch == rhs.source_epoch &&
         lhs.capture_generation == rhs.capture_generation && lhs.frame_id == rhs.frame_id;
}

inline bool text_ok(const std::string & value)
{
  return !value.empty() &&
         std::all_of(value.begin(), value.end(), [](unsigned char byte) {
           return byte >= 33 && byte <= 126;
         });
}

inline bool layout_ok(const Message & scan)
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


struct ScanOwner
{
  std::string bridge_gid, raw_gid;

  bool matches(const Receipt & receipt) const
  {
    return bridge_gid == receipt.bridge_gid && raw_gid == receipt.raw_gid;
  }

  bool operator==(const ScanOwner & other) const
  {
    return bridge_gid == other.bridge_gid && raw_gid == other.raw_gid;
  }
};

inline std::optional<ScanOwner> unique_scan_owner(
  rclcpp::Node & node, const std::string & bounded_topic, const std::string & raw_topic)
{
  const auto bridge = node.get_publishers_info_by_topic(bounded_topic);
  const auto raw = node.get_publishers_info_by_topic(raw_topic);
  if (bridge.size() != 1 || raw.size() != 1 ||
    bridge.front().node_name() != "scan_receipt" || bridge.front().node_namespace() != "/" ||
    bridge.front().topic_type() != "senior_motion_msgs/msg/BoundedRawLaserScan" ||
    raw.front().topic_type() != "sensor_msgs/msg/LaserScan")
  {
    return std::nullopt;
  }
  ScanOwner owner{gid_hex(bridge.front().endpoint_gid()), gid_hex(raw.front().endpoint_gid())};
  if (owner.bridge_gid.empty() || owner.raw_gid.empty()) {
    return std::nullopt;
  }
  return owner;
}
}  // namespace senior_slam_bridge::scan_receipt_admission
#endif  // SENIOR_SLAM_BRIDGE__SCAN_RECEIPT_ADMISSION_HPP_
