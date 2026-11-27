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
#include <senior_sensor_ingress/sample_time.hpp>
#include <senior_sensor_ingress/scan_layout.hpp>
#include "senior_interfaces/msg/bounded_raw_laser_scan.hpp"

namespace senior_mapping::scan_receipt_admission
{
using Receipt = senior_interfaces::msg::RawCaptureHeader;
using Message = senior_interfaces::msg::BoundedRawLaserScan;

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
    return -1;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}

inline bool sample_valid(const Receipt & receipt)
{
  return receipt.clock_generation > 0 && time_ns(receipt.stamp) >= 0 &&
    time_ns(receipt.received_ros) >= time_ns(receipt.stamp) &&
    receipt.sample_steady_ns <= receipt.received_steady_ns;
}

inline double steady_seconds()
{
  return static_cast<double>(senior_sensor_ingress::time::monotonic_ns()) / 1e9;
}

inline bool same_source(const Receipt & lhs, const Receipt & rhs)
{
  return lhs.bridge_boot_id == rhs.bridge_boot_id &&
         lhs.capture_generation == rhs.capture_generation;
}

inline bool text_ok(const std::string & value)
{
  return !value.empty();
}

inline bool layout_ok(const Message & scan)
{
  return senior_sensor_ingress::scan_layout_ok(scan);
}

}  // namespace senior_mapping::scan_receipt_admission
#endif  // SENIOR_SLAM_BRIDGE__SCAN_RECEIPT_ADMISSION_HPP_
