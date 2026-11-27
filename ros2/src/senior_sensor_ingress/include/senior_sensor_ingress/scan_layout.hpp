#pragma once

#include <cmath>

namespace senior_sensor_ingress {
// Validate scan-wide metadata only. Individual returns belong to the algorithm.
template<typename Scan>
bool scan_layout_ok(const Scan & scan)
{
  if (scan.ranges.empty() || !std::isfinite(scan.angle_min) ||
      !std::isfinite(scan.angle_max) || !std::isfinite(scan.angle_increment) ||
      scan.angle_increment <= 0 || scan.angle_max < scan.angle_min ||
      !std::isfinite(scan.scan_time) || scan.scan_time < 0 ||
      !std::isfinite(scan.range_min) || scan.range_min < 0 ||
      !std::isfinite(scan.range_max) || scan.range_max <= scan.range_min) {
    return false;
  }
  const double intervals =
    (static_cast<double>(scan.angle_max) - scan.angle_min) / scan.angle_increment;
  const double count = static_cast<double>(scan.ranges.size());
  //Compatible: ROS drivers publish both inclusive and exclusive end angles.
  // WHEELTEC X10 uses an exclusive end; no metadata or ray is rewritten here.
  constexpr double rounding_tolerance = 0.01;
  return std::abs(intervals - (count - 1)) <= rounding_tolerance ||
         std::abs(intervals - count) <= rounding_tolerance;
}
}  // namespace senior_sensor_ingress
