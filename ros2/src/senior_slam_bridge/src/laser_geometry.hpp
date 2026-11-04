#ifndef SENIOR_SLAM_BRIDGE__LASER_GEOMETRY_HPP_
#define SENIOR_SLAM_BRIDGE__LASER_GEOMETRY_HPP_

#include <cmath>
#include <cstddef>

namespace senior_slam_bridge
{
// Translation and quaternion of base <- laser, looked up at the original scan stamp.
struct LaserTransform
{
  double x, y, z;
  double qx, qy, qz, qw;
};

struct NativeLaserOffset
{
  double x, y, yaw;
};

struct LaserRayLayout
{
  double angle_min, angle_increment;
  std::size_t ray_count;
  bool inverted;
};

// Numeric agreement with the cached planar Karto mount, not calibration or TF authority.
inline bool matches_native_laser_geometry(
  LaserTransform transform, NativeLaserOffset offset, LaserRayLayout rays)
{
  constexpr double tolerance = 1e-5;
  constexpr double full_turn = 6.28318530717958647692;
  if (!std::isfinite(transform.x) || !std::isfinite(transform.y) || !std::isfinite(transform.z) ||
    !std::isfinite(offset.x) || !std::isfinite(offset.y) || !std::isfinite(offset.yaw) ||
    std::hypot(transform.x - offset.x, transform.y - offset.y) > tolerance ||
    !std::isfinite(rays.angle_min) || !std::isfinite(rays.angle_increment) ||
    rays.angle_increment <= 0 || rays.ray_count < 2 || rays.ray_count > 2048)
  {
    return false;
  }
  const double span = static_cast<double>(rays.ray_count - 1) * rays.angle_increment;
  if (span > full_turn + tolerance) {
    return false;
  }
  if (rays.inverted) {
    //Compatible: Humble reverses the ranges but preserves angle_min and angle_increment.
    // Reversal maps a raw bearing theta to (first + last) - theta.
    const double phase = 2 * rays.angle_min + span;
    if (!std::isfinite(phase) || std::abs(std::remainder(phase, full_turn)) > tolerance) {
      return false;
    }
  }

  const double norm_squared = transform.qx * transform.qx + transform.qy * transform.qy +
    transform.qz * transform.qz + transform.qw * transform.qw;
  if (!std::isfinite(norm_squared) || std::abs(norm_squared - 1) > tolerance) {
    return false;
  }
  const double norm = std::sqrt(norm_squared);
  const double x = transform.qx / norm, y = transform.qy / norm;
  const double z = transform.qz / norm, w = transform.qw / norm;
  const double cos_yaw = std::cos(offset.yaw), sin_yaw = std::sin(offset.yaw);
  const double handedness = rays.inverted ? -1.0 : 1.0;
  // Compare both transformed unit rays against the native yaw and reversal.
  return std::abs(1 - 2 * (y * y + z * z) - cos_yaw) <= tolerance &&
         std::abs(2 * (x * y + z * w) - sin_yaw) <= tolerance &&
         std::abs(2 * (x * z - y * w)) <= tolerance &&
         std::abs(2 * (x * y - z * w) + handedness * sin_yaw) <= tolerance &&
         std::abs(1 - 2 * (x * x + z * z) - handedness * cos_yaw) <= tolerance &&
         std::abs(2 * (y * z + x * w)) <= tolerance;
}
}  // namespace senior_slam_bridge

#endif  // SENIOR_SLAM_BRIDGE__LASER_GEOMETRY_HPP_
