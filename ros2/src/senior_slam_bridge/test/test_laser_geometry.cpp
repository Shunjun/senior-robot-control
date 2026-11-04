#include "laser_geometry.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
using senior_slam_bridge::LaserRayLayout;
using senior_slam_bridge::LaserTransform;
using senior_slam_bridge::matches_native_laser_geometry;
using senior_slam_bridge::NativeLaserOffset;

constexpr double pi = 3.14159265358979323846;
constexpr double yaw = 0.7;
const NativeLaserOffset offset{0.23, -0.17, yaw};
const LaserRayLayout symmetric{-pi / 2, pi / 180, 181, false};
const LaserTransform upright{
  offset.x, offset.y, 0.42, 0, 0, std::sin(yaw / 2), std::cos(yaw / 2)};

void require(bool condition, const char * reason)
{
  if (!condition) {
    throw std::runtime_error(reason);
  }
}

void checks_cached_mount_against_the_exact_tf_sample()
{
  require(matches_native_laser_geometry(upright, offset, symmetric), "nonzero planar mount");

  auto shifted = upright;
  shifted.x += 0.01;
  require(!matches_native_laser_geometry(shifted, offset, symmetric), "translation drift");

  auto turned = upright;
  turned.qz = std::sin((yaw + 0.01) / 2);
  turned.qw = std::cos((yaw + 0.01) / 2);
  require(!matches_native_laser_geometry(turned, offset, symmetric), "yaw drift");

  auto tilted = upright;
  const double roll = 0.01;
  tilted.qx = upright.qw * std::sin(roll / 2);
  tilted.qy = upright.qz * std::sin(roll / 2);
  tilted.qz *= std::cos(roll / 2);
  tilted.qw *= std::cos(roll / 2);
  require(!matches_native_laser_geometry(tilted, offset, symmetric), "tilt drift");

  const NativeLaserOffset wrapped{offset.x, offset.y, offset.yaw - 2 * pi};
  require(matches_native_laser_geometry(upright, wrapped, symmetric), "periodic yaw equality");
}

void checks_the_effect_of_the_cached_reversal()
{
  const LaserTransform inverted{
    offset.x, offset.y, upright.z, std::cos(yaw / 2), std::sin(yaw / 2), 0, 0};
  auto rays = symmetric;
  rays.inverted = true;
  require(matches_native_laser_geometry(inverted, offset, rays), "inverted symmetric layout");
  require(!matches_native_laser_geometry(upright, offset, rays), "incorrect cached reversal");
  require(!matches_native_laser_geometry(inverted, offset, symmetric), "missing reversal");

  const LaserRayLayout full_circle{0, pi / 180, 361, true};
  require(matches_native_laser_geometry(inverted, offset, full_circle), "periodic full circle");

  rays.angle_min = 0;
  rays.ray_count = 91;
  require(!matches_native_laser_geometry(inverted, offset, rays), "asymmetric inverted layout");
}

void rejects_invalid_numeric_geometry()
{
  auto invalid = upright;
  invalid.qw *= 0.99;
  require(!matches_native_laser_geometry(invalid, offset, symmetric), "nonunit quaternion");
  invalid = upright;
  invalid.z = std::numeric_limits<double>::quiet_NaN();
  require(!matches_native_laser_geometry(invalid, offset, symmetric), "invalid mount height");
  auto rays = symmetric;
  rays.angle_increment = std::numeric_limits<double>::infinity();
  require(!matches_native_laser_geometry(upright, offset, rays), "invalid angle increment");
  rays = symmetric;
  rays.ray_count = 2049;
  require(!matches_native_laser_geometry(upright, offset, rays), "unbounded layout");
}
}  // namespace

int main()
{
  checks_cached_mount_against_the_exact_tf_sample();
  checks_the_effect_of_the_cached_reversal();
  rejects_invalid_numeric_geometry();
}
