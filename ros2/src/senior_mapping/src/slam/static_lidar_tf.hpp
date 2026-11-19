#ifndef SENIOR_SLAM_BRIDGE__STATIC_LIDAR_TF_HPP_
#define SENIOR_SLAM_BRIDGE__STATIC_LIDAR_TF_HPP_

#include "laser_geometry.hpp"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace senior_slam_bridge
{
struct StaticLidarEdge
{
  std::string parent, child;
  LaserTransform mount;
};

struct StaticLidarCursor
{
  std::uint64_t generation, sequence;
  bool valid, admitted;
};

// Ordering for one writer before any TF lifetime has been admitted.
inline bool replaces_pending_static_lidar(StaticLidarCursor pending, StaticLidarCursor incoming)
{
  if (incoming.generation != pending.generation) {
    return incoming.generation > pending.generation;
  }
  if (incoming.valid != pending.valid) {
    return !incoming.valid;
  }
  if (incoming.sequence != pending.sequence) {
    return incoming.sequence > pending.sequence;
  }
  return incoming.valid && incoming.admitted && !pending.admitted;
}

inline bool valid_static_lidar_transform(LaserTransform pose)
{
  const double squared_norm = pose.qx * pose.qx + pose.qy * pose.qy +
    pose.qz * pose.qz + pose.qw * pose.qw;
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
         std::abs(squared_norm - 1) <= 1e-6;
}

inline LaserTransform compose_static_lidar_mounts(LaserTransform parent, LaserTransform child)
{
  // Rotate the child translation by the parent quaternion, then translate.
  const auto norm = std::sqrt(parent.qx * parent.qx + parent.qy * parent.qy +
    parent.qz * parent.qz + parent.qw * parent.qw);
  const double x = parent.qx / norm, y = parent.qy / norm;
  const double z = parent.qz / norm, w = parent.qw / norm;
  const double cx = 2 * (y * child.z - z * child.y);
  const double cy = 2 * (z * child.x - x * child.z);
  const double cz = 2 * (x * child.y - y * child.x);
  return {
    parent.x + child.x + w * cx + y * cz - z * cy,
    parent.y + child.y + w * cy + z * cx - x * cz,
    parent.z + child.z + w * cz + x * cy - y * cx,
    w * child.qx + x * child.qw + y * child.qz - z * child.qy,
    w * child.qy - x * child.qz + y * child.qw + z * child.qx,
    w * child.qz + x * child.qy - y * child.qx + z * child.qw,
    w * child.qw - x * child.qx - y * child.qy - z * child.qz,
  };
}

inline std::optional<LaserTransform> compose_static_lidar_chain(
  const std::vector<StaticLidarEdge> & edges, const std::string & base, const std::string & lidar)
{
  const auto frame_ok = [](const std::string & frame) {
      return !frame.empty() && frame.size() <= 128 && frame.front() != '/' &&
             std::all_of(frame.begin(), frame.end(), [](unsigned char byte) {
               return byte >= 33 && byte <= 126;
             });
    };
  if (edges.empty() || edges.size() > 32 || base == lidar || !frame_ok(base) || !frame_ok(lidar)) {
    return std::nullopt;
  }
  const auto edge_for = [&](const std::string & child) {
      return std::find_if(edges.begin(), edges.end(), [&](const auto & edge) {
        return edge.child == child;
      });
    };
  for (auto edge = edges.begin(); edge != edges.end(); ++edge) {
    if (!frame_ok(edge->parent) || !frame_ok(edge->child) || edge_for(edge->child) != edge ||
      !valid_static_lidar_transform(edge->mount))
    {
      return std::nullopt;
    }
    auto ancestor = edge;
    for (std::size_t steps = 0; ancestor != edges.end(); ++steps) {
      if (steps >= edges.size()) {
        return std::nullopt;
      }
      ancestor = edge_for(ancestor->parent);
    }
  }

  LaserTransform result{0, 0, 0, 0, 0, 0, 1};
  for (auto edge = edge_for(lidar); edge != edges.end(); edge = edge_for(edge->parent)) {
    result = compose_static_lidar_mounts(edge->mount, result);
    if (edge->parent == base && valid_static_lidar_transform(result)) {
      return result;
    }
  }
  return std::nullopt;
}

inline bool matches_static_lidar_mount(LaserTransform expected, LaserTransform actual)
{
  if (!valid_static_lidar_transform(expected) || !valid_static_lidar_transform(actual)) {
    return false;
  }
  constexpr double tolerance = 1e-5;
  const double dot = expected.qx * actual.qx + expected.qy * actual.qy +
    expected.qz * actual.qz + expected.qw * actual.qw;
  const double sign = dot < 0 ? -1 : 1;
  return std::hypot(expected.x - actual.x, expected.y - actual.y, expected.z - actual.z) <=
         tolerance &&
         std::hypot(
    std::hypot(expected.qx - sign * actual.qx, expected.qy - sign * actual.qy),
    std::hypot(expected.qz - sign * actual.qz, expected.qw - sign * actual.qw)) <= tolerance;
}
}  // namespace senior_slam_bridge

#endif  // SENIOR_SLAM_BRIDGE__STATIC_LIDAR_TF_HPP_
