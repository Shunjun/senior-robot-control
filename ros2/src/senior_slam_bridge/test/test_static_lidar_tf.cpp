#include "static_lidar_tf.hpp"

#include <cassert>
#include <cmath>
#include <vector>

using senior_slam_bridge::LaserTransform;
using senior_slam_bridge::StaticLidarEdge;
using senior_slam_bridge::compose_static_lidar_chain;
using senior_slam_bridge::matches_static_lidar_mount;
using senior_slam_bridge::replaces_pending_static_lidar;

int main()
{
  assert(!replaces_pending_static_lidar({3, 1, true, true}, {2, 9, true, true}));
  assert(!replaces_pending_static_lidar({1, 1, false, false}, {1, 1, true, true}));
  assert(replaces_pending_static_lidar({1, 3, true, true}, {1, 2, false, false}));
  assert(replaces_pending_static_lidar({1, 1, false, false}, {2, 1, true, true}));
  assert(replaces_pending_static_lidar({2, 1, true, false}, {2, 1, true, true}));
  assert(!replaces_pending_static_lidar({2, 1, true, true}, {2, 1, true, false}));
  assert(!replaces_pending_static_lidar({2, 1, true, true}, {2, 1, true, true}));
  const auto half = std::sqrt(0.5);
  const LaserTransform identity{0, 0, 0, 0, 0, 0, 1};
  const LaserTransform turn{1, 0, 0.5, 0, 0, half, half};
  const LaserTransform offset{2, 0, 0.25, 0, 0, 0, 1};
  std::vector<StaticLidarEdge> edges{{"base", "mount", turn}, {"mount", "lidar", offset}};
  const auto result = compose_static_lidar_chain(edges, "base", "lidar");
  assert(result);
  assert(matches_static_lidar_mount(*result, {1, 2, 0.75, 0, 0, -half, -half}));
  assert(!matches_static_lidar_mount(*result, {1, 2, 0.8, 0, 0, half, half}));
  assert(!matches_static_lidar_mount(*result, {1, 2, 0.75, 0, 0, 0, 1}));
  assert(!compose_static_lidar_chain(edges, "base", "missing"));
  edges.push_back({"other", "lidar", identity});
  assert(!compose_static_lidar_chain(edges, "base", "lidar"));
  edges.back() = {"one", "two", identity};
  edges.push_back({"two", "one", identity});
  assert(!compose_static_lidar_chain(edges, "base", "lidar"));
  edges.pop_back();
  edges.back().mount.qw = 0;
  assert(!compose_static_lidar_chain(edges, "base", "lidar"));
}
