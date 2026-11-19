#ifndef SENIOR_SLAM_BRIDGE__SNAPSHOT_HPP_
#define SENIOR_SLAM_BRIDGE__SNAPSHOT_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace senior_slam_bridge
{
struct Keyframe
{
  std::int32_t id;
  double x;
  double y;
  double yaw;
};

enum class SnapshotStatus {ok, empty, unavailable};

inline SnapshotStatus validate_snapshot(std::vector<Keyframe> & frames, std::size_t limit)
{
  if (frames.empty()) {
    return SnapshotStatus::empty;
  }
  const auto invalid = [](const Keyframe & frame) {
      return frame.id < 0 || !std::isfinite(frame.x) ||
             !std::isfinite(frame.y) || !std::isfinite(frame.yaw);
    };
  if (frames.size() > limit || std::any_of(frames.begin(), frames.end(), invalid)) {
    frames.clear();
    return SnapshotStatus::unavailable;
  }

  std::sort(frames.begin(), frames.end(), [](const Keyframe & a, const Keyframe & b) {
    return a.id < b.id;
  });
  const auto duplicate = [](const Keyframe & a, const Keyframe & b) {return a.id == b.id;};
  if (std::adjacent_find(frames.begin(), frames.end(), duplicate) != frames.end()) {
    frames.clear();
    return SnapshotStatus::unavailable;
  }
  return SnapshotStatus::ok;
}
}  // namespace senior_slam_bridge

#endif  // SENIOR_SLAM_BRIDGE__SNAPSHOT_HPP_
