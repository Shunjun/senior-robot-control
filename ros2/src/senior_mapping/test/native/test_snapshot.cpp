#include "snapshot.hpp"

#include <limits>
#include <stdexcept>

using senior_slam_bridge::Keyframe;
using senior_slam_bridge::SnapshotStatus;
using senior_slam_bridge::validate_snapshot;

int main()
{
  std::vector<Keyframe> frames;
  if (validate_snapshot(frames, 2) != SnapshotStatus::empty) {
    throw std::runtime_error("Empty snapshot");
  }

  frames = {{4, 2.0, 3.0, -0.7}, {0, -2.0, 1.0, 0.3}};
  if (validate_snapshot(frames, 2) != SnapshotStatus::ok ||
    frames.size() != 2 || frames[0].id != 0 || frames[0].yaw != 0.3 ||
    frames[1].id != 4 || frames[1].yaw != -0.7)
  {
    throw std::runtime_error("Complete sorted snapshot at the bound");
  }

  const auto rejected = [](std::vector<Keyframe> input, std::size_t limit) {
      return validate_snapshot(input, limit) == SnapshotStatus::unavailable && input.empty();
    };
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  const auto inf = std::numeric_limits<double>::infinity();
  if (!rejected(frames, 1) ||
    !rejected({{4, 0.0, 0.0, 0.0}, {4, 1.0, 1.0, 1.0}}, 2) ||
    !rejected({{-1, 0.0, 0.0, 0.0}}, 2) ||
    !rejected({{4, nan, 0.0, 0.0}}, 2) ||
    !rejected({{4, 0.0, inf, 0.0}}, 2) ||
    !rejected({{4, 0.0, 0.0, nan}}, 2))
  {
    throw std::runtime_error("Reject the entire invalid or over-limit snapshot");
  }
}
