#pragma once

#include <cstdint>
#include <ctime>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace senior_motion_msgs::time {
struct SampleTime {
  int64_t stamp_ns, received_ros_ns;
  uint64_t received_steady_ns, sample_steady_ns, clock_generation;
};
struct StampInterval {
  int64_t start_ns, end_ns;
};
struct ClockObservation {
  uint64_t generation;
  bool discontinuity;
};

inline SampleTime map_sample(int64_t stamp_ns, int64_t received_ros_ns,
    uint64_t received_steady_ns, uint64_t clock_generation) {
  if (stamp_ns < 0 || received_ros_ns < 0)
    throw std::invalid_argument("negative ROS coordinate");
  if (stamp_ns > received_ros_ns)
    throw std::invalid_argument("measurement is in the future");
  const auto age = static_cast<uint64_t>(received_ros_ns - stamp_ns);
  if (age > received_steady_ns)
    throw std::invalid_argument("mapping precedes monotonic origin");
  return {stamp_ns, received_ros_ns, received_steady_ns,
    received_steady_ns - age, clock_generation};
}

// Only endpoints already accepted by the same source/window owner belong here.
// Different ingress generations are local and are not a global clock identity.
inline StampInterval interval_ns(const SampleTime &first, const SampleTime &last) {
  if (first.clock_generation != last.clock_generation)
    throw std::invalid_argument("clock generation changed within source interval");
  if (first.stamp_ns < 0 || last.stamp_ns < first.stamp_ns)
    throw std::invalid_argument("reversed or negative ROS stamp interval");
  return {first.stamp_ns, last.stamp_ns};
}

class ClockTracker {
 public:
  ClockObservation observe(int64_t ros_ns, uint64_t steady_ns) {
    if (ros_ns < 0) throw std::invalid_argument("negative ROS coordinate");
    const bool rollback = previous_ &&
      (ros_ns < previous_->first || steady_ns < previous_->second);
    if (rollback) {
      if (generation_ == std::numeric_limits<uint64_t>::max())
        throw std::invalid_argument("clock generation exhausted");
      ++generation_;
    }
    previous_ = std::make_pair(ros_ns, steady_ns);
    return {generation_, rollback};
  }
 private:
  uint64_t generation_ = 1;
  std::optional<std::pair<int64_t, uint64_t>> previous_;
};

inline uint64_t monotonic_ns() {
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0)
    throw std::runtime_error("CLOCK_MONOTONIC unavailable");
  return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL + value.tv_nsec;
}
}  // namespace senior_motion_msgs::time
