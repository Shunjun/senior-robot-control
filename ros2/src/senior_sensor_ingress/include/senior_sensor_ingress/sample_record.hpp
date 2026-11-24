#pragma once

#include <optional>
#include <senior_sensor_ingress/sample_time.hpp>
#include <senior_interfaces/msg/raw_capture_header.hpp>

namespace senior_sensor_ingress {
// Receipt coordinates and their serialized owner clock segment travel together.
struct ReceiptClock {
  int64_t ros_ns;
  uint64_t steady_ns, generation;
};

inline double steady_seconds(uint64_t ns) {return static_cast<double>(ns) / 1e9;}

inline int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000u)
    throw std::invalid_argument("invalid ROS stamp");
  return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
}

// A bounded frontier belongs to one real publisher and one local clock segment.
// Check it before mapping so a retransmission never acquires a new receipt.
class StampFrontier {
public:
  std::optional<senior_sensor_ingress::time::SampleTime> accept(
    const builtin_interfaces::msg::Time & stamp, int64_t ros, uint64_t steady,
    uint64_t generation)
  {
    const auto ns = stamp_ns(stamp);
    if (latest_ && ns == *latest_) return std::nullopt;
    if (latest_ && ns < *latest_) throw std::invalid_argument("raw_stamp_out_of_order");
    auto sample = senior_sensor_ingress::time::map_sample(ns, ros, steady, generation);
    latest_ = ns;
    return sample;
  }
  void reset() {latest_.reset();}
private:
  std::optional<int64_t> latest_;
};

inline void set_sample(senior_interfaces::msg::RawCaptureHeader & header,
                       const senior_sensor_ingress::time::SampleTime & sample)
{
  header.received_steady_ns = sample.received_steady_ns;
  header.sample_steady_ns = sample.sample_steady_ns;
  header.clock_generation = sample.clock_generation;
}
}  // namespace senior_sensor_ingress
