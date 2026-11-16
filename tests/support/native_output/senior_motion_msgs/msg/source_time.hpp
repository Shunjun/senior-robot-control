#pragma once
#include <cstdint>
#include <string>
namespace senior_motion_msgs::msg {
struct SourceTime {
  std::string clock_domain, source_epoch, time_basis, measurement_epoch;
  uint64_t sequence=0, capture_start_steady_ns=0, capture_end_steady_ns=0, measurement_tick_ns=0;
};
}
