#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>

#include "senior_sensor_ingress/sample_record.hpp"

namespace senior_sensor_ingress {
// Bounded first-receipt metadata only; never retains an image payload.
class CaptureLedger {
public:
  using Header = senior_interfaces::msg::RawCaptureHeader;
  enum class Match {exact, old_generation, missing};

  explicit CaptureLedger(size_t capacity = 128, double age_limit_ms = 1000.0)
  : capacity_(capacity), age_limit_(age_limit_ms / 1000)
  {
    if (capacity == 0 || capacity > 128 || !std::isfinite(age_limit_ms) ||
        age_limit_ms <= 0.0 || age_limit_ms > 1000.0) {
      throw std::invalid_argument("invalid capture ledger limits");
    }
  }

  bool advance(const std::string & boot, uint64_t generation)
  {
    if (boot.empty() || generation == 0) return false;
    // Explicit owner transition; observe() never switches boot on an arriving packet.
    if (boot != boot_) {
      for (auto & values : entries_) values.clear();
      boot_ = boot;
      generation_ = 0;
    }
    if (generation < generation_) return false;
    if (boot_.empty()) boot_ = boot;
    if (generation > generation_) {
      for (auto & values : entries_) values.clear();
      generation_ = generation;
    }
    return true;
  }

  bool observe(size_t stream, const Header & header, double now_steady)
  {
    if (stream >= entries_.size() || !header.valid ||
        !std::isfinite(now_steady) || header.clock_generation == 0 ||
        header.sample_steady_ns > header.received_steady_ns ||
        now_steady < steady_seconds(header.received_steady_ns) ||
        now_steady - steady_seconds(header.sample_steady_ns) > age_limit_ ||
        (!boot_.empty() && header.bridge_boot_id != boot_) ||
        !advance(header.bridge_boot_id, header.capture_generation)) return false;
    auto & values = entries_[stream];
    expire(stream, now_steady);
    while (values.size() >= capacity_) evict_front(stream);
    values.push_back(header);
    return true;
  }

  Match match(size_t stream, const Header & header, double now_steady)
  {
    if (stream >= entries_.size() || !std::isfinite(now_steady)) {
      return Match::missing;
    }
    if (header.bridge_boot_id == boot_ && header.capture_generation < generation_) {
      return Match::old_generation;
    }
    if (header.bridge_boot_id != boot_ || header.capture_generation != generation_) {
      return Match::missing;
    }
    expire(stream, now_steady);
    for (const auto & issued : entries_[stream]) {
      if (same_capture_coordinates(issued, header)) return Match::exact;
    }
    return Match::missing;
  }

private:
  void evict_front(size_t stream)
  {
    auto & values = entries_[stream];
    values.pop_front();
  }

  void expire(size_t stream, double now_steady)
  {
    auto & values = entries_[stream];
    while (!values.empty() &&
           (now_steady < steady_seconds(values.front().received_steady_ns) ||
            now_steady - steady_seconds(values.front().sample_steady_ns) > age_limit_)) {
      evict_front(stream);
    }
  }

  size_t capacity_;
  double age_limit_;
  std::array<std::deque<Header>, 4> entries_;
  std::string boot_;
  uint64_t generation_ = 0;
};
}  // namespace senior_sensor_ingress
