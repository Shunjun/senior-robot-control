#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <stdexcept>
#include <string>

#include <senior_interfaces/msg/bounded_raw_laser_scan.hpp>
#include "senior_sensor_ingress/sample_record.hpp"

namespace senior_sensor_ingress {
// One bounded raw scan source, independent of the image process and its clocks.
class ScanLedger {
public:
  using Scan = senior_interfaces::msg::BoundedRawLaserScan;
  using Header = senior_interfaces::msg::RawCaptureHeader;
  enum class Match {exact, old_generation, missing};

  explicit ScanLedger(size_t capacity = 128, double age_limit = 3.0)
  : capacity_(capacity), age_limit_(age_limit)
  {
    if (capacity == 0 || capacity > 128 || !std::isfinite(age_limit) ||
        age_limit <= 0 || age_limit > 5) throw std::invalid_argument("invalid scan ledger limits");
  }

  bool advance(const std::string & boot, uint64_t generation)
  {
    if (boot.empty() || generation == 0) return false;
    // Explicit owner transition; observe() never switches boot on an arriving packet.
    if (boot != boot_) {
      values_.clear();
      boot_ = boot;
      generation_ = 0;
    }
    if (generation < generation_) return false;
    if (boot_.empty()) boot_ = boot;
    if (generation > generation_) {
      values_.clear();
      generation_ = generation;
    }
    return true;
  }

  bool observe(const Scan & scan, double now)
  {
    const auto & header = scan.capture;
    if (!header.valid || !std::isfinite(now) ||
        header.clock_generation == 0 || header.sample_steady_ns > header.received_steady_ns ||
        now < steady_seconds(header.received_steady_ns) ||
        now - steady_seconds(header.sample_steady_ns) > age_limit_ ||
        (!boot_.empty() && header.bridge_boot_id != boot_) ||
        !advance(header.bridge_boot_id, header.capture_generation)) return false;
    expire(now);
    while (values_.size() >= capacity_) evict_front();
    values_.push_back(scan);
    return true;
  }

  Match match(const Header & header, double now)
  {
    if (!std::isfinite(now)) return Match::missing;
    if (header.bridge_boot_id == boot_ && header.capture_generation < generation_) {
      return Match::old_generation;
    }
    if (header.bridge_boot_id != boot_ || header.capture_generation != generation_) {
      return Match::missing;
    }
    expire(now);
    for (const auto & scan : values_) if (same_capture_coordinates(scan.capture, header)) return Match::exact;
    return Match::missing;
  }

  const Scan * find(const Header & header, double now)
  {
    if (match(header, now) != Match::exact) return nullptr;
    for (const auto & scan : values_) if (same_capture_coordinates(scan.capture, header)) return &scan;
    return nullptr;
  }

  template<typename Ranges>
  bool same_ranges(const Header & header, const Ranges & ranges, double now)
  {
    const auto * source = find(header, now);
    if (source == nullptr || source->ranges.size() != ranges.size()) return false;
    for (size_t i = 0; i < ranges.size(); ++i) {
      if (std::memcmp(&source->ranges[i], &ranges[i], sizeof(float)) != 0) return false;
    }
    return true;
  }

  template<typename Ranges>
  bool filtered_ranges(const Header & header, const Ranges & ranges, double now)
  {
    const auto * source = find(header, now);
    if (source == nullptr || source->ranges.size() != ranges.size()) return false;
    for (size_t i = 0; i < ranges.size(); ++i) {
      if (!std::isnan(ranges[i]) &&
          std::memcmp(&source->ranges[i], &ranges[i], sizeof(float)) != 0) return false;
    }
    return true;
  }

private:
  void evict_front()
  {
    values_.pop_front();
  }

  void expire(double now)
  {
    while (!values_.empty() &&
           (now < steady_seconds(values_.front().capture.received_steady_ns) ||
            now - steady_seconds(values_.front().capture.sample_steady_ns) > age_limit_)) evict_front();
  }

  size_t capacity_;
  double age_limit_;
  std::deque<Scan> values_;
  std::string boot_;
  uint64_t generation_ = 0;
};
}  // namespace senior_sensor_ingress
