#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <set>
#include <stdexcept>
#include <string>

#include <senior_motion_msgs/msg/bounded_raw_laser_scan.hpp>
#include <senior_motion_msgs/msg/raw_capture_header.hpp>

namespace senior_sensor_ingress {
// One bounded raw scan source, independent of the image process and its clocks.
class ScanLedger {
public:
  using Scan = senior_motion_msgs::msg::BoundedRawLaserScan;
  using Header = senior_motion_msgs::msg::RawCaptureHeader;
  enum class Match {exact, retired, missing};

  explicit ScanLedger(size_t capacity = 128, double age_limit = 3.0)
  : capacity_(capacity), age_limit_(age_limit)
  {
    if (capacity == 0 || capacity > 128 || !std::isfinite(age_limit) ||
        age_limit <= 0 || age_limit > 5) throw std::invalid_argument("invalid scan ledger limits");
  }

  bool advance(const std::string & boot, uint64_t generation)
  {
    if (closed_ || boot.empty() || boot.size() > 64 || generation == 0 ||
        retired_boots_.count(boot) != 0) return false;
    if (!boot_.empty() && boot != boot_) {
      retired_boots_.insert(boot_);
      if (retired_boots_.size() >= 256) {closed_ = true; return false;}
      boot_ = boot;
      generation_ = 0;
    }
    if (generation < generation_) return false;
    if (boot_.empty()) boot_ = boot;
    if (generation > generation_) {
      values_.clear();
      retired_sequence_ = 0;
      generation_ = generation;
    }
    return true;
  }

  bool observe(const Scan & scan, double now)
  {
    const auto & header = scan.capture;
    if (!header.valid || header.sequence == 0 || scan.ranges.size() > 2048 ||
        scan.intensities.size() > 2048 || !std::isfinite(now) ||
        !std::isfinite(header.received_steady) || now < header.received_steady ||
        now - header.received_steady > age_limit_ ||
        !advance(header.bridge_boot_id, header.capture_generation)) return false;
    expire(now);
    if ((!values_.empty() && header.sequence <= values_.back().capture.sequence) ||
        header.sequence <= retired_sequence_) return false;
    while (values_.size() >= capacity_) retire_front();
    values_.push_back(scan);
    return true;
  }

  Match match(const Header & header, double now)
  {
    if (closed_ || header.sequence == 0 || !std::isfinite(now)) return Match::missing;
    if (retired_boots_.count(header.bridge_boot_id) != 0 ||
        (header.bridge_boot_id == boot_ && header.capture_generation < generation_)) {
      return Match::retired;
    }
    if (header.bridge_boot_id != boot_ || header.capture_generation != generation_) {
      return Match::missing;
    }
    expire(now);
    if (header.sequence <= retired_sequence_) return Match::retired;
    for (const auto & scan : values_) if (scan.capture == header) return Match::exact;
    return Match::missing;
  }

  const Scan * find(const Header & header, double now)
  {
    if (match(header, now) != Match::exact) return nullptr;
    for (const auto & scan : values_) if (scan.capture == header) return &scan;
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
  void retire_front()
  {
    if (values_.front().capture.sequence > retired_sequence_) {
      retired_sequence_ = values_.front().capture.sequence;
    }
    values_.pop_front();
  }

  void expire(double now)
  {
    while (!values_.empty() &&
           (now < values_.front().capture.received_steady ||
            now - values_.front().capture.received_steady > age_limit_)) retire_front();
  }

  size_t capacity_;
  double age_limit_;
  std::deque<Scan> values_;
  std::string boot_;
  std::set<std::string> retired_boots_;
  uint64_t generation_ = 0, retired_sequence_ = 0;
  bool closed_ = false;
};
}  // namespace senior_sensor_ingress
