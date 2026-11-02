#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <set>
#include <stdexcept>
#include <string>

#include <senior_motion_msgs/msg/raw_capture_header.hpp>

namespace senior_sensor_ingress {
// Bounded first-receipt metadata only; never retains an image payload.
class CaptureLedger {
public:
  using Header = senior_motion_msgs::msg::RawCaptureHeader;
  enum class Match {exact, retired, missing};

  explicit CaptureLedger(size_t capacity = 128, double age_limit = 1.0)
  : capacity_(capacity), age_limit_(age_limit)
  {
    if (capacity == 0 || capacity > 128 || !std::isfinite(age_limit) ||
        age_limit <= 0.0 || age_limit > 1.0) {
      throw std::invalid_argument("invalid capture ledger limits");
    }
  }

  bool advance(const std::string & boot, uint64_t generation)
  {
    if (closed_ || boot.empty() || boot.size() > 64 || generation == 0 ||
        retired_boots_.count(boot) != 0) return false;
    if (!boot_.empty() && boot != boot_) {
      retired_boots_.insert(boot_);
      if (retired_boots_.size() >= 256) {
        closed_ = true;
        return false;
      }
      boot_ = boot;
      generation_ = 0;
    }
    if (generation < generation_) return false;
    if (boot_.empty()) boot_ = boot;
    if (generation > generation_) {
      for (auto & values : entries_) values.clear();
      retired_sequence_.fill(0);
      generation_ = generation;
    }
    return true;
  }

  bool observe(size_t stream, const Header & header, double now_steady)
  {
    if (stream >= entries_.size() || !header.valid || header.sequence == 0 ||
        !std::isfinite(now_steady) || !std::isfinite(header.received_steady) ||
        now_steady < header.received_steady ||
        now_steady - header.received_steady > age_limit_ ||
        !advance(header.bridge_boot_id, header.capture_generation)) return false;
    auto & values = entries_[stream];
    expire(stream, now_steady);
    if ((!values.empty() && header.sequence <= values.back().sequence) ||
        header.sequence <= retired_sequence_[stream]) return false;
    while (values.size() >= capacity_) retire_front(stream);
    values.push_back(header);
    return true;
  }

  Match match(size_t stream, const Header & header, double now_steady)
  {
    if (stream >= entries_.size() || closed_ || header.sequence == 0 ||
        !std::isfinite(now_steady)) {
      return Match::missing;
    }
    if (retired_boots_.count(header.bridge_boot_id) != 0 ||
        (header.bridge_boot_id == boot_ && header.capture_generation < generation_)) {
      return Match::retired;
    }
    if (header.bridge_boot_id != boot_ || header.capture_generation != generation_) {
      return Match::missing;
    }
    expire(stream, now_steady);
    if (header.sequence <= retired_sequence_[stream]) return Match::retired;
    for (const auto & issued : entries_[stream]) {
      if (issued == header) return Match::exact;
    }
    return Match::missing;
  }

private:
  void retire_front(size_t stream)
  {
    auto & values = entries_[stream];
    if (values.front().sequence > retired_sequence_[stream]) {
      retired_sequence_[stream] = values.front().sequence;
    }
    values.pop_front();
  }

  void expire(size_t stream, double now_steady)
  {
    auto & values = entries_[stream];
    while (!values.empty() &&
           (now_steady < values.front().received_steady ||
            now_steady - values.front().received_steady > age_limit_)) {
      retire_front(stream);
    }
  }

  size_t capacity_;
  double age_limit_;
  std::array<std::deque<Header>, 4> entries_;
  std::array<uint64_t, 4> retired_sequence_{};
  std::string boot_;
  uint64_t generation_ = 0;
  std::set<std::string> retired_boots_;
  bool closed_ = false;
};
}  // namespace senior_sensor_ingress
