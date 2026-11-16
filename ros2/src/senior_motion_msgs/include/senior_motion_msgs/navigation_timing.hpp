// Copyright 2026 Senior Robot contributors. Licensed under Apache-2.0.
#ifndef SENIOR_MOTION_MSGS__NAVIGATION_TIMING_HPP_
#define SENIOR_MOTION_MSGS__NAVIGATION_TIMING_HPP_

#include <cstdint>
#include <ctime>
#include <fstream>
#include <limits>
#include <mutex>
#include <regex>
#include <stdexcept>
#include <string>

namespace senior_motion_msgs::navigation
{
inline uint64_t monotonic_ns()
{
  timespec value{};
  if (clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::runtime_error("CLOCK_MONOTONIC unavailable");
  }
  return static_cast<uint64_t>(value.tv_sec) * 1000000000ULL + value.tv_nsec;
}

inline std::string boot_domain()
{
  std::ifstream input("/proc/sys/kernel/random/boot_id");
  std::string value;
  input >> value;
  if (!std::regex_match(value, std::regex(
      "[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}"))) {
    throw std::runtime_error("navigation requires same-host Linux monotonic boot domain");
  }
  return value;
}

// Production time is supplied by the computing caller, never replaced at publication.
class Producer
{
public:
  Producer(const std::string & domain, const std::string & run)
  : domain_(domain), run_(run)
  {
    if (!std::regex_match(run_, std::regex("[0-9a-f]{32}"))) {
      throw std::invalid_argument("navigation_run_id must be a private run nonce");
    }
  }

  template<typename Message>
  Message sample(uint64_t produced, double vx, double wz)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (produced == 0 || sequence_ == std::numeric_limits<uint64_t>::max()) {
      throw std::runtime_error("invalid navigation production frontier");
    }
    Message result;
    result.clock_domain = domain_;
    result.run_id = run_;
    result.sequence = ++sequence_;
    result.produced_steady_ns = produced;
    result.vx = vx;
    result.wz = wz;
    return result;
  }

private:
  std::string domain_;
  std::string run_;
  uint64_t sequence_{0};
  std::mutex mutex_;
};
}  // namespace senior_motion_msgs::navigation
#endif  // SENIOR_MOTION_MSGS__NAVIGATION_TIMING_HPP_
