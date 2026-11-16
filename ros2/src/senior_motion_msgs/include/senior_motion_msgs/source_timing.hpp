// Copyright 2026 Senior Robot contributors. Licensed under Apache-2.0.
#pragma once
#include <atomic>
#include <cstdint>
#include <iomanip>
#include <random>
#include <sstream>
#include <string>
#include <set>
#include "senior_motion_msgs/msg/source_time.hpp"
#include "senior_motion_msgs/navigation_timing.hpp"

namespace senior_motion_msgs::timing
{
inline std::string epoch()
{
  std::random_device random;
  std::ostringstream value;
  for (int i = 0; i < 4; ++i) value << std::hex << std::setw(8) << std::setfill('0') << random();
  return value.str();
}

class DriverClock
{
public:
  DriverClock() : domain_(navigation::boot_domain()), epoch_(epoch()) {}
  msg::SourceTime sample(uint64_t start, uint64_t end, const std::string & basis = "driver_read",
    const std::string & measurement_epoch = "", uint64_t measurement_tick = 0)
  {
    if (start == 0 || end < start) throw std::invalid_argument("invalid source interval");
    if (basis != "driver_read" && basis != "sim_step" && basis != "control_compute")
      throw std::invalid_argument("unsupported source time basis");
    auto previous = sequence_.load();
    do {
      if (previous == std::numeric_limits<uint64_t>::max())
        throw std::overflow_error("source sequence exhausted");
    } while (!sequence_.compare_exchange_weak(previous, previous + 1));
    const auto sequence = previous + 1;
    msg::SourceTime proof;
    proof.clock_domain = domain_;
    proof.source_epoch = epoch_;
    proof.sequence = sequence;
    proof.capture_start_steady_ns = start;
    proof.capture_end_steady_ns = end;
    proof.time_basis = basis;
    if (basis == "sim_step") {
      if (measurement_epoch.empty() || measurement_epoch.size() > 64)
        throw std::invalid_argument("missing simulation measurement epoch");
      proof.measurement_epoch = measurement_epoch;
      proof.measurement_tick_ns = measurement_tick;
    }
    return proof;
  }
private:
  std::string domain_, epoch_;
  std::atomic<uint64_t> sequence_{0};
};

inline bool valid(const msg::SourceTime & p, uint64_t now)
{
  static const auto domain = navigation::boot_domain();
  return p.clock_domain == domain && !p.source_epoch.empty() && p.source_epoch.size() <= 64 &&
    p.measurement_epoch.size() <= 64 && p.sequence > 0 &&
    p.capture_start_steady_ns > 0 && p.capture_end_steady_ns >= p.capture_start_steady_ns &&
    p.capture_end_steady_ns <= now &&
    (p.time_basis != "sim_step" || !p.measurement_epoch.empty()) &&
    (p.time_basis == "driver_read" || p.time_basis == "sim_step" || p.time_basis == "control_compute");
}
inline uint64_t measurement_ns(const msg::SourceTime & p)
{
  return p.time_basis == "sim_step" ? p.measurement_tick_ns : p.capture_end_steady_ns;
}
inline bool same_measurement_axis(const msg::SourceTime & a, const msg::SourceTime & b)
{
  const bool sim = a.time_basis == "sim_step";
  return a.clock_domain == b.clock_domain && sim == (b.time_basis == "sim_step") &&
    (!sim || (!a.measurement_epoch.empty() && a.measurement_epoch == b.measurement_epoch));
}
// The producer identity is not replaced with a receipt-side clock epoch.
class SourceOrder
{
public:
  enum class Result {rejected, current, replaced};
  Result accept(const msg::SourceTime & proof, uint64_t received)
  {
    if (!valid(proof, received) || retired_.count(proof.source_epoch)) return Result::rejected;
    const bool replacement = !epoch_.empty() && epoch_ != proof.source_epoch;
    if (replacement && (retired_.size() >= 256 || proof.capture_end_steady_ns < end_))
      return Result::rejected;
    if (have_proof_ && same_measurement_axis(proof, previous_proof_) &&
        measurement_ns(proof) < measurement_ns(previous_proof_)) return Result::rejected;
    if (proof.source_epoch == epoch_ &&
        (proof.sequence <= sequence_ || proof.capture_end_steady_ns < end_))
      return Result::rejected;
    if (replacement) retired_.insert(epoch_);
    epoch_ = proof.source_epoch;
    sequence_ = proof.sequence;
    end_ = proof.capture_end_steady_ns;
    previous_proof_ = proof; have_proof_ = true;
    return replacement ? Result::replaced : Result::current;
  }
private:
  std::string epoch_;
  std::set<std::string> retired_;
  uint64_t sequence_ = 0, end_ = 0;
  bool have_proof_ = false;
  msg::SourceTime previous_proof_;
};
}  // namespace senior_motion_msgs::timing
