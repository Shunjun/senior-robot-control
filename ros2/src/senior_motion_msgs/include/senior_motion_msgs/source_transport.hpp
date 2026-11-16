// Copyright 2026 Senior Robot contributors. Licensed under Apache-2.0.
#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <string>

namespace senior_motion_msgs::timing
{
// Gazebo Header data travels in the same serialized payload, before transport queues.
inline constexpr const char * source_time_key = "senior_source_time_v2";

template<typename Header, typename Proof>
void attach(Header * header, const Proof & proof)
{
  decltype(header->add_data()) field = nullptr;
  for (int i = 0; i < header->data_size(); ++i) {
    if (header->data(i).key() == source_time_key) {
      field = header->mutable_data(i);
      field->clear_value();
      break;
    }
  }
  if (!field) field = header->add_data();
  field->set_key(source_time_key);
  for (const auto & value : std::array<std::string, 8>{
      proof.clock_domain, proof.source_epoch, std::to_string(proof.sequence),
      std::to_string(proof.capture_start_steady_ns), std::to_string(proof.capture_end_steady_ns),
      proof.time_basis, proof.measurement_epoch, std::to_string(proof.measurement_tick_ns)}) field->add_value(value);
}

inline bool decimal(const std::string & value, uint64_t & result)
{
  if (value.empty() || value.size() > 20 ||
      !std::all_of(value.begin(), value.end(), [](char c) {return c >= '0' && c <= '9';}))
    return false;
  try { result = std::stoull(value); return true; } catch (const std::exception &) { return false; }
}

template<typename Header, typename Proof>
bool extract(const Header & header, Proof & proof)
{
  bool seen = false;
  Proof original{};
  for (int i = 0; i < header.data_size(); ++i) {
    const auto & field = header.data(i);
    if (field.key() != source_time_key) continue;
    if (seen || field.value_size() != 8) return false;
    seen = true;
    original.clock_domain = field.value(0);
    original.source_epoch = field.value(1);
    original.time_basis = field.value(5);
    original.measurement_epoch = field.value(6);
    if (original.clock_domain.empty() || original.clock_domain.size() > 36 ||
        original.source_epoch.empty() || original.source_epoch.size() > 64 ||
        original.time_basis.empty() || original.time_basis.size() > 16 ||
        !decimal(field.value(2), original.sequence) ||
        !decimal(field.value(3), original.capture_start_steady_ns) ||
        !decimal(field.value(4), original.capture_end_steady_ns) ||
        !decimal(field.value(7), original.measurement_tick_ns) ||
        original.measurement_epoch.size() > 64 ||
        (original.time_basis == "sim_step" && original.measurement_epoch.empty()) ||
        original.sequence == 0 || original.capture_start_steady_ns == 0 ||
        original.capture_end_steady_ns < original.capture_start_steady_ns) return false;
  }
  if (!seen) return false;
  proof = original;
  return true;
}
}  // namespace senior_motion_msgs::timing
