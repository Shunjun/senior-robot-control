// Copyright 2026 Senior Robot contributors. Licensed under Apache-2.0.
#include "senior_motion_msgs/simulation_timing.hpp"
#include <iomanip>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>

namespace senior_motion_msgs::timing
{
namespace
{
struct WorldClock
{
  std::string run;
  uint64_t tick = 0, generation = 0;
  bool seen = false;
  WorldClock()
  {
    std::random_device random;
    std::ostringstream value;
    for (int i = 0; i < 4; ++i)
      value << std::hex << std::setw(8) << std::setfill('0') << random();
    run = value.str();
  }
};
std::mutex mutex;
std::map<std::string, WorldClock> worlds;
std::map<std::pair<std::uintptr_t, uint64_t>, uint64_t> worldKeys;
}
uint64_t simulation_world_key(const void * ecm, uint64_t entity)
{
  if (ecm == nullptr) throw std::invalid_argument("missing simulation ECM identity");
  std::lock_guard<std::mutex> lock(mutex);
  const auto identity = std::make_pair(reinterpret_cast<std::uintptr_t>(ecm), entity);
  const auto found = worldKeys.find(identity);
  if (found != worldKeys.end()) return found->second;
  // The ros2_control state-interface scalar carries this integer exactly.
  const auto key = static_cast<uint64_t>(worldKeys.size()) + 1;
  if (key > 9007199254740991ULL) throw std::overflow_error("world key exhausted");
  worldKeys.emplace(identity, key);
  return key;
}
std::string simulation_epoch(const std::string & world, uint64_t physical_tick_ns)
{
  if (world.empty()) throw std::invalid_argument("missing simulation world identity");
  std::lock_guard<std::mutex> lock(mutex);
  auto & clock = worlds[world];
  if (clock.seen && physical_tick_ns < clock.tick) ++clock.generation;
  clock.tick = physical_tick_ns;
  clock.seen = true;
  return clock.run + ":" + std::to_string(clock.generation);
}
void reset_simulation_epoch(const std::string & world)
{
  if (world.empty()) throw std::invalid_argument("missing simulation world identity");
  std::lock_guard<std::mutex> lock(mutex);
  auto & clock = worlds[world];
  ++clock.generation;
  // The executed reset already advances the epoch. Its first zero tick must
  // not be interpreted as a second reset by any producer plugin.
  clock.seen = false;
}

}
