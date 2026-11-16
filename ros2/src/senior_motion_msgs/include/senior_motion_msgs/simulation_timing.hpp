// Copyright 2026 Senior Robot contributors. Licensed under Apache-2.0.
#pragma once
#include <cstdint>
#include <string>

namespace senior_motion_msgs::timing
{
// Entity numbers are local to an ECM, so independent worlds can both be #1.
// This process-local key is shared by core/producer calls for the ECM lifetime.
__attribute__((visibility("default")))
uint64_t simulation_world_key(const void * ecm, uint64_t entity);

// Called on the synchronous physics input/read thread, before processing queues.
// The shared library owns one world/reset coordinate across producer plugins.
// A producer's DriverClock continues to own its independent incarnation/sequence.
__attribute__((visibility("default")))
std::string simulation_epoch(const std::string & world, uint64_t physical_tick_ns);
// Called only by the engine after it has entered the executed rewind branch.
__attribute__((visibility("default")))
void reset_simulation_epoch(const std::string & world);
}
