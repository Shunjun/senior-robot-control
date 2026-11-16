#pragma once
#include "geometry_msgs/msg/twist.hpp"
#include "senior_motion_msgs/msg/source_time.hpp"
namespace senior_motion_msgs::msg {
struct ActuatorTarget { SourceTime timing; geometry_msgs::msg::Twist data;
  uint64_t valid_until_steady_ns=0; };
}
