// Copyright 2026 Senior Robot contributors. Licensed under Apache-2.0.
#pragma once
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "senior_motion_msgs/msg/actuator_target.hpp"
#include "senior_motion_msgs/source_timing.hpp"

namespace senior_motion_msgs::timing
{
inline bool nonzero(const geometry_msgs::msg::Twist & value)
{
  return value.linear.x != 0 || value.angular.z != 0;
}

inline bool current(const msg::ActuatorTarget & target, uint64_t now)
{
  const auto & v = target.data;
  return valid(target.timing, now) && target.timing.time_basis == "control_compute" &&
    target.timing.capture_start_steady_ns == target.timing.capture_end_steady_ns &&
    v.linear.y == 0 && v.linear.z == 0 && v.angular.x == 0 && v.angular.y == 0 &&
    std::isfinite(v.linear.x) && std::isfinite(v.linear.y) && std::isfinite(v.linear.z) &&
    std::isfinite(v.angular.x) && std::isfinite(v.angular.y) && std::isfinite(v.angular.z) &&
    (!nonzero(v) || now < target.valid_until_steady_ns);
}

class ActuatorAdmission
{
public:
  template<typename Node>
  bool accept(Node & node, const std::string & topic, const msg::ActuatorTarget & target,
    const rclcpp::MessageInfo & info, uint64_t now)
  {
    if (!current(target, now)) return false;
    const auto graph = node.get_publishers_info_by_topic(topic);
    if (graph.size() != 1 || graph[0].node_name() != "motion_output" ||
        graph[0].node_namespace() != "/") return false;
    const auto & incoming = info.get_rmw_message_info().publisher_gid;
    const auto expected = graph[0].endpoint_gid();
    if (!std::any_of(std::begin(incoming.data), std::end(incoming.data),
        [](uint8_t v) {return v != 0;}) ||
        !std::equal(expected.begin(), expected.end(), std::begin(incoming.data))) return false;
    return order_.accept(target.timing, now) != SourceOrder::Result::rejected;
  }
private:
  SourceOrder order_;
};
}  // namespace senior_motion_msgs::timing
