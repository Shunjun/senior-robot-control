#pragma once

#include "laser_geometry.hpp"
#include <senior_sensor_ingress/source_order.hpp>
#include <atomic>
#include <deque>
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "senior_interfaces/msg/bounded_raw_laser_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"

namespace senior_mapping {
// One serial ingress group owns admission and capture ordering.
// Graph callbacks read availability; shutdown cancels and joins the group.
class BoundedScanIngress {
public:
  using Receipt = senior_interfaces::msg::RawCaptureHeader;
  using Scan = sensor_msgs::msg::LaserScan;
  using BoundedScan = senior_interfaces::msg::BoundedRawLaserScan;
  using Submit = std::function<void(Scan::ConstSharedPtr, BoundedScan::ConstSharedPtr,
                                  LaserTransform)>;
  BoundedScanIngress(rclcpp::Node & node, tf2_ros::Buffer & tf, std::string bounded_topic,
    std::string odom_frame, std::string base_frame, Submit submit,
    rclcpp::CallbackGroup::SharedPtr group);

  void close() {closed_.store(true); active_.store(false);}
  bool active_snapshot() const {return active_.load() && !closed_.load();}

private:
  void offer(BoundedScan::ConstSharedPtr scan);
  void poll();
  rclcpp::Node & node_;
  tf2_ros::Buffer & tf_;
  const std::string bounded_topic_, odom_frame_, base_frame_;
  const std::string expected_name_, expected_namespace_;
  const Submit submit_;
  senior_sensor_ingress::SourceOrder order_;
  std::deque<BoundedScan::ConstSharedPtr> pending_;
  std::atomic<bool> closed_{false}, active_{false};
  rclcpp::Subscription<BoundedScan>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace senior_mapping
