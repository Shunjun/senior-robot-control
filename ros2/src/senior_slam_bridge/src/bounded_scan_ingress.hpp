#ifndef SENIOR_SLAM_BRIDGE__BOUNDED_SCAN_INGRESS_HPP_
#define SENIOR_SLAM_BRIDGE__BOUNDED_SCAN_INGRESS_HPP_

#include "laser_geometry.hpp"

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "senior_motion_msgs/msg/bounded_raw_laser_scan.hpp"
#include "senior_motion_msgs/msg/bounded_static_tf.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"

namespace senior_slam_bridge
{
class StaticLidarTfIngress;

// All subscription and timer work runs on the owner's single-threaded executor.
class BoundedScanIngress
{
public:
  using Receipt = senior_motion_msgs::msg::RawCaptureHeader;
  using Scan = sensor_msgs::msg::LaserScan;
  using BoundedScan = senior_motion_msgs::msg::BoundedRawLaserScan;
  using StaticReceipt = senior_motion_msgs::msg::BoundedStaticTf;
  using Submit = std::function<void(
    Scan::ConstSharedPtr, BoundedScan::ConstSharedPtr, LaserTransform,
    StaticReceipt::ConstSharedPtr)>;

  BoundedScanIngress(
    rclcpp::Node & node, tf2_ros::Buffer & tf, std::string bounded_topic,
    std::string raw_topic, std::string odom_frame, std::string base_frame, Submit submit,
    StaticLidarTfIngress * static_lidar_tf);

  bool source_active();
  void retire_source();

private:
  using Message = BoundedScan;
  void offer(Message::ConstSharedPtr scan, const rclcpp::MessageInfo & info);
  void poll();
  bool observe_clock(std::int64_t now);
  bool current_owner(const Receipt & receipt);

  rclcpp::Node & node_;
  tf2_ros::Buffer & tf_;
  const std::string bounded_topic_, raw_topic_, odom_frame_, base_frame_;
  const Submit submit_;
  StaticLidarTfIngress * const static_lidar_tf_;
  Message::ConstSharedPtr source_;
  std::deque<Message::ConstSharedPtr> pending_;
  std::uint64_t last_sequence_{0};
  std::int64_t last_stamp_{0};
  std::int64_t last_ros_{0};
  bool closed_{false};
  rclcpp::Subscription<Message>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace senior_slam_bridge

#endif  // SENIOR_SLAM_BRIDGE__BOUNDED_SCAN_INGRESS_HPP_
