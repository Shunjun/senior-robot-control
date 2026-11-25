#ifndef SENIOR_SLAM_BRIDGE__BOUNDED_SCAN_INGRESS_HPP_
#define SENIOR_SLAM_BRIDGE__BOUNDED_SCAN_INGRESS_HPP_

#include "laser_geometry.hpp"
#include <senior_sensor_ingress/sample_time.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "senior_interfaces/msg/bounded_raw_laser_scan.hpp"
#include "senior_interfaces/msg/bounded_static_tf.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"

namespace senior_mapping
{
class StaticLidarTfIngress;

// Mutable receipt state belongs exclusively to the independent serial ingress group.
// Graph consumers only read active_snapshot() or request terminal retirement.
class BoundedScanIngress
{
public:
  using Receipt = senior_interfaces::msg::RawCaptureHeader;
  using Scan = sensor_msgs::msg::LaserScan;
  using BoundedScan = senior_interfaces::msg::BoundedRawLaserScan;
  using StaticReceipt = senior_interfaces::msg::BoundedStaticTf;
  using Submit = std::function<void(
    Scan::ConstSharedPtr, BoundedScan::ConstSharedPtr, LaserTransform,
    StaticReceipt::ConstSharedPtr)>;

  BoundedScanIngress(
    rclcpp::Node & node, tf2_ros::Buffer & tf, std::string bounded_topic,
    std::string raw_topic, std::string odom_frame, std::string base_frame, Submit submit,
    StaticLidarTfIngress * static_lidar_tf, rclcpp::CallbackGroup::SharedPtr group);

  bool source_active();
  void retire_source();
  bool active_snapshot() const {return active_.load() && !closed_.load();}
  bool retired_snapshot() const {return closed_.load();}

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
  std::int64_t last_measurement_ns_{0};
  std::int64_t last_steady_ns_{0};
  std::uint64_t receipt_floor_ns_{0};
  senior_sensor_ingress::time::ClockTracker clock_;
  std::atomic<bool> closed_{false};
  std::atomic<bool> active_{false};
  rclcpp::Subscription<Message>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace senior_mapping

#endif  // SENIOR_SLAM_BRIDGE__BOUNDED_SCAN_INGRESS_HPP_
