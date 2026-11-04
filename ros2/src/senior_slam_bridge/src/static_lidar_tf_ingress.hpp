#ifndef SENIOR_SLAM_BRIDGE__STATIC_LIDAR_TF_INGRESS_HPP_
#define SENIOR_SLAM_BRIDGE__STATIC_LIDAR_TF_INGRESS_HPP_

#include "static_lidar_tf.hpp"

#include <chrono>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "senior_motion_msgs/msg/bounded_static_tf.hpp"
#include "senior_motion_msgs/msg/raw_capture_header.hpp"

namespace senior_slam_bridge
{
// All state is owned by the graph's single-threaded executor.
class StaticLidarTfIngress
{
public:
  StaticLidarTfIngress(
    rclcpp::Node & node, std::string topic, std::string relay_node,
    std::string raw_topic, std::string raw_node, std::string base, std::string lidar);

  bool source_active();
  bool admits(const senior_motion_msgs::msg::RawCaptureHeader & scan);
  bool verify_mount(LaserTransform actual);

private:
  using Receipt = senior_motion_msgs::msg::BoundedStaticTf;
  bool current_owners(const Receipt & receipt) const;
  void offer(Receipt::ConstSharedPtr receipt, const rclcpp::MessageInfo & info);
  void poll();
  void retire();

  rclcpp::Node & node_;
  const std::string topic_, relay_node_, raw_topic_, raw_node_, base_, lidar_;
  Receipt::ConstSharedPtr active_, pending_, startup_cursor_;
  std::chrono::steady_clock::time_point pending_deadline_{};
  LaserTransform mount_{};
  std::int64_t last_ros_{0};
  bool retired_{false};
  rclcpp::Subscription<Receipt>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace senior_slam_bridge

#endif  // SENIOR_SLAM_BRIDGE__STATIC_LIDAR_TF_INGRESS_HPP_
