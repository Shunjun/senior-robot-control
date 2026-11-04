#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <senior_motion_msgs/msg/bounded_raw_laser_scan.hpp>
#include <senior_motion_msgs/msg/bounded_safety_scan.hpp>
#include <senior_motion_msgs/msg/bounded_filtered_scan.hpp>
#include <senior_motion_msgs/msg/fenced_safety_scan.hpp>
#include <senior_motion_msgs/msg/fenced_filtered_scan.hpp>
#include "senior_sensor_ingress/scan_ledger.hpp"

namespace senior_sensor_ingress {
class ScanReceipt : public rclcpp::Node {
public:
  using BeforeVerify = std::function<void(size_t, const rmw_gid_t &)>;
  // Test-only callback group injection makes delayed DDS callbacks reproducible;
  // production uses the default callback group and single-threaded executor.
  explicit ScanReceipt(const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
                       BeforeVerify before_verify = {}, bool test_reentrant_callbacks = false);

private:
  using Scan = sensor_msgs::msg::LaserScan;
  using Bounded = senior_motion_msgs::msg::BoundedRawLaserScan;
  std::string new_epoch();
  bool retire(const std::string & gid);
  bool clock_ok();
  void invalidate(const std::string & reason, const Scan * source = nullptr,
                  int64_t received_ros = 0, double received_steady = 0.0,
                  const std::string & raw_gid = "");
  void inspect_topology();
  void on_scan(const Scan & message, const rclcpp::MessageInfo & info);
  struct Owner {
    std::string topic, gid, epoch;
    std::set<std::string> retired_gids, retired_epochs;
    uint64_t generation = 0, sequence = 0, relay_sequence = 0;
    bool active = false, invalid = false;
  };
  bool qualify_owner(Owner & owner, const rmw_gid_t & incoming,
                     const std::string & epoch, uint64_t sequence);
  void inspect_owner(Owner & owner, size_t index);
  void invalidate_owner(Owner & owner, size_t index, const std::string & reason,
                        bool owner_lost, bool upstream_invalid = false,
                        const senior_motion_msgs::msg::BoundedSafetyScan * safety = nullptr,
                        const senior_motion_msgs::msg::BoundedFilteredScan * filtered = nullptr);
  void on_safety(const senior_motion_msgs::msg::BoundedSafetyScan & message,
                 const rclcpp::MessageInfo & info);
  void on_filtered(const senior_motion_msgs::msg::BoundedFilteredScan & message,
                   const rclcpp::MessageInfo & info);

  BeforeVerify before_verify_;
  ScanLedger ledger_;
  Owner safety_owner_, filter_owner_;
  rclcpp::Publisher<senior_motion_msgs::msg::FencedSafetyScan>::SharedPtr safety_publisher_;
  rclcpp::Publisher<senior_motion_msgs::msg::FencedFilteredScan>::SharedPtr filter_publisher_;
  rclcpp::Subscription<senior_motion_msgs::msg::BoundedSafetyScan>::SharedPtr safety_subscription_;
  rclcpp::Subscription<senior_motion_msgs::msg::BoundedFilteredScan>::SharedPtr filter_subscription_;
  std::mutex mutex_;
  rclcpp::CallbackGroup::SharedPtr test_group_;
  rclcpp::Subscription<Scan>::SharedPtr subscription_;
  rclcpp::Publisher<Bounded>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string input_topic_, boot_, epoch_, raw_gid_;
  std::set<std::string> retired_gids_;
  uint64_t counter_ = 0, generation_ = 1, sequence_ = 0;
  int64_t last_ros_ns_ = 0;
  bool active_ = false, invalid_ = false, clock_bad_ = false, failed_ = false;
};
}  // namespace senior_sensor_ingress
