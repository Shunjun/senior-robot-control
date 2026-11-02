#pragma once

#include <functional>
#include <mutex>
#include <set>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <senior_motion_msgs/msg/bounded_static_tf.hpp>

namespace senior_sensor_ingress {
bool validate_static_tf(const tf2_msgs::msg::TFMessage & message,
                        const std::string & base, const std::string & camera);

class StaticTfReceipt : public rclcpp::Node {
public:
  // Hook and reentrant group are only for DDS ownership-race tests.
  using BeforeVerify = std::function<void(const rmw_gid_t &)>;
  explicit StaticTfReceipt(const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
                           BeforeVerify before_verify = {}, bool test_reentrant = false,
                           std::function<void()> before_inspect = {});

private:
  using Input = tf2_msgs::msg::TFMessage;
  using Output = senior_motion_msgs::msg::BoundedStaticTf;
  void on_message(const Input & message, const rclcpp::MessageInfo & info);
  void inspect();
  bool owner_matches(const std::string & gid);
  bool update_clock(int64_t ros_ns, double steady);
  void invalidate(const std::string & reason, bool retire_source);
  std::string output_gid() const;

  BeforeVerify before_verify_;
  std::function<void()> before_inspect_;
  rclcpp::CallbackGroup::SharedPtr test_callback_group_;
  std::mutex mutex_;
  std::string input_topic_, output_topic_, base_, camera_, expected_node_, expected_namespace_;
  std::string boot_id_, raw_gid_;
  std::set<std::string> retired_;
  uint64_t generation_ = 0, sequence_ = 0;
  int64_t last_ros_ns_ = 0;
  double last_steady_ = 0.0;
  bool clock_ready_ = false, active_ = false, have_body_ = false, failed_ = false;
  Input body_;
  Output first_;
  rclcpp::Publisher<Output>::SharedPtr publisher_;
  rclcpp::Subscription<Input>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace senior_sensor_ingress
