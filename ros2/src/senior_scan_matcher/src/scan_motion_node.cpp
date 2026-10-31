#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <senior_motion_msgs/msg/scan_motion.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "senior_scan_matcher/scan_matcher.hpp"

namespace senior_scan_matcher {
class ScanMotionNode : public rclcpp::Node {
public:
  ScanMotionNode() : Node("scan_motion_node"), buffer_(get_clock()), listener_(buffer_)
  {
    body_ = declare_parameter<std::string>("body_frame", "base_link");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/perception/scan_filtered");
    scan_topic_ = scan_topic;
    max_age_ = declare_parameter<double>("scan_max_age", -1.0);
    max_speed_ = declare_parameter<double>("max_linear_speed", -1.0);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", -1.0);
    const bool static_tf = declare_parameter<bool>("transform_static", false);
    MatcherLimits limits;
    limits.max_interval = declare_parameter<double>("max_interval", -1.0);
    limits.range_limit = declare_parameter<double>("range_limit", -1.0);
    limits.min_response = declare_parameter<double>("min_response", -1.0);
    limits.max_residual = declare_parameter<double>("max_residual", -1.0);
    limits.min_inlier_fraction = declare_parameter<double>("min_inlier_fraction", -1.0);
    limits.min_spread = declare_parameter<double>("min_spread", -1.0);
    limits.min_translation_observability = declare_parameter<double>("min_translation_observability", -1.0);
    limits.min_yaw_observability = declare_parameter<double>("min_yaw_observability", -1.0);
    limits.max_variance = declare_parameter<double>("max_variance", -1.0);
    limits.reanchor_distance = declare_parameter<double>("reanchor_distance", -1.0);
    limits.reanchor_yaw = declare_parameter<double>("reanchor_yaw", -1.0);
    const int min_rays = declare_parameter<int>("min_rays", -1);
    if (body_.empty() || scan_topic.empty() || !static_tf || min_rays < 3 ||
        !positive(max_age_) || !positive(max_speed_) || !positive(max_yaw_rate_))
      throw std::invalid_argument("unqualified scan-motion profile");
    limits.min_rays = static_cast<unsigned>(min_rays);
    matcher_ = std::make_unique<LocalMatcher>(limits);
    publisher_ = create_publisher<senior_motion_msgs::msg::ScanMotion>("/motion/scan", 10);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message, const rclcpp::MessageInfo & info) {
        on_scan(*message, info);
      });
    new_epoch();
  }
private:
  static bool positive(double value) { return std::isfinite(value) && value > 0; }
  void new_epoch()
  {
    std::random_device random;
    std::ostringstream text;
    text << std::hex << random() << random() << random() << random() << '-' << ++epoch_counter_;
    epoch_ = text.str();
    pose_ = {};
    extrinsic_valid_ = false;
    published_qualified_ = false;
  }
  void invalidate(senior_motion_msgs::msg::ScanMotion & output, const std::string & reason,
                  bool restart)
  {
    if (restart) {
      matcher_->reset();
      // A sustained bad stream is one interruption, not hundreds of source
      // lifetimes. Rotate after qualified continuity is actually broken.
      if (published_qualified_) new_epoch();
      else extrinsic_valid_ = false;
    }
    output.source_epoch = epoch_;
    output.valid = false;
    output.reason = reason;
    publisher_->publish(output);
  }
  void on_scan(const sensor_msgs::msg::LaserScan & message, const rclcpp::MessageInfo & info)
  {
    senior_motion_msgs::msg::ScanMotion output;
    output.header = message.header;
    output.header.frame_id = body_;
    const auto sources = get_publishers_info_by_topic(scan_topic_);
    const auto & incoming = info.get_rmw_message_info().publisher_gid;
    if (sources.size() != 1 || sources[0].node_name() != "scan_adapter" ||
        sources[0].node_namespace() != "/") {
      bound_source_ = false;
      invalidate(output, "scan_publisher_unqualified", true); return;
    }
    const auto current = sources[0].endpoint_gid();
    if (!std::equal(current.begin(), current.end(), incoming.data)) {
      bound_source_ = false;
      invalidate(output, "scan_publisher_unqualified", true); return;
    }
    if (bound_source_ && current != source_gid_) {
      matcher_->reset();
      new_epoch();
    }
    source_gid_ = current;
    bound_source_ = true;
    const double stamp = rclcpp::Time(message.header.stamp).seconds();
    const double now = get_clock()->now().seconds();
    if (!std::isfinite(stamp) || stamp <= 0 || !std::isfinite(now) || now < stamp ||
        now - stamp > max_age_ || (last_stamp_ > 0 && stamp <= last_stamp_) ||
        message.header.frame_id.empty() || !std::isfinite(message.time_increment) ||
        message.time_increment != 0.0) {
      last_stamp_ = 0;
      invalidate(output, "time_or_deskew_unqualified", true); return;
    }
    last_stamp_ = stamp;
    try {
      const auto transform = buffer_.lookupTransform(body_, message.header.frame_id,
          rclcpp::Time(message.header.stamp), rclcpp::Duration::from_seconds(0.02));
      const auto & q = transform.transform.rotation;
      const auto & t = transform.transform.translation;
      const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (!std::isfinite(norm) || std::abs(norm-1.0) > 0.001 ||
          std::abs(q.x) > 0.001 || std::abs(q.y) > 0.001 ||
          !std::isfinite(t.x) || !std::isfinite(t.y)) {
        invalidate(output, "invalid_extrinsic", true); return;
      }
      Pose extrinsic{t.x, t.y, std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))};
      if (extrinsic_valid_ && (std::hypot(extrinsic.x-extrinsic_.x, extrinsic.y-extrinsic_.y) > 1e-5 ||
          std::abs(std::atan2(std::sin(extrinsic.yaw-extrinsic_.yaw),
                             std::cos(extrinsic.yaw-extrinsic_.yaw))) > 1e-5)) {
        invalidate(output, "extrinsic_changed", true); return;
      }
      extrinsic_ = extrinsic;
      extrinsic_valid_ = true;
      LaserScan scan;
      scan.stamp = stamp; scan.angle_min = message.angle_min;
      scan.angle_increment = message.angle_increment; scan.range_min = message.range_min;
      scan.range_max = message.range_max;
      scan.ranges.assign(message.ranges.begin(), message.ranges.end());
      auto match = matcher_->update(scan);
      if (!match.valid) {
        invalidate(output, match.reason, match.reason != "initializing"); return;
      }
      const Pose delta = to_body_delta(match.sensor_delta, extrinsic_);
      const double dt = match.end-match.start;
      if (!positive(dt) || std::hypot(delta.x, delta.y) > max_speed_*dt ||
          std::abs(delta.yaw) > max_yaw_rate_*dt) {
        invalidate(output, "motion_bound_exceeded", true); return;
      }
      pose_ = compose(pose_, delta);
      //Compatible: Humble rclcpp::Time converts to builtin_interfaces::msg::Time.
      output.interval_start = rclcpp::Time(static_cast<int64_t>(std::llround(match.start*1e9)),
                                         RCL_ROS_TIME);
      output.source_epoch = epoch_;
      output.valid = true;
      output.reason = match.reason;
      output.body_delta.x = delta.x; output.body_delta.y = delta.y; output.body_delta.theta = delta.yaw;
      output.local_pose.x = pose_.x; output.local_pose.y = pose_.y; output.local_pose.theta = pose_.yaw;
      // Conservative bound for transforming the measured sensor covariance to the body origin.
      const double r2 = extrinsic.x*extrinsic.x + extrinsic.y*extrinsic.y;
      const double position_variance = 2*(match.covariance[0] + match.covariance[4] +
          2*std::abs(match.covariance[1]) + r2*match.covariance[8]);
      output.covariance[0] = position_variance;
      output.covariance[4] = position_variance;
      output.covariance[8] = match.covariance[8];
      output.response = match.response; output.residual = match.residual;
      output.valid_rays = match.valid_rays;
      // No power-on anchor is claimed by a process that may have started after motion.
      output.startup_reference_valid = false;
      published_qualified_ = true;
      publisher_->publish(output);
    } catch (const std::exception & e) {
      invalidate(output, std::string("scan_exception: ") + e.what(), true);
    }
  }
  std::string body_, epoch_, scan_topic_;
  unsigned epoch_counter_ = 0;
  double max_age_ = 0, max_speed_ = 0, max_yaw_rate_ = 0, last_stamp_ = 0;
  Pose extrinsic_, pose_;
  bool extrinsic_valid_ = false;
  bool bound_source_ = false;
  bool published_qualified_ = false;
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> source_gid_{};
  std::unique_ptr<LocalMatcher> matcher_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<senior_motion_msgs::msg::ScanMotion>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<senior_scan_matcher::ScanMotionNode>());
  rclcpp::shutdown();
  return 0;
}
