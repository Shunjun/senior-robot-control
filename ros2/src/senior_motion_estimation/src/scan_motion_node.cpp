#include <senior_sensor_ingress/publisher_topology.hpp>
#include <senior_sensor_ingress/ros_wiring.hpp>
#include <senior_sensor_ingress/sample_time.hpp>
#include <senior_sensor_ingress/sample_record.hpp>
#include <senior_sensor_ingress/source_order.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <iomanip>
#include <set>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <senior_interfaces/msg/scan_motion.hpp>
#include <senior_interfaces/msg/raw_capture_header.hpp>
#include <senior_interfaces/msg/fenced_filtered_scan.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "senior_motion_estimation/scan_matcher.hpp"
#include "senior_motion_estimation/diagnostic_isolation.hpp"

namespace senior_motion_estimation::scan_matching {
class ScanMotionNode : public rclcpp::Node {
public:
  ScanMotionNode() : Node(senior_sensor_ingress::wiring::node_scan_motion_node, senior_sensor_ingress::wiring::ns_scan_motion_node), buffer_(get_clock()), listener_(buffer_)
  {
    body_ = declare_parameter<std::string>("body_frame", "base_link");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", senior_sensor_ingress::wiring::perception_scan_filtered);
    scan_topic_ = scan_topic;
    raw_topic_ = declare_parameter<std::string>("raw_scan_topic", senior_sensor_ingress::wiring::safety_raw_scan);
    //Compatible: legacy launch may supply this policy; raw input has no TTL.
    declare_parameter<double>("sensor_hard_age_ms", -1.0);
    max_speed_ = declare_parameter<double>("max_linear_speed", -1.0);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", -1.0);
    MatcherLimits limits;
    limits.max_interval_ms = declare_parameter<double>("max_interval_ms", -1.0);
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
    if (body_.empty() || scan_topic.empty() || min_rays < 3 ||
        !positive(max_speed_) || !positive(max_yaw_rate_)) {
      std::ostringstream detail;
      detail << "invalid scan-motion parameters: body_frame='" << body_
             << "', scan_topic='" << scan_topic << "', min_rays=" << min_rays
             << ", max_linear_speed=" << max_speed_
             << ", max_yaw_rate=" << max_yaw_rate_
             << "; frames/topics must be nonempty, min_rays >= 3, limits finite and > 0";
      throw std::invalid_argument(detail.str());
    }
    limits.min_rays = static_cast<unsigned>(min_rays);
    matcher_ = std::make_unique<LocalMatcher>(limits);
    publisher_ = create_publisher<senior_interfaces::msg::ScanMotion>(senior_sensor_ingress::wiring::motion_scan, 10);
    //Compatible: add independent diagnostics without changing ScanMotion consumers.
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(senior_sensor_ingress::wiring::motion_scan_diagnostics, rclcpp::SensorDataQoS().keep_last(1));
    subscription_ = create_subscription<senior_interfaces::msg::FencedFilteredScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](senior_interfaces::msg::FencedFilteredScan::ConstSharedPtr message,
             const rclcpp::MessageInfo & info) {
        on_scan(*message, info);
      });
    new_epoch();
  }
private:
  void publish_diagnostics(const senior_interfaces::msg::ScanMotion & output) noexcept
  {
    const bool published = emit_scan_diagnostics(output, current_diagnostics_,
      [this](const auto & array) { diagnostics_publisher_->publish(array); });
    if (!published && !diagnostic_failure_reported_) {
      diagnostic_failure_reported_ = true;
      try {
        RCLCPP_WARN(get_logger(), "Independent scan diagnostics failed; suppressing further warnings");
      } catch (...) {
        // Logging is also observational and must not invalidate motion.
      }
    }
  }
  static bool positive(double value) { return std::isfinite(value) && value > 0; }
  void new_epoch()
  {
    std::random_device random;
    std::ostringstream text;
    text << std::hex << random() << random() << random() << random() << '-' << ++epoch_counter_;
    epoch_ = text.str();
    pose_ = {};
    published_qualified_ = false;
    previous_stamp_valid_ = false;
    stamp_origin_ns_ = -1;
    last_stamp_ns_ = -1;
  }
  void invalidate(senior_interfaces::msg::ScanMotion & output, const std::string & reason,
                  bool restart)
  {
    if (restart) {
      matcher_->reset();
      previous_stamp_valid_ = false;
      stamp_origin_ns_ = -1;
      last_stamp_ns_ = -1;
      // A sustained bad stream is one interruption, not hundreds of source
      // lifetimes. Rotate after qualified continuity is actually broken.
      if (published_qualified_) new_epoch();
    }
    output.source_epoch = epoch_;
    output.valid = false;
    output.reason = reason;
    publisher_->publish(output);
    publish_diagnostics(output);
  }
  void on_scan(const senior_interfaces::msg::FencedFilteredScan & envelope,
               const rclcpp::MessageInfo &)
  {
    current_diagnostics_ = {};
    const auto & message = envelope.body;
    const auto & owner = envelope.owner;
    const auto & capture = message.raw_capture;
    senior_interfaces::msg::ScanMotion output;
    output.header.stamp = capture.stamp;
    output.header.frame_id = body_;
    const auto current_steady = senior_sensor_ingress::time::monotonic_ns();
    senior_sensor_ingress::time::ClockObservation local_clock;
    try {
      local_clock = clock_.observe(get_clock()->now().nanoseconds(), current_steady);
    } catch (const std::invalid_argument & error) {
      invalidate(output, error.what(), true);
      return;
    }
    if (local_clock.discontinuity) {
        invalidate(output, "clock_rollback", true);
      return;
    }
    const auto sources = get_publishers_info_by_topic(scan_topic_);
    if (!senior_sensor_ingress::publisher_matches(sources,
        senior_sensor_ingress::wiring::node_scan_receipt,
        senior_sensor_ingress::wiring::ns_scan_receipt,
        "senior_interfaces/msg/FencedFilteredScan")) {
      invalidate(output, "scan_owner_unavailable", true); return;
    }
    const auto generation = scan_order_.generation();
    if (!owner.valid || owner.owner_lost || owner.upstream_invalid || !message.valid || !capture.valid) {
      if (!scan_order_.observe_generation(capture.bridge_boot_id, capture.capture_generation)) return;
      invalidate(output, message.reason.empty() ? "scan_invalid" : message.reason, true); return;
    }
    int64_t stamp_ns;
    try {stamp_ns = senior_sensor_ingress::stamp_ns(capture.stamp);}
    catch (const std::invalid_argument &) {invalidate(output, "invalid_stamp", true); return;}
    if (capture.frame_id.empty() || capture.clock_generation == 0 ||
        capture.sample_steady_ns > capture.received_steady_ns) {
      invalidate(output, "invalid_capture", true); return;
    }
    const auto previous_boot = scan_order_.boot_id();
    if (!scan_order_.accept_capture(capture.bridge_boot_id, capture.capture_generation,
        stamp_ns, capture.received_steady_ns)) return;
    if (previous_boot != scan_order_.boot_id() || generation != scan_order_.generation()) {
      matcher_->reset(); new_epoch();
    }
    if (stamp_origin_ns_ < 0) stamp_origin_ns_ = stamp_ns;
    const double relative_stamp = static_cast<double>(stamp_ns - stamp_origin_ns_) / 1e9;
    last_stamp_ns_ = stamp_ns;
    try {
      // TF resolves static and dynamic edges at the scan timestamp; the
      // comparison below detects changes in the mounted sensor extrinsic.
      const auto transform = buffer_.lookupTransform(body_, capture.frame_id,
          rclcpp::Time(capture.stamp), rclcpp::Duration::from_seconds(0.02));
      const auto & q = transform.transform.rotation;
      const auto & t = transform.transform.translation;
      const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (!std::isfinite(norm) || std::abs(norm-1.0) > 0.001 ||
          std::abs(q.x) > 0.001 || std::abs(q.y) > 0.001 ||
          !std::isfinite(t.x) || !std::isfinite(t.y)) {
        invalidate(output, "invalid_extrinsic", true); return;
      }
      Pose extrinsic{t.x, t.y, std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))};
      LaserScan scan;
      scan.stamp = relative_stamp; scan.angle_min = message.angle_min;
      scan.angle_increment = message.angle_increment; scan.range_min = message.range_min;
      scan.range_max = message.range_max;
      scan.ranges.assign(message.ranges.begin(), message.ranges.end());
      auto match = matcher_->update(scan);
      current_diagnostics_ = std::move(match.diagnostics);
      if (!match.valid) {
        if (match.reason == "initializing") {
          previous_stamp_ = rclcpp::Time(capture.stamp);
          previous_capture_ = capture;
          previous_extrinsic_ = extrinsic;
          previous_stamp_valid_ = true;
        }
        invalidate(output, match.reason, match.reason != "initializing"); return;
      }
      if (!previous_stamp_valid_ || previous_capture_.clock_generation != capture.clock_generation ||
          static_cast<double>(previous_stamp_.nanoseconds() - stamp_origin_ns_) / 1e9 != match.start) {
        invalidate(output, "source_stamp_unavailable", true); return;
      }
      const Pose delta = compose(compose(previous_extrinsic_, match.sensor_delta), inverse(extrinsic));
      const double dt = static_cast<double>(stamp_ns - previous_stamp_.nanoseconds()) / 1e9;
      if (!positive(dt) || std::hypot(delta.x, delta.y) > max_speed_*dt ||
          std::abs(delta.yaw) > max_yaw_rate_*dt) {
        invalidate(output, "motion_bound_exceeded", true); return;
      }
      pose_ = compose(pose_, delta);
      //Compatible: Humble rclcpp::Time converts to builtin_interfaces::msg::Time.
      // Keep the original integer source identity; match.start is a double.
      output.interval_start = previous_stamp_;
      output.first_capture = previous_capture_;
      output.second_capture = capture;
      previous_stamp_ = rclcpp::Time(capture.stamp);
      previous_capture_ = capture;
      previous_extrinsic_ = extrinsic;
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
      publish_diagnostics(output);
    } catch (const std::exception & e) {
      invalidate(output, std::string("scan_exception: ") + e.what(), true);
    }
  }
  std::string body_, epoch_, scan_topic_, raw_topic_;
  unsigned epoch_counter_ = 0;
  senior_sensor_ingress::time::ClockTracker clock_;
  int64_t stamp_origin_ns_ = -1, last_stamp_ns_ = -1;
  double max_speed_ = 0, max_yaw_rate_ = 0;
  Pose extrinsic_, previous_extrinsic_, pose_;
  senior_sensor_ingress::SourceOrder scan_order_;
  bool published_qualified_ = false;
  rclcpp::Time previous_stamp_{static_cast<int64_t>(0), RCL_ROS_TIME};
  bool previous_stamp_valid_ = false;
  senior_interfaces::msg::RawCaptureHeader previous_capture_;
  bool diagnostic_failure_reported_ = false;
  MatchDiagnostics current_diagnostics_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  std::unique_ptr<LocalMatcher> matcher_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<senior_interfaces::msg::ScanMotion>::SharedPtr publisher_;
  rclcpp::Subscription<senior_interfaces::msg::FencedFilteredScan>::SharedPtr subscription_;
};
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto context = rclcpp::contexts::get_global_default_context();
  struct Shutdown
  {
    rclcpp::Context::SharedPtr context;
    ~Shutdown() {rclcpp::shutdown(context);}
  } shutdown{context};
  try {
    rclcpp::spin(std::make_shared<senior_motion_estimation::scan_matching::ScanMotionNode>());
  } catch (const rclcpp::exceptions::RCLError &) {
    // //Compatible: a Humble graph query may race native context shutdown.
    if (context->is_valid()) {
      throw;
    }
  }
  return 0;
}
