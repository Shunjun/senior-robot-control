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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <senior_motion_msgs/msg/scan_motion.hpp>
#include <senior_motion_msgs/msg/fenced_filtered_scan.hpp>
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
    raw_topic_ = declare_parameter<std::string>("raw_scan_topic", "/safety/raw/scan");
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
    subscription_ = create_subscription<senior_motion_msgs::msg::FencedFilteredScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      [this](senior_motion_msgs::msg::FencedFilteredScan::ConstSharedPtr message,
             const rclcpp::MessageInfo & info) {
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
    previous_stamp_valid_ = false;
  }
  void invalidate(senior_motion_msgs::msg::ScanMotion & output, const std::string & reason,
                  bool restart)
  {
    if (restart) {
      matcher_->reset();
      previous_stamp_valid_ = false;
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
  static std::string gid_hex(const uint8_t * data, size_t count)
  {
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (size_t i = 0; i < count; ++i) output << std::setw(2) << unsigned(data[i]);
    return output.str();
  }
  void on_scan(const senior_motion_msgs::msg::FencedFilteredScan & envelope,
               const rclcpp::MessageInfo & info)
  {
    const auto & message = envelope.body;
    const auto & owner = envelope.owner;
    const auto & capture = message.raw_capture;
    senior_motion_msgs::msg::ScanMotion output;
    output.header.stamp = capture.stamp;
    output.header.frame_id = body_;
    const auto sources = get_publishers_info_by_topic(scan_topic_);
    const auto & incoming = info.get_rmw_message_info().publisher_gid;
    if (sources.size() != 1 || sources[0].node_name() != "scan_receipt" ||
        sources[0].node_namespace() != "/") {
      bound_source_ = false;
      invalidate(output, "scan_publisher_unqualified", true); return;
    }
    const auto current = sources[0].endpoint_gid();
    if (!std::equal(current.begin(), current.end(), incoming.data)) return;
    if (owner.relay_gid != gid_hex(incoming.data, RMW_GID_STORAGE_SIZE)) return;
    if (bound_source_ && current != source_gid_) {
      matcher_->reset();
      new_epoch();
      relay_boot_.clear(); relay_sequence_ = 0;
      owner_gid_.clear(); owner_epoch_.clear(); owner_generation_ = owner_sequence_ = 0;
      scan_boot_.clear(); scan_epoch_.clear(); scan_raw_gid_.clear();
      scan_generation_ = scan_sequence_ = 0;
      retired_owner_epochs_.clear(); retired_scan_epochs_.clear();
    }
    source_gid_ = current;
    bound_source_ = true;
    if (owner.relay_boot.empty() || owner.relay_boot.size() > 64 ||
        (relay_boot_.size() && owner.relay_boot != relay_boot_) ||
        owner.relay_sequence <= relay_sequence_ || owner.owner_generation < owner_generation_ ||
        retired_owner_epochs_.count(owner.owner_epoch) != 0) return;
    if (!owner.valid) {
      const bool same_owner = !owner_epoch_.empty() && owner.owner_gid == owner_gid_ &&
        owner.owner_epoch == owner_epoch_;
      if (owner.owner_lost) {
        if (owner.upstream_invalid || !same_owner || owner.owner_sequence != 0 ||
            owner.owner_generation <= owner_generation_) return;
        retired_owner_epochs_.insert(owner_epoch_);
        owner_epoch_.clear(); owner_gid_.clear();
        owner_generation_ = owner.owner_generation; owner_sequence_ = 0;
      } else if (owner.upstream_invalid) {
        if (!same_owner || owner.owner_generation != owner_generation_ ||
            owner.owner_sequence != owner_sequence_) return;
      } else {
        if (owner.owner_gid.empty() || owner.owner_epoch.empty() || owner.owner_sequence == 0 ||
            owner.owner_epoch != message.source_epoch || owner.owner_sequence != message.sequence) return;
        if (owner.owner_generation == owner_generation_) {
          if (!same_owner || owner.owner_sequence <= owner_sequence_) return;
        } else {
          if (owner.owner_generation < owner_generation_) return;
          if (!owner_epoch_.empty()) retired_owner_epochs_.insert(owner_epoch_);
          owner_gid_ = owner.owner_gid; owner_epoch_ = owner.owner_epoch;
          owner_generation_ = owner.owner_generation;
        }
        owner_sequence_ = owner.owner_sequence;
      }
      relay_boot_ = owner.relay_boot;
      relay_sequence_ = owner.relay_sequence;
      if (retired_owner_epochs_.size() >= 256) {
        invalidate(output, "scan_owner_exhausted", true); return;
      }
      invalidate(output, owner.reason.empty() ? "scan_owner_invalid" : owner.reason, true);
      return;
    }
    if (owner.owner_gid.empty() || owner.owner_epoch.empty() || owner.owner_generation == 0 ||
        owner.owner_sequence == 0 || owner.owner_epoch != message.source_epoch ||
        owner.owner_sequence != message.sequence || !message.valid) {
      invalidate(output, "scan_owner_unqualified", true); return;
    }
    if (owner.owner_generation == owner_generation_ && !owner_epoch_.empty() &&
        (owner.owner_epoch != owner_epoch_ || owner.owner_gid != owner_gid_ ||
         owner.owner_sequence <= owner_sequence_)) return;
    if (owner.owner_generation > owner_generation_) {
      if (!owner_epoch_.empty()) retired_owner_epochs_.insert(owner_epoch_);
      if (retired_owner_epochs_.size() >= 256) {
        invalidate(output, "scan_owner_exhausted", true); return;
      }
      if (!owner_epoch_.empty()) {matcher_->reset(); new_epoch();}
      owner_generation_ = owner.owner_generation;
      owner_gid_ = owner.owner_gid;
      owner_epoch_ = owner.owner_epoch;
      owner_sequence_ = 0;
    }
    const auto raw_sources = get_publishers_info_by_topic(raw_topic_);
    const auto raw_gid = raw_sources.size() == 1 ?
      gid_hex(raw_sources[0].endpoint_gid().data(), RMW_GID_STORAGE_SIZE) : "";
    if (!capture.valid || raw_gid.empty() || capture.bridge_gid != raw_gid ||
        capture.bridge_boot_id.empty() || capture.source_epoch.empty() || capture.raw_gid.empty() ||
        capture.sequence == 0 || capture.capture_generation == 0) {
      invalidate(output, "raw_scan_identity_unqualified", true); return;
    }
    if (capture.bridge_boot_id == scan_boot_ && capture.capture_generation < scan_generation_) return;
    if (retired_scan_epochs_.count(capture.source_epoch) != 0) return;
    const bool scan_changed = !scan_boot_.empty() &&
      (capture.bridge_boot_id != scan_boot_ || capture.capture_generation != scan_generation_ ||
       capture.source_epoch != scan_epoch_ || capture.raw_gid != scan_raw_gid_);
    if (scan_changed) {
      if (capture.source_epoch != scan_epoch_) retired_scan_epochs_.insert(scan_epoch_);
      if (retired_scan_epochs_.size() >= 256) {
        invalidate(output, "raw_scan_epoch_exhausted", true); return;
      }
      matcher_->reset(); new_epoch();
      scan_sequence_ = 0;
    }
    if (capture.sequence <= scan_sequence_) return;
    const double steady = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const double received_ros = rclcpp::Time(capture.received_ros).seconds();
    if (!std::isfinite(capture.received_steady) || steady < capture.received_steady ||
        steady - capture.received_steady > max_age_ || received_ros <= 0 ||
        get_clock()->now().seconds() < received_ros ||
        get_clock()->now().seconds() - received_ros > max_age_) {
      invalidate(output, "raw_scan_receipt_expired", true); return;
    }
    relay_boot_ = owner.relay_boot;
    relay_sequence_ = owner.relay_sequence;
    owner_sequence_ = owner.owner_sequence;
    scan_boot_ = capture.bridge_boot_id;
    scan_generation_ = capture.capture_generation;
    scan_epoch_ = capture.source_epoch;
    scan_raw_gid_ = capture.raw_gid;
    scan_sequence_ = capture.sequence;
    const double stamp = rclcpp::Time(capture.stamp).seconds();
    const double now = get_clock()->now().seconds();
    if (!std::isfinite(stamp) || stamp <= 0 || !std::isfinite(now) || now < stamp ||
        now - stamp > max_age_ || (last_stamp_ > 0 && stamp <= last_stamp_) ||
        capture.frame_id.empty() || !std::isfinite(message.time_increment) ||
        message.time_increment != 0.0) {
      last_stamp_ = 0;
      invalidate(output, "time_or_deskew_unqualified", true); return;
    }
    last_stamp_ = stamp;
    try {
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
        if (match.reason == "initializing") {
          previous_stamp_ = rclcpp::Time(capture.stamp);
          previous_stamp_valid_ = true;
        }
        invalidate(output, match.reason, match.reason != "initializing"); return;
      }
      if (!previous_stamp_valid_ || previous_stamp_.seconds() != match.start) {
        invalidate(output, "source_stamp_unavailable", true); return;
      }
      const Pose delta = to_body_delta(match.sensor_delta, extrinsic_);
      const double dt = match.end-match.start;
      if (!positive(dt) || std::hypot(delta.x, delta.y) > max_speed_*dt ||
          std::abs(delta.yaw) > max_yaw_rate_*dt) {
        invalidate(output, "motion_bound_exceeded", true); return;
      }
      pose_ = compose(pose_, delta);
      //Compatible: Humble rclcpp::Time converts to builtin_interfaces::msg::Time.
      // Keep the original integer source identity; match.start is a double.
      output.interval_start = previous_stamp_;
      previous_stamp_ = rclcpp::Time(capture.stamp);
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
  std::string body_, epoch_, scan_topic_, raw_topic_;
  std::string relay_boot_, owner_gid_, owner_epoch_, scan_boot_, scan_epoch_, scan_raw_gid_;
  std::set<std::string> retired_owner_epochs_, retired_scan_epochs_;
  uint64_t relay_sequence_ = 0, owner_generation_ = 0, owner_sequence_ = 0;
  uint64_t scan_generation_ = 0, scan_sequence_ = 0;
  unsigned epoch_counter_ = 0;
  double max_age_ = 0, max_speed_ = 0, max_yaw_rate_ = 0, last_stamp_ = 0;
  Pose extrinsic_, pose_;
  bool extrinsic_valid_ = false;
  bool bound_source_ = false;
  bool published_qualified_ = false;
  rclcpp::Time previous_stamp_{static_cast<int64_t>(0), RCL_ROS_TIME};
  bool previous_stamp_valid_ = false;
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> source_gid_{};
  std::unique_ptr<LocalMatcher> matcher_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<senior_motion_msgs::msg::ScanMotion>::SharedPtr publisher_;
  rclcpp::Subscription<senior_motion_msgs::msg::FencedFilteredScan>::SharedPtr subscription_;
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
    rclcpp::spin(std::make_shared<senior_scan_matcher::ScanMotionNode>());
  } catch (const rclcpp::exceptions::RCLError &) {
    // //Compatible: a Humble graph query may race native context shutdown.
    if (context->is_valid()) {
      throw;
    }
  }
  return 0;
}
