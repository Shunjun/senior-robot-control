#pragma once

#include "senior_sensor_ingress/source_order.hpp"
#include "senior_sensor_ingress/sample_time.hpp"
#include "senior_sensor_ingress/rgbd_candidate_ledger.hpp"
#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <senior_interfaces/msg/bounded_raw_camera_info.hpp>
#include <senior_interfaces/msg/bounded_raw_image.hpp>
#include <senior_interfaces/msg/fenced_rgbd_candidate.hpp>
#include <senior_interfaces/msg/rgbd_motion_candidate.hpp>

namespace senior_sensor_ingress {
class RgbdCandidateReceipt : public rclcpp::Node {
public:
  explicit RgbdCandidateReceipt(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  friend class RgbdCandidateReceiptTestPeer;
  using Image = senior_interfaces::msg::BoundedRawImage;
  using Info = senior_interfaces::msg::BoundedRawCameraInfo;
  using Header = senior_interfaces::msg::RawCaptureHeader;
  using Candidate = senior_interfaces::msg::RgbdMotionCandidate;
  using Fenced = senior_interfaces::msg::FencedRgbdCandidate;
  bool graph_matches(const std::string & topic, const std::string & node);
  bool accept_raw(size_t stream, const Header & header);
  void image(size_t stream, const Image & body);
  void camera_info(size_t stream, const Info & body);
  void candidate(const Candidate & body, const rclcpp::MessageInfo & info);
  void invalidate(const std::string & reason);
  bool originals_current(const Candidate & body);
  bool actual_transforms(const Candidate & body);
  bool clocks_ok(int64_t ros_ns, uint64_t steady);
  std::string output_gid() const;

  std::mutex mutex_;
  std::array<std::string, 4> raw_topics_;
  std::string candidate_topic_, base_, camera_, relay_boot_;
  std::array<SourceOrder, 4> sources_;
  SourceOrder candidate_order_;
  std::string raw_boot_;
  uint64_t raw_last_receipt_ = 0;
  uint64_t raw_generation_ = 0, relay_sequence_ = 0;
  senior_sensor_ingress::time::ClockTracker clock_;
  RgbdCandidateLedger ledger_;
  Candidate last_candidate_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<Fenced>::SharedPtr publisher_;
  std::array<rclcpp::Subscription<Image>::SharedPtr, 2> images_;
  std::array<rclcpp::Subscription<Info>::SharedPtr, 2> infos_;
  rclcpp::Subscription<Candidate>::SharedPtr candidate_sub_;
};
}  // namespace senior_sensor_ingress
