#pragma once

#include <array>
#include <deque>
#include <utility>
#include <mutex>
#include <set>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <senior_motion_msgs/msg/bounded_raw_camera_info.hpp>
#include <senior_motion_msgs/msg/bounded_raw_image.hpp>
#include <senior_motion_msgs/msg/bounded_static_tf.hpp>
#include <senior_motion_msgs/msg/fenced_rgbd_candidate.hpp>
#include <senior_motion_msgs/msg/rgbd_motion_candidate.hpp>
#include "senior_sensor_ingress/rgbd_candidate_ledger.hpp"

namespace senior_sensor_ingress {
class RgbdCandidateReceipt : public rclcpp::Node {
public:
  explicit RgbdCandidateReceipt(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  friend class RgbdCandidateReceiptTestPeer;
  using Image = senior_motion_msgs::msg::BoundedRawImage;
  using Info = senior_motion_msgs::msg::BoundedRawCameraInfo;
  using Header = senior_motion_msgs::msg::RawCaptureHeader;
  using Static = senior_motion_msgs::msg::BoundedStaticTf;
  using Candidate = senior_motion_msgs::msg::RgbdMotionCandidate;
  using Fenced = senior_motion_msgs::msg::FencedRgbdCandidate;
  struct Source {
    std::string bridge_gid, last_bridge_gid, last_bridge_boot, raw_gid, epoch;
    uint64_t sequence = 0;
    Header last_header;
    bool has_last_header = false;
    std::set<std::string> retired_bridge, retired_raw;
  };
  bool graph_matches(const std::string & topic, const std::string & node, const std::string & gid);
  bool accept_raw(size_t stream, const Header & header, const std::string & actual);
  void image(size_t stream, const Image & body, const rclcpp::MessageInfo & info);
  void camera_info(size_t stream, const Info & body, const rclcpp::MessageInfo & info);
  void tf(const Static & body, const std::string & actual);
  void accept_tf(const Static & body, const std::string & actual);
  void retry_tf();
  void candidate(const Candidate & body, const rclcpp::MessageInfo & info);
  void inspect();
  void invalidate(const std::string & reason, bool owner_lost = false);
  bool originals_current(const Candidate & body, int64_t ros_ns, double steady);
  bool clocks_ok(int64_t ros_ns, double steady);
  std::string output_gid() const;

  std::mutex mutex_;
  std::array<std::string, 4> raw_topics_;
  std::string tf_topic_, candidate_topic_, output_topic_, base_, camera_;
  std::array<Source, 4> sources_;
  std::string raw_boot_, tf_gid_, tf_boot_, tf_raw_gid_, producer_gid_, producer_boot_, relay_boot_;
  std::set<std::string> retired_boots_, retired_tf_gids_, retired_tf_raw_gids_, retired_producer_gids_;
  uint64_t tf_generation_ = 0, raw_generation_ = 0, producer_generation_ = 0, producer_sequence_ = 0, relay_sequence_ = 0;
  int64_t producer_floor_ros_ = 0, last_ros_ = 0;
  double producer_floor_steady_ = 0.0, last_steady_ = 0.0;
  bool failed_ = false, tf_ready_ = false, tf_needs_new_generation_ = false, producer_ready_ = false;
  Static tf_first_;
  std::deque<std::pair<std::string, Static>> pending_tf_;
  RgbdCandidateLedger ledger_;
  Candidate last_candidate_;
  rclcpp::Publisher<Fenced>::SharedPtr publisher_;
  std::array<rclcpp::Subscription<Image>::SharedPtr, 2> images_;
  std::array<rclcpp::Subscription<Info>::SharedPtr, 2> infos_;
  rclcpp::Subscription<Static>::SharedPtr tf_sub_;
  rclcpp::Subscription<Candidate>::SharedPtr candidate_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace senior_sensor_ingress
