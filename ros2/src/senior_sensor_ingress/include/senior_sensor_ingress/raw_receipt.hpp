#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <set>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <senior_motion_msgs/msg/bounded_depth_evidence.hpp>
#include <senior_motion_msgs/msg/bounded_semantic_evidence.hpp>
#include <senior_motion_msgs/msg/fenced_semantic_evidence.hpp>
#include <senior_motion_msgs/msg/fenced_depth_evidence.hpp>
#include <senior_motion_msgs/msg/bounded_raw_camera_info.hpp>
#include "senior_sensor_ingress/capture_ledger.hpp"
#include <senior_motion_msgs/msg/bounded_raw_image.hpp>
#include <senior_motion_msgs/msg/raw_capture_header.hpp>

namespace senior_sensor_ingress {
std::string gid_hex(const rmw_gid_t & gid);
std::string gid_hex(const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid);
std::string gid_hex(const rmw_gid_t & gid);
std::string gid_hex(const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & gid);

class RawReceipt : public rclcpp::Node {
public:
  // The hook is supplied only by the component test to pause a real callback
  // between rmw receipt and graph verification; production has no such hook.
  using BeforeVerify = std::function<void(size_t, const rmw_gid_t &)>;
  // The reentrant group is injected only by DDS race tests; production uses
  // the default callback group and single-threaded executor.
  explicit RawReceipt(const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
                      BeforeVerify before_verify = {}, bool test_reentrant_callbacks = false);

private:
  struct Source {
    bool active = false;
    std::string raw_gid;
    std::string epoch;
    uint64_t sequence = 0;
    bool invalid = false;
    std::set<std::string> retired_gids;
  };
  using RawImage = senior_motion_msgs::msg::BoundedRawImage;
  using RawInfo = senior_motion_msgs::msg::BoundedRawCameraInfo;
  using Header = senior_motion_msgs::msg::RawCaptureHeader;
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;

  std::string new_epoch();
  std::string output_gid(size_t index) const;
  bool qualify_source(size_t index, const rmw_gid_t & incoming,
                      const std_msgs::msg::Header & source_header,
                      int64_t received_ros_ns, double received_steady);
  void inspect_topology();
  void invalidate_all(const std::string & reason, size_t trigger = 4,
                      const std_msgs::msg::Header * source_header = nullptr,
                      int64_t received_ros_ns = 0, double received_steady = 0.0,
                      const std::string & raw_gid = "");
  Header header(size_t index, const std::string & reason);
  bool clock_ok(size_t trigger = 4, const std_msgs::msg::Header * source_header = nullptr,
                int64_t received_ros_ns = 0, double received_steady = 0.0,
                const std::string & raw_gid = "");
  void on_image(size_t index, const Image & message, const rclcpp::MessageInfo & info);
  void on_info(size_t index, const Info & message, const rclcpp::MessageInfo & info);
  void on_depth(const senior_motion_msgs::msg::BoundedDepthEvidence & message,
                const rclcpp::MessageInfo & info);
  void inspect_depth_owner();
  void invalidate_depth(const std::string & reason, bool owner_lost,
                        const senior_motion_msgs::msg::BoundedDepthEvidence * body = nullptr);
  bool retire_gid(std::set<std::string> & retired, const std::string & gid);
  void on_semantic(const senior_motion_msgs::msg::BoundedSemanticEvidence & message,
                   const rclcpp::MessageInfo & info);
  void inspect_semantic_owner();
  void invalidate_semantic(const std::string & reason, bool owner_lost,
                           bool upstream_invalid = false,
                           const senior_motion_msgs::msg::BoundedSemanticEvidence * body = nullptr);
  void clear_depth_history();

  BeforeVerify before_verify_;
  std::mutex state_mutex_;
  rclcpp::CallbackGroup::SharedPtr depth_callback_group_;
  CaptureLedger ledger_;
  std::deque<senior_motion_msgs::msg::FencedDepthEvidence> depth_history_;
  uint64_t depth_history_retired_sequence_ = 0;
  std::string semantic_input_topic_, semantic_owner_gid_, semantic_owner_epoch_;
  std::set<std::string> semantic_retired_gids_, semantic_retired_epochs_;
  uint64_t semantic_owner_generation_ = 0, semantic_owner_sequence_ = 0;
  uint64_t semantic_relay_sequence_ = 0;
  bool semantic_active_ = false, semantic_invalid_ = false;
  rclcpp::Publisher<senior_motion_msgs::msg::FencedSemanticEvidence>::SharedPtr semantic_fenced_pub_;
  rclcpp::Subscription<senior_motion_msgs::msg::BoundedSemanticEvidence>::SharedPtr semantic_body_sub_;
  std::string depth_input_topic_;
  std::string depth_owner_gid_, depth_owner_epoch_;
  std::set<std::string> depth_retired_gids_, depth_retired_epochs_;
  uint64_t depth_owner_generation_ = 0, depth_owner_sequence_ = 0, depth_relay_sequence_ = 0;
  bool depth_active_ = false, depth_invalid_ = false;
  rclcpp::Publisher<senior_motion_msgs::msg::FencedDepthEvidence>::SharedPtr depth_fenced_pub_;
  rclcpp::Subscription<senior_motion_msgs::msg::BoundedDepthEvidence>::SharedPtr depth_body_sub_;
  std::array<std::string, 4> input_topics_;
  std::array<Source, 4> sources_;
  std::string boot_id_;
  uint64_t counter_ = 0;
  uint64_t capture_generation_ = 1;
  int64_t last_ros_ns_ = 0;
  bool clock_bad_ = false;
  bool failed_ = false;
  rclcpp::Publisher<RawImage>::SharedPtr rgb_publisher_, depth_publisher_;
  rclcpp::Publisher<RawInfo>::SharedPtr rgb_info_publisher_, depth_info_publisher_;
  rclcpp::Subscription<Image>::SharedPtr rgb_subscription_, depth_subscription_;
  rclcpp::Subscription<Info>::SharedPtr rgb_info_subscription_, depth_info_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace senior_sensor_ingress
