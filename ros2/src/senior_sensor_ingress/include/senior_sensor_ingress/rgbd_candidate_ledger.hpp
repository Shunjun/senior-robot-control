#pragma once

#include <array>
#include <cstddef>
#include <deque>
#include <senior_motion_msgs/msg/bounded_raw_camera_info.hpp>
#include <senior_motion_msgs/msg/bounded_raw_image.hpp>
#include <senior_motion_msgs/msg/raw_image_layout.hpp>

namespace senior_sensor_ingress {
// Caller must first authenticate actual DDS owner and observe source changes.
class RgbdCandidateLedger {
public:
  using Image = senior_motion_msgs::msg::BoundedRawImage;
  using Layout = senior_motion_msgs::msg::RawImageLayout;
  using Info = senior_motion_msgs::msg::BoundedRawCameraInfo;
  bool remember_image(size_t stream, const Image & image);
  bool remember_info(size_t stream, const Info & info);
  bool match_image(size_t stream, const Layout & layout) const;
  bool match_info(size_t stream, const Info & info) const;
  void clear();

private:
  using Header = senior_motion_msgs::msg::RawCaptureHeader;
  std::array<std::deque<Header>, 2> poisoned_images_;
  std::array<std::deque<Header>, 2> poisoned_infos_;
  bool poison_exhausted_ = false;
  std::array<std::deque<Layout>, 2> images_;
  std::array<std::deque<Info>, 2> infos_;
};
}  // namespace senior_sensor_ingress
