#include <senior_sensor_ingress/publisher_topology.hpp>
#include <senior_sensor_ingress/ros_wiring.hpp>
#include "senior_sensor_ingress/rgbd_candidate_receipt.hpp"
#include "senior_sensor_ingress/raw_receipt.hpp"
#include "senior_sensor_ingress/sample_record.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>

namespace senior_sensor_ingress {
namespace {
std::string new_boot()
{
  std::random_device random;
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (int i=0; i<4; ++i) out << std::setw(8) << random();
  return out.str();
}
bool rigid(const std::array<double, 16> & m)
{
  for (const double v : m) if (!std::isfinite(v)) return false;
  if (std::abs(m[12]) > 1e-8 || std::abs(m[13]) > 1e-8 ||
      std::abs(m[14]) > 1e-8 || std::abs(m[15]-1) > 1e-8) return false;
  for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) {
    double dot = 0;
    for (int k=0; k<3; ++k) dot += m[k*4+i] * m[k*4+j];
    if (!std::isfinite(dot) || std::abs(dot - (i==j ? 1.0 : 0.0)) > 1e-6) return false;
  }
  const double det = m[0]*(m[5]*m[10]-m[6]*m[9]) -
    m[1]*(m[4]*m[10]-m[6]*m[8]) + m[2]*(m[4]*m[9]-m[5]*m[8]);
  return std::isfinite(det) && std::abs(det-1) <= 1e-6;
}
bool sample_valid(const senior_interfaces::msg::RawCaptureHeader & h)
{
  try {
    const auto stamp = stamp_ns(h.stamp), received = stamp_ns(h.received_ros);
    return h.valid && h.clock_generation > 0 && received >= stamp &&
      h.sample_steady_ns <= h.received_steady_ns &&
      h.received_steady_ns-h.sample_steady_ns == static_cast<uint64_t>(received-stamp);
  } catch (const std::invalid_argument &) {return false;}
}
bool transform_matches(const geometry_msgs::msg::TransformStamped & a,
                       const geometry_msgs::msg::TransformStamped & b)
{
  const auto & at=a.transform.translation; const auto & bt=b.transform.translation;
  const auto & aq=a.transform.rotation; const auto & bq=b.transform.rotation;
  const std::array<double, 7> left{at.x, at.y, at.z, aq.x, aq.y, aq.z, aq.w};
  const std::array<double, 7> right{bt.x, bt.y, bt.z, bq.x, bq.y, bq.z, bq.w};
  for (size_t i=0; i<left.size(); ++i)
    if (!std::isfinite(left[i]) || std::abs(left[i]-right[i]) > 1e-9) return false;
  return a.header.frame_id == b.header.frame_id && a.child_frame_id == b.child_frame_id;
}
}

RgbdCandidateReceipt::RgbdCandidateReceipt(const rclcpp::NodeOptions & options)
: Node(senior_sensor_ingress::wiring::node_rgbd_candidate_receipt, senior_sensor_ingress::wiring::ns_rgbd_candidate_receipt, options), relay_boot_(new_boot())
{
  const auto prefix=declare_parameter<std::string>("topic_prefix", senior_sensor_ingress::wiring::rgbd_diagnostic);
  raw_topics_={wiring::rgbd_diagnostic_rgb, wiring::rgbd_diagnostic_depth,
    wiring::rgbd_diagnostic_rgb_info, wiring::rgbd_diagnostic_depth_info};
  if (prefix != wiring::rgbd_diagnostic) {
    //Compatible: explicit diagnostic prefix override remains an operator remap.
    raw_topics_={prefix+"/rgb", prefix+"/depth", prefix+"/rgb_info", prefix+"/depth_info"};
  }
  candidate_topic_=prefix == wiring::rgbd_diagnostic ? wiring::rgbd_diagnostic_candidate : prefix+"/candidate";
  base_=declare_parameter<std::string>("base_frame", "base_link");
  camera_=declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
  if (prefix.empty() || base_.empty() || camera_.empty() || base_ == camera_)
    throw std::invalid_argument("invalid RGB-D topics or frames");
  tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_=std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
  const auto qos=rclcpp::SensorDataQoS().keep_last(4);
  publisher_=create_publisher<Fenced>(prefix == wiring::rgbd_diagnostic ? wiring::rgbd_diagnostic_fenced : prefix+"/fenced", qos);
  for (size_t i=0; i<2; ++i) {
    images_[i]=create_subscription<Image>(raw_topics_[i], qos,
      [this,i](Image::ConstSharedPtr body) {image(i,*body);});
    infos_[i]=create_subscription<Info>(raw_topics_[i+2], qos,
      [this,i](Info::ConstSharedPtr body) {camera_info(i,*body);});
  }
  candidate_sub_=create_subscription<Candidate>(candidate_topic_, qos,
    [this](Candidate::ConstSharedPtr body,const rclcpp::MessageInfo & info) {candidate(*body,info);});
}
std::string RgbdCandidateReceipt::output_gid() const {return gid_hex(publisher_->get_gid());}

bool RgbdCandidateReceipt::graph_matches(const std::string & topic,const std::string & node)
{
  const auto graph=get_publishers_info_by_topic(topic);
  const std::string type = topic==candidate_topic_ ?
    "senior_interfaces/msg/RgbdMotionCandidate" :
    (topic==raw_topics_[0] || topic==raw_topics_[1] ?
      "senior_interfaces/msg/BoundedRawImage" : "senior_interfaces/msg/BoundedRawCameraInfo");
  return publisher_matches(graph, node, topic == candidate_topic_ ?
    wiring::ns_rgbd_candidate_producer : wiring::ns_raw_receipt, type);
}

void RgbdCandidateReceipt::invalidate(const std::string & reason)
{
  Fenced output;
  output.owner.relay_gid=output_gid(); output.owner.relay_boot=relay_boot_;
  output.owner.relay_sequence=++relay_sequence_;
  output.owner.owner_gid=last_candidate_.producer_gid;
  output.owner.owner_epoch=last_candidate_.producer_boot_id;
  output.owner.owner_generation=last_candidate_.owner_generation;
  output.owner.owner_sequence=last_candidate_.sequence;
  output.owner.reason=reason; output.body=last_candidate_;
  publisher_->publish(output);
}

bool RgbdCandidateReceipt::clocks_ok(int64_t ros_ns,uint64_t steady)
{
  try {
    if (clock_.observe(ros_ns,steady).discontinuity) {
      ledger_.clear(); invalidate("clock_rollback");
      return false;
    }
    return true;
  } catch (const std::invalid_argument &) {invalidate("clock_invalid"); return false;}
}

bool RgbdCandidateReceipt::accept_raw(size_t stream,const Header & h)
{
  if (!graph_matches(raw_topics_[stream],wiring::node_raw_receipt)) return false;
  auto & source=sources_[stream];
  if (!h.valid) {
    if (h.capture_generation < raw_generation_ ||
        !source.observe_generation(h.bridge_boot_id,h.capture_generation)) return false;
    if (h.capture_generation > raw_generation_) {
      ledger_.clear(); raw_generation_=h.capture_generation;
    }
    invalidate("raw_invalid");
    return false;
  }
  if (!sample_valid(h) || h.frame_id.empty()) return false;
  if (!raw_boot_.empty() && h.bridge_boot_id != raw_boot_) {
    if (h.received_steady_ns <= raw_last_receipt_) return false;
    for (auto & order : sources_) order.confirm_boot(h.bridge_boot_id);
    ledger_.clear(); raw_generation_=0;
  }
  if (h.capture_generation < raw_generation_ ||
      !source.accept_capture(h.bridge_boot_id,h.capture_generation,
                             stamp_ns(h.stamp),h.received_steady_ns)) return false;
  if (h.capture_generation > raw_generation_) {
    ledger_.clear(); raw_generation_=h.capture_generation;
  }
  raw_boot_=h.bridge_boot_id;
  raw_last_receipt_=std::max(raw_last_receipt_,h.received_steady_ns);
  return true;
}

void RgbdCandidateReceipt::image(size_t stream,const Image & body)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clocks_ok(get_clock()->now().nanoseconds(),time::monotonic_ns()) ||
      !accept_raw(stream,body.capture)) return;
  if (!ledger_.remember_image(stream,body)) {ledger_.clear(); invalidate("raw_image_invalid");}
}
void RgbdCandidateReceipt::camera_info(size_t stream,const Info & body)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clocks_ok(get_clock()->now().nanoseconds(),time::monotonic_ns()) ||
      !accept_raw(stream+2,body.capture)) return;
  if (!ledger_.remember_info(stream,body)) {ledger_.clear(); invalidate("raw_info_invalid");}
}

bool RgbdCandidateReceipt::originals_current(const Candidate & body)
{
  const std::array<const Header *,4> first{&body.first_rgb,&body.first_depth,
    &body.first_rgb_info,&body.first_depth_info};
  const std::array<const Header *,4> second{&body.second_rgb,&body.second_depth,
    &body.second_rgb_info,&body.second_depth_info};
  const auto begin=stamp_ns(first[0]->stamp), end=stamp_ns(second[0]->stamp);
  if (end <= begin || end-begin > 200000000LL ||
      stamp_ns(body.interval_start)!=begin || stamp_ns(body.interval_end)!=end) return false;
  for (size_t i=0;i<4;++i) {
    for (const auto * h : {first[i],second[i]}) {
      if (!sample_valid(*h) || h->bridge_boot_id!=sources_[i].boot_id() ||
          h->capture_generation!=raw_generation_ || h->frame_id!=camera_) return false;
    }
    if (first[i]->clock_generation!=second[i]->clock_generation ||
        stamp_ns(first[i]->stamp)!=begin || stamp_ns(second[i]->stamp)!=end) return false;
  }
  if (!same_capture_coordinates(body.first_rgb_layout.capture,body.first_rgb) ||
      !same_capture_coordinates(body.first_depth_layout.capture,body.first_depth) ||
      !same_capture_coordinates(body.second_rgb_layout.capture,body.second_rgb) ||
      !same_capture_coordinates(body.second_depth_layout.capture,body.second_depth) ||
      !same_capture_coordinates(body.first_rgb_calibration.capture,body.first_rgb_info) ||
      !same_capture_coordinates(body.first_depth_calibration.capture,body.first_depth_info) ||
      !same_capture_coordinates(body.second_rgb_calibration.capture,body.second_rgb_info) ||
      !same_capture_coordinates(body.second_depth_calibration.capture,body.second_depth_info)) return false;
  return ledger_.match_image(0,body.first_rgb_layout) && ledger_.match_image(1,body.first_depth_layout) &&
    ledger_.match_image(0,body.second_rgb_layout) && ledger_.match_image(1,body.second_depth_layout) &&
    ledger_.match_info(0,body.first_rgb_calibration) && ledger_.match_info(1,body.first_depth_calibration) &&
    ledger_.match_info(0,body.second_rgb_calibration) && ledger_.match_info(1,body.second_depth_calibration);
}

bool RgbdCandidateReceipt::actual_transforms(const Candidate & body)
{
  if (body.first_tf.transforms.size()!=1 || body.second_tf.transforms.size()!=1) return false;
  try {
    const auto first=tf_buffer_->lookupTransform(base_,camera_,rclcpp::Time(body.first_rgb.stamp));
    const auto second=tf_buffer_->lookupTransform(base_,camera_,rclcpp::Time(body.second_rgb.stamp));
    return transform_matches(first,body.first_tf.transforms.front()) &&
      transform_matches(second,body.second_tf.transforms.front());
  } catch (const tf2::TransformException &) {return false;}
}

void RgbdCandidateReceipt::candidate(const Candidate & body,const rclcpp::MessageInfo & info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!clocks_ok(get_clock()->now().nanoseconds(),time::monotonic_ns()) ||
      !graph_matches(candidate_topic_,wiring::node_rgbd_candidate_producer)) return;
  if (!body.valid) {
    if (!candidate_order_.observe_generation(body.producer_boot_id,body.owner_generation)) return;
    last_candidate_=body;
    invalidate(body.reason.empty() ? "candidate_invalid" : body.reason); return;
  }
  last_candidate_=body;
  try {
    if (!candidate_order_.accept_capture(body.producer_boot_id,body.owner_generation,
        stamp_ns(body.interval_end),body.second_rgb.received_steady_ns))
      return;
    if (body.hard_error_qualified || body.profile_id.empty() || body.calibration_id.empty() ||
        body.registration_id.empty() || body.body_frame!=base_ ||
        body.raw_rgb_encoding!="rgb8" || body.preprocessing_version!="rgb8-mono-v1" ||
        body.first_rgb_layout.encoding!="rgb8" || body.second_rgb_layout.encoding!="rgb8" ||
        body.first_depth_layout.encoding!="32FC1" || body.second_depth_layout.encoding!="32FC1" ||
        body.matched>500 || body.inliers>body.matched || !rigid(body.camera_transform) ||
        !std::isfinite(body.median_reprojection_px) || body.median_reprojection_px<0 ||
        !std::isfinite(body.median_depth_residual_m) || body.median_depth_residual_m<0 ||
        !originals_current(body) || !actual_transforms(body)) {
      invalidate("candidate_invalid"); return;
    }
  } catch (const std::invalid_argument &) {invalidate("candidate_timestamp_invalid"); return;}
  Fenced output;
  output.owner.relay_gid=output_gid(); output.owner.relay_boot=relay_boot_;
  output.owner.relay_sequence=++relay_sequence_;
  output.owner.owner_gid=gid_hex(info.get_rmw_message_info().publisher_gid);
  output.owner.owner_epoch=body.producer_boot_id; output.owner.owner_generation=body.owner_generation;
  output.owner.owner_sequence=body.sequence; output.owner.valid=true; output.body=body;
  publisher_->publish(output);
}
}  // namespace senior_sensor_ingress
