#include <senior_sensor_ingress/ros_wiring.hpp>
#include <senior_sensor_ingress/publisher_topology.hpp>
#include "bounded_scan_ingress.hpp"
#include "scan_receipt_admission.hpp"

#include <chrono>
#include <utility>

namespace senior_mapping {
namespace {
constexpr std::size_t pending_limit = 8;  // Queue capacity policy is unchanged.
}

BoundedScanIngress::BoundedScanIngress(rclcpp::Node & node, tf2_ros::Buffer & tf,
  std::string bounded_topic, std::string odom_frame, std::string base_frame, Submit submit,
  rclcpp::CallbackGroup::SharedPtr group)
: node_(node), tf_(tf), bounded_topic_(std::move(bounded_topic)),
  odom_frame_(std::move(odom_frame)), base_frame_(std::move(base_frame)),
  expected_name_(node.declare_parameter<std::string>("scan_bridge_node", senior_sensor_ingress::wiring::node_scan_receipt)),
  expected_namespace_(node.declare_parameter<std::string>("scan_bridge_namespace", senior_sensor_ingress::wiring::ns_scan_receipt)),
  submit_(std::move(submit))
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  subscription_ = node_.create_subscription<BoundedScan>(
    bounded_topic_, rclcpp::SensorDataQoS().keep_last(pending_limit),
    [this](BoundedScan::ConstSharedPtr scan) {offer(std::move(scan));}, options);
  timer_ = node_.create_wall_timer(std::chrono::milliseconds(20), [this] {poll();}, group);
}

void BoundedScanIngress::offer(BoundedScan::ConstSharedPtr scan)
{
  if (closed_.load()) return;
  const auto publishers = node_.get_publishers_info_by_topic(bounded_topic_);
  if (!senior_sensor_ingress::publisher_matches(publishers, expected_name_, expected_namespace_,
      "senior_interfaces/msg/BoundedRawLaserScan")) {
    active_.store(false);
    return;
  }
  const auto & capture = scan->capture;
  const auto previous_boot = order_.boot_id();
  const auto previous_generation = order_.generation();
  if (!capture.valid) {
    if (!order_.observe_generation(capture.bridge_boot_id, capture.capture_generation)) return;
    if (order_.generation() != previous_generation) pending_.clear();
    active_.store(false); return;
  }
  if (capture.frame_id.empty() || !scan_receipt_admission::sample_valid(capture) ||
      !scan_receipt_admission::layout_ok(*scan)) return;
  if (!order_.accept_capture(capture.bridge_boot_id, capture.capture_generation,
      scan_receipt_admission::time_ns(capture.stamp), capture.received_steady_ns)) return;
  if (order_.boot_id() != previous_boot || order_.generation() != previous_generation)
    pending_.clear();
  if (pending_.size() < pending_limit) pending_.push_back(std::move(scan));
}

void BoundedScanIngress::poll()
{
  if (closed_.load()) {pending_.clear(); return;}
  while (!pending_.empty()) {
    const auto input = std::move(pending_.front());
    pending_.pop_front();
    const auto & capture = input->capture;
    const auto stamp = rclcpp::Time(capture.stamp, node_.get_clock()->get_clock_type());
    LaserTransform mount;
    try {
      // A missing sample drops this scan only. The next scan can recover.
      tf_.lookupTransform(odom_frame_, base_frame_, stamp);
      const auto exact = tf_.lookupTransform(base_frame_, capture.frame_id, stamp).transform;
      mount = {exact.translation.x, exact.translation.y, exact.translation.z,
               exact.rotation.x, exact.rotation.y, exact.rotation.z, exact.rotation.w};
    } catch (const tf2::TransformException & error) {
      active_.store(false);
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 1000,
        "Dropping scan without capture-time TF: %s", error.what());
      continue;
    }
    auto scan = std::make_shared<Scan>();
    scan->header.stamp = capture.stamp;
    scan->header.frame_id = capture.frame_id;
    scan->angle_min = input->angle_min;
    scan->angle_max = input->angle_max;
    scan->angle_increment = input->angle_increment;
    scan->time_increment = input->time_increment;
    scan->scan_time = input->scan_time;
    scan->range_min = input->range_min;
    scan->range_max = input->range_max;
    scan->ranges.assign(input->ranges.begin(), input->ranges.end());
    scan->intensities.assign(input->intensities.begin(), input->intensities.end());
    active_.store(true);
    submit_(std::move(scan), input, mount);
  }
}
}  // namespace senior_mapping
