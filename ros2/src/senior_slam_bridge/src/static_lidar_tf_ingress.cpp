#include "static_lidar_tf_ingress.hpp"

#include <iterator>
#include <stdexcept>
#include <utility>

namespace senior_slam_bridge
{
namespace
{
using namespace std::chrono_literals;
using Receipt = senior_motion_msgs::msg::BoundedStaticTf;

template<typename Bytes>
std::string gid_hex(const Bytes & bytes)
{
  constexpr char hex[] = "0123456789abcdef";
  if (std::all_of(std::begin(bytes), std::end(bytes), [](auto byte) {return byte == 0;})) {
    return {};
  }
  std::string value;
  for (const auto byte : bytes) {
    value += hex[byte >> 4];
    value += hex[byte & 0x0f];
  }
  return value;
}

std::int64_t time_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000) {
    return -1;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1000000000 + stamp.nanosec;
}

bool sole_owner(
  rclcpp::Node & node, const std::string & topic, const std::string & full_name,
  const std::string & type, const std::string & gid)
{
  const auto publishers = node.get_publishers_info_by_topic(topic);
  if (publishers.size() != 1 || gid.empty()) {
    return false;
  }
  const auto & owner = publishers.front();
  const auto & name_space = owner.node_namespace();
  return (name_space == "/" ? name_space : name_space + "/") + owner.node_name() == full_name &&
         owner.topic_type() == type && gid_hex(owner.endpoint_gid()) == gid;
}

bool valid_times(const Receipt & receipt, std::int64_t now_ros, double now_steady)
{
  const auto first = time_ns(receipt.first_received_ros);
  const auto latest = time_ns(receipt.received_ros);
  const auto admitted = time_ns(receipt.admitted_from_ros);
  return receipt.first_sequence > 0 && receipt.sequence >= receipt.first_sequence &&
         first >= 0 && latest >= first && latest <= now_ros &&
         admitted > 0 && admitted >= first && admitted <= now_ros &&
         std::isfinite(receipt.first_received_steady) && receipt.first_received_steady > 0 &&
         std::isfinite(receipt.received_steady) &&
         receipt.received_steady >= receipt.first_received_steady &&
         receipt.received_steady <= now_steady &&
         std::isfinite(receipt.admitted_from_steady) &&
         receipt.admitted_from_steady >= receipt.first_received_steady &&
         receipt.admitted_from_steady <= now_steady &&
         (receipt.sequence != receipt.first_sequence ||
         (first == latest && receipt.first_received_steady == receipt.received_steady));
}

bool same_writer(const Receipt & first, const Receipt & latest)
{
  return first.bridge_boot_id == latest.bridge_boot_id &&
         first.bridge_gid == latest.bridge_gid && first.raw_gid == latest.raw_gid;
}

bool same_static_body(const Receipt & first, const Receipt & latest)
{
  return same_writer(first, latest) && first.generation == latest.generation &&
         first.first_sequence == latest.first_sequence &&
         first.first_received_ros == latest.first_received_ros &&
         first.first_received_steady == latest.first_received_steady && first.edges == latest.edges;
}

bool same_lifetime(const Receipt & first, const Receipt & latest)
{
  return same_static_body(first, latest) &&
         first.admitted_from_ros == latest.admitted_from_ros &&
         first.admitted_from_steady == latest.admitted_from_steady;
}
}  // namespace

StaticLidarTfIngress::StaticLidarTfIngress(
  rclcpp::Node & node, std::string topic, std::string relay_node,
  std::string raw_topic, std::string raw_node, std::string base, std::string lidar)
: node_(node), topic_(std::move(topic)), relay_node_(std::move(relay_node)),
  raw_topic_(std::move(raw_topic)), raw_node_(std::move(raw_node)), base_(std::move(base)),
  lidar_(std::move(lidar))
{
  if (topic_.empty() || raw_topic_.empty() || topic_ == raw_topic_ ||
    relay_node_.size() < 2 || relay_node_.front() != '/' ||
    raw_node_.size() < 2 || raw_node_.front() != '/' || relay_node_ == raw_node_ ||
    base_.empty() || lidar_.empty() || base_ == lidar_)
  {
    throw std::invalid_argument("Invalid verified static lidar TF configuration");
  }
  subscription_ = node_.create_subscription<Receipt>(
    topic_, rclcpp::QoS(1).reliable().transient_local(),
    [this](Receipt::ConstSharedPtr receipt, const rclcpp::MessageInfo & info) {
      offer(std::move(receipt), info);
    });
  timer_ = node_.create_wall_timer(20ms, [this] {poll();});
}

bool StaticLidarTfIngress::current_owners(const Receipt & receipt) const
{
  return sole_owner(node_, topic_, relay_node_, "senior_motion_msgs/msg/BoundedStaticTf",
           receipt.bridge_gid) &&
         sole_owner(node_, raw_topic_, raw_node_, "tf2_msgs/msg/TFMessage", receipt.raw_gid);
}

void StaticLidarTfIngress::retire()
{
  retired_ = true;
  active_.reset();
  pending_.reset();
  startup_cursor_.reset();
}

bool StaticLidarTfIngress::source_active()
{
  if (retired_) {
    return false;
  }
  const auto now = node_.now().nanoseconds();
  if (now < last_ros_ || (active_ && !current_owners(*active_))) {
    retire();
    return false;
  }
  last_ros_ = now;
  return active_ && !pending_ && now > 0;
}

void StaticLidarTfIngress::offer(Receipt::ConstSharedPtr receipt, const rclcpp::MessageInfo & info)
{
  source_active();
  const auto actual = gid_hex(info.get_rmw_message_info().publisher_gid.data);
  if (retired_ || actual.empty() || actual != receipt->bridge_gid ||
    (active_ && actual != active_->bridge_gid))
  {
    return;
  }
  if (!active_ && pending_) {
    if (same_writer(*pending_, *receipt)) {
      if (!replaces_pending_static_lidar(
          {pending_->generation, pending_->sequence, pending_->valid, pending_->admitted},
          {receipt->generation, receipt->sequence, receipt->valid, receipt->admitted}))
      {
        return;
      }
      if (receipt->valid && receipt->generation == pending_->generation &&
        !same_static_body(*pending_, *receipt))
      {
        return;
      }
    } else if (current_owners(*pending_) && !current_owners(*receipt)) {
      return;
    }
  }
  pending_ = std::move(receipt);
  pending_deadline_ = std::chrono::steady_clock::now() + 500ms;
  poll();
}

void StaticLidarTfIngress::poll()
{
  source_active();
  if (!pending_) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now >= pending_deadline_) {
    if (active_) {
      retire();
      return;
    }
    pending_.reset();
    return;
  }

  // A callback can precede its graph discovery. Only the pinned actual relay may
  // invalidate a live cut, including when its invalidation carries the old raw GID.
  if (!active_ && !current_owners(*pending_)) {
    return;
  }
  if (!active_ && startup_cursor_ && same_writer(*startup_cursor_, *pending_)) {
    const auto & previous = *startup_cursor_;
    if (pending_->generation < previous.generation ||
      (pending_->generation == previous.generation &&
      (!previous.valid || pending_->sequence < previous.sequence ||
      (pending_->valid && !same_static_body(previous, *pending_)))))
    {
      pending_.reset();
      return;
    }
  }
  if (active_ && pending_->bridge_boot_id == active_->bridge_boot_id &&
    (pending_->generation < active_->generation ||
    (pending_->generation == active_->generation && pending_->sequence < active_->sequence)))
  {
    pending_.reset();
    return;
  }
  if (active_ && (!pending_->valid || !pending_->admitted ||
    !same_lifetime(*active_, *pending_)))
  {
    retire();
    return;
  }
  if (pending_->valid && pending_->admitted &&
    (time_ns(pending_->received_ros) > last_ros_ ||
    time_ns(pending_->admitted_from_ros) > last_ros_))
  {
    // The relay may observe a /clock update before this executor does.
    return;
  }
  const auto candidate = std::move(pending_);
  if (!active_ && candidate->generation > 0 && candidate->sequence > 0 &&
    !candidate->bridge_boot_id.empty() && candidate->bridge_boot_id.size() <= 64 &&
    (!candidate->valid || !candidate->admitted))
  {
    // An authenticated invalid fence closes its generation even if this graph
    // has never admitted a scan. Unadmitted valid receipts may still be promoted.
    startup_cursor_ = candidate;
  }
  const double now_steady = std::chrono::duration<double>(now.time_since_epoch()).count();
  if (!candidate->valid || !candidate->admitted || candidate->generation == 0 ||
    candidate->bridge_boot_id.empty() || candidate->bridge_boot_id.size() > 64 ||
    !valid_times(*candidate, last_ros_, now_steady))
  {
    if (active_) {
      retire();
    }
    return;
  }

  if (active_) {
    const auto old_time = time_ns(active_->received_ros);
    const auto new_time = time_ns(candidate->received_ros);
    if (new_time < old_time ||
      candidate->received_steady < active_->received_steady ||
      (candidate->sequence == active_->sequence &&
      (new_time != old_time || candidate->received_steady != active_->received_steady)))
    {
      retire();
      return;
    }
  } else {
    std::vector<StaticLidarEdge> edges;
    edges.reserve(candidate->edges.size());
    for (const auto & edge : candidate->edges) {
      if (time_ns(edge.stamp) < 0) {
        return;
      }
      const auto & t = edge.translation;
      const auto & q = edge.rotation_xyzw;
      edges.push_back(
        {edge.parent_frame, edge.child_frame, {t[0], t[1], t[2], q[0], q[1], q[2], q[3]}});
    }
    const auto mount = compose_static_lidar_chain(edges, base_, lidar_);
    if (!mount) {
      return;
    }
    mount_ = *mount;
  }
  active_ = candidate;
  startup_cursor_.reset();
}

bool StaticLidarTfIngress::admits(const senior_motion_msgs::msg::RawCaptureHeader & scan)
{
  if (!source_active()) {
    return false;
  }
  const auto admitted = time_ns(active_->admitted_from_ros);
  // The producer and scan ingress share the host monotonic clock. A latched
  // static receipt is a fence, so later scans do not renew its admission.
  return scan.frame_id == lidar_ && time_ns(scan.stamp) >= admitted &&
         time_ns(scan.received_ros) >= admitted &&
         std::isfinite(scan.received_steady) && scan.received_steady >= active_->admitted_from_steady;
}

StaticLidarTfIngress::Receipt::ConstSharedPtr StaticLidarTfIngress::verify_mount(
  const senior_motion_msgs::msg::RawCaptureHeader & scan, LaserTransform actual)
{
  if (!admits(scan)) {
    return nullptr;
  }
  if (!matches_static_lidar_mount(mount_, actual)) {
    retire();
    return nullptr;
  }
  return active_;
}
}  // namespace senior_slam_bridge
