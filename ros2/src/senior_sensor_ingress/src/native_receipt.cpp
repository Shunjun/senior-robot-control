#include <senior_sensor_ingress/sample_time.hpp>
#include <optional>
#include <set>
// Humble-compatible DDS receipts over an inherited, private AF_UNIX channel.
// GID is read from MessageInfo for THIS sample, never synthesized from discovery.
#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "senior_interfaces/msg/scan_motion.hpp"
#include "senior_interfaces/msg/qualified_mapping_scan.hpp"
#include "senior_interfaces/msg/drive_execution_feedback.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
static_assert(RMW_GID_STORAGE_SIZE == 24, "SDR2 requires the supported 24-byte RMW GID");
constexpr size_t MAX_PAYLOAD = 20 * 1024 * 1024;
// Keep these failure codes aligned with the Python parent's failure-only labels.
enum class ExitCode : int {
  write_deadline = 10,
  send_failure = 11,
  poll_failure = 12,
  oversize_payload = 13,
  retired_source_exhaustion = 14,
  invalid_arguments = 15,
  invalid_fd_or_parent = 16,
  parent_death_setup = 17,
  unexpected_exception = 18,
};
static void number(std::vector<uint8_t> &v, uint64_t n, size_t bytes) {
  for (size_t i = bytes; i > 0; --i) v.push_back(static_cast<uint8_t>(n >> (8 * (i - 1))));
}

class NativeReceipt : public rclcpp::Node {
 public:
  NativeReceipt(int fd, const std::vector<std::string> &argv) : Node("native_receipt"), fd_(fd) {
    // Five immutable arguments per input: type, resolved topic, writer name,
    // writer namespace, QoS. One native process serves all this consumer's inputs.
    for (size_t i = 0; i < argv.size(); i += 5) {
      Input in; in.topic = argv[i+1]; in.name = argv[i+2]; in.ns = argv[i+3];
      if (in.topic.empty() || in.topic.size() > 256 || in.topic[0] != '/' ||
          in.name.empty() || in.name.size() > 128 || in.ns.empty() || in.ns[0] != '/')
        throw std::invalid_argument("invalid receipt input");
      inputs_.push_back(in);
      const auto id = inputs_.size() - 1;
      const std::string type(argv[i]), qos_name(argv[i+4]);
      rclcpp::QoS qos(4);
      if (qos_name == "sensor") qos.best_effort();
      else if (qos_name == "latched") qos.keep_last(1).transient_local();
      else if (qos_name != "reliable") throw std::invalid_argument("invalid receipt QoS");
      if (type == "nav_msgs/msg/Odometry") add<nav_msgs::msg::Odometry>(id, qos);
      else if (type == "sensor_msgs/msg/NavSatFix") add<sensor_msgs::msg::NavSatFix>(id, qos);
      else if (type == "sensor_msgs/msg/Imu") add<sensor_msgs::msg::Imu>(id, qos);
      else if (type == "senior_interfaces/msg/ScanMotion") add<senior_interfaces::msg::ScanMotion>(id, qos);
      else if (type == "senior_interfaces/msg/QualifiedMappingScan")
        add<senior_interfaces::msg::QualifiedMappingScan>(id, qos);
      else if (type == "senior_interfaces/msg/DriveExecutionFeedback")
        add<senior_interfaces::msg::DriveExecutionFeedback>(id, qos);
      else if (type == "std_msgs/msg/String") add<std_msgs::msg::String>(id, qos);
      else if (type == "std_msgs/msg/Bool") add<std_msgs::msg::Bool>(id, qos);
      else if (type == "nav_msgs/msg/OccupancyGrid") add<nav_msgs::msg::OccupancyGrid>(id, qos);
      else if (type == "geometry_msgs/msg/PoseWithCovarianceStamped")
        add<geometry_msgs::msg::PoseWithCovarianceStamped>(id, qos);
      else throw std::invalid_argument("unsupported receipt type");
    }
    timer_ = create_wall_timer(20ms, [this]() {
      if (!observe_clock(get_clock()->now().nanoseconds(), senior_sensor_ingress::time::monotonic_ns())) return;
      for (size_t id = 0; id < inputs_.size(); ++id) {
        topology(id); frame(id, inputs_[id].valid ? 2 : 0, senior_sensor_ingress::time::monotonic_ns(), nullptr, nullptr);
      }
    });
  }
 private:
  struct Input {
    std::string topic, name, ns;
    Gid gid{};
    uint64_t generation = 0, sequence = 0;
    bool valid = false;
    std::optional<int64_t> latest_stamp;
    std::set<Gid> retired_gids;
    std::string rejection_reason;
  };
  senior_sensor_ingress::time::ClockTracker clock_;
  uint64_t clock_generation_ = 1;
  int fd_;
  std::vector<Input> inputs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;

  void write_all(const uint8_t *p, size_t size, uint64_t deadline) {
    while (size) {
      if (senior_sensor_ingress::time::monotonic_ns() >= deadline) _exit(static_cast<int>(ExitCode::write_deadline));
      const auto sent = ::send(fd_, p, size, MSG_DONTWAIT | MSG_NOSIGNAL);
      if (sent > 0) { p += sent; size -= sent; continue; }
      if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) _exit(static_cast<int>(ExitCode::send_failure));
      pollfd event{fd_, POLLOUT, 0};
      if (::poll(&event, 1, 5) < 0 && errno != EINTR) _exit(static_cast<int>(ExitCode::poll_failure));
    }
  }
  void frame(size_t id, uint8_t kind, uint64_t received,
             const rmw_message_info_t *info, const rclcpp::SerializedMessage *message, uint8_t time_mode = 0,
             const std::optional<senior_sensor_ingress::time::SampleTime> &sample = std::nullopt) {
    const auto &in = inputs_[id];
    const size_t size = message ? message->size() : 0;
    if (size > MAX_PAYLOAD) _exit(static_cast<int>(ExitCode::oversize_payload));
    std::vector<uint8_t> header{'S','D','R','2', kind, static_cast<uint8_t>(id)};
    number(header, in.generation, 8); number(header, in.sequence, 8);
    number(header, received, 8);
    number(header, info ? info->source_timestamp : 0, 8);
    number(header, info ? info->received_timestamp : 0, 8);
    // For data frames this is the actual per-sample MessageInfo GID. For
    // heartbeat/revocation it is only topology state, never delivered as a sample.
    const auto *gid = info ? info->publisher_gid.data : in.gid.data();
    header.insert(header.end(), gid, gid + RMW_GID_STORAGE_SIZE);
    number(header, time_mode, 1);
    number(header, sample ? sample->stamp_ns : 0, 8);
    number(header, sample ? sample->received_ros_ns : 0, 8);
    number(header, sample ? sample->received_steady_ns : 0, 8);
    number(header, sample ? sample->sample_steady_ns : 0, 8);
    number(header, sample ? sample->clock_generation : 0, 8);
    number(header, size, 4);
    const auto deadline = senior_sensor_ingress::time::monotonic_ns() + 100000000ULL;
    write_all(header.data(), header.size(), deadline);
    if (size) write_all(message->get_rcl_serialized_message().buffer, size, deadline);
  }
  bool topology(size_t id) {
    auto &in = inputs_[id];
    const auto endpoints = get_publishers_info_by_topic(in.topic);
    bool good = endpoints.size() == 1 && endpoints[0].node_name() == in.name &&
                endpoints[0].node_namespace() == in.ns;
    Gid next{};
    if (good) next = endpoints[0].endpoint_gid();
    good = good && std::any_of(next.begin(), next.end(), [](uint8_t x) { return x != 0; });
    if (!good) next.fill(0);
    if (good && in.retired_gids.count(next)) good = false;
    if (!good) next.fill(0);
    if (in.generation == 0 || in.valid != good || in.gid != next) {
      if (in.valid && in.gid != next) in.retired_gids.insert(in.gid);
      if (in.retired_gids.size() >= 256) _exit(static_cast<int>(ExitCode::retired_source_exhaustion));
      in.latest_stamp.reset();
      in.valid = good; in.gid = next; ++in.generation;
      frame(id, 0, senior_sensor_ingress::time::monotonic_ns(), nullptr, nullptr);  // invalidate before any new sample
    }
    return good;
  }
  static bool bounded(const std_msgs::msg::String &m) { return m.data.size() <= 131072; }
  static bool bounded(const std_msgs::msg::Bool &) { return true; }
  static bool bounded(const nav_msgs::msg::Odometry &m) {
    return m.header.frame_id.size() <= 128 && m.child_frame_id.size() <= 128;
  }
  static bool bounded(const sensor_msgs::msg::NavSatFix &m) { return m.header.frame_id.size() <= 128; }
  static bool bounded(const sensor_msgs::msg::Imu &m) { return m.header.frame_id.size() <= 128; }
  static bool bounded(const senior_interfaces::msg::ScanMotion &m) {
    return m.header.frame_id.size() <= 128 && m.source_epoch.size() <= 128 && m.reason.size() <= 512;
  }
  static bool bounded(const senior_interfaces::msg::QualifiedMappingScan &m) {
    return m.scan.ranges.size() <= 2048 && m.scan.intensities.size() <= 2048 &&
      m.static_lidar_tf_receipt.edges.size() <= 32 && m.geometry_id.size() <= 128;
  }
  static bool bounded(const senior_interfaces::msg::DriveExecutionFeedback &m) {
    return m.robot_id.size() <= 128 && m.owner_boot_id.size() <= 128 &&
      m.session_id.size() <= 128 && m.request_id.size() <= 128 &&
      m.status.size() <= 32 && m.reason.size() <= 512;
  }
  static bool bounded(const geometry_msgs::msg::PoseWithCovarianceStamped &m) {
    return m.header.frame_id.size() <= 128;
  }
  static bool bounded(const nav_msgs::msg::OccupancyGrid &m) {
    return m.header.frame_id.size() <= 128 && m.data.size() <= 16777216 &&
      static_cast<uint64_t>(m.info.width) * m.info.height == m.data.size();
  }
  void reject_sample(size_t id, uint64_t received, const std::string &reason) {
    auto &in = inputs_[id];
    if (in.rejection_reason != reason) {
      in.rejection_reason = reason;
      RCLCPP_WARN(get_logger(), "native sample rejected on %s: %s", in.topic.c_str(), reason.c_str());
    }
    frame(id, 0, received, nullptr, nullptr);
  }
  bool observe_clock(int64_t ros, uint64_t steady) {
    senior_sensor_ingress::time::ClockObservation observed;
    try {
      observed = clock_.observe(ros, steady);
    } catch (const std::invalid_argument &error) {
      for (size_t id = 0; id < inputs_.size(); ++id) reject_sample(id, steady, error.what());
      return false;
    }
    clock_generation_ = observed.generation;
    if (!observed.discontinuity) return true;
    for (size_t id = 0; id < inputs_.size(); ++id) {
      inputs_[id].latest_stamp.reset();
      ++inputs_[id].generation;
      frame(id, 0, steady, nullptr, nullptr);
    }
    return true;
  }
  static int64_t stamp_ns(const builtin_interfaces::msg::Time &stamp) {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u)
      throw std::invalid_argument("invalid ROS stamp");
    return int64_t(stamp.sec) * 1000000000LL + stamp.nanosec;
  }
  template<class Message>
  static std::optional<int64_t> raw_stamp(const Message &) { return std::nullopt; }
  static std::optional<int64_t> raw_stamp(const nav_msgs::msg::Odometry &m) { return stamp_ns(m.header.stamp); }
  static std::optional<int64_t> raw_stamp(const sensor_msgs::msg::Imu &m) { return stamp_ns(m.header.stamp); }
  static std::optional<int64_t> raw_stamp(const sensor_msgs::msg::NavSatFix &m) { return stamp_ns(m.header.stamp); }
  static std::optional<int64_t> raw_stamp(const geometry_msgs::msg::PoseWithCovarianceStamped &m) {
    return stamp_ns(m.header.stamp);
  }
  template<class Message>
  static std::optional<senior_sensor_ingress::time::SampleTime> derived_sample(const Message &) {
    return std::nullopt;
  }
  static std::optional<senior_sensor_ingress::time::SampleTime> derived_sample(
      const senior_interfaces::msg::ScanMotion &m) {
    if (!m.valid) return std::nullopt;
    const auto &c = m.second_capture;
    if (!c.valid || c.clock_generation == 0 || c.stamp != m.header.stamp ||
        m.first_capture.stamp != m.interval_start)
      throw std::invalid_argument("invalid derived capture");
    return senior_sensor_ingress::time::SampleTime{stamp_ns(c.stamp), stamp_ns(c.received_ros),
      c.received_steady_ns, c.sample_steady_ns, c.clock_generation};
  }
  static std::optional<senior_sensor_ingress::time::SampleTime> derived_sample(
      const senior_interfaces::msg::QualifiedMappingScan &m) {
    if (m.status != m.QUALIFIED) return std::nullopt;
    const auto &c = m.scan.capture;
    if (!c.valid || c.clock_generation == 0 || !m.static_receipt_valid)
      throw std::invalid_argument("invalid qualified mapping capture");
    return senior_sensor_ingress::time::SampleTime{stamp_ns(c.stamp), stamp_ns(c.received_ros),
      c.received_steady_ns, c.sample_steady_ns, c.clock_generation};
  }
  template<class Message> void add(size_t id, const rclcpp::QoS &qos) {
    subscriptions_.push_back(create_subscription<Message>(inputs_[id].topic, qos,
      [this, id](typename Message::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
        const auto received = senior_sensor_ingress::time::monotonic_ns();  // original native callback receipt, before IPC/serialization
        const auto received_ros = get_clock()->now().nanoseconds();
        if (!observe_clock(received_ros, received)) return;
        if (!topology(id)) return;
        const auto &native = info.get_rmw_message_info();
        if (!std::equal(inputs_[id].gid.begin(), inputs_[id].gid.end(), native.publisher_gid.data)) return;
        if (!bounded(*msg)) { reject_sample(id, received, "raw_dimensions_invalid"); return; }
        std::optional<senior_sensor_ingress::time::SampleTime> sample;
        uint8_t time_mode = 0;
        try {
          const auto stamp = raw_stamp(*msg);
          auto &in = inputs_[id];
          if (stamp) {
            if (in.latest_stamp && *stamp == *in.latest_stamp) return;
            if (in.latest_stamp && *stamp < *in.latest_stamp) {
              reject_sample(id, received, "raw_stamp_out_of_order");
              return;
            }
            sample = senior_sensor_ingress::time::map_sample(*stamp, received_ros, received,
                                                          clock_generation_);
            in.latest_stamp = stamp;
            time_mode = 1;
          } else {
            sample = derived_sample(*msg);
            if (sample) time_mode = 2;
          }
        } catch (const std::invalid_argument &error) {
          reject_sample(id, received, error.what());
          return;
        }
        inputs_[id].rejection_reason.clear();
        rclcpp::SerializedMessage serialized;
        rclcpp::Serialization<Message> serializer;
        serializer.serialize_message(msg.get(), &serialized);
        ++inputs_[id].sequence;
        frame(id, 1, received, &native, &serialized, time_mode, sample);
      }));
  }
};

int main(int argc, char **argv) {
  try {
    // Explicit ROS arguments from the parent select the same ROS clock. Preserve
    // the private positional channel arguments while letting rclcpp parse them.
    rclcpp::init(argc, argv);
    const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
    if (arguments.size() < 8 || (arguments.size() - 3) % 5 != 0 ||
        (arguments.size() - 3) / 5 > 16) return static_cast<int>(ExitCode::invalid_arguments);
    const int fd = std::stoi(arguments[1]), parent = std::stoi(arguments[2]);
    if (fd < 3 || parent != ::getppid()) return static_cast<int>(ExitCode::invalid_fd_or_parent);
    // The socket has no filesystem listener; only this exec inherits its endpoint.
    if (::prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 || parent != ::getppid()) return static_cast<int>(ExitCode::parent_death_setup);
    rclcpp::spin(std::make_shared<NativeReceipt>(fd,
      std::vector<std::string>(arguments.begin() + 3, arguments.end())));
    rclcpp::shutdown();
  } catch (const std::exception &) { return static_cast<int>(ExitCode::unexpected_exception); }
  return 0;
}
