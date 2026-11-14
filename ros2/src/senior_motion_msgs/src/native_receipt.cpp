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
#include "senior_motion_msgs/msg/scan_motion.hpp"
#include "senior_motion_msgs/msg/drive_execution_feedback.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
static_assert(RMW_GID_STORAGE_SIZE == 24, "SDR1 requires the supported 24-byte RMW GID");
constexpr size_t MAX_PAYLOAD = 20 * 1024 * 1024;
static uint64_t steady_ns() {
  timespec t{}; clock_gettime(CLOCK_MONOTONIC, &t);
  return static_cast<uint64_t>(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}
static void number(std::vector<uint8_t> &v, uint64_t n, size_t bytes) {
  for (size_t i = bytes; i > 0; --i) v.push_back(static_cast<uint8_t>(n >> (8 * (i - 1))));
}

class NativeReceipt : public rclcpp::Node {
 public:
  NativeReceipt(int fd, int argc, char **argv) : Node("native_receipt"), fd_(fd) {
    // Five immutable arguments per input: type, resolved topic, writer name,
    // writer namespace, QoS. One native process serves all this consumer's inputs.
    for (int i = 0; i < argc; i += 5) {
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
      else if (type == "sensor_msgs/msg/Imu") add<sensor_msgs::msg::Imu>(id, qos);
      else if (type == "senior_motion_msgs/msg/ScanMotion") add<senior_motion_msgs::msg::ScanMotion>(id, qos);
      else if (type == "senior_motion_msgs/msg/DriveExecutionFeedback")
        add<senior_motion_msgs::msg::DriveExecutionFeedback>(id, qos);
      else if (type == "std_msgs/msg/String") add<std_msgs::msg::String>(id, qos);
      else if (type == "std_msgs/msg/Bool") add<std_msgs::msg::Bool>(id, qos);
      else if (type == "nav_msgs/msg/OccupancyGrid") add<nav_msgs::msg::OccupancyGrid>(id, qos);
      else if (type == "geometry_msgs/msg/PoseWithCovarianceStamped")
        add<geometry_msgs::msg::PoseWithCovarianceStamped>(id, qos);
      else throw std::invalid_argument("unsupported receipt type");
    }
    timer_ = create_wall_timer(20ms, [this]() {
      for (size_t id = 0; id < inputs_.size(); ++id) {
        topology(id); frame(id, inputs_[id].valid ? 2 : 0, steady_ns(), nullptr, nullptr);
      }
    });
  }
 private:
  struct Input {
    std::string topic, name, ns;
    Gid gid{};
    uint64_t generation = 0, sequence = 0;
    bool valid = false;
  };
  int fd_;
  std::vector<Input> inputs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;

  void write_all(const uint8_t *p, size_t size, uint64_t deadline) {
    while (size) {
      if (steady_ns() >= deadline) _exit(2);
      const auto sent = ::send(fd_, p, size, MSG_DONTWAIT | MSG_NOSIGNAL);
      if (sent > 0) { p += sent; size -= sent; continue; }
      if (sent == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) _exit(2);
      pollfd event{fd_, POLLOUT, 0};
      if (::poll(&event, 1, 5) < 0 && errno != EINTR) _exit(2);
    }
  }
  void frame(size_t id, uint8_t kind, uint64_t received,
             const rmw_message_info_t *info, const rclcpp::SerializedMessage *message) {
    const auto &in = inputs_[id];
    const size_t size = message ? message->size() : 0;
    if (size > MAX_PAYLOAD) _exit(2);
    std::vector<uint8_t> header{'S','D','R','1', kind, static_cast<uint8_t>(id)};
    number(header, in.generation, 8); number(header, in.sequence, 8);
    number(header, received, 8);
    number(header, info ? info->source_timestamp : 0, 8);
    number(header, info ? info->received_timestamp : 0, 8);
    // For data frames this is the actual per-sample MessageInfo GID. For
    // heartbeat/revocation it is only topology state, never delivered as a sample.
    const auto *gid = info ? info->publisher_gid.data : in.gid.data();
    header.insert(header.end(), gid, gid + RMW_GID_STORAGE_SIZE);
    number(header, size, 4);
    const auto deadline = steady_ns() + 100000000ULL;
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
    if (in.generation == 0 || in.valid != good || in.gid != next) {
      in.valid = good; in.gid = next; ++in.generation;
      frame(id, 0, steady_ns(), nullptr, nullptr);  // invalidate before any new sample
    }
    return good;
  }
  static bool bounded(const std_msgs::msg::String &m) { return m.data.size() <= 131072; }
  static bool bounded(const std_msgs::msg::Bool &) { return true; }
  static bool bounded(const nav_msgs::msg::Odometry &m) {
    return m.header.frame_id.size() <= 128 && m.child_frame_id.size() <= 128;
  }
  static bool bounded(const sensor_msgs::msg::Imu &m) { return m.header.frame_id.size() <= 128; }
  static bool bounded(const senior_motion_msgs::msg::ScanMotion &m) {
    return m.header.frame_id.size() <= 128 && m.source_epoch.size() <= 128 && m.reason.size() <= 512;
  }
  static bool bounded(const senior_motion_msgs::msg::DriveExecutionFeedback &m) {
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
  template<class Message> void add(size_t id, const rclcpp::QoS &qos) {
    subscriptions_.push_back(create_subscription<Message>(inputs_[id].topic, qos,
      [this, id](typename Message::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
        const auto received = steady_ns();  // original native callback receipt, before IPC/serialization
        if (!topology(id)) return;
        const auto &native = info.get_rmw_message_info();
        if (!std::equal(inputs_[id].gid.begin(), inputs_[id].gid.end(), native.publisher_gid.data)) return;
        if (!bounded(*msg)) { frame(id, 0, received, nullptr, nullptr); return; }
        rclcpp::SerializedMessage serialized;
        rclcpp::Serialization<Message> serializer;
        serializer.serialize_message(msg.get(), &serialized);
        ++inputs_[id].sequence;
        frame(id, 1, received, &native, &serialized);
      }));
  }
};

int main(int argc, char **argv) {
  if (argc < 8 || (argc - 3) % 5 != 0 || (argc - 3) / 5 > 16) return 2;
  const int fd = std::stoi(argv[1]), parent = std::stoi(argv[2]);
  if (fd < 3 || parent != ::getppid()) return 2;
  // The socket has no filesystem listener; only this exec inherits its endpoint.
  if (::prctl(PR_SET_PDEATHSIG, SIGTERM) != 0 || parent != ::getppid()) return 2;
  try {
    rclcpp::init(0, nullptr);
    rclcpp::spin(std::make_shared<NativeReceipt>(fd, argc - 3, argv + 3));
    rclcpp::shutdown();
  } catch (const std::exception &) { return 2; }
  return 0;
}
