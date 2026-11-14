#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "final_command_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "senior_motion_msgs/msg/final_command.hpp"

using Command = senior_motion_msgs::msg::FinalCommand;
using namespace std::chrono_literals;

// Private seqpacket framing: type, actual GID, four bounded UTF-8 strings,
// generation/epoch/sequence/gesture and source steady deadline/vx/wz, all network byte order.
class Guard : public rclcpp::Node {
 public:
  Guard(int fd, const std::string &robot, const std::string &topic)
      : Node("final_command_guard"), fd_(fd), robot_(robot), topic_(topic) {
    subscription_ = create_subscription<Command>(topic_, 10,
        [this](Command::ConstSharedPtr msg, const rclcpp::MessageInfo &info) { receive(*msg, info); });
    timer_ = create_wall_timer(50ms, [this]() { heartbeat(); });
  }
 private:
  using Gid = std::array<uint8_t, RMW_GID_STORAGE_SIZE>;
  int fd_;
  std::string robot_, topic_, boot_, session_, request_;
  Gid gid_{};
  bool valid_gid_ = false, invalid_boot_ = false;
  uint64_t generation_ = 0, sequence_ = 0, epoch_ = 0, gesture_ = 0;
  double deadline_ = 0, vx_ = 0, wz_ = 0;
  rclcpp::Subscription<Command>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  static double steady() { timespec t{}; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
  Command last_{};
  void send(uint8_t type, const Command *m = nullptr) {
    const Command empty;
    std::vector<uint8_t> data;
    try {
      data = final_command_frame::encode(type, gid_, m ? *m : empty);
    } catch (const std::exception &) {
      _exit(2);
    }
    if (::send(fd_, data.data(), data.size(), MSG_DONTWAIT | MSG_NOSIGNAL) != static_cast<ssize_t>(data.size())) {
      _exit(2);  // Overflow or broken channel revokes parent; no reliable backlog.
    }
  }
  bool topology() {
    const auto endpoints = get_publishers_info_by_topic(topic_);
    const bool good = endpoints.size() == 1 && endpoints[0].node_name() == "command_node" &&
                      endpoints[0].node_namespace() == "/";
    Gid next{};
    if (good) next = endpoints[0].endpoint_gid();
    const bool changed = good != valid_gid_ || (good && next != gid_);
    if (changed) {
      valid_gid_ = good; gid_ = next; boot_.clear(); session_.clear(); epoch_ = 0; invalid_boot_ = false;
      generation_ = sequence_ = 0; deadline_ = vx_ = wz_ = 0;
      send(0);
    }
    return good;
  }
  void heartbeat() { topology(); send(valid_gid_ && !invalid_boot_ ? 2 : 0); }
  void receive(const Command &m, const rclcpp::MessageInfo &info) {
    if (!topology()) return;
    const auto &source = info.get_rmw_message_info().publisher_gid;
    if (!std::equal(gid_.begin(), gid_.end(), source.data)) return;
    const bool is_barrier = final_command_frame::barrier(m);
    if ((m.source_kind != "manual" && m.source_kind != "navigation" && m.source_kind != "idle") ||
        m.source_id.size() > 128 || m.close_reason.size() > 32 ||
        !std::isfinite(m.received_steady) || m.received_steady < 0 || m.received_steady > steady() ||
        (!is_barrier && (m.source_kind == "idle" || m.close_reason != "intent" || m.source_id.empty())) ||
        m.robot_id.size()>128 || m.robot_id.find('\0')!=std::string::npos ||
        m.owner_boot_id.find('\0')!=std::string::npos || m.session_id.find('\0')!=std::string::npos ||
        m.request_id.find('\0')!=std::string::npos || m.source_id.find('\0')!=std::string::npos ||
        m.robot_id != robot_ || m.owner_boot_id.empty() || m.owner_boot_id.size() > 128 ||
        m.session_id.size() > 128 || m.request_id.size() > 128 ||
        (!is_barrier && (m.gesture == 0) != m.request_id.empty()) || !std::isfinite(m.steady_deadline) ||
        !std::isfinite(m.vx) || !std::isfinite(m.wz) || invalid_boot_) return;
    if (!boot_.empty() && boot_ != m.owner_boot_id) { invalid_boot_ = true; send(0); return; }
    const bool zero = m.vx == 0 && m.wz == 0;
    if (boot_.empty() || m.generation > generation_) {
      // A newer zero barrier supersedes any intermediate barriers missed by DDS.
      if (!is_barrier) return;
      boot_ = m.owner_boot_id; generation_ = m.generation; sequence_ = 0;
      session_.clear(); request_.clear(); epoch_ = gesture_ = 0;
      deadline_ = m.steady_deadline; vx_ = m.vx; wz_ = m.wz;
      last_ = m;
      send(1, &m);
      return;
    }
    if (m.generation != generation_ || m.sequence < sequence_) return;
    if (!session_.empty() && (session_ != m.session_id || epoch_ != m.connection_epoch)) return;
    if (m.sequence == sequence_) {
      if (final_command_frame::same_payload(m, last_)) send(1, &m);
      return;
    }
    if (m.steady_deadline <= steady() || (!zero && (m.session_id.empty() || m.connection_epoch == 0))) return;
    sequence_ = m.sequence; deadline_ = m.steady_deadline; vx_ = m.vx; wz_ = m.wz;
    session_ = m.session_id; epoch_ = m.connection_epoch;
    request_ = m.request_id; gesture_ = m.gesture;
    last_ = m;
    send(1, &m);
  }
};

int main(int argc, char **argv) {
  if (argc != 4) return 2;
  const int fd = std::atoi(argv[1]);
  if (fd < 3 || std::strlen(argv[2]) == 0 || std::strlen(argv[2]) > 128 || std::strlen(argv[3]) == 0) return 2;
  rclcpp::init(0, nullptr);
  rclcpp::spin(std::make_shared<Guard>(fd, argv[2], argv[3]));
  rclcpp::shutdown();
  return 0;
}
