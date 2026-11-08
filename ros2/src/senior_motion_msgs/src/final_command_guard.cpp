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
  static void put64(std::vector<uint8_t> &out, uint64_t n) {
    for (int i = 7; i >= 0; --i) out.push_back(static_cast<uint8_t>(n >> (8 * i)));
  }
  static void putdouble(std::vector<uint8_t> &out, double value) {
    uint64_t bits; std::memcpy(&bits, &value, sizeof(bits)); put64(out, bits);
  }
  static void putstring(std::vector<uint8_t> &out, const std::string &value) {
    std::array<uint8_t, 129> result{};
    std::copy(value.begin(), value.end(), result.begin());
    out.insert(out.end(), result.begin(), result.end());
  }
  void send(uint8_t type, const Command *m = nullptr) {
    std::vector<uint8_t> data;
    data.reserve(600); data.push_back(type); data.insert(data.end(), gid_.begin(), gid_.end());
    const Command empty;
    const auto &x = m ? *m : empty;
    putstring(data, x.robot_id); putstring(data, x.owner_boot_id); putstring(data, x.session_id);
    putstring(data, x.request_id);
    put64(data, x.generation); put64(data, x.connection_epoch); put64(data, x.sequence); put64(data, x.gesture);
    putdouble(data, x.steady_deadline); putdouble(data, x.vx); putdouble(data, x.wz);
    if (::send(fd_, data.data(), data.size(), MSG_DONTWAIT | MSG_NOSIGNAL) != static_cast<ssize_t>(data.size())) {
      _exit(2);  // Overflow or broken private channel revokes the parent immediately.
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
    if (m.robot_id != robot_ || m.owner_boot_id.empty() || m.owner_boot_id.size() > 128 ||
        m.session_id.size() > 128 || m.request_id.size() > 128 ||
        (m.gesture == 0) != m.request_id.empty() || !std::isfinite(m.steady_deadline) ||
        !std::isfinite(m.vx) || !std::isfinite(m.wz) || invalid_boot_) return;
    if (!boot_.empty() && boot_ != m.owner_boot_id) { invalid_boot_ = true; send(0); return; }
    const bool zero = m.vx == 0 && m.wz == 0;
    if (boot_.empty() || m.generation > generation_) {
      // A newer zero barrier supersedes any intermediate barriers missed by DDS.
      if (!zero || m.sequence != 0 || m.gesture != 0) return;
      boot_ = m.owner_boot_id; generation_ = m.generation; sequence_ = 0;
      session_.clear(); request_.clear(); epoch_ = gesture_ = 0;
      deadline_ = m.steady_deadline; vx_ = m.vx; wz_ = m.wz; send(1, &m); return;
    }
    if (m.generation != generation_ || m.sequence < sequence_) return;
    if (!session_.empty() && (session_ != m.session_id || epoch_ != m.connection_epoch)) return;
    if (m.sequence == sequence_) {
      if (m.steady_deadline == deadline_ && m.vx == vx_ && m.wz == wz_ &&
          m.request_id == request_ && m.gesture == gesture_) send(1, &m);
      return;
    }
    if (m.steady_deadline <= steady() || (!zero && (m.session_id.empty() || m.connection_epoch == 0))) return;
    sequence_ = m.sequence; deadline_ = m.steady_deadline; vx_ = m.vx; wz_ = m.wz;
    session_ = m.session_id; epoch_ = m.connection_epoch;
    request_ = m.request_id; gesture_ = m.gesture;
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
