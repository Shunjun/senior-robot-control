// Sole actuator transport for the in-memory control owner. No estop state lives here.
#include <array>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

#include "senior_motion_msgs/msg/actuator_target.hpp"
#include "senior_motion_msgs/source_timing.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MotionOutput : public rclcpp::Node {
 public:
  MotionOutput(int fd, const std::string &topic) : Node("motion_output"), fd_(fd) {
    if (topic == "/cmd_vel") throw std::invalid_argument("typed actuator topic must be independent of /cmd_vel");
    publisher_ = create_publisher<senior_motion_msgs::msg::ActuatorTarget>(topic, 1);
    timer_ = create_wall_timer(5ms, [this]() { poll(); });
  }
  ~MotionOutput() override { if (rclcpp::ok()) stop(); ::close(fd_); }
 private:
  // Private same-host packet: vx, wz, original deadline, calculation entry, sequence.
  static constexpr size_t packet_size = 4 * sizeof(double) + sizeof(uint64_t);
  int fd_;
  uint64_t sequence_ = 0, last_produced_ = 0;
  double deadline_ = 0;
  bool moving_ = false, disconnected_ = false;
  senior_motion_msgs::timing::DriverClock producer_;
  rclcpp::Publisher<senior_motion_msgs::msg::ActuatorTarget>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  static uint64_t steady_ns() { return senior_motion_msgs::navigation::monotonic_ns(); }
  void publish(double vx, double wz, uint64_t produced, uint64_t deadline) {
    if (produced < last_produced_) return;
    senior_motion_msgs::msg::ActuatorTarget value;
    value.timing = producer_.sample(produced, produced, "control_compute");
    value.valid_until_steady_ns = deadline;
    value.data.linear.x = vx;
    value.data.angular.z = wz;
    publisher_->publish(value);
    last_produced_ = produced;
    moving_ = vx != 0 || wz != 0;
  }
  void stop() { publish(0, 0, steady_ns(), 0); deadline_ = 0; }
  void disconnect() {
    disconnected_ = true;
    stop();
    const char stopped = 1;
    ::send(fd_, &stopped, 1, MSG_DONTWAIT | MSG_NOSIGNAL);
  }
  void poll() {
    if (disconnected_) { rclcpp::shutdown(); return; }
    bool received = false;
    double vx = 0, wz = 0, candidate_deadline = 0;
    uint64_t produced_ns = 0, deadline_ns = 0;
    for (size_t count = 0; count < 64; ++count) {
      std::array<unsigned char, packet_size + 1> bytes{};
      const auto size = ::recv(fd_, bytes.data(), bytes.size(), MSG_DONTWAIT);
      if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (size != static_cast<ssize_t>(packet_size)) { disconnect(); return; }
      double next_vx, next_wz, deadline, produced;
      uint64_t sequence;
      std::memcpy(&next_vx, bytes.data(), sizeof(double));
      std::memcpy(&next_wz, bytes.data() + sizeof(double), sizeof(double));
      std::memcpy(&deadline, bytes.data() + 2 * sizeof(double), sizeof(double));
      std::memcpy(&produced, bytes.data() + 3 * sizeof(double), sizeof(double));
      std::memcpy(&sequence, bytes.data() + 4 * sizeof(double), sizeof(uint64_t));
      if (!std::isfinite(next_vx) || !std::isfinite(next_wz) || !std::isfinite(deadline) ||
          !std::isfinite(produced) || produced <= 0 || produced > 9e9 || deadline < 0 ||
          deadline > 9e9 || sequence <= sequence_) { disconnect(); return; }
      sequence_ = sequence;
      const auto original_ns = static_cast<uint64_t>(std::llround(produced * 1e9));
      const auto expires_ns = static_cast<uint64_t>(std::llround(deadline * 1e9));
      const auto now = steady_ns();
      if (original_ns > now) { disconnect(); return; }
      if (count == 63) { disconnect(); return; }
      if (original_ns < last_produced_ || (received && original_ns < produced_ns) ||
          ((next_vx != 0 || next_wz != 0) && now >= expires_ns)) continue;
      vx = next_vx; wz = next_wz; produced_ns = original_ns; deadline_ns = expires_ns;
      candidate_deadline = deadline;
      received = true;
    }
    if (received) {
      // Recheck after draining the private queue without renewing production time.
      if ((vx != 0 || wz != 0) && steady_ns() >= deadline_ns) {
        if (moving_ && static_cast<double>(steady_ns()) / 1e9 >= deadline_) stop();
        return;
      }
      deadline_ = candidate_deadline;
      publish(vx, wz, produced_ns, deadline_ns);
    } else if (moving_ && static_cast<double>(steady_ns()) / 1e9 >= deadline_) {
      stop();
    }
  }
};

int main(int argc, char **argv) {
  if (argc != 3 || !argv[2][0]) return 2;
  const int fd = std::atoi(argv[1]);
  if (fd < 0) return 2;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionOutput>(fd, argv[2]);
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
