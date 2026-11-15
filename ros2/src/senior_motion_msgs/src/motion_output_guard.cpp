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
#include <time.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MotionOutput : public rclcpp::Node {
 public:
  MotionOutput(int fd, const std::string &topic) : Node("motion_output"), fd_(fd) {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(topic, 1);
    timer_ = create_wall_timer(5ms, [this]() { poll(); });
  }
  ~MotionOutput() override { if (rclcpp::ok()) publish(0, 0); ::close(fd_); }
 private:
  // Private same-host packet: vx, wz, original evidence deadline, monotonic sequence.
  static constexpr size_t packet_size = 3 * sizeof(double) + sizeof(uint64_t);
  int fd_;
  uint64_t sequence_ = 0;
  double deadline_ = 0;
  bool moving_ = false, disconnected_ = false;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  static double steady() {
    timespec value{};
    clock_gettime(CLOCK_MONOTONIC, &value);
    return value.tv_sec + value.tv_nsec * 1e-9;
  }
  void publish(double vx, double wz) {
    geometry_msgs::msg::Twist value;
    value.linear.x = vx;
    value.angular.z = wz;
    publisher_->publish(value);
    moving_ = vx != 0 || wz != 0;
  }
  void disconnect() {
    disconnected_ = true;
    publish(0, 0);
    deadline_ = 0;
    const char stopped = 1;
    ::send(fd_, &stopped, 1, MSG_DONTWAIT | MSG_NOSIGNAL);
  }
  void poll() {
    if (disconnected_) { rclcpp::shutdown(); return; }
    bool received = false;
    double vx = 0, wz = 0;
    for (size_t count = 0; count < 64; ++count) {
      std::array<unsigned char, packet_size + 1> bytes{};
      const auto size = ::recv(fd_, bytes.data(), bytes.size(), MSG_DONTWAIT);
      if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
      if (size != static_cast<ssize_t>(packet_size)) { disconnect(); return; }
      double deadline;
      uint64_t sequence;
      std::memcpy(&vx, bytes.data(), sizeof(double));
      std::memcpy(&wz, bytes.data() + sizeof(double), sizeof(double));
      std::memcpy(&deadline, bytes.data() + 2 * sizeof(double), sizeof(double));
      std::memcpy(&sequence, bytes.data() + 3 * sizeof(double), sizeof(uint64_t));
      if (!std::isfinite(vx) || !std::isfinite(wz) || !std::isfinite(deadline) ||
          sequence <= sequence_) { disconnect(); return; }
      sequence_ = sequence;
      deadline_ = deadline;
      received = true;
      if (count == 63) { disconnect(); return; }
    }
    if (received) {
      if ((vx != 0 || wz != 0) && steady() >= deadline_) { vx = wz = 0; }
      publish(vx, wz);
    } else if (moving_ && steady() >= deadline_) {
      publish(0, 0);
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
