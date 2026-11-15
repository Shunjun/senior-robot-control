// Run the production native receiver against a recording ROS publication boundary.
#include <array>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <cassert>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
#define private public
#define main production_main
#include "motion_output_guard.cpp"
#undef main
#undef private

using Publication = rclcpp::Publisher<geometry_msgs::msg::Twist>;
int main(int argc, char** argv) {
  assert(argc == 2);
  int pair[2];
  assert(socketpair(AF_UNIX, SOCK_SEQPACKET, 0, pair) == 0);
  MotionOutput receiver(pair[1], "/cmd_vel");
  auto send_packet = [&](double vx, double deadline, uint64_t sequence) {
    std::array<unsigned char, 32> frame{};
    double wz=-0.2;
    memcpy(frame.data(), &vx, 8);
    memcpy(frame.data()+8, &wz, 8);
    memcpy(frame.data()+16, &deadline, 8);
    memcpy(frame.data()+24, &sequence, 8);
    assert(send(pair[0], frame.data(), frame.size(), 0) == 32);
  };
  auto zero = [&]() {
    assert(!Publication::published.empty());
    auto value=Publication::published.back();
    assert(value.linear.x==0 && value.angular.z==0);
  };
  auto scenario=std::string(argv[1]);
  send_packet(0.3, MotionOutput::steady()+1, 1);
  receiver.poll();
  assert(Publication::published.back().linear.x==0.3);
  assert(Publication::published.back().angular.z==-0.2);
  if (scenario=="deadline") {
    receiver.deadline_=MotionOutput::steady()-0.001;
    receiver.poll(); zero();
    assert(!receiver.disconnected_);
    // Deadline expiry is a transport stop, never a second estop latch.
    send_packet(0.2, MotionOutput::steady()+1, 2);
    receiver.poll();
    assert(Publication::published.back().linear.x==0.2);
  } else if (scenario=="expired") {
    send_packet(0.4, MotionOutput::steady()-1, 2);
    receiver.poll(); zero();
  } else if (scenario=="eof") {
    close(pair[0]); pair[0]=-1;
    receiver.poll(); zero();
    assert(receiver.disconnected_);
    receiver.poll(); assert(!rclcpp::ok());
  } else if (scenario=="overflow") {
    for (uint64_t i=2; i<66; ++i) send_packet(0.4, MotionOutput::steady()+1, i);
    receiver.poll(); zero(); assert(receiver.disconnected_);
  } else if (scenario=="duplicate") {
    send_packet(0.4, MotionOutput::steady()+1, 1);
    receiver.poll(); zero(); assert(receiver.disconnected_);
  } else if (scenario=="malformed") {
    assert(send(pair[0], "bad", 3, 0)==3);
    receiver.poll(); zero(); assert(receiver.disconnected_);
  } else { assert(false); }
  if(pair[0]>=0) close(pair[0]);
}
