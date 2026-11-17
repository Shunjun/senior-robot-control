// Production receiver against actual Linux packet IO and a recording ROS boundary.
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>
#include <random>
#include <set>
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
  auto send_packet = [&](double vx, double deadline, uint64_t sequence, double produced) {
    std::array<unsigned char, 40> frame{};
    double wz=-0.2;
    memcpy(frame.data(), &vx, 8); memcpy(frame.data()+8, &wz, 8);
    memcpy(frame.data()+16, &deadline, 8); memcpy(frame.data()+24, &produced, 8);
    memcpy(frame.data()+32, &sequence, 8);
    assert(send(pair[0], frame.data(), frame.size(), 0) == 40);
  };
  auto now = [](){return static_cast<double>(MotionOutput::steady_ns())/1e9;};
  auto zero = [&]() {
    assert(!Publication::published.empty()); auto value=Publication::published.back();
    assert(value.linear.x==0 && value.angular.z==0);
  };
  const auto entry=now()-0.01;
  send_packet(0.3, entry+1, 1, entry); receiver.poll();
  const auto & first=Publication::published.back();
  assert(first.linear.x==0.3 && first.angular.z==-0.2);
  const auto scenario=std::string(argv[1]);
  if (scenario=="deadline") {
    receiver.deadline_=now()-0.001; receiver.poll(); zero(); assert(!receiver.disconnected_);
    const auto next=now(); send_packet(0.2, next+1, 2, next); receiver.poll();
    assert(Publication::published.back().linear.x==0.2);
  } else if (scenario=="expired") {
    send_packet(0.4, now()-1, 2, now()); receiver.poll();
    assert(Publication::published.size()==1 && Publication::published.back().linear.x==0.3);
    receiver.deadline_=now()-0.001; receiver.poll(); zero();
  } else if (scenario=="eof") {
    close(pair[0]); pair[0]=-1; receiver.poll(); zero(); assert(receiver.disconnected_);
    receiver.poll(); assert(!rclcpp::ok());
  } else if (scenario=="overflow") {
    for (uint64_t i=2; i<66; ++i) {const auto next=now();send_packet(0.4,next+1,i,next);}
    receiver.poll(); zero(); assert(receiver.disconnected_);
  } else if (scenario=="duplicate") {
    send_packet(0.4, now()+1, 1, now()); receiver.poll(); zero(); assert(receiver.disconnected_);
  } else if (scenario=="malformed") {
    assert(send(pair[0], "bad", 3, 0)==3); receiver.poll(); zero(); assert(receiver.disconnected_);
  } else {assert(false);}
  if(pair[0]>=0) close(pair[0]);
}
