// Deterministic IO/clock model around the exact production C++ receiver.
// This does not exercise Linux socket semantics, ROS, DDS or target ABI.
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
#include <deque>
#include <cassert>
#include <sys/socket.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
static uint64_t clock_ns = 10'000'000'000ULL;
#define SENIOR_MOTION_MSGS__NAVIGATION_TIMING_HPP_
namespace senior_motion_msgs::navigation {
inline uint64_t monotonic_ns() {return clock_ns;}
inline std::string boot_domain() {return "00000000-0000-0000-0000-000000000001";}
}
#include "senior_motion_msgs/source_timing.hpp"
#include "senior_motion_msgs/msg/actuator_target.hpp"
static std::deque<std::vector<unsigned char>> packets;
static bool eof = false;
static ssize_t model_recv(int, void * data, size_t size, int) {
  if (packets.empty()) {if (eof) return 0; errno=EAGAIN; return -1;}
  auto packet=packets.front(); packets.pop_front();
  const auto n=std::min(size,packet.size()); std::memcpy(data,packet.data(),n); return n;
}
static ssize_t model_send(int, const void *, size_t size, int) {return size;}
static int model_close(int) {return 0;}
#define recv model_recv
#define send model_send
#define close model_close
#define private public
#define main production_main
#include "motion_output_guard.cpp"
#undef main
#undef private
#undef recv
#undef send
#undef close
using Target=senior_motion_msgs::msg::ActuatorTarget;
using Publication=rclcpp::Publisher<Target>;
static void frame(double vx, double wz, double deadline, double produced, uint64_t sequence) {
  std::vector<unsigned char> bytes(40);
  std::memcpy(bytes.data(),&vx,8);std::memcpy(bytes.data()+8,&wz,8);
  std::memcpy(bytes.data()+16,&deadline,8);std::memcpy(bytes.data()+24,&produced,8);
  std::memcpy(bytes.data()+32,&sequence,8);packets.push_back(bytes);
}
int main(int argc,char **argv) {
  assert(argc==2);const std::string scenario=argv[1];
  MotionOutput receiver(42,"/control/actuator_target");
  frame(-0.4,0.7,10.5,9.9,1);receiver.poll();
  assert(Publication::published.size()==1);
  auto initial=Publication::published.back();
  assert(initial.data.linear.x==-0.4 && initial.data.angular.z==0.7);
  assert(initial.timing.capture_start_steady_ns==9'900'000'000ULL);
  assert(initial.valid_until_steady_ns==10'500'000'000ULL);
  if(scenario=="expired_queue") {
    frame(0.8,0.1,9.99,9.95,2);receiver.poll();
    assert(Publication::published.size()==1 && receiver.deadline_==10.5);
    clock_ns=10'500'000'000ULL;receiver.poll();
    assert(Publication::published.back().data.linear.x==0);
  } else if(scenario=="original_order") {
    frame(0.2,0.3,10.4,9.99,2);
    frame(0.9,0.9,10.5,9.98,3);receiver.poll();
    assert(Publication::published.back().data.linear.x==0.2);
    assert(Publication::published.back().timing.capture_start_steady_ns==9'990'000'000ULL);
  } else if(scenario=="zero") {
    frame(0,0,0,10,2);receiver.poll();
    assert(Publication::published.back().data.linear.x==0);
    assert(Publication::published.back().timing.capture_start_steady_ns==clock_ns);
  } else if(scenario=="future") {
    frame(0.2,0.3,11,10.1,2);receiver.poll();assert(receiver.disconnected_);
    assert(Publication::published.back().data.linear.x==0);
  } else if(scenario=="nan") {
    frame(NAN,0.3,11,10,2);receiver.poll();assert(receiver.disconnected_);
    assert(Publication::published.back().data.linear.x==0);
  } else if(scenario=="replay") {
    frame(0.2,0.3,11,10,1);receiver.poll();assert(receiver.disconnected_);
    assert(Publication::published.back().data.linear.x==0);
  } else if(scenario=="eof") {
    eof=true;receiver.poll();assert(receiver.disconnected_);
    assert(Publication::published.back().data.linear.x==0);
  } else if(scenario=="topic") {
    bool rejected=false;try {MotionOutput invalid(43,"/cmd_vel");}
    catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  } else {assert(false);}
}
