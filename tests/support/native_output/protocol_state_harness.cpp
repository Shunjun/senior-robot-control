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
static int model_clock_gettime(clockid_t, timespec *value) {
  value->tv_sec = static_cast<time_t>(clock_ns / 1'000'000'000ULL);
  value->tv_nsec = static_cast<long>(clock_ns % 1'000'000'000ULL);
  return 0;
}
#define clock_gettime model_clock_gettime
#include "senior_sensor_ingress/sample_time.hpp"
#undef clock_gettime
#include "geometry_msgs/msg/twist.hpp"
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
using Target=geometry_msgs::msg::Twist;
using Publication=rclcpp::Publisher<Target>;
static void frame(double vx, double wz, double deadline, double produced, uint64_t sequence) {
  std::vector<unsigned char> bytes(40);
  std::memcpy(bytes.data(),&vx,8);std::memcpy(bytes.data()+8,&wz,8);
  std::memcpy(bytes.data()+16,&deadline,8);std::memcpy(bytes.data()+24,&produced,8);
  std::memcpy(bytes.data()+32,&sequence,8);packets.push_back(bytes);
}
int main(int argc,char **argv) {
  assert(argc==2);const std::string scenario=argv[1];
  MotionOutput receiver(42,"/cmd_vel");
  frame(-0.4,0.7,10.5,9.9,1);receiver.poll();
  assert(Publication::published.size()==1);
  assert(receiver.deadline_==10.5);
  auto initial=Publication::published.back();
  assert(initial.linear.x==-0.4 && initial.angular.z==0.7);
  if(scenario=="watchdog_inflight") {
    clock_ns=10'500'000'000ULL;receiver.poll();
    assert(Publication::published.size()==2 && !receiver.moving_);
    assert(Publication::published[1].linear.x==0 && Publication::published[1].angular.z==0);
    assert(receiver.last_produced_==clock_ns);
    frame(0.8,0.1,11.0,10.49,2);receiver.poll();
    assert(Publication::published.size()==2 && receiver.sequence_==2);
    assert(receiver.last_produced_==10'500'000'000ULL);
    clock_ns=10'510'000'000ULL;
    frame(0.3,0.2,10.66,10.51,3);receiver.poll();
    assert(Publication::published.size()==3 && receiver.moving_);
    assert(Publication::published.back().linear.x==0.3 && Publication::published.back().angular.z==0.2);
    assert(receiver.deadline_==10.66 && receiver.last_produced_==clock_ns);
  } else if(scenario=="expired_queue") {
    frame(0.8,0.1,9.99,9.95,2);receiver.poll();
    assert(Publication::published.size()==1 && receiver.deadline_==10.5);
    clock_ns=10'500'000'000ULL;receiver.poll();
    assert(Publication::published.back().linear.x==0);
  } else if(scenario=="publish_deadline") {
    clock_ns=10'500'000'000ULL;
    receiver.publish(0.8,0.4,10'000'000'000ULL,clock_ns);
    assert(Publication::published.back().linear.x==0);
    assert(Publication::published.back().angular.z==0);
    assert(!receiver.moving_ && receiver.deadline_==0);
  } else if(scenario=="original_order") {
    frame(0.2,0.3,10.4,9.99,2);
    frame(0.9,0.9,10.5,9.98,3);receiver.poll();
    assert(Publication::published.back().linear.x==0.2);
  } else if(scenario=="zero") {
    frame(0,0,0,10,2);receiver.poll();
    assert(Publication::published.back().linear.x==0);
  } else if(scenario=="future") {
    frame(0.2,0.3,11,10.1,2);receiver.poll();assert(receiver.disconnected_);
    assert(Publication::published.back().linear.x==0);
  } else if(scenario=="nan") {
    frame(NAN,0.3,11,10,2);receiver.poll();assert(receiver.disconnected_);
    assert(Publication::published.back().linear.x==0);
  } else if(scenario=="replay") {
    frame(0.2,0.3,11,10,1);receiver.poll();assert(receiver.disconnected_);
    assert(Publication::published.back().linear.x==0);
  } else if(scenario=="eof") {
    eof=true;receiver.poll();assert(receiver.disconnected_);
    assert(Publication::published.back().linear.x==0);
  } else if(scenario=="topic") {
    assert(Publication::topics.back()=="/cmd_vel");
  } else {assert(false);}
}
