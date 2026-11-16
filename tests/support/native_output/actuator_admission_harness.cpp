// Actual admission policy with recording graph/message-info boundaries, not DDS.
#include <cassert>
#include <cstdint>
#include <limits>
#include <string>
#define SENIOR_MOTION_MSGS__NAVIGATION_TIMING_HPP_
namespace senior_motion_msgs::navigation {
inline std::string boot_domain(){return "00000000-0000-0000-0000-000000000001";}
inline uint64_t monotonic_ns(){return 100;}
}
#include "senior_motion_msgs/actuator_timing.hpp"
struct Endpoint {
  std::string name="motion_output",space="/";
  std::array<uint8_t,24> gid{};
  std::string node_name()const{return name;}std::string node_namespace()const{return space;}
  std::array<uint8_t,24> endpoint_gid()const{return gid;}
};
struct Graph {
  std::vector<Endpoint> entries{Endpoint{}};
  std::vector<Endpoint> get_publishers_info_by_topic(const std::string&){return entries;}
};
int main() {
  using namespace senior_motion_msgs::timing;
  DriverClock producer;Graph graph;graph.entries[0].gid.fill(0xab);
  rclcpp::MessageInfo info;for(auto & v:info.info.publisher_gid.data)v=0xab;
  senior_motion_msgs::msg::ActuatorTarget target;
  target.timing=producer.sample(90,90,"control_compute");target.valid_until_steady_ns=110;
  target.data.linear.x=-0.4;target.data.angular.z=0.7;
  ActuatorAdmission admission;
  assert(admission.accept(graph,"/control/actuator_target",target,info,100));
  assert(!admission.accept(graph,"/control/actuator_target",target,info,100)); // replay
  auto next=target;next.timing=producer.sample(95,95,"control_compute");
  next.valid_until_steady_ns=100;assert(!admission.accept(graph,"x",next,info,100));
  next.valid_until_steady_ns=110;
  graph.entries[0].name="foreign";assert(!admission.accept(graph,"x",next,info,100));
  graph.entries[0].name="motion_output";info.info.publisher_gid.data[0]=0;
  assert(!admission.accept(graph,"x",next,info,100));info.info.publisher_gid.data[0]=0xab;
  auto bad=next;bad.timing.clock_domain="foreign";assert(!admission.accept(graph,"x",bad,info,100));
  bad=next;bad.timing.time_basis="driver_read";assert(!admission.accept(graph,"x",bad,info,100));
  bad=next;bad.data.linear.y=0.2;assert(!admission.accept(graph,"x",bad,info,100));
  bad=next;bad.data.angular.x=std::numeric_limits<double>::quiet_NaN();
  assert(!admission.accept(graph,"x",bad,info,100));
  assert(admission.accept(graph,"x",next,info,100)); // invalid frames did not advance the frontier
  auto zero=next;zero.timing=producer.sample(99,99,"control_compute");
  zero.data.linear.x=0;zero.data.angular.z=0;zero.valid_until_steady_ns=0;
  assert(admission.accept(graph,"x",zero,info,100));
}
