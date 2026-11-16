// Real timing/transport helpers and shared world clock, with a fixed boot boundary.
#include <cassert>
#include <cstdint>
#include <string>
#include <vector>
#include <limits>
#include <stdexcept>
#define SENIOR_MOTION_MSGS__NAVIGATION_TIMING_HPP_
namespace senior_motion_msgs::navigation {
inline std::string boot_domain(){return "00000000-0000-0000-0000-000000000001";}
inline uint64_t monotonic_ns(){return 10'000'000'000ULL;}
}
#include "senior_motion_msgs/source_timing.hpp"
#include "senior_motion_msgs/source_transport.hpp"
#include "senior_motion_msgs/simulation_timing.hpp"
struct Field {
  std::string name;std::vector<std::string> values;
  const std::string& key()const{return name;}void set_key(const std::string& v){name=v;}
  int value_size()const{return static_cast<int>(values.size());}
  const std::string& value(int i)const{return values.at(i);}
  void clear_value(){values.clear();}void add_value(const std::string& s){values.push_back(s);}
};
struct Header {
  std::vector<Field> fields;
  int data_size()const{return static_cast<int>(fields.size());}
  const Field& data(int i)const{return fields.at(i);}
  Field* mutable_data(int i){return &fields.at(i);}
  Field* add_data(){fields.emplace_back();return &fields.back();}
};
int main() {
  using namespace senior_motion_msgs::timing;
  int firstWorld = 0, secondWorld = 0;
  const auto firstKey=simulation_world_key(&firstWorld,1);
  assert(firstKey==simulation_world_key(&firstWorld,1));
  assert(firstKey!=simulation_world_key(&secondWorld,1));
  const auto zero=simulation_epoch("world-a",0);
  assert(simulation_epoch("world-a",1'000'000'000ULL)==zero);
  assert(simulation_epoch("world-a",1'000'000'000ULL)==zero); // pause, no host extrapolation
  const auto other=simulation_epoch("world-b",0);assert(other!=zero);
  DriverClock first,second;
  auto a=first.sample(1,1,"sim_step",zero,0);
  auto b=second.sample(1,1,"sim_step",zero,0);
  assert(a.source_epoch!=b.source_epoch && a.measurement_epoch==b.measurement_epoch);
  assert(same_measurement_axis(a,b) && measurement_ns(a)==0 && valid(a,10));
  auto reset=simulation_epoch("world-a",0);assert(reset!=zero);
  assert(simulation_epoch("world-a",0)==reset);
  auto c=first.sample(2,2,"sim_step",reset,0);
  assert(c.source_epoch==a.source_epoch && !same_measurement_axis(a,c));
  SourceOrder order;assert(order.accept(a,10)==SourceOrder::Result::current);
  assert(order.accept(c,10)==SourceOrder::Result::current);
  assert(order.accept(a,10)==SourceOrder::Result::rejected);
  auto d=second.sample(3,3,"sim_step",reset,1);
  assert(order.accept(d,10)==SourceOrder::Result::replaced);
  assert(order.accept(c,10)==SourceOrder::Result::rejected);
  Header header;attach(&header,d);senior_motion_msgs::msg::SourceTime restored;
  assert(extract(header,restored));assert(restored.measurement_tick_ns==1);
  assert(restored.capture_end_steady_ns==3 && restored.measurement_epoch==reset);
  assert(restored.source_epoch==d.source_epoch && restored.sequence==d.sequence);
  header.fields.push_back(header.fields[0]);assert(!extract(header,restored));
  header.fields.pop_back();header.fields[0].values[7]="NaN";assert(!extract(header,restored));
  auto host=first.sample(4,4);auto host2=second.sample(5,5);
  assert(same_measurement_axis(host,host2));assert(measurement_ns(host)==4);
  host.clock_domain="foreign";assert(!valid(host,10));
}
