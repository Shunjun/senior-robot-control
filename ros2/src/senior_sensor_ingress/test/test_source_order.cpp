#include "senior_sensor_ingress/source_order.hpp"
#include "senior_sensor_ingress/scan_layout.hpp"

#include <limits>
#include <stdexcept>
#include <vector>

namespace {
void require(bool value)
{
  if (!value) {throw std::runtime_error("source order contract failed");}
}
}  // namespace

int main()
{
  struct Scan {
    float angle_min = -1, angle_max = 1, angle_increment = 1;
    float time_increment = 0.01f, scan_time = 0, range_min = 0, range_max = 10;
    std::vector<float> ranges{1, std::numeric_limits<float>::quiet_NaN(), -1};
  } scan;
  require(senior_sensor_ingress::scan_layout_ok(scan));
  scan.angle_max = 2;  // Exclusive end convention used by the real driver.
  require(senior_sensor_ingress::scan_layout_ok(scan));
  scan.angle_max = 3;
  require(!senior_sensor_ingress::scan_layout_ok(scan));
  scan.angle_max = 1;
  scan.angle_increment = 0;
  require(!senior_sensor_ingress::scan_layout_ok(scan));
  scan.angle_increment = 1;
  scan.ranges.clear();
  require(!senior_sensor_ingress::scan_layout_ok(scan));

  senior_sensor_ingress::SourceOrder order;
  require(order.accept("boot-a", 3, 100));
  require(!order.accept("boot-a", 3, 100));
  require(!order.accept("boot-a", 3, 99));
  require(order.accept("boot-a", 3, 101));
  require(order.observe_generation("boot-a", 4));
  require(!order.accept("boot-a", 3, 200));
  require(order.accept("boot-a", 4, 10));
  require(!order.accept("boot-a", 3, 200));
  require(!order.accept("boot-b", 1, 102));
  order.confirm_boot("boot-b");
  require(order.accept("boot-b", 1, 10));
  require(!order.accept("boot-a", 4, 300));
  order.confirm_boot("boot-b");
  require(!order.accept("boot-b", 1, 10));
  require(!order.accept("boot-b", 0, 11));
  require(!order.accept("boot-b", 2, -1));
  require(order.accept("boot-b", 1, 11));
  senior_sensor_ingress::SourceOrder received;
  require(received.accept_capture("boot-a", 2, 100, 1000));
  require(!received.accept_capture("boot-b", 1, 101, 1000));
  require(received.accept_capture("boot-b", 1, 10, 1001));
  require(!received.accept_capture("boot-a", 2, 200, 1000));
  require(received.accept_capture("boot-b", 1, 11, 1002));
}
