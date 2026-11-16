#include <senior_motion_msgs/msg/scan_motion.hpp>
#include <senior_motion_msgs/sensor_timing.hpp>
#include <senior_motion_msgs/navigation_timing.hpp>
#include <senior_motion_msgs/msg/navigation_sample.hpp>

// Compile both public header families using only the installed package export.
static_assert(sizeof(senior_motion_msgs::msg::ScanMotion) > 0);
static_assert(sizeof(sensor_timing::Policy) > 0);

static_assert(sizeof(senior_motion_msgs::msg::NavigationSample) > 0);
static_assert(sizeof(senior_motion_msgs::navigation::Producer) > 0);

#include "senior_motion_msgs/source_timing.hpp"
#include "senior_motion_msgs/source_transport.hpp"
#include "senior_motion_msgs/actuator_timing.hpp"
#include "senior_motion_msgs/msg/timed_laser_scan.hpp"
