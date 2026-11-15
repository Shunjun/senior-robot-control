#pragma once
namespace geometry_msgs { namespace msg {
struct Twist { struct Axis { double x=0, y=0, z=0; }; Axis linear, angular; };
}}
