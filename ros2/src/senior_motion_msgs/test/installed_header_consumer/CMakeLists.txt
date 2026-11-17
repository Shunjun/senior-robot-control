cmake_minimum_required(VERSION 3.8)
project(installed_header_consumer LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(senior_motion_msgs REQUIRED)
# Regression: the installed package must load its message typesupport before
# sensor_timing's export checks that dependency, even in a fresh consumer.
if(NOT TARGET senior_motion_msgs::sensor_timing OR
   NOT TARGET senior_motion_msgs::senior_motion_msgs__rosidl_typesupport_cpp)
  message(FATAL_ERROR "Installed timing and C++ message typesupport targets must be available")
endif()
add_library(header_consumer OBJECT header_consumer.cpp)
ament_target_dependencies(header_consumer senior_motion_msgs)
