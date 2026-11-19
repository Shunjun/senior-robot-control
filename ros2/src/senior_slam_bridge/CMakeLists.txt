cmake_minimum_required(VERSION 3.8)
project(senior_slam_bridge)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(senior_motion_msgs 0.3.0 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Keyframe.msg"
  "srv/CaptureGraph.srv"
  "srv/MatchStationaryScan.srv"
  "srv/ReadGraph.srv"
  "srv/ReadGraphCommit.srv"
  "srv/ReadLoadedGraph.srv"
  DEPENDENCIES builtin_interfaces senior_motion_msgs
)
ament_export_dependencies(rosidl_default_runtime builtin_interfaces senior_motion_msgs)
ament_package()
