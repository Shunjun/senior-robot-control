cmake_minimum_required(VERSION 3.8)
project(senior_motion_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/ScanMotion.msg" "msg/BoundedHeader.msg" "msg/MotionEvidence.msg"
  "msg/FinalCommand.msg" "msg/DriveExecutionFeedback.msg"
  "msg/BoundedSafetyScan.msg" "msg/DepthSample.msg" "msg/BoundedDepthEvidence.msg"
  "msg/SemanticObject.msg" "msg/SemanticAssociation.msg" "msg/BoundedSemanticEvidence.msg"
  "msg/RawCaptureHeader.msg" "msg/BoundedRawImage.msg" "msg/BoundedRawCameraInfo.msg"
  "msg/EgressHeader.msg" "msg/FencedDepthEvidence.msg" "msg/FencedSemanticEvidence.msg"
  "msg/BoundedRawLaserScan.msg" "msg/BoundedFilteredScan.msg"
  "msg/FencedSafetyScan.msg" "msg/FencedFilteredScan.msg"
  "msg/StaticTransformEdge.msg" "msg/BoundedStaticTf.msg" "msg/RawImageLayout.msg" "msg/RgbdMotionCandidate.msg" "msg/FencedRgbdCandidate.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)
add_executable(final_command_guard src/final_command_guard.cpp)
ament_target_dependencies(final_command_guard rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(final_command_guard "${cpp_typesupport_target}")
install(TARGETS final_command_guard DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
