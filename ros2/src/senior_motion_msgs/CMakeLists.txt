cmake_minimum_required(VERSION 3.8)
project(senior_motion_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/ScanMotion.msg" "msg/BoundedHeader.msg" "msg/MotionEvidence.msg"
  "msg/BoundedSafetyScan.msg" "msg/DepthSample.msg" "msg/BoundedDepthEvidence.msg"
  "msg/SemanticObject.msg" "msg/SemanticAssociation.msg" "msg/BoundedSemanticEvidence.msg"
  "msg/RawCaptureHeader.msg" "msg/BoundedRawImage.msg" "msg/BoundedRawCameraInfo.msg"
  "msg/EgressHeader.msg" "msg/FencedDepthEvidence.msg" "msg/FencedSemanticEvidence.msg"
  "msg/BoundedRawLaserScan.msg" "msg/BoundedFilteredScan.msg"
  "msg/FencedSafetyScan.msg" "msg/FencedFilteredScan.msg"
  "msg/StaticTransformEdge.msg" "msg/BoundedStaticTf.msg" "msg/RawImageLayout.msg" "msg/RgbdMotionCandidate.msg" "msg/FencedRgbdCandidate.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
