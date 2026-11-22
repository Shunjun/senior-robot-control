cmake_minimum_required(VERSION 3.8)
project(senior_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundedDepthEvidence.msg"
  "msg/BoundedFilteredScan.msg"
  "msg/BoundedHeader.msg"
  "msg/BoundedRawCameraInfo.msg"
  "msg/BoundedRawImage.msg"
  "msg/BoundedRawLaserScan.msg"
  "msg/BoundedSafetyScan.msg"
  "msg/BoundedSemanticEvidence.msg"
  "msg/BoundedStaticTf.msg"
  "msg/DepthSample.msg"
  "msg/DriveExecutionFeedback.msg"
  "msg/EgressHeader.msg"
  "msg/FencedDepthEvidence.msg"
  "msg/FencedFilteredScan.msg"
  "msg/FencedRgbdCandidate.msg"
  "msg/FencedSafetyScan.msg"
  "msg/FencedSemanticEvidence.msg"
  "msg/FinalCommand.msg"
  "msg/Keyframe.msg"
  "msg/RawCaptureHeader.msg"
  "msg/RawImageLayout.msg"
  "msg/RgbdMotionCandidate.msg"
  "msg/ScanMotion.msg"
  "msg/SemanticAssociation.msg"
  "msg/SemanticObject.msg"
  "msg/StaticTransformEdge.msg"
  "srv/CaptureGraph.srv"
  "srv/MatchStationaryScan.srv"
  "srv/ReadGraph.srv"
  "srv/ReadGraphCommit.srv"
  "srv/ReadLoadedGraph.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime builtin_interfaces geometry_msgs std_msgs nav_msgs sensor_msgs)
ament_package()
