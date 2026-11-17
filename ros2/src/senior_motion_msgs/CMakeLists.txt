cmake_minimum_required(VERSION 3.8)
project(senior_motion_msgs)
find_package(ament_cmake REQUIRED)
# ament_target_dependencies uses exported targets exclusively when rosidl
# targets exist, so handwritten headers need their own usage requirements.
add_library(sensor_timing INTERFACE)
target_include_directories(sensor_timing INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_features(sensor_timing INTERFACE cxx_std_17)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS sensor_timing EXPORT export_sensor_timing)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/ScanMotion.msg" "msg/BoundedHeader.msg"
  "msg/FinalCommand.msg" "msg/DriveExecutionFeedback.msg"
  "msg/BoundedSafetyScan.msg" "msg/DepthSample.msg" "msg/BoundedDepthEvidence.msg"
  "msg/SemanticObject.msg" "msg/SemanticAssociation.msg" "msg/BoundedSemanticEvidence.msg"
  "msg/RawCaptureHeader.msg" "msg/BoundedRawImage.msg" "msg/BoundedRawCameraInfo.msg"
  "msg/EgressHeader.msg" "msg/FencedDepthEvidence.msg" "msg/FencedSemanticEvidence.msg"
  "msg/BoundedRawLaserScan.msg" "msg/BoundedFilteredScan.msg"
  "msg/FencedSafetyScan.msg" "msg/FencedFilteredScan.msg"
  "msg/StaticTransformEdge.msg" "msg/BoundedStaticTf.msg" "msg/RawImageLayout.msg" "msg/RgbdMotionCandidate.msg" "msg/FencedRgbdCandidate.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs nav_msgs sensor_msgs)
add_executable(final_command_guard src/final_command_guard.cpp)
ament_target_dependencies(final_command_guard rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(final_command_guard "${cpp_typesupport_target}")
target_link_libraries(sensor_timing INTERFACE "${cpp_typesupport_target}")
add_executable(motion_output_guard src/motion_output_guard.cpp)
ament_target_dependencies(motion_output_guard rclcpp geometry_msgs)
target_link_libraries(motion_output_guard "${cpp_typesupport_target}" sensor_timing)
add_executable(native_receipt src/native_receipt.cpp)
ament_target_dependencies(native_receipt rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs)
target_link_libraries(native_receipt "${cpp_typesupport_target}")
install(TARGETS final_command_guard motion_output_guard native_receipt DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime builtin_interfaces geometry_msgs std_msgs nav_msgs sensor_msgs rclcpp)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  add_test(NAME final_command_frame_roundtrip
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_final_command_frame.py)
  set_tests_properties(final_command_frame_roundtrip PROPERTIES
    ENVIRONMENT "CXX=${CMAKE_CXX_COMPILER}")
  # colcon test runs after installation: exercise the public package export,
  # including generated and handwritten headers, without source include paths.
  add_test(NAME installed_header_consumer
    COMMAND ${CMAKE_COMMAND}
      "-DPACKAGE_PREFIX=${CMAKE_INSTALL_PREFIX}"
      "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
      "-DCONSUMER_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}/test/installed_header_consumer"
      "-DCONSUMER_BINARY=${CMAKE_CURRENT_BINARY_DIR}/installed_header_consumer"
      "-DCONSUMER_CXX=${CMAKE_CXX_COMPILER}"
      -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_installed_headers.cmake)
endif()
# Load generated message targets before the handwritten targets that link them.
ament_export_targets(export_sensor_timing HAS_LIBRARY_TARGET)
ament_package()
