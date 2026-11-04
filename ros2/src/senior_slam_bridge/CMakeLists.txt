cmake_minimum_required(VERSION 3.8)
project(senior_slam_bridge)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread)
find_package(builtin_interfaces REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(senior_motion_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox 2.6.10 EXACT REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)

# //Compatible: Humble ament uses the exported RViz target and skips the legacy library exports.
set(slam_toolbox_library_dir "${slam_toolbox_DIR}/../../../lib")
find_library(SLAM_TOOLBOX_ASYNC_LIBRARY NAMES async_slam_toolbox
  PATHS "${slam_toolbox_library_dir}" NO_DEFAULT_PATH REQUIRED)
find_library(SLAM_TOOLBOX_COMMON_LIBRARY NAMES toolbox_common
  PATHS "${slam_toolbox_library_dir}" NO_DEFAULT_PATH REQUIRED)
find_library(SLAM_TOOLBOX_KARTO_LIBRARY NAMES kartoSlamToolbox
  PATHS "${slam_toolbox_library_dir}" NO_DEFAULT_PATH REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Keyframe.msg"
  "srv/CaptureGraph.srv"
  "srv/ReadGraph.srv"
  "srv/ReadGraphCommit.srv"
  "srv/ReadLoadedGraph.srv"
  DEPENDENCIES builtin_interfaces senior_motion_msgs
)
rosidl_get_typesupport_target(graph_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(slam_graph_node
  src/slam_graph_node.cpp src/staged_graph.cpp src/native_capture.cpp src/bounded_scan_ingress.cpp)
target_compile_features(slam_graph_node PRIVATE cxx_std_17)
ament_target_dependencies(slam_graph_node rclcpp senior_motion_msgs sensor_msgs slam_toolbox tf2_ros)
target_link_libraries(slam_graph_node
  "${graph_typesupport}"
  "${SLAM_TOOLBOX_ASYNC_LIBRARY}"
  "${SLAM_TOOLBOX_COMMON_LIBRARY}"
  "${SLAM_TOOLBOX_KARTO_LIBRARY}"
  Boost::serialization Boost::thread OpenSSL::Crypto TBB::tbb Threads::Threads
)
install(TARGETS slam_graph_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/candidate_preflight.py DESTINATION lib/${PROJECT_NAME}
  RENAME candidate_preflight)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  configure_file(scripts/candidate_preflight.py candidate_preflight.py COPYONLY)
  ament_add_test(candidate_preflight
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts"
      "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test/test_candidate_preflight.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_snapshot test/test_snapshot.cpp)
  target_compile_features(test_snapshot PRIVATE cxx_std_17)
  target_include_directories(test_snapshot PRIVATE src)
  ament_add_test(snapshot
    COMMAND "$<TARGET_FILE:test_snapshot>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_graph_commit_history test/test_graph_commit_history.cpp)
  target_compile_features(test_graph_commit_history PRIVATE cxx_std_17)
  target_include_directories(test_graph_commit_history PRIVATE src)
  ament_add_test(graph_commit_history
    COMMAND "$<TARGET_FILE:test_graph_commit_history>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_laser_geometry test/test_laser_geometry.cpp)
  target_compile_features(test_laser_geometry PRIVATE cxx_std_17)
  target_include_directories(test_laser_geometry PRIVATE src)
  ament_add_test(laser_geometry
    COMMAND "$<TARGET_FILE:test_laser_geometry>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_native_capture test/test_native_capture.cpp src/native_capture.cpp)
  target_compile_features(test_native_capture PRIVATE cxx_std_17)
  target_include_directories(test_native_capture PRIVATE src)
  target_link_libraries(test_native_capture OpenSSL::Crypto Threads::Threads)
  ament_add_test(native_capture
    COMMAND "$<TARGET_FILE:test_native_capture>" "${CMAKE_CURRENT_BINARY_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_staged_graph test/test_staged_graph.cpp src/staged_graph.cpp)
  target_compile_features(test_staged_graph PRIVATE cxx_std_17)
  target_include_directories(test_staged_graph PRIVATE src)
  target_link_libraries(test_staged_graph OpenSSL::Crypto)
  ament_add_test(staged_graph
    COMMAND "$<TARGET_FILE:test_staged_graph>" "${CMAKE_CURRENT_BINARY_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(read_graph_runtime
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/probe_read_graph.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
  ament_add_test(native_capture_runtime
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/probe_native_capture.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 100
  )
  ament_add_test(bounded_scan_ingress_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probe_bounded_scan_ingress.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
  ament_add_test(bounded_graph_capture_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probe_bounded_scan_ingress.py"
      "$<TARGET_FILE:slam_graph_node>" --capture
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
  ament_add_test(graph_commit_delivery_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probe_graph_commit_delivery.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 65
  )
  ament_add_test(candidate_load_runtime
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/probe_candidate_load.py"
      "$<TARGET_FILE:slam_graph_node>" "${CMAKE_CURRENT_BINARY_DIR}/candidate_preflight.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
