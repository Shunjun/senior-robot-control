cmake_minimum_required(VERSION 3.8)
project(senior_slam_bridge)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(slam_toolbox 2.6.10 EXACT REQUIRED)
find_package(TBB REQUIRED)

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
  "srv/ReadGraph.srv"
  DEPENDENCIES builtin_interfaces
)
rosidl_get_typesupport_target(graph_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(slam_graph_node src/slam_graph_node.cpp)
target_compile_features(slam_graph_node PRIVATE cxx_std_17)
ament_target_dependencies(slam_graph_node rclcpp slam_toolbox)
target_link_libraries(slam_graph_node
  "${graph_typesupport}"
  "${SLAM_TOOLBOX_ASYNC_LIBRARY}"
  "${SLAM_TOOLBOX_COMMON_LIBRARY}"
  "${SLAM_TOOLBOX_KARTO_LIBRARY}"
  Boost::serialization Boost::thread TBB::tbb
)
install(TARGETS slam_graph_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_snapshot test/test_snapshot.cpp)
  target_compile_features(test_snapshot PRIVATE cxx_std_17)
  target_include_directories(test_snapshot PRIVATE src)
  ament_add_test(snapshot
    COMMAND "$<TARGET_FILE:test_snapshot>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(read_graph_runtime
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/probe_read_graph.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
