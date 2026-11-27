cmake_minimum_required(VERSION 3.8)
project(senior_mapping)
find_package(ament_cmake REQUIRED)
find_package(senior_sensor_ingress REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/slam_adapter DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(mapping_algorithms ${CMAKE_CURRENT_SOURCE_DIR}/test
    ENV "PYTEST_ADDOPTS=--ignore=${CMAKE_CURRENT_SOURCE_DIR}/test/native")
endif()

# Preserve the supported native backend and fail explicitly when its build dependencies are absent.
find_package(Boost REQUIRED COMPONENTS serialization thread)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(senior_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(slam_toolbox 2.6.10 EXACT REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Ceres REQUIRED)
find_package(pluginlib REQUIRED)

add_library(senior_mapping_ceres SHARED src/solver/ceres_solver.cpp)
target_compile_features(senior_mapping_ceres PRIVATE cxx_std_17)
ament_target_dependencies(senior_mapping_ceres rclcpp slam_toolbox pluginlib)
target_link_libraries(senior_mapping_ceres Ceres::ceres Boost::thread TBB::tbb)
pluginlib_export_plugin_description_file(slam_toolbox mapping_solver_plugins.xml)
install(TARGETS senior_mapping_ceres LIBRARY DESTINATION lib)
install(DIRECTORY src/solver/ DESTINATION share/${PROJECT_NAME}/solver_source)

# //Compatible: Humble ament uses the exported RViz target and skips the legacy library exports.
set(slam_toolbox_library_dir "${slam_toolbox_DIR}/../../../lib")
find_library(SLAM_TOOLBOX_ASYNC_LIBRARY NAMES async_slam_toolbox
  PATHS "${slam_toolbox_library_dir}" NO_DEFAULT_PATH REQUIRED)
find_library(SLAM_TOOLBOX_COMMON_LIBRARY NAMES toolbox_common
  PATHS "${slam_toolbox_library_dir}" NO_DEFAULT_PATH REQUIRED)
find_library(SLAM_TOOLBOX_KARTO_LIBRARY NAMES kartoSlamToolbox
  PATHS "${slam_toolbox_library_dir}" NO_DEFAULT_PATH REQUIRED)

add_executable(slam_graph_node
  src/slam/slam_graph_node.cpp src/slam/staged_graph.cpp src/slam/native_capture.cpp src/slam/bounded_scan_ingress.cpp)
target_compile_features(slam_graph_node PRIVATE cxx_std_17)
ament_target_dependencies(slam_graph_node rclcpp senior_interfaces sensor_msgs nav_msgs geometry_msgs slam_toolbox tf2_ros senior_sensor_ingress)
target_link_libraries(slam_graph_node
  "${SLAM_TOOLBOX_ASYNC_LIBRARY}"
  "${SLAM_TOOLBOX_COMMON_LIBRARY}"
  "${SLAM_TOOLBOX_KARTO_LIBRARY}"
  Boost::serialization Boost::thread OpenSSL::Crypto TBB::tbb Threads::Threads
)
add_executable(acquire_scan_receipt src/slam/acquire_scan_receipt.cpp)
target_compile_features(acquire_scan_receipt PRIVATE cxx_std_17)
ament_target_dependencies(acquire_scan_receipt rclcpp senior_interfaces senior_sensor_ingress)
install(TARGETS slam_graph_node acquire_scan_receipt DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/candidate_preflight.py DESTINATION lib/${PROJECT_NAME}
  RENAME candidate_preflight)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  configure_file(scripts/candidate_preflight.py candidate_preflight.py COPYONLY)
  ament_add_test(candidate_preflight
    COMMAND "${CMAKE_COMMAND}" -E env
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/scripts"
      "${Python3_EXECUTABLE}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test/native/test_candidate_preflight.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_snapshot test/native/test_snapshot.cpp)
  target_compile_features(test_snapshot PRIVATE cxx_std_17)
  target_include_directories(test_snapshot PRIVATE src/slam)
  ament_add_test(snapshot
    COMMAND "$<TARGET_FILE:test_snapshot>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_graph_commit_history test/native/test_graph_commit_history.cpp)
  target_compile_features(test_graph_commit_history PRIVATE cxx_std_17)
  target_include_directories(test_graph_commit_history PRIVATE src/slam)
  ament_add_test(graph_commit_history
    COMMAND "$<TARGET_FILE:test_graph_commit_history>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_laser_geometry test/native/test_laser_geometry.cpp)
  target_compile_features(test_laser_geometry PRIVATE cxx_std_17)
  target_include_directories(test_laser_geometry PRIVATE src/slam)
  ament_add_test(laser_geometry
    COMMAND "$<TARGET_FILE:test_laser_geometry>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_stationary_match_quality test/native/test_stationary_match_quality.cpp)
  target_compile_features(test_stationary_match_quality PRIVATE cxx_std_17)
  ament_target_dependencies(test_stationary_match_quality slam_toolbox)
  target_link_libraries(test_stationary_match_quality
    "${SLAM_TOOLBOX_KARTO_LIBRARY}" Boost::serialization Boost::thread TBB::tbb)
  ament_add_test(stationary_match_quality
    COMMAND "$<TARGET_FILE:test_stationary_match_quality>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )

  add_executable(test_native_capture test/native/test_native_capture.cpp src/slam/native_capture.cpp)
  target_compile_features(test_native_capture PRIVATE cxx_std_17)
  target_include_directories(test_native_capture PRIVATE src/slam)
  target_link_libraries(test_native_capture OpenSSL::Crypto Threads::Threads)
  ament_add_test(native_capture
    COMMAND "$<TARGET_FILE:test_native_capture>" "${CMAKE_CURRENT_BINARY_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(test_staged_graph test/native/test_staged_graph.cpp src/slam/staged_graph.cpp)
  target_compile_features(test_staged_graph PRIVATE cxx_std_17)
  target_include_directories(test_staged_graph PRIVATE src/slam)
  target_link_libraries(test_staged_graph OpenSSL::Crypto)
  ament_add_test(staged_graph
    COMMAND "$<TARGET_FILE:test_staged_graph>" "${CMAKE_CURRENT_BINARY_DIR}"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(production_session_runtime
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_production_session.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 90
  )
  ament_add_test(read_graph_runtime
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_read_graph.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
  ament_add_test(native_capture_runtime
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_native_capture.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 100
  )
  ament_add_test(bounded_scan_ingress_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_bounded_scan_ingress.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
  ament_add_test(bounded_scan_geometry_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_bounded_scan_ingress.py"
      "$<TARGET_FILE:slam_graph_node>" --geometry-drift
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
  ament_add_test(bounded_graph_capture_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_bounded_scan_ingress.py"
      "$<TARGET_FILE:slam_graph_node>" --capture
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
  ament_add_test(graph_commit_delivery_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_graph_commit_delivery.py"
      "$<TARGET_FILE:slam_graph_node>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 65
  )


  ament_add_test(candidate_live_scan_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_candidate_live_scan.py"
      "$<TARGET_FILE:slam_graph_node>" "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/candidate_preflight"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 75
  )
  ament_add_test(candidate_stationary_match_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_candidate_stationary_match.py"
      "$<TARGET_FILE:slam_graph_node>" "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/candidate_preflight"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120
  )
  ament_add_test(acquire_scan_receipt_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_acquire_scan_receipt.py"
      "$<TARGET_FILE:acquire_scan_receipt>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30
  )
  ament_add_test(candidate_match_ambiguity_runtime
    COMMAND "${Python3_EXECUTABLE}"
      "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_candidate_match_ambiguity.py"
      "$<TARGET_FILE:slam_graph_node>" "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/candidate_preflight"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120
  )
  ament_add_test(candidate_load_runtime
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/probes/probe_candidate_load.py"
      "$<TARGET_FILE:slam_graph_node>" "${CMAKE_CURRENT_BINARY_DIR}/candidate_preflight.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120
  )
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
