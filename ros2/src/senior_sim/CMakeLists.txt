cmake_minimum_required(VERSION 3.8)
project(senior_sim)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY urdf worlds launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/simulation_camera.py DESTINATION lib/${PROJECT_NAME}
  RENAME simulation_camera)
install(PROGRAMS scripts/world_reference.py DESTINATION lib/${PROJECT_NAME}
  RENAME world_reference)
include("${CMAKE_CURRENT_SOURCE_DIR}/../../../tools/ros_wiring.cmake")
senior_configure_wiring(config/simulation_rgbd.yaml)
senior_configure_wiring(config/simulation_motion.yaml)
ament_package()
