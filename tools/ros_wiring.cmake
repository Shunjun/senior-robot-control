# Generate native wiring and standard ROS parameter files from one source.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
get_filename_component(SENIOR_WIRING_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(SENIOR_WIRING_DEFINITION "${SENIOR_WIRING_ROOT}/vehicle_core/ros_wiring.json")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SENIOR_WIRING_DEFINITION}")

function(senior_generate_wiring output)
  execute_process(COMMAND "${Python3_EXECUTABLE}"
    "${SENIOR_WIRING_ROOT}/tools/generate_ros_wiring.py"
    --definition "${SENIOR_WIRING_DEFINITION}" --output "${output}"
    RESULT_VARIABLE wiring_status)
  if(NOT wiring_status EQUAL 0)
    message(FATAL_ERROR "Project ROS wiring generation failed")
  endif()
endfunction()

function(senior_configure_wiring relative)
  set(output "${CMAKE_CURRENT_BINARY_DIR}/wiring/${relative}")
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${relative}")
  execute_process(COMMAND "${Python3_EXECUTABLE}"
    "${SENIOR_WIRING_ROOT}/tools/generate_ros_wiring.py"
    --definition "${SENIOR_WIRING_DEFINITION}" --output "${output}"
    --template "${CMAKE_CURRENT_SOURCE_DIR}/${relative}" RESULT_VARIABLE wiring_status)
  if(NOT wiring_status EQUAL 0)
    message(FATAL_ERROR "Project ROS parameter rendering failed: ${relative}")
  endif()
  get_filename_component(parent "${relative}" DIRECTORY)
  install(FILES "${output}" DESTINATION "share/${PROJECT_NAME}/${parent}")
endfunction()
