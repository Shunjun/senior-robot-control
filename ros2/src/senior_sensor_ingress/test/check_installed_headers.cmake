# Select this installation explicitly rather than a cached/underlay package.
execute_process(
  COMMAND "${CMAKE_COMMAND}" -S "${CONSUMER_SOURCE}" -B "${CONSUMER_BINARY}"
    "-Dsenior_sensor_ingress_DIR=${PACKAGE_PREFIX}/share/senior_sensor_ingress/cmake"
    "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}"
    "-DCMAKE_CXX_COMPILER=${CONSUMER_CXX}"
  RESULT_VARIABLE configure_result)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR "Installed header consumer configuration failed")
endif()
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${CONSUMER_BINARY}" --clean-first
  RESULT_VARIABLE build_result)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR "Installed header consumer compilation failed")
endif()
