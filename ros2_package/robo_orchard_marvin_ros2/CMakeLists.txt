cmake_minimum_required(VERSION 3.8)
project(robo_orchard_marvin_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(MARVIN_DRIVER_WARNING_FLAGS -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robo_orchard_marvin_msg_ros2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

set(MARVIN_SDK_REVISION 2d219b3697381d169b61574086d22979c33d58ff)
set(MARVIN_SDK_SUBMODULE_ROOT
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/marvin_sdk)
set(MARVIN_CONTROL_SDK_ROOT
  ${MARVIN_SDK_SUBMODULE_ROOT}/contrlSDK100343)
set(MARVIN_KINEMATICS_SDK_ROOT
  ${MARVIN_SDK_SUBMODULE_ROOT}/kinematicsSDK)
set(MARVIN_KINEMATICS_CONFIG
  ${MARVIN_SDK_SUBMODULE_ROOT}/CommonConfig/ccs_m6_40.MvKDCfg)

if(NOT EXISTS ${MARVIN_CONTROL_SDK_ROOT}/MarvinSDK.cpp)
  message(FATAL_ERROR
    "Marvin SDK submodule is not initialized. Run: "
    "git submodule update --init --recursive")
endif()

if(EXISTS ${MARVIN_SDK_SUBMODULE_ROOT}/.git)
  find_package(Git REQUIRED)
  if(IS_DIRECTORY ${MARVIN_SDK_SUBMODULE_ROOT}/.git)
    set(MARVIN_SDK_GIT_DIR ${MARVIN_SDK_SUBMODULE_ROOT}/.git)
  else()
    file(READ ${MARVIN_SDK_SUBMODULE_ROOT}/.git MARVIN_SDK_GIT_FILE)
    string(STRIP "${MARVIN_SDK_GIT_FILE}" MARVIN_SDK_GIT_FILE)
    string(REGEX REPLACE "^gitdir: " ""
      MARVIN_SDK_GIT_DIR_RELATIVE "${MARVIN_SDK_GIT_FILE}")
    get_filename_component(MARVIN_SDK_GIT_DIR
      "${MARVIN_SDK_SUBMODULE_ROOT}/${MARVIN_SDK_GIT_DIR_RELATIVE}"
      ABSOLUTE)
  endif()
  execute_process(
    COMMAND ${GIT_EXECUTABLE} --git-dir=${MARVIN_SDK_GIT_DIR} rev-parse HEAD
    RESULT_VARIABLE MARVIN_SDK_REVISION_RESULT
    OUTPUT_VARIABLE MARVIN_SDK_ACTUAL_REVISION
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT MARVIN_SDK_REVISION_RESULT EQUAL 0 OR
      NOT MARVIN_SDK_ACTUAL_REVISION STREQUAL MARVIN_SDK_REVISION)
    message(FATAL_ERROR
      "Marvin SDK must be checked out at ${MARVIN_SDK_REVISION}; found "
      "${MARVIN_SDK_ACTUAL_REVISION}")
  endif()
  # core.fileMode=false because CI checks the submodule out as root, which
  # leaves permission bits that do not match the index and reports every
  # tracked file as modified. Only content matters for what gets compiled.
  execute_process(
    COMMAND ${GIT_EXECUTABLE} --git-dir=${MARVIN_SDK_GIT_DIR}
      --work-tree=${MARVIN_SDK_SUBMODULE_ROOT}
      -c core.fileMode=false
      status --porcelain --untracked-files=all
    RESULT_VARIABLE MARVIN_SDK_STATUS_RESULT
    OUTPUT_VARIABLE MARVIN_SDK_STATUS
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT MARVIN_SDK_STATUS_RESULT EQUAL 0 OR MARVIN_SDK_STATUS)
    message(FATAL_ERROR
      "Marvin SDK submodule must remain pristine; local changes found:\n"
      "${MARVIN_SDK_STATUS}")
  endif()
endif()

add_library(marvin_control_sdk STATIC
  ${MARVIN_CONTROL_SDK_ROOT}/ACB.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/CAxisSpPln.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/FXDG.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/FileOP.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/FilePortal.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/MarvinSDK.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/Parser.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/PointSet.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/Robot.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/ShMem.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/TCPAgent.cpp
  ${MARVIN_CONTROL_SDK_ROOT}/TCPFileClient.cpp
)
target_compile_features(marvin_control_sdk PUBLIC cxx_std_17)
target_compile_definitions(marvin_control_sdk PUBLIC CMPL_LIN)
target_include_directories(marvin_control_sdk PUBLIC
  $<BUILD_INTERFACE:${MARVIN_CONTROL_SDK_ROOT}>)
target_link_libraries(marvin_control_sdk PUBLIC Threads::Threads rt)
set_target_properties(marvin_control_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(marvin_kinematics_sdk SHARED
  ${MARVIN_KINEMATICS_SDK_ROOT}/AxisPln.cpp
  ${MARVIN_KINEMATICS_SDK_ROOT}/FXDG.cpp
  ${MARVIN_KINEMATICS_SDK_ROOT}/FXMath.cpp
  ${MARVIN_KINEMATICS_SDK_ROOT}/FXMatrix.cpp
  ${MARVIN_KINEMATICS_SDK_ROOT}/FxRobot.cpp
  ${MARVIN_KINEMATICS_SDK_ROOT}/LoadIdenPub.cpp
  ${MARVIN_KINEMATICS_SDK_ROOT}/O3Polynorm.cpp
  ${MARVIN_KINEMATICS_SDK_ROOT}/PointSet.cpp
  ${MARVIN_KINEMATICS_SDK_ROOT}/fxrobot_cpi.cpp
)
target_compile_features(marvin_kinematics_sdk PUBLIC cxx_std_17)
target_include_directories(marvin_kinematics_sdk PUBLIC
  $<BUILD_INTERFACE:${MARVIN_KINEMATICS_SDK_ROOT}>)
target_link_libraries(marvin_kinematics_sdk PUBLIC Threads::Threads rt)

add_executable(marvin_driver_node
  src/marvin_driver_node.cpp
  src/marvin_forward_kinematics.cpp
)
target_compile_features(marvin_driver_node PUBLIC cxx_std_17)
target_compile_options(marvin_driver_node PRIVATE ${MARVIN_DRIVER_WARNING_FLAGS})
target_include_directories(marvin_driver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${MARVIN_CONTROL_SDK_ROOT}
  ${MARVIN_KINEMATICS_SDK_ROOT}
)
ament_target_dependencies(
  marvin_driver_node
  ament_index_cpp
  geometry_msgs
  rclcpp
  robo_orchard_marvin_msg_ros2
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(
  marvin_driver_node
  marvin_control_sdk
  marvin_kinematics_sdk
)

install(TARGETS marvin_driver_node marvin_kinematics_sdk
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib)
install(DIRECTORY config launch meshes urdf DESTINATION share/${PROJECT_NAME})
install(FILES ${MARVIN_KINEMATICS_CONFIG}
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES third_party/marvin_sdk/LICENSE
  DESTINATION share/${PROJECT_NAME}/third_party/marvin_sdk)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # third_party/AMENT_IGNORE keeps the vendored SDK out of the linters that
  # honour that marker. Two do not, so they are dropped from the automatic hook:
  #   pep257  crawls via pydocstyle, which ignores AMENT_IGNORE, and its CMake
  #           macro takes no EXCLUDE. It is re-added below for our own sources.
  #   xmllint validates package.xml against the schema URL in its <?xml-model?>
  #           processing instruction, so it needs to reach download.ros.org.
  #           The CI container has no route there and the fetch returns empty.
  # flake8 is dropped for an unrelated reason: it runs the apt flake8 4.0.1,
  # which dies loading plugins when CI's pip pulls a pycodestyle newer than
  # 2.8. Our Python is covered by ruff and by ament_pep257(launch) below.
  set(AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_pep257
    ament_cmake_xmllint
    ament_cmake_flake8)
  ament_lint_auto_find_test_dependencies()
  ament_pep257(launch)
  ament_add_gtest(
    marvin_forward_kinematics_test
    tests/marvin_forward_kinematics_test.cpp
    src/marvin_forward_kinematics.cpp)
  if(TARGET marvin_forward_kinematics_test)
    target_compile_features(marvin_forward_kinematics_test PUBLIC cxx_std_17)
    target_include_directories(marvin_forward_kinematics_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      ${MARVIN_KINEMATICS_SDK_ROOT})
    target_compile_definitions(
      marvin_forward_kinematics_test PRIVATE
      MARVIN_KINEMATICS_CONFIG="${MARVIN_KINEMATICS_CONFIG}")
    target_link_libraries(
      marvin_forward_kinematics_test
      marvin_kinematics_sdk)
  endif()
endif()

ament_package()
