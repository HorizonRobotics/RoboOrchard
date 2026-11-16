cmake_minimum_required(VERSION 3.16)
project(robo_orchard_wuji2_ros2 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wuji_msgs REQUIRED)

include(cmake/wuji_sdk.cmake)
add_subdirectory(driver_common)

add_library(wuji_hand2_driver_impl STATIC
  src/wuji_hand2_driver/driver_config.cpp
  src/wuji_hand2_driver/hand2_node.cpp
  src/wuji_hand2_driver/control/control_plane.cpp
  src/wuji_hand2_driver/control/hand_action_services.cpp
  src/wuji_hand2_driver/control/hand_configuration_services.cpp
  src/wuji_hand2_driver/control/joint_command_controller.cpp
  src/wuji_hand2_driver/control/tactile_calibration_service.cpp
  src/wuji_hand2_driver/stream/fingertip_stream.cpp
  src/wuji_hand2_driver/stream/imu_stream.cpp
  src/wuji_hand2_driver/stream/joint_diagnostics_stream.cpp
  src/wuji_hand2_driver/stream/joint_state_stream.cpp
  src/wuji_hand2_driver/conversion/tactile_decoder.cpp
  src/wuji_hand2_driver/conversion/imu_conversion.cpp
  src/wuji_hand2_driver/conversion/joint_command_conversion.cpp
  src/wuji_hand2_driver/conversion/joint_diagnostics_conversion.cpp
  src/wuji_hand2_driver/conversion/joint_state_conversion.cpp
  src/wuji_hand2_driver/conversion/ros_header_conversion.cpp
  src/wuji_hand2_driver/sdk/hand2_sdk_api.cpp
  src/wuji_hand2_driver/sdk/hand2_session.cpp)
target_include_directories(wuji_hand2_driver_impl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_compile_features(wuji_hand2_driver_impl PUBLIC cxx_std_17)
ament_target_dependencies(wuji_hand2_driver_impl PUBLIC
  rclcpp sensor_msgs std_msgs std_srvs wuji_msgs)
target_link_libraries(wuji_hand2_driver_impl PUBLIC
  wuji_driver_common_stream WujiSdk::C PRIVATE nlohmann_json::nlohmann_json)

add_executable(wuji_hand2_node src/main.cpp)
target_link_libraries(wuji_hand2_node PRIVATE wuji_hand2_driver_impl)
# The separately installed SDK remains an external runtime dependency.
set_target_properties(wuji_hand2_node PROPERTIES
  INSTALL_RPATH "${WUJI_SDK_LIBRARY_DIR}")
install(TARGETS wuji_hand2_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(test_name stream_controller stream_registry)
    ament_add_gtest(test_${test_name} test/common/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} wuji_driver_common_stream)
  endforeach()
  foreach(test_name joint_conversion tactile_decoder)
    ament_add_gtest(test_${test_name} test/unit/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} wuji_hand2_driver_impl)
  endforeach()
  foreach(test_name hand2_session command_and_actions
      configuration_and_calibration fingertip_stream hand2_streams hand2_startup)
    ament_add_gtest(test_${test_name} test/unit/test_${test_name}.cpp
      test/fake_sdk/fake_hand2_sdk.cpp)
    target_include_directories(test_${test_name} PRIVATE test)
    target_link_libraries(test_${test_name} wuji_hand2_driver_impl)
    set_tests_properties(test_${test_name} PROPERTIES RUN_SERIAL TRUE)
  endforeach()
  ament_add_gtest(test_hand2_node test/fake_sdk/test_hand2_node.cpp
    test/fake_sdk/fake_hand2_sdk.cpp)
  target_include_directories(test_hand2_node PRIVATE test)
  target_link_libraries(test_hand2_node wuji_hand2_driver_impl)
  set_tests_properties(test_hand2_node PROPERTIES RUN_SERIAL TRUE)

  add_executable(wuji_hand2_test_node test/fake_sdk/fake_hand2_node_main.cpp
    test/fake_sdk/fake_hand2_sdk.cpp)
  target_include_directories(wuji_hand2_test_node PRIVATE test)
  target_link_libraries(wuji_hand2_test_node PRIVATE wuji_hand2_driver_impl)
  ament_add_pytest_test(test_launch test/test_launch.py)
  ament_add_pytest_test(test_process test/test_process.py
    ENV "WUJI_HAND2_TEST_NODE=$<TARGET_FILE:wuji_hand2_test_node>"
    TIMEOUT 20)
endif()

ament_package()
