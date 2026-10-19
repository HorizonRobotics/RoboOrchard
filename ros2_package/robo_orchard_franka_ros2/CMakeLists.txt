cmake_minimum_required(VERSION 3.8)
project(robo_orchard_franka_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(Franka 0.13.2 REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(franka_semantic_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ruckig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(
  ${PROJECT_NAME}_controller SHARED
  src/franka_robot_state_broadcaster.cpp
  src/joint_position_command_controller.cpp
)
target_compile_features(${PROJECT_NAME}_controller PUBLIC cxx_std_17)
target_include_directories(
  ${PROJECT_NAME}_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${PROJECT_NAME}_controller
  controller_interface
  franka_msgs
  franka_semantic_components
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_srvs
)
target_link_libraries(
  ${PROJECT_NAME}_controller
  Franka::Franka
  ruckig::ruckig
)

pluginlib_export_plugin_description_file(
  controller_interface robo_orchard_franka_ros2.xml
)

install(
  TARGETS ${PROJECT_NAME}_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_controller)
ament_export_dependencies(
  controller_interface
  franka_msgs
  franka_semantic_components
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  ruckig
  sensor_msgs
  std_srvs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  ament_add_gtest(
    joint_position_command_controller_test
    tests/joint_position_command_controller_test.cpp
    TIMEOUT 60
    APPEND_ENV ROS_LOCALHOST_ONLY=1
  )
  if(TARGET joint_position_command_controller_test)
    target_link_libraries(
      joint_position_command_controller_test ${PROJECT_NAME}_controller
    )
    ament_target_dependencies(
      joint_position_command_controller_test
      controller_interface hardware_interface lifecycle_msgs
      rclcpp rclcpp_lifecycle sensor_msgs std_srvs
    )
  endif()
endif()

ament_package()
