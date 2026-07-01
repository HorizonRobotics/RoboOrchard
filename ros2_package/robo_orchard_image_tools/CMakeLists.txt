cmake_minimum_required(VERSION 3.8)
project(robo_orchard_image_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(compressed_image_encoder src/compressed_image_encoder.cpp)
target_compile_features(compressed_image_encoder PUBLIC cxx_std_17)
ament_target_dependencies(
  compressed_image_encoder
  cv_bridge
  rclcpp
  sensor_msgs
)
target_link_libraries(compressed_image_encoder ${OpenCV_LIBS})

install(TARGETS compressed_image_encoder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
