cmake_minimum_required(VERSION 3.16)
project(wuji_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FingertipTactile.msg"
  "msg/JointDiagnostics.msg"
  "msg/JointDiagnosticsArray.msg"
  "srv/SetEnabled.srv"
  "srv/ClearFaults.srv"
  "srv/SetEffortLimit.srv"
  "srv/SetEffortLimits.srv"
  "srv/SetMitParams.srv"
  DEPENDENCIES geometry_msgs std_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
