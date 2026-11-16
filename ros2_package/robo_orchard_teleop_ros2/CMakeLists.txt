cmake_minimum_required(VERSION 3.16)
project(robo_orchard_teleop_ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdfdom REQUIRED)

ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_ik
  src/ik/bindings.cpp
  src/ik/kinematics.cpp
  src/ik/solver.cpp)
target_compile_features(_ik PRIVATE cxx_std_17)
target_compile_definitions(_ik PRIVATE EIGEN_DONT_PARALLELIZE)
target_link_libraries(_ik PRIVATE
  Eigen3::Eigen pinocchio::pinocchio ${urdfdom_LIBRARIES})
target_include_directories(_ik PRIVATE ${urdfdom_INCLUDE_DIRS})
set_target_properties(_ik PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS _ik
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})

# Keep the existing ROS executable names and the selected build interpreter.
set(TELEOP_ENTRYPOINTS
  "teleop_engagement|robo_orchard_teleop_ros2.engagement"
  "pico_bridge|robo_orchard_teleop_ros2.bridge.pico.node"
  "hexfellow_chassis_pico_vr_teleop|robo_orchard_teleop_ros2.robot.hexfellow.chassis.pico_vr"
  "hexfellow_lift_pico_vr_teleop|robo_orchard_teleop_ros2.robot.hexfellow.lift.pico_vr"
  "keyboard_trigger|robo_orchard_teleop_ros2.take_over.trigger.keyboard"
  "piper_pico_vr_teleop|robo_orchard_teleop_ros2.robot.piper.pico_vr"
  "franka_pico_vr_teleop|robo_orchard_teleop_ros2.robot.franka.pico_vr"
  "robotiq_pico_vr_teleop|robo_orchard_teleop_ros2.robot.robotiq.pico_vr"
  "marvin_pico_vr_teleop|robo_orchard_teleop_ros2.robot.marvin.pico_vr"
  "teleop_keyboard_node|robo_orchard_teleop_ros2.activation.keyboard")
foreach(entry IN LISTS TELEOP_ENTRYPOINTS)
  string(REPLACE "|" ";" entry_parts "${entry}")
  list(GET entry_parts 0 entry_name)
  list(GET entry_parts 1 entry_module)
  configure_file(scripts/node_entry.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/scripts/${entry_name} @ONLY)
  install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/scripts/${entry_name}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch config resource DESTINATION share/${PROJECT_NAME})
install(FILES README.md src/ik/SCIPY_LICENSE.txt
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(teleop_tests tests
    TIMEOUT 180
    ENV "PYTHONDONTWRITEBYTECODE=1"
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}")
  ament_add_pytest_test(ik_tests test/test_ik.py
    TIMEOUT 180
    ENV "PYTHONDONTWRITEBYTECODE=1"
    APPEND_ENV "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}")
endif()

ament_package()
