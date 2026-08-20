// Project RoboOrchard
//
// Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
// implied. See the License for the specific language governing
// permissions and limitations under the License.

#include "marvin_forward_kinematics.hpp"

#include "FxRobot.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <stdexcept>

namespace robo_orchard_marvin_ros2
{

namespace
{

constexpr double kDegreesPerRadian = 180.0 / 3.14159265358979323846;

using Matrix4Array = std::array<double, 16>;

constexpr Matrix4Array kLeftBaseToRobotStandMat = {
  1.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 1.0, 0.037,
  0.0, -1.0, 0.0, 0.14,
  0.0, 0.0, 0.0, 1.0,
};

constexpr Matrix4Array kRightBaseToRobotStandMat = {
  1.0, 0.0, 0.0, 0.0,
  0.0, 0.0, -1.0, -0.037,
  0.0, 1.0, 0.0, 0.14,
  0.0, 0.0, 0.0, 1.0,
};

std::mutex kinematics_mutex;

const Matrix4Array & arm_base_to_robot_stand_mat(std::size_t arm_index)
{
  return arm_index == 0 ?
         kLeftBaseToRobotStandMat : kRightBaseToRobotStandMat;
}

Matrix4Array multiply(
  const Matrix4Array & left, const Matrix4Array & right)
{
  Matrix4Array result{};
  for (std::size_t row = 0; row < 4; ++row) {
    for (std::size_t column = 0; column < 4; ++column) {
      for (std::size_t index = 0; index < 4; ++index) {
        result[row * 4 + column] +=
          left[row * 4 + index] * right[index * 4 + column];
      }
    }
  }
  return result;
}

std::optional<std::array<double, 4>> rotation_matrix_to_quaternion(
  const Matrix4Array & matrix)
{
  std::array<double, 4> quaternion{};
  const double trace = matrix[0] + matrix[5] + matrix[10];
  if (trace > 0.0) {
    const double scale = std::sqrt(trace + 1.0) * 2.0;
    quaternion[3] = 0.25 * scale;
    quaternion[0] = (matrix[9] - matrix[6]) / scale;
    quaternion[1] = (matrix[2] - matrix[8]) / scale;
    quaternion[2] = (matrix[4] - matrix[1]) / scale;
  } else if (matrix[0] > matrix[5] && matrix[0] > matrix[10]) {
    const double scale = std::sqrt(1.0 + matrix[0] - matrix[5] - matrix[10]) * 2.0;
    quaternion[3] = (matrix[9] - matrix[6]) / scale;
    quaternion[0] = 0.25 * scale;
    quaternion[1] = (matrix[1] + matrix[4]) / scale;
    quaternion[2] = (matrix[2] + matrix[8]) / scale;
  } else if (matrix[5] > matrix[10]) {
    const double scale = std::sqrt(1.0 + matrix[5] - matrix[0] - matrix[10]) * 2.0;
    quaternion[3] = (matrix[2] - matrix[8]) / scale;
    quaternion[0] = (matrix[1] + matrix[4]) / scale;
    quaternion[1] = 0.25 * scale;
    quaternion[2] = (matrix[6] + matrix[9]) / scale;
  } else {
    const double scale = std::sqrt(1.0 + matrix[10] - matrix[0] - matrix[5]) * 2.0;
    quaternion[3] = (matrix[4] - matrix[1]) / scale;
    quaternion[0] = (matrix[2] + matrix[8]) / scale;
    quaternion[1] = (matrix[6] + matrix[9]) / scale;
    quaternion[2] = 0.25 * scale;
  }

  double norm_squared = 0.0;
  for (const double value : quaternion) {
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
    norm_squared += value * value;
  }
  if (norm_squared <= 1e-12) {
    return std::nullopt;
  }
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  for (double & value : quaternion) {
    value *= inverse_norm;
  }
  return quaternion;
}

void initialize_kinematics(const std::string & config_path)
{
  std::lock_guard<std::mutex> lock(kinematics_mutex);
  FX_LOG_SWITCH(0);

  FX_INT32L robot_types[2]{};
  FX_DOUBLE gravity[2][3]{};
  FX_DOUBLE dh_parameters[2][8][4]{};
  FX_DOUBLE joint_limits[2][7][4]{};
  FX_DOUBLE coupled_limits[2][4][3]{};
  FX_DOUBLE masses[2][7]{};
  FX_DOUBLE mass_centers[2][7][3]{};
  FX_DOUBLE inertias[2][7][6]{};
  if (
    LOADMvCfg(
      const_cast<char *>(config_path.c_str()), robot_types, gravity,
      dh_parameters, joint_limits, coupled_limits, masses, mass_centers,
      inertias) == FX_FALSE)
  {
    throw std::runtime_error("failed to load Marvin kinematics configuration");
  }

  for (int arm_index = 0; arm_index < 2; ++arm_index) {
    if (
      FX_Robot_Init_Type(arm_index, robot_types[arm_index]) == FX_FALSE ||
      FX_Robot_Init_Kine(arm_index, dh_parameters[arm_index]) == FX_FALSE ||
      FX_Robot_Init_Lmt(
        arm_index, joint_limits[arm_index], coupled_limits[arm_index]) == FX_FALSE)
    {
      throw std::runtime_error("failed to initialize Marvin kinematics");
    }
  }
}

}  // namespace

MarvinForwardKinematics::MarvinForwardKinematics(const std::string & config_path)
{
  initialize_kinematics(config_path);
}

std::optional<EndEffectorPose> MarvinForwardKinematics::forward(
  std::size_t arm_index, const MarvinJointArray & joints_rad) const
{
  if (arm_index > 1) {
    return std::nullopt;
  }

  std::array<double, 7> joints_degrees{};
  for (std::size_t joint = 0; joint < joints_rad.size(); ++joint) {
    if (!std::isfinite(joints_rad[joint])) {
      return std::nullopt;
    }
    joints_degrees[joint] = joints_rad[joint] * kDegreesPerRadian;
  }

  Matrix4Array vendor_pose{};
  {
    std::lock_guard<std::mutex> lock(kinematics_mutex);
    Matrix4 ee_to_arm_base{};
    if (
      FX_Robot_Kine_FK(
        static_cast<FX_INT32L>(arm_index), joints_degrees.data(),
        ee_to_arm_base) == FX_FALSE)
    {
      return std::nullopt;
    }
    for (std::size_t row = 0; row < 4; ++row) {
      for (std::size_t column = 0; column < 4; ++column) {
        vendor_pose[row * 4 + column] = ee_to_arm_base[row][column];
      }
    }
  }

  for (std::size_t row = 0; row < 3; ++row) {
    vendor_pose[row * 4 + 3] /= 1000.0;
  }
  const auto ee_to_robot_stand_mat = multiply(
    arm_base_to_robot_stand_mat(arm_index), vendor_pose);
  const auto orientation = rotation_matrix_to_quaternion(ee_to_robot_stand_mat);
  if (!orientation.has_value()) {
    return std::nullopt;
  }

  EndEffectorPose pose;
  pose.position_m = {
    ee_to_robot_stand_mat[3],
    ee_to_robot_stand_mat[7],
    ee_to_robot_stand_mat[11],
  };
  if (
    !std::all_of(
      pose.position_m.begin(), pose.position_m.end(),
      [](double value) {return std::isfinite(value);}))
  {
    return std::nullopt;
  }
  pose.orientation_xyzw = *orientation;
  return pose;
}

}  // namespace robo_orchard_marvin_ros2
