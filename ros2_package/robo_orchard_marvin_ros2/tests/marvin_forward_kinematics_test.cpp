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

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

namespace
{

using robo_orchard_marvin_ros2::EndEffectorPose;
using robo_orchard_marvin_ros2::MarvinForwardKinematics;
using robo_orchard_marvin_ros2::MarvinJointArray;

constexpr std::array<MarvinJointArray, 2> kReferenceJoints = {{
  {
    1.529088193659988, -1.4519708206606186, -1.5572770064089485,
    -1.5387608083745408, -1.7303194204271783, 0.06482327374832139,
    -0.04159817739203285,
  },
  {
    -1.6407159619590415, -1.37823240509311, 1.5677856338352063,
    -1.484165164042906, 1.9133154471695317, -0.0002548180707911721,
    0.13003226526133355,
  },
}};

constexpr std::array<std::array<double, 3>, 2> kExpectedPositions = {{
  {0.43862824512749776, 0.23750579033548468, -0.1542936688028004},
  {0.40058642665103467, -0.2772349243869303, -0.21091606565048904},
}};

constexpr std::array<std::array<double, 4>, 2> kExpectedOrientations = {{
  {0.49469944481185246, 0.4786025728980184, 0.48331228929888453,
    0.5409447915738845},
  {0.4591832602897753, -0.5501018474510744, 0.4645087949995592,
    -0.5203559073063599},
}};

double quaternion_norm(const EndEffectorPose & pose)
{
  double norm_squared = 0.0;
  for (const double value : pose.orientation_xyzw) {
    norm_squared += value * value;
  }
  return std::sqrt(norm_squared);
}

}  // namespace

TEST(MarvinForwardKinematicsTest, ConvertsVendorPoseToRobotStand) {
  const MarvinForwardKinematics kinematics(MARVIN_KINEMATICS_CONFIG);
  for (std::size_t arm_index = 0; arm_index < kReferenceJoints.size(); ++arm_index) {
    const auto pose = kinematics.forward(arm_index, kReferenceJoints[arm_index]);
    ASSERT_TRUE(pose.has_value());
    for (std::size_t axis = 0; axis < 3; ++axis) {
      EXPECT_NEAR(pose->position_m[axis], kExpectedPositions[arm_index][axis], 1e-9);
    }
    double orientation_dot = 0.0;
    for (std::size_t axis = 0; axis < 4; ++axis) {
      orientation_dot +=
        pose->orientation_xyzw[axis] * kExpectedOrientations[arm_index][axis];
    }
    EXPECT_NEAR(std::abs(orientation_dot), 1.0, 1e-9);
    EXPECT_NEAR(quaternion_norm(*pose), 1.0, 1e-12);
  }
}

TEST(MarvinForwardKinematicsTest, RejectsInvalidFeedback) {
  const MarvinForwardKinematics kinematics(MARVIN_KINEMATICS_CONFIG);
  EXPECT_FALSE(kinematics.forward(2, MarvinJointArray{}).has_value());

  MarvinJointArray invalid_feedback{};
  invalid_feedback[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(kinematics.forward(0, invalid_feedback).has_value());
}
