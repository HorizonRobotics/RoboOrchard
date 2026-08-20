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

#ifndef MARVIN_FORWARD_KINEMATICS_HPP_
#define MARVIN_FORWARD_KINEMATICS_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <string>

namespace robo_orchard_marvin_ros2
{

using MarvinJointArray = std::array<double, 7>;

struct EndEffectorPose
{
  std::array<double, 3> position_m{};
  std::array<double, 4> orientation_xyzw{};
};

class MarvinForwardKinematics
{
public:
  explicit MarvinForwardKinematics(const std::string & config_path);

  std::optional<EndEffectorPose> forward(
    std::size_t arm_index, const MarvinJointArray & joints_rad) const;
};

}  // namespace robo_orchard_marvin_ros2

#endif  // MARVIN_FORWARD_KINEMATICS_HPP_
