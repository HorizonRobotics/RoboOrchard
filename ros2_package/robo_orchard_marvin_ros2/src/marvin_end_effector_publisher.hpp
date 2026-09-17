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

#ifndef MARVIN_END_EFFECTOR_PUBLISHER_HPP_
#define MARVIN_END_EFFECTOR_PUBLISHER_HPP_

#include "marvin_forward_kinematics.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace robo_orchard_marvin_ros2
{

class MarvinEndEffectorPublisher
{
public:
  explicit MarvinEndEffectorPublisher(rclcpp::Node & node);

  void publish(
    std::size_t arm_index, const std::optional<EndEffectorPose> & pose,
    const rclcpp::Time & stamp);

private:
  struct ArmFrames
  {
    std::string base_frame_id;
    std::string ee_frame_id;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  };

  std::array<ArmFrames, 2> arms_{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace robo_orchard_marvin_ros2

#endif  // MARVIN_END_EFFECTOR_PUBLISHER_HPP_
