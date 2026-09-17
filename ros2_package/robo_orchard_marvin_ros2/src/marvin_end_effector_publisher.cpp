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

#include "marvin_end_effector_publisher.hpp"

#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace robo_orchard_marvin_ros2
{

MarvinEndEffectorPublisher::MarvinEndEffectorPublisher(rclcpp::Node & node)
{
  const bool publish_ee_tf = node.declare_parameter<bool>("publish_ee_tf", true);
  const std::array<std::string, 2> sides{"left", "right"};
  const std::array<std::string, 2> default_ee_frames{"TCP_Link_L", "TCP_Link_R"};
  for (std::size_t arm_index = 0; arm_index < arms_.size(); ++arm_index) {
    auto & arm = arms_[arm_index];
    const auto & side = sides[arm_index];
    arm.base_frame_id = node.declare_parameter<std::string>(
      side + "_base_frame_id", "robot_stand");
    arm.ee_frame_id = node.declare_parameter<std::string>(
      side + "_ee_frame_id", default_ee_frames[arm_index]);
    if (
      arm.base_frame_id.find_first_not_of(" \t\r\n") == std::string::npos ||
      arm.ee_frame_id.find_first_not_of(" \t\r\n") == std::string::npos)
    {
      throw std::invalid_argument(
              side + "_base_frame_id and " + side + "_ee_frame_id must be non-empty");
    }
    if (arm.base_frame_id == arm.ee_frame_id) {
      throw std::invalid_argument(
              side + "_base_frame_id and " + side + "_ee_frame_id must differ");
    }
  }
  if (arms_[0].ee_frame_id == arms_[1].ee_frame_id) {
    throw std::invalid_argument("left_ee_frame_id and right_ee_frame_id must differ");
  }

  for (std::size_t arm_index = 0; arm_index < arms_.size(); ++arm_index) {
    arms_[arm_index].pose_publisher =
      node.create_publisher<geometry_msgs::msg::PoseStamped>(
      "/robot/" + sides[arm_index] + "/ee_pose", 10);
  }
  if (publish_ee_tf) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  }
}

void MarvinEndEffectorPublisher::publish(
  std::size_t arm_index, const std::optional<EndEffectorPose> & pose,
  const rclcpp::Time & stamp)
{
  if (!pose.has_value()) {
    return;
  }
  const auto & arm = arms_.at(arm_index);
  geometry_msgs::msg::PoseStamped ee_pose;
  ee_pose.header.stamp = stamp;
  ee_pose.header.frame_id = arm.base_frame_id;
  ee_pose.pose.position.x = pose->position_m[0];
  ee_pose.pose.position.y = pose->position_m[1];
  ee_pose.pose.position.z = pose->position_m[2];
  ee_pose.pose.orientation.x = pose->orientation_xyzw[0];
  ee_pose.pose.orientation.y = pose->orientation_xyzw[1];
  ee_pose.pose.orientation.z = pose->orientation_xyzw[2];
  ee_pose.pose.orientation.w = pose->orientation_xyzw[3];
  arm.pose_publisher->publish(ee_pose);

  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = ee_pose.header;
    transform.child_frame_id = arm.ee_frame_id;
    transform.transform.translation.x = ee_pose.pose.position.x;
    transform.transform.translation.y = ee_pose.pose.position.y;
    transform.transform.translation.z = ee_pose.pose.position.z;
    transform.transform.rotation = ee_pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
}

}  // namespace robo_orchard_marvin_ros2
