/*
 * Project RoboOrchard
 *
 * Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>
#include <franka_semantic_components/franka_robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace robo_orchard_franka_ros2 {

class FrankaRobotStateBroadcaster
    : public controller_interface::ControllerInterface {
public:
  using CallbackReturn = controller_interface::CallbackReturn;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  CallbackReturn on_init() override;
  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

private:
  static constexpr const char *kStateInterfaceName = "robot_state";

  std::string arm_id_;
  double publish_rate_{100.0};
  std::int64_t publish_period_ns_{10'000'000};
  std::int64_t last_publish_time_ns_{0};

  std::unique_ptr<franka_semantic_components::FrankaRobotState>
      franka_robot_state_;
  std::shared_ptr<rclcpp::Publisher<franka_msgs::msg::FrankaRobotState>>
      robot_state_publisher_;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaRobotState>>
      realtime_robot_state_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>
      current_pose_publisher_;
  std::shared_ptr<
      realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>
      realtime_current_pose_publisher_;
};

} // namespace robo_orchard_franka_ros2
