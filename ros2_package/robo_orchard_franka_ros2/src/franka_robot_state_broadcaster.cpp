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

#include "robo_orchard_franka_ros2/franka_robot_state_broadcaster.hpp"

#include <cmath>
#include <exception>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace robo_orchard_franka_ros2 {

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  if (franka_robot_state_) {
    configuration.names = franka_robot_state_->get_state_interface_names();
  }
  return configuration;
}

controller_interface::return_type
FrankaRobotStateBroadcaster::update(const rclcpp::Time &time,
                                    const rclcpp::Duration & /*period*/) {
  const std::int64_t current_time_ns = time.nanoseconds();
  if (last_publish_time_ns_ != 0 &&
      current_time_ns - last_publish_time_ns_ < publish_period_ns_) {
    return controller_interface::return_type::OK;
  }

  if (!realtime_robot_state_publisher_ ||
      !realtime_robot_state_publisher_->trylock()) {
    return controller_interface::return_type::OK;
  }

  auto &robot_state_message = realtime_robot_state_publisher_->msg_;
  robot_state_message.header.stamp = time;
  if (!franka_robot_state_->get_values_as_message(robot_state_message)) {
    realtime_robot_state_publisher_->unlock();
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                          1000,
                          "Failed to read the Franka robot state interface");
    return controller_interface::return_type::ERROR;
  }

  if (realtime_current_pose_publisher_ &&
      realtime_current_pose_publisher_->trylock()) {
    realtime_current_pose_publisher_->msg_ = robot_state_message.o_t_ee;
    realtime_current_pose_publisher_->unlockAndPublish();
  }

  realtime_robot_state_publisher_->unlockAndPublish();
  last_publish_time_ns_ = current_time_ns;
  return controller_interface::return_type::OK;
}

FrankaRobotStateBroadcaster::CallbackReturn
FrankaRobotStateBroadcaster::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("publish_rate", 100.0);
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("FrankaRobotStateBroadcaster"),
                 "Failed to declare parameters: %s", exception.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

FrankaRobotStateBroadcaster::CallbackReturn
FrankaRobotStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
  if (arm_id_.empty() || !std::isfinite(publish_rate_) ||
      publish_rate_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "arm_id must be non-empty and publish_rate must be positive");
    return CallbackReturn::ERROR;
  }
  publish_period_ns_ = static_cast<std::int64_t>(1.0e9 / publish_rate_);

  std::string robot_description;
  if (!get_node()->get_parameter("robot_description", robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get robot_description parameter");
    return CallbackReturn::ERROR;
  }

  try {
    franka_robot_state_ =
        std::make_unique<franka_semantic_components::FrankaRobotState>(
            arm_id_ + "/" + kStateInterfaceName, robot_description);

    const auto qos = rclcpp::QoS(1).reliable();
    robot_state_publisher_ =
        get_node()->create_publisher<franka_msgs::msg::FrankaRobotState>(
            "~/robot_state", qos);
    realtime_robot_state_publisher_ = std::make_shared<
        realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaRobotState>>(
        robot_state_publisher_);
    franka_robot_state_->initialize_robot_state_msg(
        realtime_robot_state_publisher_->msg_);

    current_pose_publisher_ =
        get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/current_pose", qos);
    realtime_current_pose_publisher_ = std::make_shared<
        realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
        current_pose_publisher_);
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to configure Franka state publishers: %s",
                 exception.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Publishing Franka robot state at %.1f Hz", publish_rate_);
  return CallbackReturn::SUCCESS;
}

FrankaRobotStateBroadcaster::CallbackReturn
FrankaRobotStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  last_publish_time_ns_ = 0;
  return CallbackReturn::SUCCESS;
}

FrankaRobotStateBroadcaster::CallbackReturn
FrankaRobotStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  franka_robot_state_->release_interfaces();
  last_publish_time_ns_ = 0;
  return CallbackReturn::SUCCESS;
}

} // namespace robo_orchard_franka_ros2

PLUGINLIB_EXPORT_CLASS(robo_orchard_franka_ros2::FrankaRobotStateBroadcaster,
                       controller_interface::ControllerInterface)
