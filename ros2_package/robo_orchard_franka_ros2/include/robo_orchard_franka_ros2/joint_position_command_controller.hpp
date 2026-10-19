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

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <ruckig/ruckig.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace robo_orchard_franka_ros2 {

class JointPositionCommandController
    : public controller_interface::ControllerInterface {
public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

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
  static constexpr std::size_t kNumberOfJoints = 7;

  enum class ResetState : std::uint8_t {
    kIdle,
    kRequested,
    kRunning,
    kSucceeded,
    kFailed,
    kCancelling,
  };

  struct JointCommand {
    std::array<double, kNumberOfJoints> positions{};
    std::uint64_t sequence{0};
    bool valid{false};
    bool is_reset{false};
  };

  void commandCallback(const sensor_msgs::msg::JointState::SharedPtr message);
  void createCommandSubscription();
  void resetServiceCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool loadParameters();

  std::string arm_id_;
  std::string command_topic_;
  std::string reset_service_name_;
  std::string state_interface_;
  double velocity_limit_scale_{0.5};
  double acceleration_limit_scale_{0.5};
  double jerk_limit_scale_{0.2};
  bool enable_target_step_limit_{true};
  double max_target_step_{0.2};
  double reset_timeout_{30.0};

  std::array<std::string, kNumberOfJoints> joint_names_{};
  std::array<double, kNumberOfJoints> max_velocity_{};
  std::array<double, kNumberOfJoints> lower_limits_{};
  std::array<double, kNumberOfJoints> upper_limits_{};
  std::array<double, kNumberOfJoints> reset_joint_position_{};
  ruckig::Ruckig<kNumberOfJoints> trajectory_generator_{0.001};
  ruckig::InputParameter<kNumberOfJoints> trajectory_input_;
  ruckig::OutputParameter<kNumberOfJoints> trajectory_output_;

  realtime_tools::RealtimeBuffer<JointCommand> command_buffer_;
  std::mutex command_write_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      command_subscription_;
  std::atomic_uint64_t command_subscription_generation_{0};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;

  std::atomic_bool subscriber_is_active_{false};
  std::atomic_bool reset_cancel_requested_{false};
  std::atomic<ResetState> reset_state_{ResetState::kIdle};
  std::atomic_uint64_t next_command_sequence_{1};
  std::uint64_t last_processed_sequence_{0};
  bool initialization_required_{true};
};

} // namespace robo_orchard_franka_ros2
