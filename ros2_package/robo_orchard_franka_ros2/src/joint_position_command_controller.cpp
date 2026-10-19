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

#include "robo_orchard_franka_ros2/joint_position_command_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <thread>
#include <utility>

#include <franka/rate_limiting.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace robo_orchard_franka_ros2 {

controller_interface::InterfaceConfiguration
JointPositionCommandController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    configuration.names.push_back(joint_name + "/" +
                                  hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
JointPositionCommandController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration configuration;
  configuration.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    configuration.names.push_back(joint_name + "/" + state_interface_);
  }
  return configuration;
}

controller_interface::return_type
JointPositionCommandController::update(const rclcpp::Time & /*time*/,
                                       const rclcpp::Duration & /*period*/) {
  if (initialization_required_) {
    if (state_interfaces_.size() != kNumberOfJoints ||
        command_interfaces_.size() != kNumberOfJoints) {
      return controller_interface::return_type::ERROR;
    }

    for (std::size_t index = 0; index < kNumberOfJoints; ++index) {
      const double initial_position = state_interfaces_[index].get_value();
      if (!std::isfinite(initial_position)) {
        return controller_interface::return_type::ERROR;
      }

      trajectory_input_.current_position[index] = initial_position;
      trajectory_input_.current_velocity[index] = 0.0;
      trajectory_input_.current_acceleration[index] = 0.0;
      trajectory_input_.target_position[index] = initial_position;
      trajectory_input_.target_velocity[index] = 0.0;
      trajectory_input_.target_acceleration[index] = 0.0;
      trajectory_input_.max_velocity[index] =
          max_velocity_[index] * velocity_limit_scale_;
      trajectory_input_.max_acceleration[index] =
          franka::kMaxJointAcceleration[index] * acceleration_limit_scale_;
      trajectory_input_.max_jerk[index] =
          franka::kMaxJointJerk[index] * jerk_limit_scale_;
      command_interfaces_[index].set_value(initial_position);
    }

    trajectory_input_.duration_discretization =
        ruckig::DurationDiscretization::Discrete;
    initialization_required_ = false;
    return controller_interface::return_type::OK;
  }

  const JointCommand *command = command_buffer_.readFromRT();
  if (reset_cancel_requested_.exchange(false)) {
    if (command != nullptr && command->valid && command->is_reset) {
      last_processed_sequence_ = command->sequence;
    }
    trajectory_input_.target_position = trajectory_input_.current_position;
    trajectory_input_.target_velocity.fill(0.0);
    trajectory_input_.target_acceleration.fill(0.0);
    reset_state_.store(ResetState::kIdle);
  } else if (command != nullptr && command->valid &&
             command->sequence != last_processed_sequence_) {
    last_processed_sequence_ = command->sequence;

    bool target_step_is_valid = true;
    if (enable_target_step_limit_ && !command->is_reset) {
      for (std::size_t index = 0; index < kNumberOfJoints; ++index) {
        if (std::abs(command->positions[index] -
                     trajectory_input_.current_position[index]) >
            max_target_step_) {
          target_step_is_valid = false;
          break;
        }
      }
    }

    if (target_step_is_valid) {
      trajectory_input_.target_position = command->positions;
      if (command->is_reset) {
        reset_state_.store(ResetState::kRunning);
      }
    } else {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Rejected joint command: at least one target differs from "
                  "the current "
                  "commanded position by more than %.3f rad",
                  max_target_step_);
    }
  }

  const auto result =
      trajectory_generator_.update(trajectory_input_, trajectory_output_);
  if (static_cast<int>(result) < 0) {
    const auto reset_state = reset_state_.load();
    if (reset_state == ResetState::kRequested ||
        reset_state == ResetState::kRunning) {
      reset_state_.store(ResetState::kFailed);
    }
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Ruckig update failed with result %d",
                 static_cast<int>(result));
    return controller_interface::return_type::ERROR;
  }

  for (std::size_t index = 0; index < kNumberOfJoints; ++index) {
    if (!std::isfinite(trajectory_output_.new_position[index])) {
      const auto reset_state = reset_state_.load();
      if (reset_state == ResetState::kRequested ||
          reset_state == ResetState::kRunning) {
        reset_state_.store(ResetState::kFailed);
      }
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Ruckig generated a non-finite position command");
      return controller_interface::return_type::ERROR;
    }
    command_interfaces_[index].set_value(
        trajectory_output_.new_position[index]);
  }
  trajectory_output_.pass_to_input(trajectory_input_);
  if (result == ruckig::Result::Finished &&
      reset_state_.load() == ResetState::kRunning) {
    reset_state_.store(ResetState::kSucceeded);
  }

  return controller_interface::return_type::OK;
}

JointPositionCommandController::CallbackReturn
JointPositionCommandController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::string>("command_topic", "~/command");
    auto_declare<std::string>("reset_service", "~/reset_ctrl");
    auto_declare<std::string>("state_interface", "position");
    auto_declare<double>("velocity_limit_scale", 0.5);
    auto_declare<double>("acceleration_limit_scale", 0.5);
    auto_declare<double>("jerk_limit_scale", 0.2);
    auto_declare<bool>("enable_target_step_limit", true);
    auto_declare<double>("max_target_step", 0.2);
    auto_declare<double>("reset_timeout", 30.0);
    auto_declare<std::vector<double>>("reset_joint_position", {});
    auto_declare<std::vector<double>>(
        "max_velocity", {2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26});
    auto_declare<std::vector<double>>(
        "lower_limits",
        {-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159});
    auto_declare<std::vector<double>>(
        "upper_limits",
        {2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159});
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(rclcpp::get_logger("JointPositionCommandController"),
                 "Failed to declare parameters: %s", exception.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

JointPositionCommandController::CallbackReturn
JointPositionCommandController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!loadParameters()) {
    return CallbackReturn::ERROR;
  }

  for (std::size_t index = 0; index < kNumberOfJoints; ++index) {
    joint_names_[index] = arm_id_ + "_joint" + std::to_string(index + 1);
  }

  createCommandSubscription();
  reset_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
      reset_service_name_,
      std::bind(&JointPositionCommandController::resetServiceCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  command_buffer_.writeFromNonRT(JointCommand{});
  if (enable_target_step_limit_) {
    RCLCPP_INFO(get_node()->get_logger(),
                "Target step limit enabled at %.3f rad", max_target_step_);
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Target step limit disabled");
  }
  return CallbackReturn::SUCCESS;
}

JointPositionCommandController::CallbackReturn
JointPositionCommandController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  subscriber_is_active_.store(false);
  command_buffer_.writeFromNonRT(JointCommand{});
  trajectory_generator_.reset();
  reset_cancel_requested_.store(false);
  reset_state_.store(ResetState::kIdle);
  last_processed_sequence_ = 0;
  initialization_required_ = true;
  subscriber_is_active_.store(true);
  return CallbackReturn::SUCCESS;
}

JointPositionCommandController::CallbackReturn
JointPositionCommandController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  subscriber_is_active_.store(false);
  reset_cancel_requested_.store(false);
  const auto reset_state = reset_state_.load();
  if (reset_state != ResetState::kIdle) {
    reset_state_.store(ResetState::kFailed);
  }
  initialization_required_ = true;
  return CallbackReturn::SUCCESS;
}

void JointPositionCommandController::createCommandSubscription() {
  const auto generation = command_subscription_generation_.fetch_add(1) + 1;
  command_subscription_.reset();
  command_subscription_ =
      get_node()->create_subscription<sensor_msgs::msg::JointState>(
          command_topic_, rclcpp::QoS(1).reliable(),
          [this, generation](
              const sensor_msgs::msg::JointState::SharedPtr message) {
            // An executor may still hold an old intra-process waitable.
            if (generation == command_subscription_generation_.load()) {
              commandCallback(message);
            }
          });
}

void JointPositionCommandController::commandCallback(
    const sensor_msgs::msg::JointState::SharedPtr message) {
  if (!subscriber_is_active_.load() ||
      reset_state_.load() != ResetState::kIdle) {
    return;
  }

  if (message->name.size() != kNumberOfJoints ||
      message->position.size() != kNumberOfJoints) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Rejected joint command: expected exactly 7 names and 7 positions");
    return;
  }

  JointCommand command;
  for (std::size_t joint_index = 0; joint_index < kNumberOfJoints;
       ++joint_index) {
    const auto joint = std::find(message->name.begin(), message->name.end(),
                                 joint_names_[joint_index]);
    if (joint == message->name.end()) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Rejected joint command: missing joint '%s'",
                  joint_names_[joint_index].c_str());
      return;
    }

    const auto message_index =
        static_cast<std::size_t>(std::distance(message->name.begin(), joint));
    const double position = message->position[message_index];
    if (!std::isfinite(position) || position < lower_limits_[joint_index] ||
        position > upper_limits_[joint_index]) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Rejected joint command: target for '%s' is invalid or "
                  "outside limits",
                  joint_names_[joint_index].c_str());
      return;
    }
    command.positions[joint_index] = position;
  }

  command.sequence = next_command_sequence_.fetch_add(1);
  command.valid = true;
  {
    const std::lock_guard<std::mutex> lock(command_write_mutex_);
    if (!subscriber_is_active_.load() ||
        reset_state_.load() != ResetState::kIdle) {
      return;
    }
    command_buffer_.writeFromNonRT(command);
  }
}

void JointPositionCommandController::resetServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!subscriber_is_active_.load()) {
    response->success = false;
    response->message = "Reset failed: controller is not active.";
    return;
  }

  JointCommand command;
  command.positions = reset_joint_position_;
  command.sequence = next_command_sequence_.fetch_add(1);
  command.valid = true;
  command.is_reset = true;
  {
    const std::lock_guard<std::mutex> lock(command_write_mutex_);
    ResetState expected_state = ResetState::kIdle;
    if (!subscriber_is_active_.load() ||
        !reset_state_.compare_exchange_strong(expected_state,
                                              ResetState::kRequested)) {
      response->success = false;
      response->message = "Reset failed: another reset is already in progress.";
      return;
    }
    command_buffer_.writeFromNonRT(command);
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Resetting arm to configured home pose");
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(reset_timeout_);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto reset_state = reset_state_.load();
    if (reset_state == ResetState::kSucceeded ||
        reset_state == ResetState::kFailed || !subscriber_is_active_.load()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  const auto reset_state = reset_state_.load();
  if (reset_state == ResetState::kSucceeded) {
    reset_state_.store(ResetState::kIdle);
    response->success = true;
    response->message = "Arm reset successfully.";
  } else if (reset_state == ResetState::kFailed ||
             !subscriber_is_active_.load()) {
    reset_state_.store(ResetState::kIdle);
    response->success = false;
    response->message = "Reset failed while moving to the home pose.";
  } else {
    reset_state_.store(ResetState::kCancelling);
    reset_cancel_requested_.store(true);
    {
      const std::lock_guard<std::mutex> lock(command_write_mutex_);
      command_buffer_.writeFromNonRT(JointCommand{});
    }
    const auto cancellation_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < cancellation_deadline &&
           reset_state_.load() == ResetState::kCancelling) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    response->success = false;
    response->message = "Reset timed out before reaching the home pose.";
  }

  // The mutually exclusive service callback prevents command callbacks from
  // draining their queue during reset. Discard that queue before returning.
  createCommandSubscription();
}

bool JointPositionCommandController::loadParameters() {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  reset_service_name_ = get_node()->get_parameter("reset_service").as_string();
  state_interface_ = get_node()->get_parameter("state_interface").as_string();
  velocity_limit_scale_ =
      get_node()->get_parameter("velocity_limit_scale").as_double();
  acceleration_limit_scale_ =
      get_node()->get_parameter("acceleration_limit_scale").as_double();
  jerk_limit_scale_ =
      get_node()->get_parameter("jerk_limit_scale").as_double();
  enable_target_step_limit_ =
      get_node()->get_parameter("enable_target_step_limit").as_bool();
  max_target_step_ = get_node()->get_parameter("max_target_step").as_double();
  reset_timeout_ = get_node()->get_parameter("reset_timeout").as_double();
  const auto reset_joint_position =
      get_node()->get_parameter("reset_joint_position").as_double_array();
  const auto max_velocity =
      get_node()->get_parameter("max_velocity").as_double_array();
  const auto lower_limits =
      get_node()->get_parameter("lower_limits").as_double_array();
  const auto upper_limits =
      get_node()->get_parameter("upper_limits").as_double_array();

  if (arm_id_.empty() || command_topic_.empty() ||
      reset_service_name_.empty() || state_interface_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "arm_id, command_topic, reset_service and state_interface "
                 "must be set");
    return false;
  }
  if (!std::isfinite(velocity_limit_scale_) ||
      velocity_limit_scale_ <= 0.0 || velocity_limit_scale_ > 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "velocity_limit_scale must be finite and in (0, 1]");
    return false;
  }
  if (!std::isfinite(acceleration_limit_scale_) ||
      acceleration_limit_scale_ <= 0.0 ||
      acceleration_limit_scale_ > 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "acceleration_limit_scale must be finite and in (0, 1]");
    return false;
  }
  if (!std::isfinite(jerk_limit_scale_) || jerk_limit_scale_ <= 0.0 ||
      jerk_limit_scale_ > 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "jerk_limit_scale must be finite and in (0, 1]");
    return false;
  }
  if (!std::isfinite(max_target_step_) ||
      (enable_target_step_limit_ && max_target_step_ <= 0.0)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "max_target_step must be positive and finite");
    return false;
  }
  if (!std::isfinite(reset_timeout_) || reset_timeout_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "reset_timeout must be positive and finite");
    return false;
  }
  if (max_velocity.size() != kNumberOfJoints ||
      lower_limits.size() != kNumberOfJoints ||
      upper_limits.size() != kNumberOfJoints ||
      reset_joint_position.size() != kNumberOfJoints) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "max_velocity, lower_limits, upper_limits and reset_joint_position "
        "must contain 7 values");
    return false;
  }

  for (std::size_t index = 0; index < kNumberOfJoints; ++index) {
    if (!std::isfinite(max_velocity[index]) || max_velocity[index] <= 0.0) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid maximum velocity for joint %zu", index + 1);
      return false;
    }
    if (!std::isfinite(lower_limits[index]) ||
        !std::isfinite(upper_limits[index]) ||
        lower_limits[index] >= upper_limits[index]) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid position limits for joint %zu", index + 1);
      return false;
    }
    if (!std::isfinite(reset_joint_position[index]) ||
        reset_joint_position[index] < lower_limits[index] ||
        reset_joint_position[index] > upper_limits[index]) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Invalid reset position for joint %zu", index + 1);
      return false;
    }
    max_velocity_[index] = max_velocity[index];
    lower_limits_[index] = lower_limits[index];
    upper_limits_[index] = upper_limits[index];
    reset_joint_position_[index] = reset_joint_position[index];
  }
  return true;
}

} // namespace robo_orchard_franka_ros2

PLUGINLIB_EXPORT_CLASS(robo_orchard_franka_ros2::JointPositionCommandController,
                       controller_interface::ControllerInterface)
