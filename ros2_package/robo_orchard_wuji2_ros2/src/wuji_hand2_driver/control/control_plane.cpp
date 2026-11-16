// Copyright 2026 Wuji Technology
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "wuji_hand2_driver/control/control_plane.hpp"

#include <utility>

namespace wuji_hand2_driver::control {

ControlPlane::ControlPlane(const OperationContext& context, TactileCalibrationTiming timing,
                           DriverConfig config)
    : context_(context),
      joint_command_(context, std::move(config)),
      hand_actions_(context),
      hand_configuration_(context),
      tactile_calibration_(context, std::move(timing)) {}

ControlPlane::~ControlPlane() noexcept {
  stop();
}

std::function<void(const WujiJointStateFrame&)> ControlPlane::feedback_observer() const {
  return joint_command_.feedback_observer();
}

sdk::Status ControlPlane::start(const rclcpp::CallbackGroup::SharedPtr& command_group,
                                const rclcpp::CallbackGroup::SharedPtr& control_group,
                                const rclcpp::CallbackGroup::SharedPtr& emergency_group,
                                const rclcpp::CallbackGroup::SharedPtr& tactile_group) {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (running_) {
    return sdk::Status::success();
  }
  if (!context_.gate.accepting()) {
    return sdk::Status::failure("control operations are stopping");
  }

  bool command_started = false;
  bool actions_started = false;
  bool configuration_started = false;

  auto status = joint_command_.start(command_group, emergency_group);
  if (!status) {
    rollback(command_started, actions_started, configuration_started);
    return status;
  }
  command_started = true;

  status = hand_actions_.start(control_group, emergency_group);
  if (!status) {
    rollback(command_started, actions_started, configuration_started);
    return status;
  }
  actions_started = true;

  status = hand_configuration_.start(control_group);
  if (!status) {
    rollback(command_started, actions_started, configuration_started);
    return status;
  }
  configuration_started = true;

  status = tactile_calibration_.start(tactile_group);
  if (!status) {
    rollback(command_started, actions_started, configuration_started);
    return status;
  }

  running_ = true;
  return sdk::Status::success();
}

void ControlPlane::rollback(bool command_started, bool actions_started,
                            bool configuration_started) noexcept {
  context_.gate.stop();
  if (configuration_started) {
    hand_configuration_.stop();
  }
  if (actions_started) {
    hand_actions_.stop();
  }
  if (command_started) {
    joint_command_.stop();
  }
  running_ = false;
}

void ControlPlane::stop() noexcept {
  context_.gate.stop();
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);

  tactile_calibration_.stop();
  hand_configuration_.stop();
  hand_actions_.stop();
  joint_command_.stop();
  running_ = false;
}

}  // namespace wuji_hand2_driver::control
