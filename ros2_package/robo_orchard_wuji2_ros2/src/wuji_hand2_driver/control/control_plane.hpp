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

#ifndef WUJI_HAND2_DRIVER__CONTROL__CONTROL_PLANE_HPP_
#define WUJI_HAND2_DRIVER__CONTROL__CONTROL_PLANE_HPP_

#include <mutex>

#include <rclcpp/callback_group.hpp>

#include "wuji_hand2_driver/control/hand_action_services.hpp"
#include "wuji_hand2_driver/control/hand_configuration_services.hpp"
#include "wuji_hand2_driver/control/joint_command_controller.hpp"
#include "wuji_hand2_driver/control/operation_context.hpp"
#include "wuji_hand2_driver/control/tactile_calibration_service.hpp"
#include "wuji_hand2_driver/sdk/sdk_status.hpp"

namespace wuji_hand2_driver::control {

class ControlPlane {
 public:
  explicit ControlPlane(const OperationContext& context,
                        TactileCalibrationTiming timing = TactileCalibrationTiming::production(),
                        DriverConfig config = {});
  ~ControlPlane() noexcept;

  ControlPlane(const ControlPlane&) = delete;
  ControlPlane& operator=(const ControlPlane&) = delete;

  sdk::Status start(const rclcpp::CallbackGroup::SharedPtr& command_group,
                    const rclcpp::CallbackGroup::SharedPtr& control_group,
                    const rclcpp::CallbackGroup::SharedPtr& emergency_group,
                    const rclcpp::CallbackGroup::SharedPtr& tactile_group);
  void stop() noexcept;
  std::function<void(const WujiJointStateFrame&)> feedback_observer() const;

 private:
  void rollback(bool command_started, bool actions_started, bool configuration_started) noexcept;

  const OperationContext context_;
  JointCommandController joint_command_;
  HandActionServices hand_actions_;
  HandConfigurationServices hand_configuration_;
  TactileCalibrationService tactile_calibration_;
  mutable std::mutex lifecycle_mutex_;
  bool running_{false};
};

}  // namespace wuji_hand2_driver::control

#endif  // WUJI_HAND2_DRIVER__CONTROL__CONTROL_PLANE_HPP_
