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

#ifndef WUJI_HAND2_DRIVER__CONTROL__JOINT_COMMAND_CONTROLLER_HPP_
#define WUJI_HAND2_DRIVER__CONTROL__JOINT_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <functional>
#include <mutex>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/service.hpp>

#include "wuji_hand2_driver/control/operation_context.hpp"
#include "wuji_hand2_driver/sdk/sdk_status.hpp"
#include "wuji_hand2_driver/driver_config.hpp"

namespace wuji_hand2_driver::control {

class JointCommandController {
 public:
  explicit JointCommandController(const OperationContext& context, DriverConfig config = {});
  ~JointCommandController() noexcept;

  JointCommandController(const JointCommandController&) = delete;
  JointCommandController& operator=(const JointCommandController&) = delete;

  sdk::Status start(const rclcpp::CallbackGroup::SharedPtr& command_group,
                    const rclcpp::CallbackGroup::SharedPtr& reset_group);
  void stop() noexcept;
  std::function<void(const WujiJointStateFrame&)> feedback_observer() const;

 private:
  class CallbackState;

  const OperationContext context_;
  std::shared_ptr<CallbackState> callback_state_;
  mutable std::mutex lifecycle_mutex_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::CallbackGroup::SharedPtr reset_group_;
};

}  // namespace wuji_hand2_driver::control

#endif  // WUJI_HAND2_DRIVER__CONTROL__JOINT_COMMAND_CONTROLLER_HPP_
