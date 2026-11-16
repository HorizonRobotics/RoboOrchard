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

#ifndef WUJI_HAND2_DRIVER__CONTROL__HAND_ACTION_SERVICES_HPP_
#define WUJI_HAND2_DRIVER__CONTROL__HAND_ACTION_SERVICES_HPP_

#include <memory>
#include <mutex>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wuji_msgs/srv/clear_faults.hpp>
#include <wuji_msgs/srv/set_enabled.hpp>

#include "wuji_hand2_driver/control/operation_context.hpp"
#include "wuji_hand2_driver/sdk/sdk_status.hpp"

namespace wuji_hand2_driver::control {

class HandActionServices {
 public:
  explicit HandActionServices(const OperationContext& context);
  ~HandActionServices() noexcept;

  HandActionServices(const HandActionServices&) = delete;
  HandActionServices& operator=(const HandActionServices&) = delete;

  sdk::Status start(const rclcpp::CallbackGroup::SharedPtr& control_group,
                    const rclcpp::CallbackGroup::SharedPtr& emergency_group);
  void stop() noexcept;

 private:
  class CallbackState;

  void reset_endpoints() noexcept;

  const OperationContext context_;
  std::shared_ptr<CallbackState> callback_state_;
  mutable std::mutex lifecycle_mutex_;
  rclcpp::Service<wuji_msgs::srv::SetEnabled>::SharedPtr set_enabled_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<wuji_msgs::srv::ClearFaults>::SharedPtr clear_faults_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
};

}  // namespace wuji_hand2_driver::control

#endif  // WUJI_HAND2_DRIVER__CONTROL__HAND_ACTION_SERVICES_HPP_
