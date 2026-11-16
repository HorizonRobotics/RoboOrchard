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

#include "wuji_hand2_driver/control/hand_action_services.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <exception>
#include <string>

#include <rclcpp/qos.hpp>

namespace wuji_hand2_driver::control {
namespace {

constexpr const char* kStoppingMessage = "control operations are stopping";

template <typename ResponseT>
void set_response(const sdk::Status& status, ResponseT& response) {
  response.success = status.ok;
  response.message = status.ok ? std::string{} : status.message;
}

std::array<std::uint8_t, conversion::JointMapping::kJointCount> to_sdk_mask(
    const std::array<bool, conversion::JointMapping::kJointCount>& mask) {
  std::array<std::uint8_t, conversion::JointMapping::kJointCount> converted{};
  std::transform(mask.begin(), mask.end(), converted.begin(),
                 [](bool selected) { return static_cast<std::uint8_t>(selected ? 1U : 0U); });
  return converted;
}

}  // namespace

class HandActionServices::CallbackState {
 public:
  explicit CallbackState(const OperationContext& context) : context_(context) {}

  void enable(std_srvs::srv::Trigger::Response& response) {
    wuji_msgs::srv::SetEnabled::Request request;
    request.mask.fill(true);
    request.enabled = true;
    wuji_msgs::srv::SetEnabled::Response result;
    set_enabled(request, result);
    response.success = result.success;
    response.message = result.message;
  }

  void set_enabled(const wuji_msgs::srv::SetEnabled::Request& request,
                   wuji_msgs::srv::SetEnabled::Response& response) {
    std::unique_lock<std::mutex> motion_lock(context_.gate.command_mutex, std::defer_lock);
    if (!request.enabled && std::any_of(request.mask.begin(), request.mask.end(),
                                       [](bool selected) { return selected; })) {
      motion_lock.lock();
      ++context_.gate.motion_epoch;
    }
    if (!context_.gate.accepting()) {
      set_response(sdk::Status::failure(kStoppingMessage), response);
      return;
    }
    WujiDevice* const device = context_.session.device();
    if (device == nullptr) {
      set_response(sdk::Status::failure("device is not connected"), response);
      return;
    }

    const auto mask = to_sdk_mask(request.mask);
    const WujiStatus result = request.enabled ? context_.sdk.enable(device, mask.data())
                                              : context_.sdk.disable(device, mask.data());
    const char* operation = request.enabled ? "wuji_hand_2_enable" : "wuji_hand_2_disable";
    const sdk::Status status = sdk::sdk_status(context_.sdk, operation, result);
    set_response(status, response);
    if (!status) {
      RCLCPP_ERROR(context_.logger, "%s", status.message.c_str());
    }
  }

  void clear_faults(const wuji_msgs::srv::ClearFaults::Request& request,
                    wuji_msgs::srv::ClearFaults::Response& response) {
    if (!context_.gate.accepting()) {
      set_response(sdk::Status::failure(kStoppingMessage), response);
      return;
    }
    WujiDevice* const device = context_.session.device();
    if (device == nullptr) {
      set_response(sdk::Status::failure("device is not connected"), response);
      return;
    }

    const auto mask = to_sdk_mask(request.mask);
    const WujiStatus result = context_.sdk.clear_fault(device, mask.data());
    const sdk::Status status = sdk::sdk_status(context_.sdk, "wuji_hand_2_clear_fault", result);
    set_response(status, response);
    if (!status) {
      RCLCPP_ERROR(context_.logger, "%s", status.message.c_str());
    }
  }

  void emergency_stop(std_srvs::srv::Trigger::Response& response) {
    {
      const std::lock_guard<std::mutex> motion_lock(context_.gate.command_mutex);
      ++context_.gate.motion_epoch;
    }
    if (!context_.gate.accepting()) {
      set_response(sdk::Status::failure(kStoppingMessage), response);
      return;
    }
    WujiDevice* const device = context_.session.device();
    if (device == nullptr) {
      set_response(sdk::Status::failure("device is not connected"), response);
      return;
    }

    const WujiStatus result = context_.sdk.emergency_stop(device);
    const sdk::Status status = sdk::sdk_status(context_.sdk, "wuji_hand_2_emergency_stop", result);
    set_response(status, response);
    if (!status) {
      RCLCPP_ERROR(context_.logger, "%s", status.message.c_str());
    }
  }

 private:
  const OperationContext context_;
};

HandActionServices::HandActionServices(const OperationContext& context)
    : context_(context), callback_state_(std::make_shared<CallbackState>(context)) {}

HandActionServices::~HandActionServices() noexcept {
  stop();
}

sdk::Status HandActionServices::start(const rclcpp::CallbackGroup::SharedPtr& control_group,
                                      const rclcpp::CallbackGroup::SharedPtr& emergency_group) {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (set_enabled_service_ != nullptr && enable_service_ != nullptr &&
      clear_faults_service_ != nullptr &&
      emergency_stop_service_ != nullptr) {
    return sdk::Status::success();
  }
  reset_endpoints();
  if (control_group == nullptr || emergency_group == nullptr) {
    return sdk::Status::failure("Hand action callback group is null");
  }
  if (context_.sdk.enable == nullptr || context_.sdk.disable == nullptr ||
      context_.sdk.clear_fault == nullptr || context_.sdk.emergency_stop == nullptr) {
    return sdk::Status::failure("Hand action SDK bindings are incomplete");
  }
  if (!context_.gate.accepting()) {
    return sdk::Status::failure(kStoppingMessage);
  }

  try {
    const auto state = callback_state_;
    enable_service_ = context_.node.create_service<std_srvs::srv::Trigger>(
        "enable",
        [state](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          state->enable(*response);
        },
        rmw_qos_profile_services_default, control_group);
    set_enabled_service_ = context_.node.create_service<wuji_msgs::srv::SetEnabled>(
        "set_enabled",
        [state](const std::shared_ptr<wuji_msgs::srv::SetEnabled::Request> request,
                std::shared_ptr<wuji_msgs::srv::SetEnabled::Response> response) {
          state->set_enabled(*request, *response);
        },
        rmw_qos_profile_services_default, control_group);
    clear_faults_service_ = context_.node.create_service<wuji_msgs::srv::ClearFaults>(
        "clear_faults",
        [state](const std::shared_ptr<wuji_msgs::srv::ClearFaults::Request> request,
                std::shared_ptr<wuji_msgs::srv::ClearFaults::Response> response) {
          state->clear_faults(*request, *response);
        },
        rmw_qos_profile_services_default, control_group);
    emergency_stop_service_ = context_.node.create_service<std_srvs::srv::Trigger>(
        "emergency_stop",
        [state](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          state->emergency_stop(*response);
        },
        rmw_qos_profile_services_default, emergency_group);
    if (set_enabled_service_ == nullptr || enable_service_ == nullptr ||
        clear_faults_service_ == nullptr ||
        emergency_stop_service_ == nullptr) {
      reset_endpoints();
      return sdk::Status::failure("Hand action service creation returned null");
    }
    if (!context_.gate.accepting()) {
      reset_endpoints();
      return sdk::Status::failure(kStoppingMessage);
    }
    return sdk::Status::success();
  } catch (const std::exception& error) {
    reset_endpoints();
    return sdk::Status::failure(error.what());
  } catch (...) {
    reset_endpoints();
    return sdk::Status::failure("Hand action service creation failed");
  }
}

void HandActionServices::stop() noexcept {
  context_.gate.stop();
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  reset_endpoints();
}

void HandActionServices::reset_endpoints() noexcept {
  emergency_stop_service_.reset();
  clear_faults_service_.reset();
  set_enabled_service_.reset();
  enable_service_.reset();
}

}  // namespace wuji_hand2_driver::control
