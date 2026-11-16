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

#include "wuji_hand2_driver/control/hand_configuration_services.hpp"

#include <array>
#include <cmath>
#include <cstddef>
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

sdk::Status validate_value(float value, const char* field, std::size_t index) {
  if (!std::isfinite(value) || value < 0.0F) {
    return sdk::Status::failure(std::string{field} + "[" + std::to_string(index) +
                                "] must be finite and nonnegative");
  }
  return sdk::Status::success();
}

template <std::size_t Size>
sdk::Status validate_array(const std::array<float, Size>& values, const char* field) {
  for (std::size_t index = 0; index < values.size(); ++index) {
    auto status = validate_value(values[index], field, index);
    if (!status) {
      return status;
    }
  }
  return sdk::Status::success();
}

}  // namespace

class HandConfigurationServices::CallbackState {
 public:
  explicit CallbackState(const OperationContext& context) : context_(context) {}

  void set_effort_limit(const wuji_msgs::srv::SetEffortLimit::Request& request,
                        wuji_msgs::srv::SetEffortLimit::Response& response) {
    WujiDevice* device = nullptr;
    auto status = enter(device);
    if (!status) {
      set_response(status, response);
      return;
    }
    if (!std::isfinite(request.effort_limit) || request.effort_limit < 0.0F) {
      set_response(sdk::Status::failure("effort_limit must be finite and nonnegative"), response);
      return;
    }

    const WujiStatus result = context_.sdk.set_effort_limit(device, request.effort_limit);
    status = sdk::sdk_status(context_.sdk, "wuji_hand_2_set_all_effort_limit", result);
    finish(status, response);
  }

  void set_effort_limits(const wuji_msgs::srv::SetEffortLimits::Request& request,
                         wuji_msgs::srv::SetEffortLimits::Response& response) {
    WujiDevice* device = nullptr;
    auto status = enter(device);
    if (!status) {
      set_response(status, response);
      return;
    }
    const std::array<float, conversion::JointMapping::kJointCount> limits = request.effort_limits;
    status = validate_array(limits, "effort_limits");
    if (!status) {
      set_response(status, response);
      return;
    }

    const WujiStatus result = context_.sdk.set_effort_limits(device, limits.data());
    status = sdk::sdk_status(context_.sdk, "wuji_hand_2_set_all_effort_limit_per_joint", result);
    finish(status, response);
  }

  void set_mit_params(const wuji_msgs::srv::SetMitParams::Request& request,
                      wuji_msgs::srv::SetMitParams::Response& response) {
    WujiDevice* device = nullptr;
    auto status = enter(device);
    if (!status) {
      set_response(status, response);
      return;
    }
    const std::array<float, conversion::JointMapping::kJointCount> kp = request.kp;
    const std::array<float, conversion::JointMapping::kJointCount> kd = request.kd;
    status = validate_array(kp, "kp");
    if (status) {
      status = validate_array(kd, "kd");
    }
    if (!status) {
      set_response(status, response);
      return;
    }

    const WujiStatus result = context_.sdk.set_mit_params(device, kp.data(), kd.data());
    status = sdk::sdk_status(context_.sdk, "wuji_hand_2_set_all_mit_params", result);
    finish(status, response);
  }

 private:
  sdk::Status enter(WujiDevice*& device) const {
    if (!context_.gate.accepting()) {
      return sdk::Status::failure(kStoppingMessage);
    }
    device = context_.session.device();
    if (device == nullptr) {
      return sdk::Status::failure("device is not connected");
    }
    return sdk::Status::success();
  }

  template <typename ResponseT>
  void finish(const sdk::Status& status, ResponseT& response) const {
    set_response(status, response);
    if (!status) {
      RCLCPP_ERROR(context_.logger, "%s", status.message.c_str());
    }
  }

  const OperationContext context_;
};

HandConfigurationServices::HandConfigurationServices(const OperationContext& context)
    : context_(context), callback_state_(std::make_shared<CallbackState>(context)) {}

HandConfigurationServices::~HandConfigurationServices() noexcept {
  stop();
}

sdk::Status HandConfigurationServices::start(
    const rclcpp::CallbackGroup::SharedPtr& control_group) {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (effort_limit_service_ != nullptr && effort_limits_service_ != nullptr &&
      mit_params_service_ != nullptr) {
    return sdk::Status::success();
  }
  reset_endpoints();
  if (control_group == nullptr) {
    return sdk::Status::failure("Hand configuration callback group is null");
  }
  if (context_.sdk.set_effort_limit == nullptr || context_.sdk.set_effort_limits == nullptr ||
      context_.sdk.set_mit_params == nullptr) {
    return sdk::Status::failure("Hand configuration SDK bindings are incomplete");
  }
  if (!context_.gate.accepting()) {
    return sdk::Status::failure(kStoppingMessage);
  }

  try {
    const auto state = callback_state_;
    effort_limit_service_ = context_.node.create_service<wuji_msgs::srv::SetEffortLimit>(
        "set_effort_limit",
        [state](const std::shared_ptr<wuji_msgs::srv::SetEffortLimit::Request> request,
                std::shared_ptr<wuji_msgs::srv::SetEffortLimit::Response> response) {
          state->set_effort_limit(*request, *response);
        },
        rmw_qos_profile_services_default, control_group);
    effort_limits_service_ = context_.node.create_service<wuji_msgs::srv::SetEffortLimits>(
        "set_effort_limits",
        [state](const std::shared_ptr<wuji_msgs::srv::SetEffortLimits::Request> request,
                std::shared_ptr<wuji_msgs::srv::SetEffortLimits::Response> response) {
          state->set_effort_limits(*request, *response);
        },
        rmw_qos_profile_services_default, control_group);
    mit_params_service_ = context_.node.create_service<wuji_msgs::srv::SetMitParams>(
        "set_mit_params",
        [state](const std::shared_ptr<wuji_msgs::srv::SetMitParams::Request> request,
                std::shared_ptr<wuji_msgs::srv::SetMitParams::Response> response) {
          state->set_mit_params(*request, *response);
        },
        rmw_qos_profile_services_default, control_group);
    if (effort_limit_service_ == nullptr || effort_limits_service_ == nullptr ||
        mit_params_service_ == nullptr) {
      reset_endpoints();
      return sdk::Status::failure("Hand configuration service creation returned null");
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
    return sdk::Status::failure("Hand configuration service creation failed");
  }
}

void HandConfigurationServices::stop() noexcept {
  context_.gate.stop();
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  reset_endpoints();
}

void HandConfigurationServices::reset_endpoints() noexcept {
  mit_params_service_.reset();
  effort_limits_service_.reset();
  effort_limit_service_.reset();
}

}  // namespace wuji_hand2_driver::control
