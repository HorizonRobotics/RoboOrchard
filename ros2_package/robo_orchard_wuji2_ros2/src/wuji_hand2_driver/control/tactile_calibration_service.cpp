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

#include "wuji_hand2_driver/control/tactile_calibration_service.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/qos.hpp>

namespace wuji_hand2_driver::control {
namespace {

constexpr const char* kStoppingMessage = "control operations are stopping";
constexpr std::uint8_t kFingerCount = 5U;

void set_response(const sdk::Status& status, std_srvs::srv::Trigger::Response& response) {
  response.success = status.ok;
  response.message =
      status.ok ? "tactile calibration completed for all five fingers" : status.message;
}

sdk::Status deadline_status(const std::string& last_timeout) {
  std::string message = "tactile calibration did not complete before the 30 second deadline";
  if (!last_timeout.empty()) {
    message += ". Last tactile status timeout: ";
    message += last_timeout;
  }
  return sdk::Status::failure(std::move(message));
}

}  // namespace

TactileCalibrationTiming TactileCalibrationTiming::production() {
  return TactileCalibrationTiming{
      []() { return Clock::now(); },
      [](Clock::duration duration) { std::this_thread::sleep_for(duration); },
      std::chrono::milliseconds{50}, std::chrono::seconds{30}};
}

class TactileCalibrationService::CallbackState {
 public:
  CallbackState(const OperationContext& context, TactileCalibrationTiming timing)
      : context_(context), timing_(std::move(timing)) {}

  void calibrate(std_srvs::srv::Trigger::Response& response) {
    sdk::Status status = sdk::Status::success();
    try {
      status = run();
    } catch (const std::exception& error) {
      status = sdk::Status::failure(std::string{"tactile calibration failed: "} + error.what());
    } catch (...) {
      status = sdk::Status::failure("tactile calibration failed: unknown exception");
    }
    finish(status, response);
  }

 private:
  sdk::Status run() {
    if (!context_.gate.accepting()) {
      return sdk::Status::failure(kStoppingMessage);
    }
    WujiDevice* const device = context_.session.device();
    if (device == nullptr) {
      return sdk::Status::failure("device is not connected");
    }

    const auto deadline = timing_.now() + timing_.timeout;
    auto status = sdk::sdk_status(context_.sdk, "wuji_hand_2_calibrate_fingertip",
                                  context_.sdk.calibrate_fingertip(device));
    if (!status) {
      return status;
    }
    std::string last_timeout;

    while (true) {
      bool all_ready = true;
      for (std::uint8_t finger = 0; finger < kFingerCount; ++finger) {
        if (timing_.now() >= deadline) {
          return deadline_status(last_timeout);
        }

        WujiFingertipSensorStatus sensor{};
        const WujiStatus result = context_.sdk.get_fingertip_status[finger](device, &sensor);
        status = sdk::sdk_status(context_.sdk, "wuji_hand_2_get_fingertip_status", result);
        if (result == WUJI_STATUS_ERR_TIMEOUT) {
          last_timeout = status.message;
          all_ready = false;
          break;
        }
        if (!status) {
          return status;
        }

        if (sensor.state == WUJI_FINGERTIP_SENSOR_STATE_READY) {
          continue;
        }
        if (sensor.state == WUJI_FINGERTIP_SENSOR_STATE_CALIBRATING) {
          all_ready = false;
          continue;
        }
        return sdk::Status::failure(
            "wuji_hand_2_get_fingertip_status returned unknown fingertip state " +
            std::to_string(static_cast<int>(sensor.state)) + " for finger " +
            std::to_string(static_cast<unsigned int>(finger)));
      }

      if (all_ready) {
        return sdk::Status::success();
      }
      const auto now = timing_.now();
      if (now < deadline) {
        timing_.sleep_for(std::min(timing_.poll_interval, deadline - now));
      }
    }
  }

  void finish(const sdk::Status& status, std_srvs::srv::Trigger::Response& response) const {
    set_response(status, response);
    if (!status) {
      RCLCPP_ERROR(context_.logger, "%s", status.message.c_str());
    }
  }

  const OperationContext context_;
  const TactileCalibrationTiming timing_;
};

TactileCalibrationService::TactileCalibrationService(const OperationContext& context,
                                                     TactileCalibrationTiming timing)
    : context_(context),
      timing_(std::move(timing)),
      callback_state_(std::make_shared<CallbackState>(context, timing_)) {}

TactileCalibrationService::~TactileCalibrationService() noexcept {
  stop();
}

sdk::Status TactileCalibrationService::start(
    const rclcpp::CallbackGroup::SharedPtr& tactile_group) {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (service_ != nullptr) {
    return sdk::Status::success();
  }
  if (tactile_group == nullptr) {
    return sdk::Status::failure("tactile calibration callback group is null");
  }
  if (!timing_.now || !timing_.sleep_for ||
      timing_.poll_interval <= TactileCalibrationTiming::Clock::duration::zero() ||
      timing_.timeout <= TactileCalibrationTiming::Clock::duration::zero()) {
    return sdk::Status::failure("tactile calibration timing is invalid");
  }
  if (!context_.gate.accepting()) {
    return sdk::Status::failure(kStoppingMessage);
  }

  try {
    const auto state = callback_state_;
    service_ = context_.node.create_service<std_srvs::srv::Trigger>(
        "calibrate_tactile",
        [state](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          state->calibrate(*response);
        },
        rmw_qos_profile_services_default, tactile_group);
    if (service_ == nullptr) {
      return sdk::Status::failure("tactile calibration service creation returned null");
    }
    if (!context_.gate.accepting()) {
      service_.reset();
      return sdk::Status::failure(kStoppingMessage);
    }
    return sdk::Status::success();
  } catch (const std::exception& error) {
    service_.reset();
    return sdk::Status::failure(error.what());
  } catch (...) {
    service_.reset();
    return sdk::Status::failure("tactile calibration service creation failed");
  }
}

void TactileCalibrationService::stop() noexcept {
  context_.gate.stop();
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  service_.reset();
}

}  // namespace wuji_hand2_driver::control
