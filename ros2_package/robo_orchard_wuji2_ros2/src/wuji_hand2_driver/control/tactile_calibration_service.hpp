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

#ifndef WUJI_HAND2_DRIVER__CONTROL__TACTILE_CALIBRATION_SERVICE_HPP_
#define WUJI_HAND2_DRIVER__CONTROL__TACTILE_CALIBRATION_SERVICE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/service.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "wuji_hand2_driver/control/operation_context.hpp"
#include "wuji_hand2_driver/sdk/sdk_status.hpp"

namespace wuji_hand2_driver::control {

struct TactileCalibrationTiming {
  using Clock = std::chrono::steady_clock;

  std::function<Clock::time_point()> now;
  std::function<void(Clock::duration)> sleep_for;
  Clock::duration poll_interval{std::chrono::milliseconds{50}};
  Clock::duration timeout{std::chrono::seconds{30}};

  static TactileCalibrationTiming production();
};

class TactileCalibrationService {
 public:
  explicit TactileCalibrationService(
      const OperationContext& context,
      TactileCalibrationTiming timing = TactileCalibrationTiming::production());
  ~TactileCalibrationService() noexcept;

  TactileCalibrationService(const TactileCalibrationService&) = delete;
  TactileCalibrationService& operator=(const TactileCalibrationService&) = delete;

  sdk::Status start(const rclcpp::CallbackGroup::SharedPtr& tactile_group);
  void stop() noexcept;

 private:
  class CallbackState;

  const OperationContext context_;
  const TactileCalibrationTiming timing_;
  std::shared_ptr<CallbackState> callback_state_;
  mutable std::mutex lifecycle_mutex_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

}  // namespace wuji_hand2_driver::control

#endif  // WUJI_HAND2_DRIVER__CONTROL__TACTILE_CALIBRATION_SERVICE_HPP_
