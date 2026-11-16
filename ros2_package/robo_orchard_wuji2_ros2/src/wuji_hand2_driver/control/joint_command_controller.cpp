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

#include "wuji_hand2_driver/control/joint_command_controller.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <rclcpp/context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>

#include "wuji_hand2_driver/conversion/joint_command_conversion.hpp"

namespace wuji_hand2_driver::control {

class JointCommandController::CallbackState : public std::enable_shared_from_this<CallbackState> {
 public:
  using Clock = std::chrono::steady_clock;

  CallbackState(const OperationContext& context, DriverConfig config)
      : context_(context), config_(std::move(config)) {}

  void subscribe(const rclcpp::CallbackGroup::SharedPtr& group) {
    const std::lock_guard<std::mutex> lock(context_.gate.command_mutex);
    command_group_ = group;
    subscribe_locked();
  }

  void stop() {
    std::unique_lock<std::mutex> lock(context_.gate.command_mutex);
    subscription_.reset();
    ++subscription_epoch_;
    finished_.wait(lock, [this]() { return !reset_active_; });
  }

  void observe(const WujiJointStateFrame& frame) {
    const std::lock_guard<std::mutex> lock(feedback_mutex_);
    // Sparse frames remain legal public state, but cannot prove a full-hand
    // reset origin/arrival. Never merge partial frames from different times.
    if (frame.header.timestamp_us <= device_timestamp_) {
      return;
    }
    device_timestamp_ = frame.header.timestamp_us;
    complete_ = false;
    if (frame.joints_len != conversion::JointMapping::kJointCount || frame.joints == nullptr ||
        frame.num_joints != frame.joints_len) {
      return;
    }
    std::array<bool, conversion::JointMapping::kJointCount> seen{};
    for (std::size_t item = 0; item < frame.joints_len; ++item) {
      const auto& joint = frame.joints[item];
      const auto index = context_.mapping.index_from_nid(joint.nid);
      if (!index || seen[*index] || !std::isfinite(joint.position)) {
        return;
      }
      seen[*index] = true;
      positions_[*index] = joint.position;
    }
    complete_ = true;
    received_at_ = Clock::now();
  }

  void reset(std_srvs::srv::Trigger::Response& response) {
    std::uint64_t epoch;
    {
      const std::lock_guard<std::mutex> lock(context_.gate.command_mutex);
      if (reset_active_) {
        response.message = "reset is already in progress";
        return;
      }
      if (!context_.gate.accepting()) {
        response.message = "control operations are stopping";
        return;
      }
      reset_active_ = true;
      epoch = context_.gate.motion_epoch;
      // Discard the DDS queue, and invalidate even callbacks already taken
      // by the executor. A new subscription is admitted only after reset exits.
      ++subscription_epoch_;
      subscription_.reset();
    }
    sdk::Status status;
    try {
      status = move_to_reset(epoch);
    } catch (const std::exception& error) {
      status = sdk::Status::failure(error.what());
    } catch (...) {
      status = sdk::Status::failure("reset raised an unknown exception");
    }
    {
      const std::lock_guard<std::mutex> lock(context_.gate.command_mutex);
      // Emergency/shutdown may have arrived after the final feedback check.
      const bool context_valid = context_.node.get_node_base_interface()->get_context()->is_valid();
      if (!context_.gate.accepting() || epoch != context_.gate.motion_epoch || !context_valid) {
        status = sdk::Status::failure("reset interrupted by stop, disable or shutdown");
      }
      if (context_.gate.accepting() && context_valid) {
        try {
          subscribe_locked();
        } catch (const std::exception& error) {
          context_.gate.stop();
          status = sdk::Status::failure(
              std::string{"command subscription restore failed: "} + error.what());
        }
      }
      reset_active_ = false;
      finished_.notify_all();
    }
    response.success = status.ok;
    response.message = status.ok ? "reset target reached using fresh joint feedback" : status.message;
    if (!status) {
      RCLCPP_ERROR(context_.logger, "reset failed: %s", status.message.c_str());
    }
  }

  void receive(const sensor_msgs::msg::JointState& message, std::uint64_t epoch) {
    const std::lock_guard<std::mutex> lock(context_.gate.command_mutex);
    if (epoch != subscription_epoch_ || reset_active_) {
      return;
    }
    if (!context_.gate.accepting()) {
      report_failure("control operations are stopping");
      return;
    }

    std::array<WujiJointCommand, conversion::JointMapping::kJointCount> commands{};
    sdk::Status status = conversion::to_joint_commands(message, context_.mapping, commands);
    if (!status) {
      report_failure(status.message);
      return;
    }

    status = context_.session.send(commands);
    if (!status) {
      report_failure(status.message);
    }
  }

 private:
  void subscribe_locked() {
    if (subscription_) {
      return;
    }
    rclcpp::SubscriptionOptions options;
    options.callback_group = command_group_;
    const std::weak_ptr<CallbackState> weak = shared_from_this();
    const auto epoch = subscription_epoch_;
    subscription_ = context_.node.create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands", rclcpp::SensorDataQoS().keep_last(1),
        [weak, epoch](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          if (const auto state = weak.lock()) {
            state->receive(*message, epoch);
          }
        }, options);
    if (!subscription_) {
      throw std::runtime_error("joint command subscription creation returned null");
    }
  }

  sdk::Status snapshot(std::array<float, conversion::JointMapping::kJointCount>& positions,
                        Clock::time_point& received_at) {
    const std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!complete_) {
      return sdk::Status::failure("reset requires a complete 20-joint feedback frame");
    }
    if (std::chrono::duration<double>(Clock::now() - received_at_).count() >
        config_.reset_feedback_timeout_s) {
      return sdk::Status::failure("reset joint feedback is stale");
    }
    positions = positions_;
    received_at = received_at_;
    return sdk::Status::success();
  }

  sdk::Status move_to_reset(std::uint64_t epoch) {
    std::array<float, conversion::JointMapping::kJointCount> origin{};
    Clock::time_point received_at;
    auto status = snapshot(origin, received_at);
    if (!status) {
      return status;
    }
    const auto started = Clock::now();
    std::optional<Clock::time_point> final_sent;
    bool first_sample = true;
    while (true) {
      const auto now = Clock::now();
      {
        const std::lock_guard<std::mutex> lock(context_.gate.command_mutex);
        if (!context_.gate.accepting() || epoch != context_.gate.motion_epoch ||
            !context_.node.get_node_base_interface()->get_context()->is_valid()) {
          return sdk::Status::failure("reset interrupted by stop, disable or shutdown");
        }
        if (!context_.session.is_connected()) {
          return sdk::Status::failure("reset device disconnected");
        }
        std::array<float, conversion::JointMapping::kJointCount> actual{};
        status = snapshot(actual, received_at);
        if (!status) {
          return status;
        }
        bool arrived = true;
        const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
        if (elapsed >= config_.reset_timeout_s) {
          return sdk::Status::failure("reset arrival timed out; enable the hand before resetting");
        }
        for (std::size_t index = 0; index < actual.size(); ++index) {
          arrived = arrived && std::abs(actual[index] - config_.reset_joint_position[index]) <=
                                   config_.reset_position_tolerance;
        }
        if ((final_sent && received_at > *final_sent && arrived) ||
            (first_sample && arrived)) {
          return sdk::Status::success();
        }
        first_sample = false;
        const double t = std::min(1.0, elapsed / config_.reset_duration_s);
        // Quintic smoothstep: zero commanded slope/acceleration at both ends.
        const double blend = t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
        std::array<WujiJointCommand, conversion::JointMapping::kJointCount> commands{};
        for (std::size_t index = 0; index < commands.size(); ++index) {
          commands[index].position = static_cast<float>(
              origin[index] + (config_.reset_joint_position[index] - origin[index]) * blend);
        }
        status = context_.session.send(commands);
        if (!status) {
          return status;
        }
        if (t >= 1.0 && !final_sent) {
          final_sent = Clock::now();
        }
      }
      std::this_thread::sleep_until(now + std::chrono::milliseconds(10));
    }
  }

  void report_failure(const std::string& message) {
    RCLCPP_ERROR_THROTTLE(context_.logger, *context_.node.get_clock(), 1000,
                          "joint command rejected: %s", message.c_str());
  }

  const OperationContext context_;
  const DriverConfig config_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::uint64_t subscription_epoch_{0};
  bool reset_active_{false};
  std::condition_variable finished_;
  std::mutex feedback_mutex_;
  std::array<float, conversion::JointMapping::kJointCount> positions_{};
  Clock::time_point received_at_{};
  std::uint64_t device_timestamp_{0};
  bool complete_{false};
};

JointCommandController::JointCommandController(const OperationContext& context, DriverConfig config)
    : context_(context), callback_state_(std::make_shared<CallbackState>(context, std::move(config))) {}

std::function<void(const WujiJointStateFrame&)> JointCommandController::feedback_observer() const {
  const auto state = callback_state_;
  return [state](const WujiJointStateFrame& frame) { state->observe(frame); };
}

JointCommandController::~JointCommandController() noexcept {
  stop();
}

sdk::Status JointCommandController::start(const rclcpp::CallbackGroup::SharedPtr& command_group,
                                          const rclcpp::CallbackGroup::SharedPtr& reset_group) {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (reset_service_ != nullptr) {
    return sdk::Status::success();
  }
  if (command_group == nullptr || reset_group == nullptr) {
    return sdk::Status::failure("joint command or reset callback group is null");
  }
  if (!context_.gate.accepting()) {
    return sdk::Status::failure("control operations are stopping");
  }

  try {
    const auto state = callback_state_;
    state->subscribe(command_group);
    reset_group_ = reset_group;
    reset_service_ = context_.node.create_service<std_srvs::srv::Trigger>(
        "reset_ctrl",
        [state](std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response) { state->reset(*response); },
        rmw_qos_profile_services_default, reset_group_);
    if (!reset_service_) {
      throw std::runtime_error("reset service creation returned null");
    }
    if (!context_.gate.accepting()) {
      callback_state_->stop();
      reset_service_.reset();
      return sdk::Status::failure("control operations are stopping");
    }
    return sdk::Status::success();
  } catch (const std::exception& error) {
    callback_state_->stop();
    reset_service_.reset();
    return sdk::Status::failure(error.what());
  } catch (...) {
    callback_state_->stop();
    reset_service_.reset();
    return sdk::Status::failure("joint command subscription creation failed");
  }
}

void JointCommandController::stop() noexcept {
  context_.gate.stop();
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  callback_state_->stop();
  reset_service_.reset();
  reset_group_.reset();
}

}  // namespace wuji_hand2_driver::control
