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

#include "wuji_hand2_driver/sdk/hand2_session.hpp"

#include <cmath>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace wuji_hand2_driver::sdk {
namespace {

constexpr char kDeviceAlias[] = "wuji_hand_2";
constexpr char kHand2Model[] = "0x0002";
constexpr std::size_t kIdentityReadAttempts = 3;
constexpr std::size_t kIdentityTerminatorAndProbeBytes = 2;

struct SdkRuntimeState {
  std::mutex mutex;
  decltype(&wuji_init) init{nullptr};
  decltype(&wuji_shutdown) shutdown{nullptr};
  std::size_t users{0};
};

SdkRuntimeState& runtime_state() {
  static SdkRuntimeState state;
  return state;
}

class ConnectRollback {
 public:
  explicit ConnectRollback(Hand2Session& session) noexcept : session_(session) {}

  ~ConnectRollback() noexcept {
    if (active_) {
      session_.disconnect();
    }
  }

  void dismiss() noexcept {
    active_ = false;
  }

 private:
  Hand2Session& session_;
  bool active_{true};
};

}  // namespace

Hand2Session::Hand2Session(Hand2SdkApi& sdk) noexcept : sdk_(sdk) {}

Hand2Session::~Hand2Session() noexcept {
  disconnect();
}

Status Hand2Session::acquire_runtime() {
  auto& runtime = runtime_state();
  std::lock_guard<std::mutex> lock(runtime.mutex);
  if (runtime.users != 0) {
    if (runtime.init != sdk_.init || runtime.shutdown != sdk_.shutdown) {
      return Status::failure("another SDK function table owns the process runtime");
    }
    ++runtime.users;
    runtime_acquired_ = true;
    return Status::success();
  }

  const auto status = sdk_.init(nullptr);
  const auto result = sdk_status(sdk_, "wuji_init", status);
  if (!result) {
    return result;
  }

  runtime.init = sdk_.init;
  runtime.shutdown = sdk_.shutdown;
  runtime.users = 1;
  runtime_acquired_ = true;
  return Status::success();
}

void Hand2Session::release_runtime() noexcept {
  if (!runtime_acquired_) {
    return;
  }
  runtime_acquired_ = false;

  auto& runtime = runtime_state();
  std::lock_guard<std::mutex> lock(runtime.mutex);
  if (runtime.users == 0) {
    return;
  }
  --runtime.users;
  if (runtime.users == 0) {
    const auto shutdown = runtime.shutdown;
    runtime.init = nullptr;
    runtime.shutdown = nullptr;
    shutdown();
  }
}

Status Hand2Session::read_device_string(decltype(&wuji_dev_serial_number) getter,
                                        const char* operation, std::string& output,
                                        WujiStatus* failure_result_output) {
  for (std::size_t attempt = 0; attempt < kIdentityReadAttempts; ++attempt) {
    const auto required = getter(device_, nullptr, 0);
    if (required < 0) {
      const auto result = static_cast<WujiStatus>(required);
      if (failure_result_output != nullptr) {
        *failure_result_output = result;
      }
      return sdk_status(sdk_, operation, result);
    }

    std::vector<char> buffer(static_cast<std::size_t>(required) + kIdentityTerminatorAndProbeBytes,
                             '\0');
    const auto written = getter(device_, buffer.data(), buffer.size());
    if (written < 0) {
      const auto result = static_cast<WujiStatus>(written);
      if (failure_result_output != nullptr) {
        *failure_result_output = result;
      }
      return sdk_status(sdk_, operation, result);
    }

    const auto confirmed = getter(device_, nullptr, 0);
    if (confirmed < 0) {
      const auto result = static_cast<WujiStatus>(confirmed);
      if (failure_result_output != nullptr) {
        *failure_result_output = result;
      }
      return sdk_status(sdk_, operation, result);
    }
    if (written > required) {
      continue;
    }
    if (confirmed == written) {
      output.assign(buffer.data(), static_cast<std::size_t>(written));
      return Status::success();
    }
  }

  return Status::failure(std::string{operation} + " did not stabilize after " +
                         std::to_string(kIdentityReadAttempts) + " attempts");
}

Status Hand2Session::connect(const Hand2ConnectConfig& config) {
  Status status = connect_device(config);
  if (!status) {
    return status;
  }
  status = open_command_publisher();
  if (!status) {
    disconnect();
  }
  return status;
}

Status Hand2Session::connect_device(const Hand2ConnectConfig& config,
                                    WujiStatus* failure_result_output) {
  if (failure_result_output != nullptr) {
    *failure_result_output = WUJI_STATUS_OK;
  }
  if (device_ != nullptr || command_publisher_ != nullptr || runtime_acquired_) {
    return Status::failure("session is already connected");
  }
  if (config.endpoint.empty() && config.serial_number.empty()) {
    return Status::failure("connect requires endpoint or serial_number");
  }
  if (!config.hand_side.empty() && config.hand_side != "left" && config.hand_side != "right") {
    return Status::failure("hand_side must be left, right, or empty");
  }
  constexpr double kMaxTimeoutSeconds =
      static_cast<double>(std::numeric_limits<std::uint16_t>::max()) / 1000.0;
  if (!std::isfinite(config.connect_timeout) || config.connect_timeout < 0.0 ||
      config.connect_timeout > kMaxTimeoutSeconds) {
    return Status::failure("connect_timeout is outside the SDK millisecond range");
  }

  auto runtime_status = acquire_runtime();
  if (!runtime_status) {
    return runtime_status;
  }
  ConnectRollback rollback(*this);

  auto options = sdk_.connect_options_default();
  // The driver owns its device connection. Without the Bridge the SDK never falls back
  // to Zenoh, so wuji_connect returns once device discovery has answered.
  options.enable_bridge = false;
  const double timeout_ms = std::ceil(config.connect_timeout * 1000.0);
  options.timeout_ms = static_cast<std::uint32_t>(
      std::clamp(timeout_ms, 1.0, static_cast<double>(std::numeric_limits<std::uint16_t>::max())));
  if (config.retry_count) {
    options.retry_count = *config.retry_count;
  }

  const std::string target_value = config.endpoint.empty() ? config.serial_number : config.endpoint;
  const std::string alias{kDeviceAlias};
  const WujiConnectTarget target{
      static_cast<std::uint8_t>(config.endpoint.empty() ? WUJI_CONNECT_TARGET_KIND_SN
                                                        : WUJI_CONNECT_TARGET_KIND_ADDR),
      target_value.c_str()};

  WujiDevice* connected_device = nullptr;
  const auto connect_result = sdk_.connect(&target, alias.c_str(), &options, &connected_device);
  device_ = connected_device;
  if (failure_result_output != nullptr) {
    *failure_result_output = connect_result;
  }
  if (connect_result != WUJI_STATUS_OK) {
    return sdk_status(sdk_, "wuji_connect", connect_result);
  }
  if (device_ == nullptr) {
    return Status::failure("wuji_connect succeeded without a device handle");
  }

  auto identity_status = read_device_string(sdk_.device_serial_number, "wuji_dev_serial_number",
                                            serial_number_, failure_result_output);
  if (!identity_status) {
    return identity_status;
  }
  if (serial_number_.empty()) {
    return Status::failure("wuji_dev_serial_number returned an empty value");
  }
  if (!config.serial_number.empty() && serial_number_ != config.serial_number) {
    return Status::failure("connected serial_number is " + serial_number_ + ", expected " +
                           config.serial_number);
  }

  identity_status =
      read_device_string(sdk_.device_model, "wuji_dev_model", model_, failure_result_output);
  if (!identity_status) {
    return identity_status;
  }
  if (model_.empty()) {
    return Status::failure("wuji_dev_model returned an empty value");
  }
  if (model_ != kHand2Model) {
    return Status::failure("connected model is " + model_ + ", expected " + kHand2Model);
  }
  if (sdk_.device_is_connected(device_) == 0) {
    return Status::failure("wuji_dev_is_connected reported a disconnected device");
  }

  WujiHandedness handedness{};
  const auto handedness_result = sdk_.get_handedness(device_, &handedness);
  if (handedness_result != WUJI_STATUS_OK) {
    if (failure_result_output != nullptr) {
      *failure_result_output = handedness_result;
    }
    return sdk_status(sdk_, "wuji_hand_2_get_handedness", handedness_result);
  }
  const char* actual_hand_side = nullptr;
  if (handedness == WUJI_HANDEDNESS_LEFT) {
    actual_hand_side = "left";
  } else if (handedness == WUJI_HANDEDNESS_RIGHT) {
    actual_hand_side = "right";
  } else {
    return Status::failure("wuji_hand_2_get_handedness returned unknown value " +
                           std::to_string(static_cast<int>(handedness)));
  }
  if (!config.hand_side.empty() && actual_hand_side != config.hand_side) {
    return Status::failure("connected hand_side is " + std::string{actual_hand_side} +
                           ", expected " + config.hand_side);
  }
  hand_side_ = actual_hand_side;

  rollback.dismiss();
  return Status::success();
}

Status Hand2Session::open_command_publisher() {
  if (device_ == nullptr) {
    return Status::failure("device is not connected");
  }
  if (command_publisher_ != nullptr) {
    return Status::success();
  }

  WujiJointCommandPublisher* publisher = nullptr;
  const auto publisher_result = sdk_.open_joint_command_publisher(device_, &publisher);
  command_publisher_ = publisher;
  if (publisher_result != WUJI_STATUS_OK) {
    const Status status = sdk_status(sdk_, "wuji_hand_2_joint_command_publish", publisher_result);
    close_command_publisher();
    return status;
  }
  if (command_publisher_ == nullptr) {
    return Status::failure(
        "wuji_hand_2_joint_command_publish succeeded without a publisher handle");
  }
  return Status::success();
}

Status Hand2Session::send(const std::array<WujiJointCommand, WUJI_HAND_2_JOINT_COUNT>& commands) {
  if (command_publisher_ == nullptr) {
    return Status::failure("joint command publisher is not open");
  }
  const auto result = sdk_.send_joint_commands(command_publisher_, commands.data());
  return sdk_status(sdk_, "wuji_joint_command_publisher_send", result);
}

Status Hand2Session::enable_all(WujiStatus* result_output) noexcept {
  if (result_output != nullptr) {
    *result_output = WUJI_STATUS_OK;
  }
  if (device_ == nullptr) {
    return Status::failure("device is not connected");
  }
  const auto result = sdk_.enable(device_, nullptr);
  if (result_output != nullptr) {
    *result_output = result;
  }
  return sdk_status(sdk_, "wuji_hand_2_enable", result);
}

Status Hand2Session::disable_all() noexcept {
  if (device_ == nullptr) {
    return Status::failure("device is not connected");
  }
  const auto result = sdk_.disable(device_, nullptr);
  return sdk_status(sdk_, "wuji_hand_2_disable", result);
}

void Hand2Session::close_command_publisher() noexcept {
  if (command_publisher_ != nullptr) {
    auto* publisher = std::exchange(command_publisher_, nullptr);
    sdk_.close_joint_command_publisher(publisher);
  }
}

void Hand2Session::disconnect() noexcept {
  close_command_publisher();
  if (device_ != nullptr) {
    auto* device = std::exchange(device_, nullptr);
    static_cast<void>(sdk_.disconnect(device));
    sdk_.release(device);
  }
  serial_number_.clear();
  model_.clear();
  release_runtime();
  hand_side_.clear();
}

WujiDevice* Hand2Session::device() const noexcept {
  return device_;
}

std::string Hand2Session::serial_number() const {
  return serial_number_;
}

std::string Hand2Session::model() const {
  return model_;
}

std::string Hand2Session::hand_side() const {
  return hand_side_;
}

bool Hand2Session::is_connected() const noexcept {
  return device_ != nullptr && sdk_.device_is_connected(device_) != 0;
}

}  // namespace wuji_hand2_driver::sdk
