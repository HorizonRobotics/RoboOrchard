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

#include "fake_sdk/fake_hand2_sdk.hpp"

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <utility>

struct WujiDevice {
  std::uint64_t token;
  std::uint8_t target_kind;
  std::string target_value;
};

struct WujiJointCommandPublisher {
  std::uint64_t token;
};

struct WujiSub {
  std::uint64_t token;
  std::size_t stream_index;
};

namespace wuji_hand2_driver::test {
namespace {

WujiFingertipSensorInfo default_fingertip_info(std::uint8_t finger) {
  WujiFingertipSensorInfo info{};
  info.header.seq = finger;
  info.header.timestamp_us = 111111;
  std::snprintf(info.header.frame_id, sizeof(info.header.frame_id), "%s_tip_sensor_frame",
                finger == 0   ? "thumb"
                : finger == 1 ? "index"
                : finger == 2 ? "middle"
                : finger == 3 ? "ring"
                              : "pinky");
  info.digest = 0xA1000000U + finger;
  info.device_type = finger == 0 ? 0x0004U : 0x0003U;
  info.rate_hz = 100.0F;
  const unsigned int point_count = finger == 0 ? 40U : 34U;
  std::snprintf(info.format, sizeof(info.format),
                "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":%u,"
                "\"point_stride\":6,\"point_fields\":["
                "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\",\"scale\":0.01},"
                "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\",\"scale\":0.01},"
                "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\",\"scale\":0.01}],"
                "\"aggregate_stride\":8,\"aggregate_fields\":["
                "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\",\"scale\":0.01},"
                "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\",\"scale\":0.01},"
                "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\",\"scale\":0.01},"
                "{\"name\":\"temperature\",\"offset\":6,\"type\":\"i16\",\"scale\":0.1}]}",
                point_count);
  return info;
}

}  // namespace

FakeHand2Sdk* FakeHand2Sdk::current_ = nullptr;
thread_local std::string FakeHand2Sdk::thread_error_;

FakeHand2Sdk::FakeHand2Sdk()
    : api_{&FakeHand2Sdk::init,
           &FakeHand2Sdk::shutdown,
           &FakeHand2Sdk::last_error,
           &FakeHand2Sdk::connect_options_default,
           &FakeHand2Sdk::scan,
           &FakeHand2Sdk::discovered_free,
           &FakeHand2Sdk::connect,
           &FakeHand2Sdk::disconnect,
           &FakeHand2Sdk::release,
           &FakeHand2Sdk::device_serial_number,
           &FakeHand2Sdk::device_model,
           &FakeHand2Sdk::device_is_connected,
           &FakeHand2Sdk::sub_close,
           &FakeHand2Sdk::subscribe_joint_states,
           &FakeHand2Sdk::subscribe_joint_diagnostics,
           &FakeHand2Sdk::subscribe_imu,
           {&FakeHand2Sdk::subscribe_fingertip_thumb, &FakeHand2Sdk::subscribe_fingertip_index,
            &FakeHand2Sdk::subscribe_fingertip_middle, &FakeHand2Sdk::subscribe_fingertip_ring,
            &FakeHand2Sdk::subscribe_fingertip_pinky},
           {&FakeHand2Sdk::get_fingertip_info<0>, &FakeHand2Sdk::get_fingertip_info<1>,
            &FakeHand2Sdk::get_fingertip_info<2>, &FakeHand2Sdk::get_fingertip_info<3>,
            &FakeHand2Sdk::get_fingertip_info<4>},
           &FakeHand2Sdk::fingertip_online,
           &FakeHand2Sdk::get_handedness,
           &FakeHand2Sdk::get_hw_version,
           &FakeHand2Sdk::enable,
           &FakeHand2Sdk::disable,
           &FakeHand2Sdk::clear_fault,
           &FakeHand2Sdk::emergency_stop,
           &FakeHand2Sdk::set_effort_limit,
           &FakeHand2Sdk::set_effort_limits,
           &FakeHand2Sdk::set_mit_params,
           &FakeHand2Sdk::calibrate_fingertip,
           {&FakeHand2Sdk::get_fingertip_status<0>, &FakeHand2Sdk::get_fingertip_status<1>,
            &FakeHand2Sdk::get_fingertip_status<2>, &FakeHand2Sdk::get_fingertip_status<3>,
            &FakeHand2Sdk::get_fingertip_status<4>},
           &FakeHand2Sdk::open_joint_command_publisher,
           &FakeHand2Sdk::send_joint_commands,
           &FakeHand2Sdk::close_joint_command_publisher} {
  if (current_ != nullptr) {
    throw std::logic_error("Only one FakeHand2Sdk may be active in a process");
  }
  current_ = this;
  default_connect_options_.timeout_ms = 1000;
  default_connect_options_.retry_count = 3;
  default_connect_options_.enable_bridge = true;
  default_connect_options_.auto_time_sync_interval_ms = 30000;
  default_connect_options_.auto_time_sync_interval_enabled = true;
  for (std::uint8_t finger = 0; finger < fingertip_info_.size(); ++finger) {
    fingertip_info_[finger] = default_fingertip_info(finger);
  }
}

FakeHand2Sdk::~FakeHand2Sdk() {
  current_ = nullptr;
}

sdk::Hand2SdkApi& FakeHand2Sdk::api() noexcept {
  return api_;
}

const std::vector<std::string>& FakeHand2Sdk::calls() const noexcept {
  return calls_;
}

std::vector<std::string> FakeHand2Sdk::calls_snapshot() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return calls_;
}

std::size_t FakeHand2Sdk::count(std::string_view operation) const noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<std::size_t>(std::count(calls_.begin(), calls_.end(), operation));
}

std::vector<FakeHand2Sdk::TypedStream> FakeHand2Sdk::closed_streams_snapshot() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return closed_streams_;
}
const std::vector<FakeHand2Sdk::ConnectCall>& FakeHand2Sdk::connect_calls() const noexcept {
  return connect_calls_;
}

const std::vector<std::array<WujiJointCommand, WUJI_HAND_2_JOINT_COUNT>>&
FakeHand2Sdk::sent_commands() const noexcept {
  return sent_commands_;
}

std::vector<std::array<WujiJointCommand, WUJI_HAND_2_JOINT_COUNT>>
FakeHand2Sdk::sent_commands_snapshot() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return sent_commands_;
}

std::vector<FakeHand2Sdk::MaskCall> FakeHand2Sdk::enable_calls() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return enable_calls_;
}

std::vector<FakeHand2Sdk::MaskCall> FakeHand2Sdk::disable_calls() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return disable_calls_;
}

std::vector<FakeHand2Sdk::MaskCall> FakeHand2Sdk::clear_fault_calls() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return clear_fault_calls_;
}

std::vector<WujiDevice*> FakeHand2Sdk::emergency_stop_devices() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return emergency_stop_devices_;
}

std::vector<FakeHand2Sdk::EffortLimitCall> FakeHand2Sdk::effort_limit_calls() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return effort_limit_calls_;
}

std::vector<FakeHand2Sdk::EffortLimitsCall> FakeHand2Sdk::effort_limits_calls() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return effort_limits_calls_;
}

std::vector<FakeHand2Sdk::MitParamsCall> FakeHand2Sdk::mit_params_calls() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return mit_params_calls_;
}

std::vector<WujiDevice*> FakeHand2Sdk::tactile_calibrate_devices() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return tactile_calibrate_devices_;
}

std::vector<FakeHand2Sdk::TactileStatusCall> FakeHand2Sdk::tactile_status_calls() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return tactile_status_calls_;
}

const std::vector<std::size_t>& FakeHand2Sdk::model_buffer_lengths() const noexcept {
  return model_.buffer_lengths;
}

std::vector<std::uint8_t> FakeHand2Sdk::fingertip_info_requests() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return fingertip_info_requests_;
}

bool FakeHand2Sdk::last_disable_used_whole_hand() const noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  return last_disable_used_whole_hand_;
}

void FakeHand2Sdk::set_subscribe_failure(TypedStream stream, WujiStatus status, std::string error,
                                         bool return_subscription) {
  const std::lock_guard<std::mutex> lock(mutex_);
  subscribe_failures_[stream_index(stream)] =
      Failure{status, std::move(error), return_subscription};
}

void FakeHand2Sdk::set_synchronous_joint_state(WujiFrameKind kind,
                                               const WujiJointStateFrame* frame) noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  joint_state_subscription_.synchronous_kind = kind;
  joint_state_subscription_.synchronous_frame = frame;
}

void FakeHand2Sdk::set_synchronous_joint_diagnostics(
    WujiFrameKind kind, const WujiJointDiagnosticsFrame* frame) noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  joint_diagnostics_subscription_.synchronous_kind = kind;
  joint_diagnostics_subscription_.synchronous_frame = frame;
}

void FakeHand2Sdk::set_synchronous_imu(WujiFrameKind kind, const WujiImuData* frame) noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  imu_subscription_.synchronous_kind = kind;
  imu_subscription_.synchronous_frame = frame;
}

void FakeHand2Sdk::set_synchronous_fingertip(std::uint8_t finger, WujiFrameKind kind,
                                             const WujiFingertipSensorData* frame) noexcept {
  if (finger >= fingertip_subscriptions_.size()) {
    return;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  fingertip_subscriptions_[finger].synchronous_kind = kind;
  fingertip_subscriptions_[finger].synchronous_frame = frame;
}

void FakeHand2Sdk::set_callback_hook(TypedStream stream, std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t index = stream_index(stream);
  if (index >= stream_index(TypedStream::FingertipThumb)) {
    fingertip_subscriptions_[index - stream_index(TypedStream::FingertipThumb)].callback_hook =
        std::move(hook);
    return;
  }
  switch (stream) {
    case TypedStream::JointState:
      joint_state_subscription_.callback_hook = std::move(hook);
      return;
    case TypedStream::JointDiagnostics:
      joint_diagnostics_subscription_.callback_hook = std::move(hook);
      return;
    case TypedStream::Imu:
      imu_subscription_.callback_hook = std::move(hook);
      return;
  }
}

void FakeHand2Sdk::set_callback_return_hook(TypedStream stream, std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t index = stream_index(stream);
  if (index >= stream_index(TypedStream::FingertipThumb)) {
    fingertip_subscriptions_[index - stream_index(TypedStream::FingertipThumb)]
        .callback_return_hook = std::move(hook);
    return;
  }
  switch (stream) {
    case TypedStream::JointState:
      joint_state_subscription_.callback_return_hook = std::move(hook);
      return;
    case TypedStream::JointDiagnostics:
      joint_diagnostics_subscription_.callback_return_hook = std::move(hook);
      return;
    case TypedStream::Imu:
      imu_subscription_.callback_return_hook = std::move(hook);
      return;
  }
}

bool FakeHand2Sdk::is_closing(TypedStream stream) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t index = stream_index(stream);
  if (index >= stream_index(TypedStream::FingertipThumb)) {
    return fingertip_subscriptions_[index - stream_index(TypedStream::FingertipThumb)].closing;
  }
  switch (stream) {
    case TypedStream::JointState:
      return joint_state_subscription_.closing;
    case TypedStream::JointDiagnostics:
      return joint_diagnostics_subscription_.closing;
    case TypedStream::Imu:
      return imu_subscription_.closing;
  }
  return false;
}

std::size_t FakeHand2Sdk::callbacks_in_flight(TypedStream stream) const {
  const std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t index = stream_index(stream);
  if (index >= stream_index(TypedStream::FingertipThumb)) {
    return fingertip_subscriptions_[index - stream_index(TypedStream::FingertipThumb)]
        .callbacks_in_flight;
  }
  switch (stream) {
    case TypedStream::JointState:
      return joint_state_subscription_.callbacks_in_flight;
    case TypedStream::JointDiagnostics:
      return joint_diagnostics_subscription_.callbacks_in_flight;
    case TypedStream::Imu:
      return imu_subscription_.callbacks_in_flight;
  }
  return 0;
}

void FakeHand2Sdk::set_sub_close_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  sub_close_hook_ = std::move(hook);
}

void FakeHand2Sdk::emit_joint_state(WujiFrameKind kind, const WujiJointStateFrame* frame) {
  emit_typed(joint_state_subscription_, kind, frame, true);
}

void FakeHand2Sdk::emit_joint_diagnostics(WujiFrameKind kind,
                                          const WujiJointDiagnosticsFrame* frame) {
  emit_typed(joint_diagnostics_subscription_, kind, frame, true);
}

void FakeHand2Sdk::emit_imu(WujiFrameKind kind, const WujiImuData* frame) {
  emit_typed(imu_subscription_, kind, frame, true);
}

void FakeHand2Sdk::emit_fingertip(std::uint8_t finger, WujiFrameKind kind,
                                  const WujiFingertipSensorData* frame) {
  if (finger >= fingertip_subscriptions_.size()) {
    return;
  }
  emit_typed(fingertip_subscriptions_[finger], kind, frame, true);
}

void FakeHand2Sdk::emit_closed_joint_state(WujiFrameKind kind, const WujiJointStateFrame* frame) {
  emit_typed(joint_state_subscription_, kind, frame, false);
}

void FakeHand2Sdk::emit_closed_joint_diagnostics(WujiFrameKind kind,
                                                 const WujiJointDiagnosticsFrame* frame) {
  emit_typed(joint_diagnostics_subscription_, kind, frame, false);
}

void FakeHand2Sdk::emit_closed_imu(WujiFrameKind kind, const WujiImuData* frame) {
  emit_typed(imu_subscription_, kind, frame, false);
}

void FakeHand2Sdk::emit_closed_fingertip(std::uint8_t finger, WujiFrameKind kind,
                                         const WujiFingertipSensorData* frame) {
  if (finger >= fingertip_subscriptions_.size()) {
    return;
  }
  emit_typed(fingertip_subscriptions_[finger], kind, frame, false);
}

void FakeHand2Sdk::set_default_connect_options(const WujiConnectOptions& options) noexcept {
  default_connect_options_ = options;
}

void FakeHand2Sdk::set_scan_results_sequence(std::vector<std::vector<Discovery>> results) {
  const std::lock_guard<std::mutex> lock(mutex_);
  scan_results_sequence_ = std::move(results);
  scan_result_index_ = 0;
}

void FakeHand2Sdk::set_scan_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  scan_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_identity_from_connect_target(bool enabled) noexcept {
  const std::lock_guard<std::mutex> lock(mutex_);
  identity_from_connect_target_ = enabled;
}

void FakeHand2Sdk::set_init_failure(WujiStatus status, std::string error) {
  init_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_connect_failure(WujiStatus status, std::string error, bool return_device) {
  connect_failure_ = Failure{status, std::move(error), return_device};
}

void FakeHand2Sdk::set_connect_replies(
    std::vector<ConnectReply> replies,
    std::function<void(std::chrono::milliseconds)> time_advance) {
  const std::lock_guard<std::mutex> lock(mutex_);
  connect_replies_ = std::move(replies);
  connect_reply_index_ = 0;
  connect_time_advance_ = std::move(time_advance);
}

void FakeHand2Sdk::set_open_publisher_failure(WujiStatus status, std::string error,
                                              bool return_publisher) {
  open_publisher_failure_ = Failure{status, std::move(error), return_publisher};
}

void FakeHand2Sdk::set_open_publisher_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  open_publisher_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_send_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  send_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_enable_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  enable_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_disable_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  disable_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_clear_fault_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  clear_fault_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_emergency_stop_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  emergency_stop_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_effort_limit_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  effort_limit_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_effort_limits_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  effort_limits_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_mit_params_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  mit_params_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_tactile_calibrate_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  tactile_calibrate_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_tactile_status_replies(std::vector<TactileStatusReply> replies) {
  const std::lock_guard<std::mutex> lock(mutex_);
  tactile_status_replies_ = std::move(replies);
  tactile_status_reply_index_ = 0;
}

void FakeHand2Sdk::set_send_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  send_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_enable_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  enable_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_disable_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  disable_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_handedness_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  handedness_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_fingertip_info_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  fingertip_info_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_emergency_stop_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  emergency_stop_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_effort_limit_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  effort_limit_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_tactile_calibrate_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  tactile_calibrate_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_tactile_status_hook(std::function<void()> hook) {
  const std::lock_guard<std::mutex> lock(mutex_);
  tactile_status_hook_ = std::move(hook);
}

void FakeHand2Sdk::set_disconnect_failure(WujiStatus status, std::string error) {
  disconnect_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_serial_number(std::string serial_number) {
  serial_number_.value = std::move(serial_number);
}

void FakeHand2Sdk::set_serial_number_failure(std::string error) {
  serial_number_.failure = std::move(error);
}

void FakeHand2Sdk::set_serial_number_sequence(std::vector<std::string> values) {
  serial_number_.sequence = std::move(values);
  serial_number_.calls = 0;
}

void FakeHand2Sdk::set_serial_number_failure_on_call(std::size_t call, std::string error,
                                                     WujiStatus status) {
  serial_number_.failure_call = call;
  serial_number_.call_failure = std::move(error);
  serial_number_.call_failure_status = status;
  serial_number_.calls = 0;
}

void FakeHand2Sdk::set_model(std::string model) {
  model_.value = std::move(model);
}

void FakeHand2Sdk::set_model_failure(std::string error) {
  model_.failure = std::move(error);
}

void FakeHand2Sdk::set_model_sequence(std::vector<std::string> values) {
  model_.sequence = std::move(values);
  model_.calls = 0;
}

void FakeHand2Sdk::set_model_failure_on_call(std::size_t call, std::string error,
                                             WujiStatus status) {
  model_.failure_call = call;
  model_.call_failure = std::move(error);
  model_.call_failure_status = status;
  model_.calls = 0;
}

void FakeHand2Sdk::set_connected(bool connected) noexcept {
  connected_ = connected;
}

void FakeHand2Sdk::set_handedness(WujiHandedness handedness) noexcept {
  handedness_ = handedness;
}

void FakeHand2Sdk::set_handedness_failure(WujiStatus status, std::string error) {
  handedness_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_handedness_failure_on_call(std::size_t call, WujiStatus status,
                                                  std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  handedness_calls_ = 0;
  handedness_failure_call_ = call;
  handedness_call_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_fingertip_info(std::uint8_t finger, const WujiFingertipSensorInfo& info) {
  const std::size_t index = fingertip_index(finger);
  const std::lock_guard<std::mutex> lock(mutex_);
  fingertip_info_[index] = info;
}

void FakeHand2Sdk::set_fingertip_info_failure(std::uint8_t finger, WujiStatus status,
                                              std::string error) {
  const std::size_t index = fingertip_index(finger);
  const std::lock_guard<std::mutex> lock(mutex_);
  fingertip_info_failures_[index] = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::set_fingertip_online(const std::array<bool, 5>& online) {
  const std::lock_guard<std::mutex> lock(mutex_);
  fingertip_online_ = online;
}

void FakeHand2Sdk::set_fingertip_online_failure(WujiStatus status, std::string error) {
  const std::lock_guard<std::mutex> lock(mutex_);
  fingertip_online_failure_ = Failure{status, std::move(error), false};
}

void FakeHand2Sdk::overwrite_last_error(std::string error) {
  thread_error_ = std::move(error);
}

FakeHand2Sdk& FakeHand2Sdk::current() {
  assert(current_ != nullptr);
  return *current_;
}

void FakeHand2Sdk::record(std::string operation) {
  auto& fake = current();
  const std::lock_guard<std::mutex> lock(fake.mutex_);
  fake.calls_.push_back(std::move(operation));
  thread_error_.clear();
}

WujiStatus FakeHand2Sdk::finish_status(const Failure& failure) {
  if (failure.status != WUJI_STATUS_OK) {
    thread_error_ = failure.error;
  }
  return failure.status;
}

int32_t FakeHand2Sdk::copy_device_string(std::string_view operation, DeviceStringControl& control,
                                         char* buffer, std::size_t buffer_length) {
  record(std::string(operation));
  std::string value;
  std::string failure;
  WujiStatus failure_status = WUJI_STATUS_ERR_INTERNAL;
  {
    auto& fake = current();
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    ++control.calls;
    control.buffer_lengths.push_back(buffer_length);
    value = control.sequence.empty()
                ? control.value
                : control.sequence[std::min(control.calls - 1, control.sequence.size() - 1)];
    failure = control.failure;
    failure_status = control.failure_status;
    if (control.failure_call == control.calls) {
      failure = control.call_failure;
      failure_status = control.call_failure_status;
    }
  }
  if (!failure.empty()) {
    thread_error_ = failure;
    return failure_status;
  }
  if (buffer_length == 0) {
    return static_cast<int32_t>(value.size());
  }
  const auto copied = std::min(value.size(), buffer_length - 1);
  std::memcpy(buffer, value.data(), copied);
  buffer[copied] = '\0';
  return static_cast<int32_t>(copied);
}

std::size_t FakeHand2Sdk::stream_index(TypedStream stream) noexcept {
  return static_cast<std::size_t>(stream);
}

std::size_t FakeHand2Sdk::fingertip_index(std::uint8_t finger) {
  if (finger >= 5) {
    throw std::out_of_range("fingertip index must be in [0, 4]");
  }
  return finger;
}

template <typename CallbackT, typename FrameT>
WujiStatus FakeHand2Sdk::subscribe_typed(std::string operation, TypedStream stream,
                                         CallbackT callback, void* user_data, WujiSub** output,
                                         SubscriptionSlot<CallbackT, FrameT>& slot) {
  record(std::move(operation));
  if (callback == nullptr || output == nullptr) {
    thread_error_ = "subscription callback or output is null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  *output = nullptr;

  auto& fake = current();
  Failure failure;
  std::optional<WujiFrameKind> synchronous_kind;
  const FrameT* synchronous_frame = nullptr;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    slot.callback = callback;
    slot.user_data = user_data;
    slot.subscription = nullptr;
    slot.closing = false;
    slot.callbacks_in_flight = 0;
    slot.terminal_delivered = false;
    failure = fake.subscribe_failures_[stream_index(stream)];
    synchronous_kind = slot.synchronous_kind;
    synchronous_frame = slot.synchronous_frame;
    slot.synchronous_kind.reset();
    slot.synchronous_frame = nullptr;
  }

  if (synchronous_kind) {
    callback(*synchronous_kind, synchronous_frame, user_data);
  }

  WujiSub* subscription = nullptr;
  if (failure.status == WUJI_STATUS_OK || failure.return_handle) {
    subscription =
        new WujiSub{fake.next_token_.fetch_add(1, std::memory_order_relaxed), stream_index(stream)};
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    slot.subscription = subscription;
    slot.terminal_delivered = synchronous_kind && (*synchronous_kind == WUJI_FRAME_KIND_END ||
                                                   *synchronous_kind == WUJI_FRAME_KIND_ERROR);
  }
  *output = subscription;
  return finish_status(failure);
}

template <typename CallbackT, typename FrameT>
void FakeHand2Sdk::emit_typed(SubscriptionSlot<CallbackT, FrameT>& slot, WujiFrameKind kind,
                              const FrameT* frame, bool require_active) {
  CallbackT callback = nullptr;
  void* user_data = nullptr;
  std::function<void()> callback_hook;
  std::function<void()> callback_return_hook;
  bool callback_registered = false;
  {
    auto& fake = current();
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    const bool active = slot.subscription != nullptr;
    if ((require_active && (!active || slot.closing)) || (!require_active && active)) {
      return;
    }
    callback = slot.callback;
    user_data = slot.user_data;
    if (require_active) {
      callback_hook = slot.callback_hook;
      callback_return_hook = slot.callback_return_hook;
      ++slot.callbacks_in_flight;
      callback_registered = true;
    }
    if (kind == WUJI_FRAME_KIND_END || kind == WUJI_FRAME_KIND_ERROR) {
      slot.terminal_delivered = true;
    }
  }

  const auto complete_callback = [&]() {
    if (!callback_registered) {
      return;
    }
    auto& fake = current();
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    assert(slot.callbacks_in_flight > 0);
    --slot.callbacks_in_flight;
    if (slot.callbacks_in_flight == 0) {
      fake.callbacks_drained_.notify_all();
    }
  };
  try {
    if (callback_hook) {
      callback_hook();
    }
    if (callback != nullptr) {
      callback(kind, frame, user_data);
      if (callback_return_hook) {
        callback_return_hook();
      }
    }
  } catch (...) {
    complete_callback();
    throw;
  }
  complete_callback();
}

WujiStatus FakeHand2Sdk::init(const WujiInitOptions*) {
  record("init");
  return finish_status(current().init_failure_);
}

void FakeHand2Sdk::shutdown() {
  record("shutdown");
}

const char* FakeHand2Sdk::last_error() {
  auto& fake = current();
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    fake.calls_.push_back("last_error");
  }
  return thread_error_.empty() ? nullptr : thread_error_.c_str();
}

WujiConnectOptions FakeHand2Sdk::connect_options_default() {
  record("connect_options_default");
  return current().default_connect_options_;
}

WujiStatus FakeHand2Sdk::scan(WujiDiscovered** output, std::size_t* output_size) {
  record("scan");
  if (output == nullptr || output_size == nullptr) {
    thread_error_ = "scan output is null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  *output = nullptr;
  *output_size = 0;
  auto& fake = current();
  Failure failure;
  std::vector<Discovery> discoveries;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    failure = fake.scan_failure_;
    if (!fake.scan_results_sequence_.empty()) {
      const auto index = std::min(fake.scan_result_index_, fake.scan_results_sequence_.size() - 1);
      discoveries = fake.scan_results_sequence_[index];
      ++fake.scan_result_index_;
    }
  }
  if (failure.status != WUJI_STATUS_OK) {
    return finish_status(failure);
  }
  if (discoveries.empty()) {
    return WUJI_STATUS_OK;
  }
  auto* result = new WujiDiscovered[discoveries.size()]{};
  for (std::size_t index = 0; index < discoveries.size(); ++index) {
    const auto& source = discoveries[index];
    std::snprintf(result[index].serial_number, sizeof(result[index].serial_number), "%s",
                  source.serial_number.c_str());
    std::snprintf(result[index].model, sizeof(result[index].model), "%s", source.model.c_str());
    result[index].device_id = source.device_id;
    result[index].transport = source.transport;
    std::snprintf(result[index].address, sizeof(result[index].address), "%s",
                  source.address.c_str());
  }
  *output = result;
  *output_size = discoveries.size();
  return WUJI_STATUS_OK;
}

void FakeHand2Sdk::discovered_free(WujiDiscovered* discovered, std::size_t) {
  record("discovered_free");
  delete[] discovered;
}

WujiStatus FakeHand2Sdk::connect(const WujiConnectTarget* target, const char* alias,
                                 const WujiConnectOptions* options, WujiDevice** output) {
  auto& fake = current();
  const ConnectCall call{
      target == nullptr ? std::uint8_t{255} : target->kind,
      target == nullptr || target->value == nullptr ? std::string{} : target->value,
      alias == nullptr ? std::string{} : alias,
      options == nullptr ? WujiConnectOptions{} : *options};
  record("connect");
  Failure failure;
  std::chrono::milliseconds elapsed{0};
  std::function<void(std::chrono::milliseconds)> time_advance;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    fake.connect_calls_.push_back(call);
    failure = fake.connect_failure_;
    if (!fake.connect_replies_.empty()) {
      const auto index = std::min(fake.connect_reply_index_, fake.connect_replies_.size() - 1);
      const ConnectReply& reply = fake.connect_replies_[index];
      failure = Failure{reply.status, reply.error, reply.return_device};
      elapsed = reply.elapsed;
      ++fake.connect_reply_index_;
    }
    time_advance = fake.connect_time_advance_;
  }
  if (time_advance) {
    time_advance(elapsed);
  }
  if (output == nullptr) {
    thread_error_ = "connect output is null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  *output = nullptr;
  if (failure.status == WUJI_STATUS_OK || failure.return_handle) {
    *output = new WujiDevice{fake.next_token_.fetch_add(1, std::memory_order_relaxed),
                             call.target_kind, call.target_value};
  }
  return finish_status(failure);
}

WujiStatus FakeHand2Sdk::disconnect(WujiDevice* device) {
  static_cast<void>(device);
  record("disconnect");
  return finish_status(current().disconnect_failure_);
}

void FakeHand2Sdk::release(WujiDevice* device) {
  record("release");
  delete device;
}

int32_t FakeHand2Sdk::device_serial_number(WujiDevice* device, char* buffer,
                                           std::size_t buffer_length) {
  auto& fake = current();
  bool use_target = false;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    use_target = fake.identity_from_connect_target_ && device != nullptr &&
                 device->target_kind == WUJI_CONNECT_TARGET_KIND_SN;
  }
  if (use_target) {
    DeviceStringControl target{device->target_value};
    return copy_device_string("device_serial_number", target, buffer, buffer_length);
  }
  return copy_device_string("device_serial_number", fake.serial_number_, buffer, buffer_length);
}

int32_t FakeHand2Sdk::device_model(WujiDevice*, char* buffer, std::size_t buffer_length) {
  auto& fake = current();
  return copy_device_string("device_model", fake.model_, buffer, buffer_length);
}

std::uint8_t FakeHand2Sdk::device_is_connected(WujiDevice* device) {
  static_cast<void>(device);
  record("device_is_connected");
  return current().connected_ ? std::uint8_t{1} : std::uint8_t{0};
}

void FakeHand2Sdk::sub_close(WujiSub* subscription) {
  record("sub_close");
  if (subscription == nullptr) {
    return;
  }
  auto& fake = current();
  std::function<void()> close_hook;
  std::function<void()> final_callback;
  {
    std::unique_lock<std::mutex> lock(fake.mutex_);
    fake.closed_streams_.push_back(static_cast<TypedStream>(subscription->stream_index));
    close_hook = fake.sub_close_hook_;
    const auto close_slot = [&fake, &lock, subscription, &final_callback](auto& slot) {
      if (slot.subscription != subscription) {
        return false;
      }
      slot.closing = true;
      slot.subscription = nullptr;
      fake.callbacks_drained_.wait(lock, [&slot]() { return slot.callbacks_in_flight == 0; });
      if (!slot.terminal_delivered && slot.callback != nullptr) {
        const auto callback = slot.callback;
        void* const user_data = slot.user_data;
        final_callback = [callback, user_data]() {
          callback(WUJI_FRAME_KIND_END, nullptr, user_data);
        };
        slot.terminal_delivered = true;
      }
      return true;
    };
    switch (subscription->stream_index) {
      case 0:
        close_slot(fake.joint_state_subscription_);
        break;
      case 1:
        close_slot(fake.joint_diagnostics_subscription_);
        break;
      case 2:
        close_slot(fake.imu_subscription_);
        break;
      case 3:
        close_slot(fake.fingertip_subscriptions_[0]);
        break;
      case 4:
        close_slot(fake.fingertip_subscriptions_[1]);
        break;
      case 5:
        close_slot(fake.fingertip_subscriptions_[2]);
        break;
      case 6:
        close_slot(fake.fingertip_subscriptions_[3]);
        break;
      case 7:
        close_slot(fake.fingertip_subscriptions_[4]);
        break;
      default:
        break;
    }
  }
  if (final_callback) {
    final_callback();
  }
  if (close_hook) {
    close_hook();
  }
  delete subscription;
}

WujiStatus FakeHand2Sdk::subscribe_joint_states(WujiDevice*,
                                                WujiJointStateFrameSubCallback callback,
                                                void* user_data, WujiSub** output) {
  auto& fake = current();
  return subscribe_typed("subscribe_joint_states", TypedStream::JointState, callback, user_data,
                         output, fake.joint_state_subscription_);
}

WujiStatus FakeHand2Sdk::subscribe_joint_diagnostics(WujiDevice*,
                                                     WujiJointDiagnosticsFrameSubCallback callback,
                                                     void* user_data, WujiSub** output) {
  auto& fake = current();
  return subscribe_typed("subscribe_joint_diagnostics", TypedStream::JointDiagnostics, callback,
                         user_data, output, fake.joint_diagnostics_subscription_);
}

WujiStatus FakeHand2Sdk::subscribe_imu(WujiDevice*, WujiImuDataSubCallback callback,
                                       void* user_data, WujiSub** output) {
  auto& fake = current();
  return subscribe_typed("subscribe_imu", TypedStream::Imu, callback, user_data, output,
                         fake.imu_subscription_);
}

WujiStatus FakeHand2Sdk::subscribe_fingertip_thumb(WujiDevice*,
                                                   WujiFingertipSensorDataSubCallback callback,
                                                   void* user_data, WujiSub** output) {
  auto& fake = current();
  return subscribe_typed("subscribe_fingertip_thumb", TypedStream::FingertipThumb, callback,
                         user_data, output, fake.fingertip_subscriptions_[0]);
}

WujiStatus FakeHand2Sdk::subscribe_fingertip_index(WujiDevice*,
                                                   WujiFingertipSensorDataSubCallback callback,
                                                   void* user_data, WujiSub** output) {
  auto& fake = current();
  return subscribe_typed("subscribe_fingertip_index", TypedStream::FingertipIndex, callback,
                         user_data, output, fake.fingertip_subscriptions_[1]);
}

WujiStatus FakeHand2Sdk::subscribe_fingertip_middle(WujiDevice*,
                                                    WujiFingertipSensorDataSubCallback callback,
                                                    void* user_data, WujiSub** output) {
  auto& fake = current();
  return subscribe_typed("subscribe_fingertip_middle", TypedStream::FingertipMiddle, callback,
                         user_data, output, fake.fingertip_subscriptions_[2]);
}

WujiStatus FakeHand2Sdk::subscribe_fingertip_ring(WujiDevice*,
                                                  WujiFingertipSensorDataSubCallback callback,
                                                  void* user_data, WujiSub** output) {
  auto& fake = current();
  return subscribe_typed("subscribe_fingertip_ring", TypedStream::FingertipRing, callback,
                         user_data, output, fake.fingertip_subscriptions_[3]);
}

WujiStatus FakeHand2Sdk::subscribe_fingertip_pinky(WujiDevice*,
                                                   WujiFingertipSensorDataSubCallback callback,
                                                   void* user_data, WujiSub** output) {
  auto& fake = current();
  return subscribe_typed("subscribe_fingertip_pinky", TypedStream::FingertipPinky, callback,
                         user_data, output, fake.fingertip_subscriptions_[4]);
}

WujiStatus FakeHand2Sdk::fingertip_online(WujiDevice*, bool* output) {
  record("fingertip_online");
  if (output == nullptr) {
    thread_error_ = "fingertip online output is null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  auto& fake = current();
  Failure failure;
  std::array<bool, 5> online{};
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    failure = fake.fingertip_online_failure_;
    online = fake.fingertip_online_;
  }
  if (failure.status != WUJI_STATUS_OK) {
    return finish_status(failure);
  }
  std::copy(online.begin(), online.end(), output);
  return WUJI_STATUS_OK;
}

template <std::uint8_t Finger>
WujiStatus FakeHand2Sdk::get_fingertip_info(WujiDevice*, WujiFingertipSensorInfo* output) {
  return read_fingertip_info(Finger, output);
}

WujiStatus FakeHand2Sdk::read_fingertip_info(std::uint8_t finger, WujiFingertipSensorInfo* output) {
  record("get_fingertip_info");
  if (output == nullptr) {
    thread_error_ = "fingertip info output is null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  if (finger >= 5) {
    thread_error_ = "fingertip index must be in [0, 4]";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  auto& fake = current();
  Failure failure;
  WujiFingertipSensorInfo info{};
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    fake.fingertip_info_requests_.push_back(finger);
    failure = fake.fingertip_info_failures_[finger];
    info = fake.fingertip_info_[finger];
    hook = fake.fingertip_info_hook_;
  }
  if (hook) {
    hook();
  }
  if (failure.status != WUJI_STATUS_OK) {
    return finish_status(failure);
  }
  *output = info;
  return WUJI_STATUS_OK;
}

WujiStatus FakeHand2Sdk::get_handedness(WujiDevice* device, WujiHandedness* output) {
  record("get_handedness");
  auto& fake = current();
  Failure failure;
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    ++fake.handedness_calls_;
    failure = fake.handedness_failure_call_ == fake.handedness_calls_
                  ? fake.handedness_call_failure_
                  : fake.handedness_failure_;
    hook = fake.handedness_hook_;
  }
  if (hook) {
    hook();
  }
  if (failure.status != WUJI_STATUS_OK) {
    return finish_status(failure);
  }
  if (output == nullptr) {
    thread_error_ = "handedness output is null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  bool use_target = false;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    use_target = fake.identity_from_connect_target_ && device != nullptr &&
                 device->target_kind == WUJI_CONNECT_TARGET_KIND_SN &&
                 device->target_value.size() > 3;
  }
  if (use_target && device->target_value[3] == 'J') {
    *output = WUJI_HANDEDNESS_LEFT;
    return WUJI_STATUS_OK;
  }
  if (use_target && device->target_value[3] == 'K') {
    *output = WUJI_HANDEDNESS_RIGHT;
    return WUJI_STATUS_OK;
  }
  *output = fake.handedness_;
  return WUJI_STATUS_OK;
}

WujiStatus FakeHand2Sdk::get_hw_version(WujiDevice*, WujiHwVersion* output) {
  record("get_hw_version");
  if (output == nullptr) {
    thread_error_ = "hardware version output is null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  *output = WujiHwVersion{1, 2, 3};
  return WUJI_STATUS_OK;
}

WujiStatus FakeHand2Sdk::enable(WujiDevice* device, const std::uint8_t* mask) {
  record("enable");
  auto& fake = current();
  Failure failure;
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    MaskCall call;
    call.device = device;
    call.used_null = mask == nullptr;
    if (mask != nullptr) {
      std::copy_n(mask, call.mask.size(), call.mask.begin());
    }
    fake.enable_calls_.push_back(call);
    failure = fake.enable_failure_;
    hook = fake.enable_hook_;
  }
  if (hook) {
    hook();
  }
  return finish_status(failure);
}

WujiStatus FakeHand2Sdk::disable(WujiDevice* device, const std::uint8_t* mask) {
  record("disable");
  auto& fake = current();
  Failure failure;
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    MaskCall call;
    call.device = device;
    call.used_null = mask == nullptr;
    if (mask != nullptr) {
      std::copy_n(mask, call.mask.size(), call.mask.begin());
    }
    fake.disable_calls_.push_back(call);
    fake.last_disable_used_whole_hand_ = call.used_null;
    failure = fake.disable_failure_;
    hook = fake.disable_hook_;
  }
  if (hook) {
    hook();
  }
  return finish_status(failure);
}

WujiStatus FakeHand2Sdk::clear_fault(WujiDevice* device, const std::uint8_t* mask) {
  record("clear_fault");
  auto& fake = current();
  Failure failure;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    MaskCall call;
    call.device = device;
    call.used_null = mask == nullptr;
    if (mask != nullptr) {
      std::copy_n(mask, call.mask.size(), call.mask.begin());
    }
    fake.clear_fault_calls_.push_back(call);
    failure = fake.clear_fault_failure_;
  }
  return finish_status(failure);
}

WujiStatus FakeHand2Sdk::emergency_stop(WujiDevice* device) {
  record("emergency_stop");
  auto& fake = current();
  Failure failure;
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    fake.emergency_stop_devices_.push_back(device);
    failure = fake.emergency_stop_failure_;
    hook = fake.emergency_stop_hook_;
  }
  if (hook) {
    hook();
  }
  return finish_status(failure);
}

WujiStatus FakeHand2Sdk::set_effort_limit(WujiDevice* device, float limit) {
  record("set_effort_limit");
  auto& fake = current();
  Failure failure;
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    fake.effort_limit_calls_.push_back(EffortLimitCall{device, limit});
    failure = fake.effort_limit_failure_;
    hook = fake.effort_limit_hook_;
  }
  if (hook) {
    hook();
  }
  return finish_status(failure);
}

WujiStatus FakeHand2Sdk::set_effort_limits(WujiDevice* device, const float* limits) {
  record("set_effort_limits");
  if (limits == nullptr) {
    thread_error_ = "effort limits are null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  auto& fake = current();
  Failure failure;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    EffortLimitsCall call;
    call.device = device;
    std::copy_n(limits, call.limits.size(), call.limits.begin());
    fake.effort_limits_calls_.push_back(call);
    failure = fake.effort_limits_failure_;
  }
  return finish_status(failure);
}

WujiStatus FakeHand2Sdk::set_mit_params(WujiDevice* device, const float* kp, const float* kd) {
  record("set_mit_params");
  if (kp == nullptr || kd == nullptr) {
    thread_error_ = "MIT params are null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  auto& fake = current();
  Failure failure;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    MitParamsCall call;
    call.device = device;
    std::copy_n(kp, call.kp.size(), call.kp.begin());
    std::copy_n(kd, call.kd.size(), call.kd.begin());
    fake.mit_params_calls_.push_back(call);
    failure = fake.mit_params_failure_;
  }
  return finish_status(failure);
}

WujiStatus FakeHand2Sdk::calibrate_fingertip(WujiDevice* device) {
  record("calibrate_fingertip");
  auto& fake = current();
  Failure failure;
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    fake.tactile_calibrate_devices_.push_back(device);
    failure = fake.tactile_calibrate_failure_;
    hook = fake.tactile_calibrate_hook_;
  }
  if (hook) {
    hook();
  }
  return finish_status(failure);
}

template <std::uint8_t Finger>
WujiStatus FakeHand2Sdk::get_fingertip_status(WujiDevice* device,
                                              WujiFingertipSensorStatus* output) {
  return read_fingertip_status(device, Finger, output);
}

WujiStatus FakeHand2Sdk::read_fingertip_status(WujiDevice* device, std::uint8_t finger,
                                               WujiFingertipSensorStatus* output) {
  record("fingertip_status");
  if (output == nullptr) {
    thread_error_ = "fingertip status output is null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  auto& fake = current();
  TactileStatusReply reply;
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    fake.tactile_status_calls_.push_back(TactileStatusCall{device, finger});
    if (fake.tactile_status_reply_index_ < fake.tactile_status_replies_.size()) {
      reply = fake.tactile_status_replies_[fake.tactile_status_reply_index_++];
    }
    hook = fake.tactile_status_hook_;
  }
  if (hook) {
    hook();
  }
  if (reply.status != WUJI_STATUS_OK) {
    thread_error_ = reply.error;
    return reply.status;
  }
  output->model = reply.model;
  output->state = reply.state;
  return WUJI_STATUS_OK;
}

WujiStatus FakeHand2Sdk::open_joint_command_publisher(WujiDevice*,
                                                      WujiJointCommandPublisher** output) {
  record("open_joint_command_publisher");
  auto& fake = current();
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    hook = fake.open_publisher_hook_;
  }
  if (hook) {
    hook();
  }
  if (output == nullptr) {
    thread_error_ = "publisher output is null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  *output = nullptr;
  if (fake.open_publisher_failure_.status == WUJI_STATUS_OK ||
      fake.open_publisher_failure_.return_handle) {
    *output = new WujiJointCommandPublisher{1};
  }
  return finish_status(fake.open_publisher_failure_);
}

WujiStatus FakeHand2Sdk::send_joint_commands(WujiJointCommandPublisher*,
                                             const WujiJointCommand* commands) {
  record("send_joint_commands");
  auto& fake = current();
  if (commands == nullptr) {
    thread_error_ = "commands are null";
    return WUJI_STATUS_ERR_INVALID_ARG;
  }
  std::array<WujiJointCommand, WUJI_HAND_2_JOINT_COUNT> copied{};
  std::copy_n(commands, WUJI_HAND_2_JOINT_COUNT, copied.begin());
  Failure failure;
  std::function<void()> hook;
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    failure = fake.send_failure_;
    hook = fake.send_hook_;
  }
  if (hook) {
    hook();
  }
  {
    const std::lock_guard<std::mutex> lock(fake.mutex_);
    fake.sent_commands_.push_back(copied);
  }
  return finish_status(failure);
}

void FakeHand2Sdk::close_joint_command_publisher(WujiJointCommandPublisher* publisher) {
  record("close_joint_command_publisher");
  delete publisher;
}

}  // namespace wuji_hand2_driver::test
