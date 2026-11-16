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

#ifndef FAKE_SDK__FAKE_HAND2_SDK_HPP_
#define FAKE_SDK__FAKE_HAND2_SDK_HPP_

#include <wuji_sdk.h>

#include <atomic>
#include <array>
#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"

namespace wuji_hand2_driver::test {

class FakeHand2Sdk {
 public:
  enum class TypedStream : std::uint8_t {
    JointState,
    JointDiagnostics,
    Imu,
    FingertipThumb,
    FingertipIndex,
    FingertipMiddle,
    FingertipRing,
    FingertipPinky,
  };

  struct ConnectCall {
    std::uint8_t target_kind;
    std::string target_value;
    std::string alias;
    WujiConnectOptions options;
  };

  struct ConnectReply {
    WujiStatus status{WUJI_STATUS_OK};
    std::string error;
    bool return_device{false};
    std::chrono::milliseconds elapsed{0};
  };

  struct Discovery {
    std::string serial_number;
    std::string model{"WujiHand2"};
    WujiDeviceType device_id{WUJI_DEVICE_TYPE_WUJI_HAND_2};
    WujiTransportType transport{};
    std::string address;
  };

  struct MaskCall {
    WujiDevice* device{nullptr};
    bool used_null{false};
    std::array<std::uint8_t, WUJI_HAND_2_JOINT_COUNT> mask{};
  };

  struct EffortLimitCall {
    WujiDevice* device{nullptr};
    float limit{0.0F};
  };

  struct EffortLimitsCall {
    WujiDevice* device{nullptr};
    std::array<float, WUJI_HAND_2_JOINT_COUNT> limits{};
  };

  struct MitParamsCall {
    WujiDevice* device{nullptr};
    std::array<float, WUJI_HAND_2_JOINT_COUNT> kp{};
    std::array<float, WUJI_HAND_2_JOINT_COUNT> kd{};
  };

  struct TactileStatusCall {
    WujiDevice* device{nullptr};
    std::uint8_t finger{0};
  };

  struct TactileStatusReply {
    WujiStatus status{WUJI_STATUS_OK};
    WujiFingertipSensorModel model{WUJI_FINGERTIP_SENSOR_MODEL_STANDARD};
    WujiFingertipSensorState state{WUJI_FINGERTIP_SENSOR_STATE_READY};
    std::string error;
  };

  FakeHand2Sdk();
  ~FakeHand2Sdk();

  FakeHand2Sdk(const FakeHand2Sdk&) = delete;
  FakeHand2Sdk& operator=(const FakeHand2Sdk&) = delete;

  sdk::Hand2SdkApi& api() noexcept;
  const std::vector<std::string>& calls() const noexcept;
  std::vector<std::string> calls_snapshot() const;
  std::size_t count(std::string_view operation) const noexcept;
  std::vector<TypedStream> closed_streams_snapshot() const;
  const std::vector<ConnectCall>& connect_calls() const noexcept;
  const std::vector<std::array<WujiJointCommand, WUJI_HAND_2_JOINT_COUNT>>& sent_commands()
      const noexcept;
  std::vector<std::array<WujiJointCommand, WUJI_HAND_2_JOINT_COUNT>> sent_commands_snapshot() const;
  std::vector<MaskCall> enable_calls() const;
  std::vector<MaskCall> disable_calls() const;
  std::vector<MaskCall> clear_fault_calls() const;
  std::vector<WujiDevice*> emergency_stop_devices() const;
  std::vector<EffortLimitCall> effort_limit_calls() const;
  std::vector<EffortLimitsCall> effort_limits_calls() const;
  std::vector<MitParamsCall> mit_params_calls() const;
  std::vector<WujiDevice*> tactile_calibrate_devices() const;
  std::vector<TactileStatusCall> tactile_status_calls() const;
  const std::vector<std::size_t>& model_buffer_lengths() const noexcept;
  std::vector<std::uint8_t> fingertip_info_requests() const;
  bool last_disable_used_whole_hand() const noexcept;

  void set_subscribe_failure(TypedStream stream, WujiStatus status, std::string error,
                             bool return_subscription);
  void set_synchronous_joint_state(WujiFrameKind kind, const WujiJointStateFrame* frame) noexcept;
  void set_synchronous_joint_diagnostics(WujiFrameKind kind,
                                         const WujiJointDiagnosticsFrame* frame) noexcept;
  void set_synchronous_imu(WujiFrameKind kind, const WujiImuData* frame) noexcept;
  void set_synchronous_fingertip(std::uint8_t finger, WujiFrameKind kind,
                                 const WujiFingertipSensorData* frame) noexcept;
  void set_callback_hook(TypedStream stream, std::function<void()> hook);
  void set_callback_return_hook(TypedStream stream, std::function<void()> hook);
  bool is_closing(TypedStream stream) const;
  std::size_t callbacks_in_flight(TypedStream stream) const;
  void set_sub_close_hook(std::function<void()> hook);
  void emit_joint_state(WujiFrameKind kind, const WujiJointStateFrame* frame);
  void emit_joint_diagnostics(WujiFrameKind kind, const WujiJointDiagnosticsFrame* frame);
  void emit_imu(WujiFrameKind kind, const WujiImuData* frame);
  void emit_fingertip(std::uint8_t finger, WujiFrameKind kind,
                      const WujiFingertipSensorData* frame);
  void emit_closed_joint_state(WujiFrameKind kind, const WujiJointStateFrame* frame);
  void emit_closed_joint_diagnostics(WujiFrameKind kind, const WujiJointDiagnosticsFrame* frame);
  void emit_closed_imu(WujiFrameKind kind, const WujiImuData* frame);
  void emit_closed_fingertip(std::uint8_t finger, WujiFrameKind kind,
                             const WujiFingertipSensorData* frame);

  void set_default_connect_options(const WujiConnectOptions& options) noexcept;
  void set_scan_results_sequence(std::vector<std::vector<Discovery>> results);
  void set_scan_failure(WujiStatus status, std::string error);
  void set_identity_from_connect_target(bool enabled) noexcept;
  void set_init_failure(WujiStatus status, std::string error);
  void set_connect_failure(WujiStatus status, std::string error, bool return_device);
  void set_connect_replies(std::vector<ConnectReply> replies,
                           std::function<void(std::chrono::milliseconds)> time_advance = {});
  void set_open_publisher_failure(WujiStatus status, std::string error, bool return_publisher);
  void set_open_publisher_hook(std::function<void()> hook);
  void set_send_failure(WujiStatus status, std::string error);
  void set_enable_failure(WujiStatus status, std::string error);
  void set_disable_failure(WujiStatus status, std::string error);
  void set_clear_fault_failure(WujiStatus status, std::string error);
  void set_emergency_stop_failure(WujiStatus status, std::string error);
  void set_effort_limit_failure(WujiStatus status, std::string error);
  void set_effort_limits_failure(WujiStatus status, std::string error);
  void set_mit_params_failure(WujiStatus status, std::string error);
  void set_tactile_calibrate_failure(WujiStatus status, std::string error);
  void set_tactile_status_replies(std::vector<TactileStatusReply> replies);
  void set_send_hook(std::function<void()> hook);
  void set_enable_hook(std::function<void()> hook);
  void set_disable_hook(std::function<void()> hook);
  void set_handedness_hook(std::function<void()> hook);
  void set_fingertip_info_hook(std::function<void()> hook);
  void set_emergency_stop_hook(std::function<void()> hook);
  void set_effort_limit_hook(std::function<void()> hook);
  void set_tactile_calibrate_hook(std::function<void()> hook);
  void set_tactile_status_hook(std::function<void()> hook);
  void set_disconnect_failure(WujiStatus status, std::string error);
  void set_serial_number(std::string serial_number);
  void set_serial_number_failure(std::string error);
  void set_serial_number_sequence(std::vector<std::string> values);
  void set_serial_number_failure_on_call(std::size_t call, std::string error,
                                         WujiStatus status = WUJI_STATUS_ERR_INVALID_ARG);
  void set_model(std::string model);
  void set_model_failure(std::string error);
  void set_model_sequence(std::vector<std::string> values);
  void set_model_failure_on_call(std::size_t call, std::string error,
                                 WujiStatus status = WUJI_STATUS_ERR_INVALID_ARG);
  void set_connected(bool connected) noexcept;
  void set_handedness(WujiHandedness handedness) noexcept;
  void set_handedness_failure(WujiStatus status, std::string error);
  void set_handedness_failure_on_call(std::size_t call, WujiStatus status, std::string error);
  void set_fingertip_info(std::uint8_t finger, const WujiFingertipSensorInfo& info);
  void set_fingertip_info_failure(std::uint8_t finger, WujiStatus status, std::string error);
  void set_fingertip_online(const std::array<bool, 5>& online);
  void set_fingertip_online_failure(WujiStatus status, std::string error);
  void overwrite_last_error(std::string error);

 private:
  struct Failure {
    WujiStatus status{WUJI_STATUS_OK};
    std::string error;
    bool return_handle{false};
  };

  template <typename CallbackT, typename FrameT>
  struct SubscriptionSlot {
    CallbackT callback{nullptr};
    void* user_data{nullptr};
    WujiSub* subscription{nullptr};
    bool closing{false};
    bool terminal_delivered{false};
    std::size_t callbacks_in_flight{0};
    std::function<void()> callback_hook;
    std::function<void()> callback_return_hook;
    std::optional<WujiFrameKind> synchronous_kind;
    const FrameT* synchronous_frame{nullptr};
  };

  struct DeviceStringControl {
    std::string value;
    std::string failure;
    std::vector<std::string> sequence;
    WujiStatus failure_status{WUJI_STATUS_ERR_INVALID_ARG};
    std::size_t calls{0};
    std::size_t failure_call{0};
    std::string call_failure;
    std::vector<std::size_t> buffer_lengths;
    WujiStatus call_failure_status{WUJI_STATUS_ERR_INVALID_ARG};
  };

  static FakeHand2Sdk& current();
  static void record(std::string operation);
  static WujiStatus finish_status(const Failure& failure);
  static int32_t copy_device_string(std::string_view operation, DeviceStringControl& control,
                                    char* buffer, std::size_t buffer_length);
  static std::size_t stream_index(TypedStream stream) noexcept;
  static std::size_t fingertip_index(std::uint8_t finger);
  template <typename CallbackT, typename FrameT>
  static WujiStatus subscribe_typed(std::string operation, TypedStream stream, CallbackT callback,
                                    void* user_data, WujiSub** output,
                                    SubscriptionSlot<CallbackT, FrameT>& slot);
  template <typename CallbackT, typename FrameT>
  static void emit_typed(SubscriptionSlot<CallbackT, FrameT>& slot, WujiFrameKind kind,
                         const FrameT* frame, bool require_active);

  static WujiStatus init(const WujiInitOptions* options);
  static void shutdown();
  static const char* last_error();
  static WujiConnectOptions connect_options_default();
  static WujiStatus scan(WujiDiscovered** output, std::size_t* output_size);
  static void discovered_free(WujiDiscovered* discovered, std::size_t size);
  static WujiStatus connect(const WujiConnectTarget* target, const char* alias,
                            const WujiConnectOptions* options, WujiDevice** output);
  static WujiStatus disconnect(WujiDevice* device);
  static void release(WujiDevice* device);
  static int32_t device_serial_number(WujiDevice* device, char* buffer, std::size_t buffer_length);
  static int32_t device_model(WujiDevice* device, char* buffer, std::size_t buffer_length);
  static std::uint8_t device_is_connected(WujiDevice* device);
  static void sub_close(WujiSub* subscription);
  static WujiStatus subscribe_joint_states(WujiDevice* device,
                                           WujiJointStateFrameSubCallback callback, void* user_data,
                                           WujiSub** output);
  static WujiStatus subscribe_joint_diagnostics(WujiDevice* device,
                                                WujiJointDiagnosticsFrameSubCallback callback,
                                                void* user_data, WujiSub** output);
  static WujiStatus subscribe_imu(WujiDevice* device, WujiImuDataSubCallback callback,
                                  void* user_data, WujiSub** output);
  static WujiStatus subscribe_fingertip_thumb(WujiDevice* device,
                                              WujiFingertipSensorDataSubCallback callback,
                                              void* user_data, WujiSub** output);
  static WujiStatus subscribe_fingertip_index(WujiDevice* device,
                                              WujiFingertipSensorDataSubCallback callback,
                                              void* user_data, WujiSub** output);
  static WujiStatus subscribe_fingertip_middle(WujiDevice* device,
                                               WujiFingertipSensorDataSubCallback callback,
                                               void* user_data, WujiSub** output);
  static WujiStatus subscribe_fingertip_ring(WujiDevice* device,
                                             WujiFingertipSensorDataSubCallback callback,
                                             void* user_data, WujiSub** output);
  static WujiStatus subscribe_fingertip_pinky(WujiDevice* device,
                                              WujiFingertipSensorDataSubCallback callback,
                                              void* user_data, WujiSub** output);
  template <std::uint8_t Finger>
  static WujiStatus get_fingertip_info(WujiDevice* device, WujiFingertipSensorInfo* output);
  static WujiStatus fingertip_online(WujiDevice* device, bool* output);
  static WujiStatus get_handedness(WujiDevice* device, WujiHandedness* output);
  static WujiStatus get_hw_version(WujiDevice* device, WujiHwVersion* output);
  static WujiStatus enable(WujiDevice* device, const std::uint8_t* mask);
  static WujiStatus disable(WujiDevice* device, const std::uint8_t* mask);
  static WujiStatus clear_fault(WujiDevice* device, const std::uint8_t* mask);
  static WujiStatus emergency_stop(WujiDevice* device);
  static WujiStatus set_effort_limit(WujiDevice* device, float limit);
  static WujiStatus set_effort_limits(WujiDevice* device, const float* limits);
  static WujiStatus set_mit_params(WujiDevice* device, const float* kp, const float* kd);
  static WujiStatus calibrate_fingertip(WujiDevice* device);
  template <std::uint8_t Finger>
  static WujiStatus get_fingertip_status(WujiDevice* device, WujiFingertipSensorStatus* output);
  static WujiStatus read_fingertip_info(std::uint8_t finger, WujiFingertipSensorInfo* output);
  static WujiStatus read_fingertip_status(WujiDevice* device, std::uint8_t finger,
                                          WujiFingertipSensorStatus* output);
  static WujiStatus open_joint_command_publisher(WujiDevice* device,
                                                 WujiJointCommandPublisher** output);
  static WujiStatus send_joint_commands(WujiJointCommandPublisher* publisher,
                                        const WujiJointCommand* commands);
  static void close_joint_command_publisher(WujiJointCommandPublisher* publisher);

  static FakeHand2Sdk* current_;

  sdk::Hand2SdkApi api_;
  std::vector<std::string> calls_;
  std::vector<ConnectCall> connect_calls_;
  std::vector<TypedStream> closed_streams_;
  std::vector<std::vector<Discovery>> scan_results_sequence_;
  std::size_t scan_result_index_{0};
  Failure scan_failure_;
  bool identity_from_connect_target_{false};
  std::vector<std::array<WujiJointCommand, WUJI_HAND_2_JOINT_COUNT>> sent_commands_;
  WujiConnectOptions default_connect_options_{};
  Failure init_failure_;
  Failure connect_failure_;
  Failure open_publisher_failure_;
  std::vector<ConnectReply> connect_replies_;
  std::function<void()> open_publisher_hook_;
  std::size_t connect_reply_index_{0};
  std::function<void(std::chrono::milliseconds)> connect_time_advance_;
  Failure send_failure_;
  Failure enable_failure_;
  Failure disable_failure_;
  Failure clear_fault_failure_;
  Failure emergency_stop_failure_;
  Failure effort_limit_failure_;
  Failure effort_limits_failure_;
  Failure mit_params_failure_;
  Failure tactile_calibrate_failure_;
  Failure disconnect_failure_;
  Failure handedness_failure_;
  std::array<Failure, 8> subscribe_failures_;
  std::size_t handedness_calls_{0};
  std::size_t handedness_failure_call_{0};
  Failure handedness_call_failure_;
  std::vector<MaskCall> enable_calls_;
  std::vector<MaskCall> disable_calls_;
  std::vector<MaskCall> clear_fault_calls_;
  std::vector<WujiDevice*> emergency_stop_devices_;
  std::vector<EffortLimitCall> effort_limit_calls_;
  std::vector<EffortLimitsCall> effort_limits_calls_;
  std::vector<MitParamsCall> mit_params_calls_;
  std::vector<WujiDevice*> tactile_calibrate_devices_;
  std::vector<TactileStatusCall> tactile_status_calls_;
  std::vector<TactileStatusReply> tactile_status_replies_;
  std::size_t tactile_status_reply_index_{0};
  std::function<void()> send_hook_;
  std::function<void()> enable_hook_;
  std::function<void()> disable_hook_;
  std::function<void()> handedness_hook_;
  std::function<void()> fingertip_info_hook_;
  std::function<void()> emergency_stop_hook_;
  std::function<void()> effort_limit_hook_;
  std::function<void()> tactile_calibrate_hook_;
  std::function<void()> tactile_status_hook_;
  SubscriptionSlot<WujiJointStateFrameSubCallback, WujiJointStateFrame> joint_state_subscription_;
  SubscriptionSlot<WujiJointDiagnosticsFrameSubCallback, WujiJointDiagnosticsFrame>
      joint_diagnostics_subscription_;
  SubscriptionSlot<WujiImuDataSubCallback, WujiImuData> imu_subscription_;
  std::array<SubscriptionSlot<WujiFingertipSensorDataSubCallback, WujiFingertipSensorData>, 5>
      fingertip_subscriptions_;
  std::array<WujiFingertipSensorInfo, 5> fingertip_info_{};
  std::array<Failure, 5> fingertip_info_failures_{};
  std::array<bool, 5> fingertip_online_{true, true, true, true, true};
  Failure fingertip_online_failure_;
  std::vector<std::uint8_t> fingertip_info_requests_;
  std::function<void()> sub_close_hook_;
  DeviceStringControl serial_number_{"WH2J000000000001"};
  DeviceStringControl model_{"0x0002"};
  bool connected_{true};
  WujiHandedness handedness_{WUJI_HANDEDNESS_LEFT};
  bool last_disable_used_whole_hand_{false};
  std::atomic<std::uint64_t> next_token_{1};
  mutable std::mutex mutex_;
  std::condition_variable callbacks_drained_;

  static thread_local std::string thread_error_;
};

}  // namespace wuji_hand2_driver::test

#endif  // FAKE_SDK__FAKE_HAND2_SDK_HPP_
