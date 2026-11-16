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

#ifndef WUJI_HAND2_DRIVER__SDK__HAND2_SDK_API_HPP_
#define WUJI_HAND2_DRIVER__SDK__HAND2_SDK_API_HPP_

#include <wuji_sdk.h>

#include <array>

namespace wuji_hand2_driver::sdk {

struct Hand2SdkApi {
  using FingertipSubscribeFn = decltype(&wuji_hand_2_subscribe_fingertip_thumb_data);
  using FingertipInfoFn = decltype(&wuji_hand_2_get_fingertip_thumb_info);
  using FingertipStatusFn = decltype(&wuji_hand_2_get_fingertip_thumb_status);

  decltype(&wuji_init) init;
  decltype(&wuji_shutdown) shutdown;
  decltype(&wuji_last_error) last_error;
  decltype(&wuji_connect_options_default) connect_options_default;
  decltype(&wuji_scan) scan;
  decltype(&wuji_discovered_free) discovered_free;
  decltype(&wuji_connect) connect;
  decltype(&wuji_dev_disconnect) disconnect;
  decltype(&wuji_dev_release) release;
  decltype(&wuji_dev_serial_number) device_serial_number;
  decltype(&wuji_dev_model) device_model;
  decltype(&wuji_dev_is_connected) device_is_connected;
  decltype(&wuji_sub_close) sub_close;
  decltype(&wuji_hand_2_subscribe_joint_states) subscribe_joint_states;
  decltype(&wuji_hand_2_subscribe_joint_diagnostics) subscribe_joint_diagnostics;
  decltype(&wuji_hand_2_subscribe_imu) subscribe_imu;
  std::array<FingertipSubscribeFn, 5> subscribe_fingertip;
  std::array<FingertipInfoFn, 5> get_fingertip_info;
  decltype(&wuji_hand_2_fingertip_online) fingertip_online;
  decltype(&wuji_hand_2_get_handedness) get_handedness;
  decltype(&wuji_hand_2_get_hw_version) get_hw_version;
  decltype(&wuji_hand_2_enable) enable;
  decltype(&wuji_hand_2_disable) disable;
  decltype(&wuji_hand_2_clear_fault) clear_fault;
  decltype(&wuji_hand_2_emergency_stop) emergency_stop;
  decltype(&wuji_hand_2_set_all_effort_limit) set_effort_limit;
  decltype(&wuji_hand_2_set_all_effort_limit_per_joint) set_effort_limits;
  decltype(&wuji_hand_2_set_all_mit_params) set_mit_params;
  decltype(&wuji_hand_2_calibrate_fingertip) calibrate_fingertip;
  std::array<FingertipStatusFn, 5> get_fingertip_status;
  decltype(&wuji_hand_2_joint_command_publish) open_joint_command_publisher;
  decltype(&wuji_joint_command_publisher_send) send_joint_commands;
  decltype(&wuji_joint_command_publisher_close) close_joint_command_publisher;

  static Hand2SdkApi production() noexcept;
};

}  // namespace wuji_hand2_driver::sdk

#endif  // WUJI_HAND2_DRIVER__SDK__HAND2_SDK_API_HPP_
