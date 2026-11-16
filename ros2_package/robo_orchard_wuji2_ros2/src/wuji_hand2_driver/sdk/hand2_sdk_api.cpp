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

#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"

namespace wuji_hand2_driver::sdk {

Hand2SdkApi Hand2SdkApi::production() noexcept {
  return Hand2SdkApi{
      &wuji_init,
      &wuji_shutdown,
      &wuji_last_error,
      &wuji_connect_options_default,
      &wuji_scan,
      &wuji_discovered_free,
      &wuji_connect,
      &wuji_dev_disconnect,
      &wuji_dev_release,
      &wuji_dev_serial_number,
      &wuji_dev_model,
      &wuji_dev_is_connected,
      &wuji_sub_close,
      &wuji_hand_2_subscribe_joint_states,
      &wuji_hand_2_subscribe_joint_diagnostics,
      &wuji_hand_2_subscribe_imu,
      {&wuji_hand_2_subscribe_fingertip_thumb_data, &wuji_hand_2_subscribe_fingertip_index_data,
       &wuji_hand_2_subscribe_fingertip_middle_data, &wuji_hand_2_subscribe_fingertip_ring_data,
       &wuji_hand_2_subscribe_fingertip_pinky_data},
      {&wuji_hand_2_get_fingertip_thumb_info, &wuji_hand_2_get_fingertip_index_info,
       &wuji_hand_2_get_fingertip_middle_info, &wuji_hand_2_get_fingertip_ring_info,
       &wuji_hand_2_get_fingertip_pinky_info},
      &wuji_hand_2_fingertip_online,
      &wuji_hand_2_get_handedness,
      &wuji_hand_2_get_hw_version,
      &wuji_hand_2_enable,
      &wuji_hand_2_disable,
      &wuji_hand_2_clear_fault,
      &wuji_hand_2_emergency_stop,
      &wuji_hand_2_set_all_effort_limit,
      &wuji_hand_2_set_all_effort_limit_per_joint,
      &wuji_hand_2_set_all_mit_params,
      &wuji_hand_2_calibrate_fingertip,
      {&wuji_hand_2_get_fingertip_thumb_status, &wuji_hand_2_get_fingertip_index_status,
       &wuji_hand_2_get_fingertip_middle_status, &wuji_hand_2_get_fingertip_ring_status,
       &wuji_hand_2_get_fingertip_pinky_status},
      &wuji_hand_2_joint_command_publish,
      &wuji_joint_command_publisher_send,
      &wuji_joint_command_publisher_close};
}

}  // namespace wuji_hand2_driver::sdk
