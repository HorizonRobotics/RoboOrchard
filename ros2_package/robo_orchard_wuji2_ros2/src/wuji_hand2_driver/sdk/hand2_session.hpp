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

#ifndef WUJI_HAND2_DRIVER__SDK__HAND2_SESSION_HPP_
#define WUJI_HAND2_DRIVER__SDK__HAND2_SESSION_HPP_

#include <wuji_sdk.h>

#include <array>
#include <cstdint>
#include <optional>
#include <string>

#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"
#include "wuji_hand2_driver/sdk/sdk_status.hpp"

namespace wuji_hand2_driver::sdk {

struct Hand2ConnectConfig {
  std::string endpoint;
  std::string serial_number;
  std::string hand_side;
  double connect_timeout{10.0};
  std::optional<std::uint32_t> retry_count;
};

class Hand2Session {
 public:
  explicit Hand2Session(Hand2SdkApi& sdk) noexcept;
  ~Hand2Session() noexcept;

  Hand2Session(const Hand2Session&) = delete;
  Hand2Session& operator=(const Hand2Session&) = delete;

  Status connect(const Hand2ConnectConfig& config);
  Status connect_device(const Hand2ConnectConfig& config, WujiStatus* connect_result = nullptr);
  Status open_command_publisher();
  Status send(const std::array<WujiJointCommand, WUJI_HAND_2_JOINT_COUNT>& commands);
  Status enable_all(WujiStatus* result_output = nullptr) noexcept;
  Status disable_all() noexcept;
  void close_command_publisher() noexcept;
  void disconnect() noexcept;

  WujiDevice* device() const noexcept;
  std::string serial_number() const;
  std::string model() const;
  std::string hand_side() const;
  bool is_connected() const noexcept;

 private:
  Status acquire_runtime();
  void release_runtime() noexcept;
  Status read_device_string(decltype(&wuji_dev_serial_number) getter, const char* operation,
                            std::string& output, WujiStatus* failure_result_output);

  Hand2SdkApi& sdk_;
  WujiDevice* device_{nullptr};
  WujiJointCommandPublisher* command_publisher_{nullptr};
  std::string serial_number_;
  std::string model_;
  std::string hand_side_;
  bool runtime_acquired_{false};
};

}  // namespace wuji_hand2_driver::sdk

#endif  // WUJI_HAND2_DRIVER__SDK__HAND2_SESSION_HPP_
