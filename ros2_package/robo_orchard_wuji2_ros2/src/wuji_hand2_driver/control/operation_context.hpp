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

#ifndef WUJI_HAND2_DRIVER__CONTROL__OPERATION_CONTEXT_HPP_
#define WUJI_HAND2_DRIVER__CONTROL__OPERATION_CONTEXT_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include "wuji_hand2_driver/conversion/joint_mapping.hpp"
#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"
#include "wuji_hand2_driver/sdk/hand2_session.hpp"

namespace wuji_hand2_driver::control {

class OperationGate {
 public:
  // Serialize publisher writes with emergency-stop cancellation. An epoch
  // change interrupts an active reset without latching ordinary control off.
  std::mutex command_mutex;
  std::uint64_t motion_epoch{0};  // Protected by command_mutex.

  bool accepting() const noexcept {
    return accepting_.load(std::memory_order_acquire);
  }

  void stop() noexcept {
    accepting_.store(false, std::memory_order_release);
  }

 private:
  std::atomic<bool> accepting_{true};
};

struct OperationContext {
  sdk::Hand2SdkApi& sdk;
  sdk::Hand2Session& session;
  const conversion::JointMapping& mapping;
  OperationGate& gate;
  rclcpp::Node& node;
  rclcpp::Logger logger;
};

}  // namespace wuji_hand2_driver::control

#endif  // WUJI_HAND2_DRIVER__CONTROL__OPERATION_CONTEXT_HPP_
