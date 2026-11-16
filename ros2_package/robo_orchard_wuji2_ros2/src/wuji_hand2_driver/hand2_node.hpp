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

#ifndef WUJI_HAND2_DRIVER__HAND2_NODE_HPP_
#define WUJI_HAND2_DRIVER__HAND2_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"
#include "wuji_hand2_driver/sdk/sdk_status.hpp"

namespace wuji_hand2_driver {

struct Hand2NodeHooks {
  using Clock = std::chrono::steady_clock;

  std::function<Clock::time_point()> now;
  std::function<void(Clock::duration)> sleep_for;
  std::function<bool()> cancelled;
  std::function<void()> request_shutdown;
};

class Hand2Node final : public rclcpp::Node {
 public:
  explicit Hand2Node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  Hand2Node(sdk::Hand2SdkApi sdk, Hand2NodeHooks hooks,
            const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~Hand2Node() noexcept override;

  Hand2Node(const Hand2Node&) = delete;
  Hand2Node& operator=(const Hand2Node&) = delete;

  sdk::Status start();
  void stop() noexcept;

  bool exit_requested() const noexcept;
  int exit_code() const noexcept;
  std::string exit_reason() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace wuji_hand2_driver

#endif  // WUJI_HAND2_DRIVER__HAND2_NODE_HPP_
