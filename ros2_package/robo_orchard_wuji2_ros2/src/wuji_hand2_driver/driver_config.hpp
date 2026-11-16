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

#ifndef WUJI_HAND2_DRIVER__DRIVER_CONFIG_HPP_
#define WUJI_HAND2_DRIVER__DRIVER_CONFIG_HPP_

#include <string>
#include <vector>

#include <rclcpp/node.hpp>

#include "wuji_hand2_driver/sdk/sdk_status.hpp"

namespace wuji_hand2_driver {

struct DriverConfig {
  std::string endpoint;
  std::string serial_number;
  std::string hand_side;
  double connect_timeout{10.0};
  bool auto_enable{false};
  bool publish_joint_diagnostics{true};
  bool publish_imu{true};
  bool publish_fingertip_tactile{false};
  std::vector<double> reset_joint_position = std::vector<double>(20, 0.0);
  double reset_duration_s{2.0};
  double reset_timeout_s{5.0};
  double reset_feedback_timeout_s{0.2};
  double reset_position_tolerance{0.2};
};

void declare_driver_parameters(rclcpp::Node& node);
sdk::Status read_driver_config(const rclcpp::Node& node, DriverConfig& output);

}  // namespace wuji_hand2_driver

#endif  // WUJI_HAND2_DRIVER__DRIVER_CONFIG_HPP_
