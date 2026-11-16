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

#include "wuji_hand2_driver/driver_config.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <utility>
namespace wuji_hand2_driver {

void declare_driver_parameters(rclcpp::Node& node) {
  node.declare_parameter<std::string>("endpoint", "");
  node.declare_parameter<std::string>("serial_number", "");
  node.declare_parameter<std::string>("hand_side", "");
  node.declare_parameter<double>("connect_timeout", 10.0);
  node.declare_parameter<bool>("auto_enable", false);
  node.declare_parameter<bool>("publish_joint_diagnostics", true);
  node.declare_parameter<bool>("publish_imu", true);
  node.declare_parameter<bool>("publish_fingertip_tactile", false);
  node.declare_parameter<std::vector<double>>("reset_joint_position", std::vector<double>(20, 0.0));
  node.declare_parameter<double>("reset_duration_s", 2.0);
  node.declare_parameter<double>("reset_timeout_s", 5.0);
  node.declare_parameter<double>("reset_feedback_timeout_s", 0.2);
  node.declare_parameter<double>("reset_position_tolerance", 0.2);
}

sdk::Status read_driver_config(const rclcpp::Node& node, DriverConfig& output) {
  DriverConfig config;
  config.endpoint = node.get_parameter("endpoint").as_string();
  config.serial_number = node.get_parameter("serial_number").as_string();
  config.hand_side = node.get_parameter("hand_side").as_string();
  config.connect_timeout = node.get_parameter("connect_timeout").as_double();
  config.auto_enable = node.get_parameter("auto_enable").as_bool();
  config.publish_joint_diagnostics = node.get_parameter("publish_joint_diagnostics").as_bool();
  config.publish_imu = node.get_parameter("publish_imu").as_bool();
  config.publish_fingertip_tactile = node.get_parameter("publish_fingertip_tactile").as_bool();
  config.reset_joint_position = node.get_parameter("reset_joint_position").as_double_array();
  config.reset_duration_s = node.get_parameter("reset_duration_s").as_double();
  config.reset_timeout_s = node.get_parameter("reset_timeout_s").as_double();
  config.reset_feedback_timeout_s = node.get_parameter("reset_feedback_timeout_s").as_double();
  config.reset_position_tolerance = node.get_parameter("reset_position_tolerance").as_double();

  if (!config.hand_side.empty() && config.hand_side != "left" && config.hand_side != "right") {
    return sdk::Status::failure("hand_side must be left, right, or empty");
  }
  if (!std::isfinite(config.connect_timeout) || config.connect_timeout < 0.0) {
    return sdk::Status::failure("connect_timeout must be finite and nonnegative");
  }
  if (config.reset_joint_position.size() != 20U) {
    return sdk::Status::failure("reset_joint_position must contain exactly 20 positions in radians");
  }
  for (const double value : config.reset_joint_position) {
    if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
      return sdk::Status::failure("reset_joint_position must contain finite float-representable values");
    }
  }
  if (!std::isfinite(config.reset_duration_s) || config.reset_duration_s <= 0.0 ||
      !std::isfinite(config.reset_timeout_s) || config.reset_timeout_s <= config.reset_duration_s ||
      !std::isfinite(config.reset_feedback_timeout_s) || config.reset_feedback_timeout_s <= 0.0 ||
      !std::isfinite(config.reset_position_tolerance) || config.reset_position_tolerance <= 0.0) {
    return sdk::Status::failure(
        "reset duration, feedback timeout and tolerance must be finite and positive; "
        "reset_timeout_s must exceed reset_duration_s");
  }

  output = std::move(config);
  return sdk::Status::success();
}

}  // namespace wuji_hand2_driver
