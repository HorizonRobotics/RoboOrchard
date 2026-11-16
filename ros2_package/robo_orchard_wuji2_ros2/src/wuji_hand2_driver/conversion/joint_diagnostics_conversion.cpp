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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "wuji_hand2_driver/conversion/joint_diagnostics_conversion.hpp"

#include <array>
#include <cmath>
#include <utility>

#include "wuji_hand2_driver/conversion/ros_header_conversion.hpp"

namespace wuji_hand2_driver::conversion {

Status to_joint_diagnostics(const WujiJointDiagnosticsFrame& input, const JointMapping& mapping,
                            wuji_msgs::msg::JointDiagnosticsArray& output) {
  if (input.joints_len == 0 || input.joints_len > JointMapping::kJointCount) {
    return Status::failure("joint diagnostics frame must contain 1 to 20 joints");
  }
  if (input.num_joints != input.joints_len) {
    return Status::failure("joint diagnostics count does not match its array length");
  }
  if (input.joints == nullptr) {
    return Status::failure("joint diagnostics array is null");
  }

  wuji_msgs::msg::JointDiagnosticsArray converted;
  const Status header_status = to_ros_header(input.header, converted.header);
  if (!header_status) {
    return header_status;
  }
  converted.joints.reserve(input.joints_len);
  std::array<bool, JointMapping::kJointCount> seen{};

  for (std::size_t item_index = 0; item_index < input.joints_len; ++item_index) {
    const auto& item = input.joints[item_index];
    const auto dense_index = mapping.index_from_nid(item.nid);
    const auto name = mapping.name_from_nid(item.nid);
    if (!dense_index || !name) {
      return Status::failure("joint diagnostics contains an invalid nid");
    }
    if (seen[*dense_index]) {
      return Status::failure("joint diagnostics contains a duplicate nid");
    }
    if (!std::isfinite(item.current) || !std::isfinite(item.vbus_v_fb) ||
        !std::isfinite(item.mcu_temp_c_fb)) {
      return Status::failure("joint diagnostics contains a nonfinite value");
    }
    seen[*dense_index] = true;

    wuji_msgs::msg::JointDiagnostics joint;
    joint.nid = item.nid;
    joint.name = *name;
    joint.status_word = item.status_word;
    joint.current = item.current;
    joint.bus_voltage = item.vbus_v_fb;
    joint.mcu_temperature = item.mcu_temp_c_fb;
    joint.error_code = item.error_code_current;
    joint.comm_response_rate_pct = item.comm_response_rate_pct;
    joint.comm_timeout_total = item.comm_timeout_total;
    converted.joints.push_back(std::move(joint));
  }

  output = std::move(converted);
  return Status::success();
}

}  // namespace wuji_hand2_driver::conversion
