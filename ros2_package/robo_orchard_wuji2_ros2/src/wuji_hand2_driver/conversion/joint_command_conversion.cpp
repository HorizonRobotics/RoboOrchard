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

#include "wuji_hand2_driver/conversion/joint_command_conversion.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace wuji_hand2_driver::conversion {
namespace {

bool command_value_fits_float(double value) noexcept {
  return std::isfinite(value) &&
         value >= static_cast<double>(std::numeric_limits<float>::lowest()) &&
         value <= static_cast<double>(std::numeric_limits<float>::max());
}

bool to_finite_float(double input, float& output) {
  if (!command_value_fits_float(input)) {
    return false;
  }
  const float converted = static_cast<float>(input);
  output = converted;
  return true;
}

Status fill_command(const sensor_msgs::msg::JointState& input, std::size_t source_index,
                    std::size_t dense_index,
                    std::array<WujiJointCommand, JointMapping::kJointCount>& converted) {
  WujiJointCommand command{};
  if (!to_finite_float(input.position[source_index], command.position)) {
    return Status::failure("joint command contains an unconvertible position");
  }
  if (!input.velocity.empty() && !to_finite_float(input.velocity[source_index], command.velocity)) {
    return Status::failure("joint command contains an unconvertible velocity");
  }
  if (!input.effort.empty() && !to_finite_float(input.effort[source_index], command.effort)) {
    return Status::failure("joint command contains an unconvertible effort");
  }
  converted[dense_index] = command;
  return Status::success();
}

}  // namespace

Status to_joint_commands(const sensor_msgs::msg::JointState& input, const JointMapping& mapping,
                         std::array<WujiJointCommand, JointMapping::kJointCount>& output) {
  const std::size_t expected_size =
      input.name.empty() ? JointMapping::kJointCount : input.name.size();
  if (input.position.size() != expected_size) {
    return Status::failure("joint command position length is invalid");
  }
  if (!input.velocity.empty() && input.velocity.size() != expected_size) {
    return Status::failure("joint command velocity length is invalid");
  }
  if (!input.effort.empty() && input.effort.size() != expected_size) {
    return Status::failure("joint command effort length is invalid");
  }

  std::array<WujiJointCommand, JointMapping::kJointCount> converted{};
  if (input.name.empty()) {
    for (std::size_t index = 0; index < converted.size(); ++index) {
      const Status status = fill_command(input, index, index, converted);
      if (!status) {
        return status;
      }
    }
  } else {
    std::array<bool, JointMapping::kJointCount> seen{};
    for (std::size_t source_index = 0; source_index < input.name.size(); ++source_index) {
      const auto dense_index = mapping.index_from_name(input.name[source_index]);
      if (!dense_index) {
        return Status::failure("joint command contains an unknown or opposite-side name");
      }
      if (seen[*dense_index]) {
        return Status::failure("joint command contains a duplicate name");
      }
      seen[*dense_index] = true;
      const Status status = fill_command(input, source_index, *dense_index, converted);
      if (!status) {
        return status;
      }
    }
  }

  output = converted;
  return Status::success();
}

}  // namespace wuji_hand2_driver::conversion
