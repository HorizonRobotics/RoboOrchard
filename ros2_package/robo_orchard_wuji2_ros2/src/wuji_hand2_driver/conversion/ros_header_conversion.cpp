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

#include "wuji_hand2_driver/conversion/ros_header_conversion.hpp"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

namespace wuji_hand2_driver::conversion {

Status to_ros_header(const WujiFrameHeader& input, std_msgs::msg::Header& output) {
  if (input.timestamp_us == 0) {
    return Status::failure("SDK timestamp must be nonzero");
  }

  constexpr std::uint64_t kMicrosecondsPerSecond = 1000000;
  constexpr std::uint64_t kNanosecondsPerMicrosecond = 1000;
  const std::uint64_t seconds = input.timestamp_us / kMicrosecondsPerSecond;
  if (seconds > static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max())) {
    return Status::failure("SDK timestamp exceeds the ROS Time range");
  }

  const auto terminator = std::find(std::begin(input.frame_id), std::end(input.frame_id), '\0');
  if (terminator == std::end(input.frame_id)) {
    return Status::failure("SDK frame_id is not NUL-terminated");
  }

  std_msgs::msg::Header converted;
  converted.stamp.sec = static_cast<std::int32_t>(seconds);
  converted.stamp.nanosec = static_cast<std::uint32_t>(
      (input.timestamp_us % kMicrosecondsPerSecond) * kNanosecondsPerMicrosecond);
  converted.frame_id.assign(input.frame_id, terminator);
  output = std::move(converted);
  return Status::success();
}

}  // namespace wuji_hand2_driver::conversion
