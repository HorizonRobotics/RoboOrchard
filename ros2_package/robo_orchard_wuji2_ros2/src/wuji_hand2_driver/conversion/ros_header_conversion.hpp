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

#ifndef WUJI_HAND2_DRIVER__CONVERSION__ROS_HEADER_CONVERSION_HPP_
#define WUJI_HAND2_DRIVER__CONVERSION__ROS_HEADER_CONVERSION_HPP_

#include <wuji_sdk.h>

#include <std_msgs/msg/header.hpp>

#include "wuji_driver_common/stream/status.hpp"

namespace wuji_hand2_driver::conversion {

using Status = wuji_driver_common::stream::Status;

Status to_ros_header(const WujiFrameHeader& input, std_msgs::msg::Header& output);

}  // namespace wuji_hand2_driver::conversion

#endif  // WUJI_HAND2_DRIVER__CONVERSION__ROS_HEADER_CONVERSION_HPP_
