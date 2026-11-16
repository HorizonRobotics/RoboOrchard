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

#include "wuji_hand2_driver/stream/joint_state_stream.hpp"

#include <exception>
#include <string>
#include <utility>

#include <rclcpp/qos.hpp>

#include "wuji_hand2_driver/conversion/joint_state_conversion.hpp"
#include "wuji_hand2_driver/sdk/sdk_status.hpp"

namespace wuji_hand2_driver::stream {

using Status = wuji_driver_common::stream::Status;
using StreamPolicy = wuji_driver_common::stream::StreamPolicy;

JointStateStream::JointStateStream(wuji_driver_common::stream::StreamKey key, WujiDevice* device,
                                   sdk::Hand2SdkApi& sdk, rclcpp::Node& node,
                                   const conversion::JointMapping& mapping,
                                   std::function<void(const WujiJointStateFrame&)> observer)
    : StreamController(key, device, StreamPolicy::Required),
      sdk_(sdk),
      node_(node),
      mapping_(mapping),
      observer_(std::move(observer)) {}

Status JointStateStream::prepare_impl() {
  publisher_ = node_.create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS().keep_last(5));
  return Status::success();
}

Status JointStateStream::subscribe_impl(WujiSub** output) {
  const WujiStatus status =
      sdk_.subscribe_joint_states(device(), &JointStateStream::sdk_callback, this, output);
  return sdk::sdk_status(sdk_, "wuji_hand_2_subscribe_joint_states", status);
}

void JointStateStream::unsubscribe_impl(WujiSub* subscription) noexcept {
  sdk_.sub_close(subscription);
}

void JointStateStream::cleanup_impl() noexcept {
  publisher_.reset();
}

void JointStateStream::sdk_callback(WujiFrameKind kind, const WujiJointStateFrame* frame,
                                    void* user_data) noexcept {
  if (user_data == nullptr) {
    return;
  }
  auto* const self = static_cast<JointStateStream*>(user_data);
  self->dispatch(kind, frame, [self](const WujiJointStateFrame& received) {
    return self->publish_frame(received);
  });
}

Status JointStateStream::publish_frame(const WujiJointStateFrame& frame) {
  try {
    sensor_msgs::msg::JointState message;
    const Status status = conversion::to_joint_state(frame, mapping_, message);
    if (!status) {
      return status;
    }
    if (observer_) {
      observer_(frame);
    }
    if (publisher_ == nullptr) {
      const std::string message = "joint state publisher is unavailable";
      mark_terminal(wuji_driver_common::stream::TerminalKind::Error, message);
      return Status::failure(message);
    }
    publisher_->publish(message);
    return Status::success();
  } catch (const std::exception& error) {
    const std::string message = error.what();
    mark_terminal(wuji_driver_common::stream::TerminalKind::Error, message);
    return Status::failure(message);
  } catch (...) {
    const std::string message = "joint state publish threw an unknown exception";
    mark_terminal(wuji_driver_common::stream::TerminalKind::Error, message);
    return Status::failure(message);
  }
}

}  // namespace wuji_hand2_driver::stream
