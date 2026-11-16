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

#include "wuji_hand2_driver/stream/fingertip_stream.hpp"

#include <exception>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/qos.hpp>

#include "wuji_hand2_driver/sdk/sdk_status.hpp"

namespace wuji_hand2_driver::stream {
namespace {

using Status = wuji_driver_common::stream::Status;
using StreamPolicy = wuji_driver_common::stream::StreamPolicy;
using TerminalKind = wuji_driver_common::stream::TerminalKind;

struct FingerProperties {
  std::uint8_t index;
  const char* name;
  const char* subscribe_operation;
};

Status finger_properties(Fingertip finger, FingerProperties& output) {
  switch (finger) {
    case Fingertip::Thumb:
      output = FingerProperties{0, "thumb", "wuji_hand_2_subscribe_fingertip_thumb_data"};
      return Status::success();
    case Fingertip::Index:
      output = FingerProperties{1, "index", "wuji_hand_2_subscribe_fingertip_index_data"};
      return Status::success();
    case Fingertip::Middle:
      output = FingerProperties{2, "middle", "wuji_hand_2_subscribe_fingertip_middle_data"};
      return Status::success();
    case Fingertip::Ring:
      output = FingerProperties{3, "ring", "wuji_hand_2_subscribe_fingertip_ring_data"};
      return Status::success();
    case Fingertip::Pinky:
      output = FingerProperties{4, "pinky", "wuji_hand_2_subscribe_fingertip_pinky_data"};
      return Status::success();
  }
  return Status::failure("invalid fingertip enum value");
}

}  // namespace

FingertipStream::FingertipStream(wuji_driver_common::stream::StreamKey key, WujiDevice* device,
                                 sdk::Hand2SdkApi& sdk, rclcpp::Node& node, Fingertip finger,
                                 sdk::Hand2SdkApi::FingertipSubscribeFn subscribe) noexcept
    : StreamController(key, device, StreamPolicy::Optional),
      sdk_(sdk),
      node_(node),
      finger_(finger),
      subscribe_(subscribe) {}

Status FingertipStream::prepare_impl() {
  FingerProperties properties{};
  Status status = finger_properties(finger_, properties);
  if (!status) {
    return status;
  }
  if (subscribe_ == nullptr || subscribe_ != sdk_.subscribe_fingertip[properties.index]) {
    return Status::failure("fingertip subscribe function does not match the configured finger");
  }

  WujiFingertipSensorInfo info{};
  const WujiStatus sdk_result = sdk_.get_fingertip_info[properties.index](device(), &info);
  if (sdk_result != WUJI_STATUS_OK) {
    return sdk::sdk_status(sdk_, "get fingertip metadata", sdk_result);
  }

  auto decoder = std::make_unique<conversion::TactileDecoder>();
  status = decoder->configure(info);
  if (!status) {
    return status;
  }
  auto publisher = node_.create_publisher<wuji_msgs::msg::FingertipTactile>(
      std::string("fingertip_tactile/") + properties.name, rclcpp::SensorDataQoS().keep_last(5));

  decoder_ = std::move(decoder);
  publisher_ = std::move(publisher);
  return Status::success();
}

Status FingertipStream::subscribe_impl(WujiSub** output) {
  FingerProperties properties{};
  const Status properties_status = finger_properties(finger_, properties);
  if (!properties_status) {
    return properties_status;
  }
  const WujiStatus status = subscribe_(device(), &FingertipStream::sdk_callback, this, output);
  return sdk::sdk_status(sdk_, properties.subscribe_operation, status);
}

void FingertipStream::unsubscribe_impl(WujiSub* subscription) noexcept {
  sdk_.sub_close(subscription);
}

void FingertipStream::cleanup_impl() noexcept {
  publisher_.reset();
  decoder_.reset();
}

void FingertipStream::sdk_callback(WujiFrameKind kind, const WujiFingertipSensorData* frame,
                                   void* user_data) noexcept {
  if (user_data == nullptr) {
    return;
  }
  auto* const self = static_cast<FingertipStream*>(user_data);
  self->dispatch(kind, frame, [self](const WujiFingertipSensorData& received) {
    return self->publish_frame(received);
  });
}

Status FingertipStream::publish_frame(const WujiFingertipSensorData& frame) {
  try {
    if (decoder_ == nullptr) {
      const std::string message = "fingertip decoder is unavailable";
      mark_terminal(TerminalKind::Error, message);
      return Status::failure(message);
    }
    if (!decoder_->digest_matches(frame)) {
      const std::string message = "fingertip metadata digest mismatch (expected " +
                                  std::to_string(decoder_->info_digest()) + ", received " +
                                  std::to_string(frame.info_digest) + ")";
      mark_terminal(TerminalKind::Error, message);
      return Status::failure(message);
    }

    wuji_msgs::msg::FingertipTactile message;
    const Status status = decoder_->decode(frame, message);
    if (!status) {
      return status;
    }
    if (publisher_ == nullptr) {
      const std::string message = "fingertip publisher is unavailable";
      mark_terminal(TerminalKind::Error, message);
      return Status::failure(message);
    }
    publisher_->publish(message);
    return Status::success();
  } catch (const std::exception& error) {
    const std::string message = error.what();
    mark_terminal(TerminalKind::Error, message);
    return Status::failure(message);
  } catch (...) {
    const std::string message = "fingertip publish threw an unknown exception";
    mark_terminal(TerminalKind::Error, message);
    return Status::failure(message);
  }
}

}  // namespace wuji_hand2_driver::stream
