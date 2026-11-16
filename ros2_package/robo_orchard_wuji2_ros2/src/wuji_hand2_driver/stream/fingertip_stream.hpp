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

#ifndef WUJI_HAND2_DRIVER__STREAM__FINGERTIP_STREAM_HPP_
#define WUJI_HAND2_DRIVER__STREAM__FINGERTIP_STREAM_HPP_

#include <wuji_sdk.h>

#include <cstdint>
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <wuji_msgs/msg/fingertip_tactile.hpp>

#include "wuji_driver_common/stream/status.hpp"
#include "wuji_driver_common/stream/stream_controller.hpp"
#include "wuji_driver_common/stream/stream_types.hpp"
#include "wuji_hand2_driver/conversion/tactile_decoder.hpp"
#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"

namespace wuji_hand2_driver::stream {

enum class Fingertip : std::uint8_t {
  Thumb = 0,
  Index = 1,
  Middle = 2,
  Ring = 3,
  Pinky = 4,
};

class FingertipStream final : public wuji_driver_common::stream::StreamController {
 public:
  FingertipStream(wuji_driver_common::stream::StreamKey key, WujiDevice* device,
                  sdk::Hand2SdkApi& sdk, rclcpp::Node& node, Fingertip finger,
                  sdk::Hand2SdkApi::FingertipSubscribeFn subscribe) noexcept;

 protected:
  wuji_driver_common::stream::Status prepare_impl() override;
  wuji_driver_common::stream::Status subscribe_impl(WujiSub** output) override;
  void unsubscribe_impl(WujiSub* subscription) noexcept override;
  void cleanup_impl() noexcept override;

 private:
  static void sdk_callback(WujiFrameKind kind, const WujiFingertipSensorData* frame,
                           void* user_data) noexcept;
  wuji_driver_common::stream::Status publish_frame(const WujiFingertipSensorData& frame);

  sdk::Hand2SdkApi& sdk_;
  rclcpp::Node& node_;
  Fingertip finger_;
  sdk::Hand2SdkApi::FingertipSubscribeFn subscribe_;
  std::unique_ptr<conversion::TactileDecoder> decoder_;
  rclcpp::Publisher<wuji_msgs::msg::FingertipTactile>::SharedPtr publisher_;
};

}  // namespace wuji_hand2_driver::stream

#endif  // WUJI_HAND2_DRIVER__STREAM__FINGERTIP_STREAM_HPP_
