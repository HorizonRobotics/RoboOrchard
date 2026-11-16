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

#ifndef WUJI_HAND2_DRIVER__CONVERSION__TACTILE_DECODER_HPP_
#define WUJI_HAND2_DRIVER__CONVERSION__TACTILE_DECODER_HPP_

#include <wuji_sdk.h>

#include <cstdint>
#include <memory>

#include <wuji_msgs/msg/fingertip_tactile.hpp>

#include "wuji_driver_common/stream/status.hpp"

namespace wuji_hand2_driver::conversion {

class TactileDecoder {
 public:
  using Status = wuji_driver_common::stream::Status;

  Status configure(const WujiFingertipSensorInfo& info);
  Status decode(const WujiFingertipSensorData& input,
                wuji_msgs::msg::FingertipTactile& output) const;

  bool configured() const noexcept;
  std::uint32_t info_digest() const noexcept;
  bool digest_matches(const WujiFingertipSensorData& input) const noexcept;

 private:
  struct Layout;
  std::shared_ptr<const Layout> layout_;
};

}  // namespace wuji_hand2_driver::conversion

#endif  // WUJI_HAND2_DRIVER__CONVERSION__TACTILE_DECODER_HPP_
