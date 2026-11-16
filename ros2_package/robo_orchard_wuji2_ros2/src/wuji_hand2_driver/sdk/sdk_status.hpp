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

#ifndef WUJI_HAND2_DRIVER__SDK__SDK_STATUS_HPP_
#define WUJI_HAND2_DRIVER__SDK__SDK_STATUS_HPP_

#include <wuji_sdk.h>

#include <string>
#include <string_view>
#include <utility>

#include "wuji_driver_common/stream/status.hpp"
#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"

namespace wuji_hand2_driver::sdk {

using Status = wuji_driver_common::stream::Status;

inline Status sdk_status(Hand2SdkApi& sdk, std::string_view operation, WujiStatus status) {
  if (status == WUJI_STATUS_OK) {
    return Status::success();
  }

  const char* raw_error = sdk.last_error();
  const std::string error = raw_error == nullptr ? std::string{} : std::string{raw_error};

  std::string message{operation};
  message += " failed (status ";
  message += std::to_string(static_cast<int>(status));
  message += ")";
  if (!error.empty()) {
    message += ": ";
    message += error;
  }
  return Status::failure(std::move(message));
}

}  // namespace wuji_hand2_driver::sdk

#endif  // WUJI_HAND2_DRIVER__SDK__SDK_STATUS_HPP_
