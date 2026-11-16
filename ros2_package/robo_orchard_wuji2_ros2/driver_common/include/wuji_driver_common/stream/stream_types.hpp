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

#ifndef INCLUDE__WUJI_DRIVER_COMMON__STREAM__STREAM_TYPES_HPP_
#define INCLUDE__WUJI_DRIVER_COMMON__STREAM__STREAM_TYPES_HPP_

#include <cstdint>
#include <string>

namespace wuji_driver_common::stream {

struct StreamKey {
  std::uint16_t value;

  friend constexpr bool operator==(StreamKey lhs, StreamKey rhs) noexcept {
    return lhs.value == rhs.value;
  }
};

enum class StreamPolicy : std::uint8_t {
  Required,
  Optional,
};

enum class StreamState : std::uint8_t {
  Created,
  Preparing,
  Subscribing,
  Active,
  Terminal,
  Stopping,
  Stopped,
};

enum class TerminalKind : std::uint8_t {
  None,
  End,
  Error,
};

struct StreamFault {
  TerminalKind kind{TerminalKind::None};
  std::string reason;
};

struct StreamSnapshot {
  StreamKey key{0};
  StreamPolicy policy{StreamPolicy::Required};
  StreamState state{StreamState::Created};
  StreamFault fault;
};

}  // namespace wuji_driver_common::stream

#endif  // INCLUDE__WUJI_DRIVER_COMMON__STREAM__STREAM_TYPES_HPP_
