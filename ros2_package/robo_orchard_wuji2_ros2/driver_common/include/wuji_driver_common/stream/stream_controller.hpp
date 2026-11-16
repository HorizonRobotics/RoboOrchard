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

#ifndef INCLUDE__WUJI_DRIVER_COMMON__STREAM__STREAM_CONTROLLER_HPP_
#define INCLUDE__WUJI_DRIVER_COMMON__STREAM__STREAM_CONTROLLER_HPP_

#include <wuji_sdk.h>

#include <atomic>
#include <mutex>
#include <string>
#include <utility>

#include "wuji_driver_common/stream/status.hpp"
#include "wuji_driver_common/stream/stream_types.hpp"

namespace wuji_driver_common::stream {

class StreamController {
 public:
  StreamController(StreamKey key, WujiDevice* device, StreamPolicy policy);
  virtual ~StreamController() noexcept;

  StreamController(const StreamController&) = delete;
  StreamController& operator=(const StreamController&) = delete;
  StreamController(StreamController&&) = delete;
  StreamController& operator=(StreamController&&) = delete;

  Status start();
  void stop() noexcept;
  StreamSnapshot snapshot() const noexcept;
  StreamKey key() const noexcept {
    return key_;
  }
  StreamPolicy policy() const noexcept {
    return policy_;
  }

 protected:
  WujiDevice* device() const noexcept {
    return device_;
  }

  virtual Status prepare_impl() = 0;
  virtual Status subscribe_impl(WujiSub** output) = 0;
  virtual void unsubscribe_impl(WujiSub* subscription) noexcept = 0;
  virtual void cleanup_impl() noexcept = 0;

  template <class Frame, class OnOk>
  void dispatch(WujiFrameKind kind, const Frame* frame, OnOk&& on_ok) noexcept {
    try {
      if (!accepts_callback()) {
        return;
      }

      switch (kind) {
        case WUJI_FRAME_KIND_OK:
          if (frame == nullptr) {
            return;
          }
          static_cast<void>(std::forward<OnOk>(on_ok)(*frame));
          return;
        case WUJI_FRAME_KIND_LAG:
          return;
        case WUJI_FRAME_KIND_END:
          mark_terminal(TerminalKind::End, "stream ended");
          return;
        case WUJI_FRAME_KIND_ERROR:
          mark_terminal(TerminalKind::Error, "stream error");
          return;
      }
    } catch (...) {
      mark_terminal(TerminalKind::Error, "stream callback threw an exception");
    }
  }

  void mark_terminal(TerminalKind kind, std::string message) noexcept;

 private:
  class StartRollbackGuard;
  void rollback_failed_start() noexcept;
  bool mark_preparing() noexcept;
  void mark_subscribing() noexcept;
  bool activate_if_subscribing() noexcept;
  bool accepts_callback() const noexcept;
  void mark_stopping() noexcept;
  void mark_stopped() noexcept;

  StreamKey key_;
  WujiDevice* device_;
  StreamPolicy policy_;
  StreamState state_{StreamState::Created};
  StreamFault fault_;
  mutable std::mutex lifecycle_mutex_;
  WujiSub* subscription_{nullptr};
  std::atomic<bool> prepared_{false};
};

}  // namespace wuji_driver_common::stream

#endif  // INCLUDE__WUJI_DRIVER_COMMON__STREAM__STREAM_CONTROLLER_HPP_
