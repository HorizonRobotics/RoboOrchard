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

#ifndef INCLUDE__WUJI_DRIVER_COMMON__STREAM__STREAM_REGISTRY_HPP_
#define INCLUDE__WUJI_DRIVER_COMMON__STREAM__STREAM_REGISTRY_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include "wuji_driver_common/stream/status.hpp"
#include "wuji_driver_common/stream/stream_controller.hpp"
#include "wuji_driver_common/stream/stream_types.hpp"

namespace wuji_driver_common::stream {

class StreamRegistry {
 public:
  StreamRegistry() = default;
  ~StreamRegistry() noexcept;

  StreamRegistry(const StreamRegistry&) = delete;
  StreamRegistry& operator=(const StreamRegistry&) = delete;
  StreamRegistry(StreamRegistry&&) = delete;
  StreamRegistry& operator=(StreamRegistry&&) = delete;

  Status add(std::unique_ptr<StreamController> stream);
  Status start_required();
  // Starts the optional streams in order. `cancelled`, when set, is checked before each
  // stream so a shutdown request skips the ones not yet started.
  Status start_optional(const std::function<bool()>& cancelled = {});
  void stop(StreamKey key) noexcept;
  void stop_all() noexcept;
  std::vector<StreamSnapshot> snapshots() const;

 private:
  enum class State {
    Building,
    StartingRequired,
    RequiredRunning,
    StartingOptional,
    Running,
    Stopped,
  };

  mutable std::mutex mutex_;
  std::vector<std::unique_ptr<StreamController>> streams_;
  State state_{State::Building};
};

}  // namespace wuji_driver_common::stream

#endif  // INCLUDE__WUJI_DRIVER_COMMON__STREAM__STREAM_REGISTRY_HPP_
