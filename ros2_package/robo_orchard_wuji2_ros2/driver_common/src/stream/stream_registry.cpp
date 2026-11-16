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

#include "wuji_driver_common/stream/stream_registry.hpp"

#include <algorithm>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace wuji_driver_common::stream {

namespace {

Status start_stream(StreamController& stream) {
  try {
    return stream.start();
  } catch (const std::exception& error) {
    return Status::failure(error.what());
  } catch (...) {
    return Status::failure("stream start threw an unknown exception");
  }
}

}  // namespace

StreamRegistry::~StreamRegistry() noexcept {
  stop_all();
}

Status StreamRegistry::add(std::unique_ptr<StreamController> stream) {
  const std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::Building) {
    return Status::failure("stream registry is not building");
  }
  if (stream == nullptr) {
    return Status::failure("stream must not be null");
  }
  const StreamKey key = stream->key();
  const auto duplicate = std::find_if(streams_.cbegin(), streams_.cend(),
                                      [key](const auto& entry) { return entry->key() == key; });
  if (duplicate != streams_.cend()) {
    return Status::failure("duplicate stream key");
  }
  streams_.push_back(std::move(stream));
  return Status::success();
}

Status StreamRegistry::start_required() {
  std::vector<StreamController*> streams;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::Building) {
      return Status::failure("stream registry has already started");
    }
    state_ = State::StartingRequired;
    streams.reserve(streams_.size());
    for (const auto& stream : streams_) {
      if (stream->policy() == StreamPolicy::Required) {
        streams.push_back(stream.get());
      }
    }
  }

  std::vector<StreamController*> started;
  started.reserve(streams.size());
  for (StreamController* stream : streams) {
    const Status result = start_stream(*stream);
    if (result) {
      started.push_back(stream);
      continue;
    }
    for (auto iterator = started.rbegin(); iterator != started.rend(); ++iterator) {
      (*iterator)->stop();
    }
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      state_ = State::Stopped;
    }
    return result;
  }

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    state_ = State::RequiredRunning;
  }
  return Status::success();
}

Status StreamRegistry::start_optional(const std::function<bool()>& cancelled) {
  std::vector<StreamController*> streams;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != State::RequiredRunning) {
      return Status::failure("required streams are not running");
    }
    state_ = State::StartingOptional;
    streams.reserve(streams_.size());
    for (const auto& stream : streams_) {
      if (stream->policy() == StreamPolicy::Optional) {
        streams.push_back(stream.get());
      }
    }
  }

  for (StreamController* stream : streams) {
    if (cancelled && cancelled()) {
      break;
    }
    static_cast<void>(start_stream(*stream));
  }

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    state_ = State::Running;
  }
  return Status::success();
}

void StreamRegistry::stop(StreamKey key) noexcept {
  StreamController* stream = nullptr;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    const auto entry = std::find_if(streams_.begin(), streams_.end(), [key](const auto& candidate) {
      return candidate->key() == key;
    });
    if (entry != streams_.end()) {
      stream = entry->get();
    }
  }
  if (stream != nullptr) {
    stream->stop();
  }
}

void StreamRegistry::stop_all() noexcept {
  std::size_t stream_count = 0;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    state_ = State::Stopped;
    stream_count = streams_.size();
  }
  while (stream_count > 0) {
    StreamController* stream = nullptr;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      stream = streams_[--stream_count].get();
    }
    stream->stop();
  }
}

std::vector<StreamSnapshot> StreamRegistry::snapshots() const {
  std::vector<StreamSnapshot> result;
  const std::lock_guard<std::mutex> lock(mutex_);
  result.reserve(streams_.size());
  for (const auto& stream : streams_) {
    result.push_back(stream->snapshot());
  }
  return result;
}

}  // namespace wuji_driver_common::stream
