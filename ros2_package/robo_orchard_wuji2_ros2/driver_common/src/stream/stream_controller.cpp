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

#include "wuji_driver_common/stream/stream_controller.hpp"

#include <cassert>
#include <exception>
#include <mutex>
#include <string>
#include <utility>

namespace wuji_driver_common::stream {

class StreamController::StartRollbackGuard {
 public:
  explicit StartRollbackGuard(StreamController& controller) noexcept : controller_(controller) {}

  ~StartRollbackGuard() noexcept {
    if (armed_) {
      controller_.rollback_failed_start();
    }
  }

  void dismiss() noexcept {
    armed_ = false;
  }

 private:
  StreamController& controller_;
  bool armed_{true};
};

StreamController::StreamController(StreamKey key, WujiDevice* device, StreamPolicy policy)
    : key_(key), device_(device), policy_(policy) {}

StreamController::~StreamController() noexcept {
  assert(subscription_ == nullptr && "StreamController destroyed with an open subscription");
}

Status StreamController::start() {
  if (!mark_preparing()) {
    return Status::failure("stream has already started");
  }

  Status prepare_status = Status::success();
  try {
    prepare_status = prepare_impl();
  } catch (const std::exception& error) {
    prepare_status = Status::failure(error.what());
  } catch (...) {
    prepare_status = Status::failure("prepare threw an unknown exception");
  }
  if (!prepare_status) {
    if (prepare_status.message.empty()) {
      prepare_status.message = "prepare failed";
    }
    mark_terminal(TerminalKind::Error, prepare_status.message);
    return prepare_status;
  }
  prepared_.store(true, std::memory_order_release);
  StartRollbackGuard rollback_guard(*this);
  mark_subscribing();

  WujiSub* local_subscription = nullptr;
  Status subscribe_status = Status::failure("subscribe failed");
  std::exception_ptr subscribe_exception;
  try {
    subscribe_status = subscribe_impl(&local_subscription);
  } catch (...) {
    subscribe_exception = std::current_exception();
  }

  {
    const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    subscription_ = local_subscription;
  }

  std::string exception_message;
  if (subscribe_exception) {
    try {
      std::rethrow_exception(subscribe_exception);
    } catch (const std::exception& error) {
      exception_message = error.what();
    } catch (...) {
      exception_message = "subscribe threw an unknown exception";
    }
  }

  if (!subscribe_exception && subscribe_status && local_subscription != nullptr &&
      activate_if_subscribing()) {
    rollback_guard.dismiss();
    return Status::success();
  }

  const StreamFault current = snapshot().fault;
  std::string failure_message;
  if (current.kind != TerminalKind::None) {
    failure_message = current.reason;
    if (failure_message.empty()) {
      failure_message = "stream terminated during subscribe";
    }
  } else if (!exception_message.empty()) {
    failure_message = std::move(exception_message);
  } else if (!subscribe_status) {
    failure_message = std::move(subscribe_status.message);
    if (failure_message.empty()) {
      failure_message = "subscribe failed";
    }
  } else {
    failure_message = "subscribe returned a null handle";
  }

  if (current.kind == TerminalKind::None) {
    mark_terminal(TerminalKind::Error, failure_message);
  }
  return Status::failure(std::move(failure_message));
}

void StreamController::stop() noexcept {
  mark_stopping();

  WujiSub* subscription = nullptr;
  bool prepared = false;
  {
    const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    subscription = std::exchange(subscription_, nullptr);
    prepared = prepared_.exchange(false, std::memory_order_acq_rel);
  }

  if (subscription != nullptr) {
    unsubscribe_impl(subscription);
  }
  if (prepared) {
    cleanup_impl();
  }
  mark_stopped();
}

StreamSnapshot StreamController::snapshot() const noexcept {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  StreamSnapshot snapshot;
  snapshot.key = key_;
  snapshot.policy = policy_;
  snapshot.state = state_;
  snapshot.fault = fault_;
  return snapshot;
}

void StreamController::mark_terminal(TerminalKind kind, std::string message) noexcept {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (kind == TerminalKind::None || fault_.kind != TerminalKind::None ||
      state_ == StreamState::Terminal || state_ == StreamState::Stopping ||
      state_ == StreamState::Stopped) {
    return;
  }
  fault_.kind = kind;
  fault_.reason = std::move(message);
  state_ = StreamState::Terminal;
}

bool StreamController::mark_preparing() noexcept {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (state_ != StreamState::Created) {
    return false;
  }
  state_ = StreamState::Preparing;
  return true;
}

void StreamController::mark_subscribing() noexcept {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (state_ == StreamState::Preparing) {
    state_ = StreamState::Subscribing;
  }
}

bool StreamController::activate_if_subscribing() noexcept {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (state_ != StreamState::Subscribing) {
    return false;
  }
  state_ = StreamState::Active;
  return true;
}

bool StreamController::accepts_callback() const noexcept {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  return state_ == StreamState::Subscribing || state_ == StreamState::Active;
}

void StreamController::mark_stopping() noexcept {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  if (state_ != StreamState::Stopped) {
    state_ = StreamState::Stopping;
  }
}

void StreamController::mark_stopped() noexcept {
  const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
  state_ = StreamState::Stopped;
}

void StreamController::rollback_failed_start() noexcept {
  WujiSub* subscription = nullptr;
  bool prepared = false;
  {
    const std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    subscription = std::exchange(subscription_, nullptr);
    prepared = prepared_.exchange(false, std::memory_order_acq_rel);
  }

  if (subscription != nullptr) {
    unsubscribe_impl(subscription);
  }
  if (prepared) {
    cleanup_impl();
  }
}

}  // namespace wuji_driver_common::stream
