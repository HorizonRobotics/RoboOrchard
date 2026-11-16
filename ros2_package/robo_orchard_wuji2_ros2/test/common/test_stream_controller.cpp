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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <wuji_driver_common/stream/status.hpp>
#include <wuji_driver_common/stream/stream_controller.hpp>

namespace wuji_driver_common::stream {
namespace {

struct TestFrame {
  int value;
};

constexpr std::chrono::seconds kCoordinationTimeout{2};

WujiSub* fake_subscription(std::uintptr_t value = 1) {
  return reinterpret_cast<WujiSub*>(value);
}

class TestStream final : public StreamController {
 public:
  explicit TestStream(Status subscribe_status = Status::success(),
                      WujiSub* subscription = fake_subscription(),
                      StreamPolicy policy = StreamPolicy::Required)
      : StreamController(StreamKey{7}, nullptr, policy),
        subscribe_status_(subscribe_status),
        subscription_(subscription) {}

  void set_prepare_status(Status status) {
    prepare_status_ = std::move(status);
  }

  void set_synchronous_callback(WujiFrameKind kind, bool has_frame = true) {
    synchronous_kind_ = kind;
    synchronous_has_frame_ = has_frame;
  }

  void set_emit_during_unsubscribe(bool enabled) {
    emit_during_unsubscribe_ = enabled;
  }

  void emit(WujiFrameKind kind, const TestFrame* frame) noexcept {
    dispatch(kind, frame, [this](const TestFrame& received) -> Status {
      ++on_ok_calls_;
      converted_value_ = received.value;
      return Status::success();
    });
  }

  void emit_throwing_ok(const TestFrame* frame) noexcept {
    dispatch(WUJI_FRAME_KIND_OK, frame,
             [](const TestFrame&) -> Status { throw std::runtime_error("conversion failed"); });
  }

  void terminate(TerminalKind kind, std::string message) noexcept {
    mark_terminal(kind, std::move(message));
  }

  int prepare_calls() const noexcept {
    return prepare_calls_;
  }
  int subscribe_calls() const noexcept {
    return subscribe_calls_;
  }
  int unsubscribe_calls() const noexcept {
    return unsubscribe_calls_;
  }
  int cleanup_calls() const noexcept {
    return cleanup_calls_;
  }
  int on_ok_calls() const noexcept {
    return on_ok_calls_;
  }
  int converted_value() const noexcept {
    return converted_value_;
  }
  StreamState prepare_state() const noexcept {
    return prepare_state_;
  }
  StreamState subscribe_state() const noexcept {
    return subscribe_state_;
  }
  WujiSub* unsubscribed_handle() const noexcept {
    return unsubscribed_handle_;
  }

 protected:
  Status prepare_impl() override {
    ++prepare_calls_;
    prepare_state_ = snapshot().state;
    return prepare_status_;
  }

  Status subscribe_impl(WujiSub** output) override {
    ++subscribe_calls_;
    subscribe_state_ = snapshot().state;
    if (synchronous_kind_) {
      const TestFrame* frame = synchronous_has_frame_ ? &frame_ : nullptr;
      emit(*synchronous_kind_, frame);
    }
    *output = subscription_;
    return subscribe_status_;
  }

  void unsubscribe_impl(WujiSub* subscription) noexcept override {
    ++unsubscribe_calls_;
    unsubscribed_handle_ = subscription;
    if (emit_during_unsubscribe_) {
      emit(WUJI_FRAME_KIND_END, nullptr);
      emit(WUJI_FRAME_KIND_OK, &frame_);
    }
  }

  void cleanup_impl() noexcept override {
    ++cleanup_calls_;
  }

 private:
  Status subscribe_status_;
  WujiSub* subscription_;
  Status prepare_status_{Status::success()};
  std::optional<WujiFrameKind> synchronous_kind_;
  bool synchronous_has_frame_{true};
  bool emit_during_unsubscribe_{false};
  int prepare_calls_{0};
  int subscribe_calls_{0};
  int unsubscribe_calls_{0};
  int cleanup_calls_{0};
  int on_ok_calls_{0};
  int converted_value_{0};
  StreamState prepare_state_{StreamState::Created};
  StreamState subscribe_state_{StreamState::Created};
  WujiSub* unsubscribed_handle_{nullptr};
  TestFrame frame_{42};
};

class CoordinatedCloseStream final : public StreamController {
 public:
  CoordinatedCloseStream() : StreamController(StreamKey{8}, nullptr, StreamPolicy::Required) {}

  void run_blocking_callback() noexcept {
    dispatch(WUJI_FRAME_KIND_OK, &frame_, [this](const TestFrame&) {
      std::unique_lock<std::mutex> lock(coordination_mutex_);
      callback_entered_ = true;
      condition_.notify_all();
      if (!condition_.wait_for(lock, kCoordinationTimeout,
                               [this]() { return release_callback_; })) {
        callback_wait_timed_out_ = true;
      }
      return Status::success();
    });
    {
      const std::lock_guard<std::mutex> lock(coordination_mutex_);
      callback_returned_ = true;
    }
    condition_.notify_all();
  }

  bool wait_for_callback_entry(std::chrono::steady_clock::duration timeout = kCoordinationTimeout) {
    std::unique_lock<std::mutex> lock(coordination_mutex_);
    return condition_.wait_for(lock, timeout, [this]() { return callback_entered_; });
  }

  bool wait_for_close_entry(std::chrono::steady_clock::duration timeout = kCoordinationTimeout) {
    std::unique_lock<std::mutex> lock(coordination_mutex_);
    return condition_.wait_for(lock, timeout, [this]() { return close_entered_; });
  }

  void release_callback() {
    {
      const std::lock_guard<std::mutex> lock(coordination_mutex_);
      release_callback_ = true;
    }
    condition_.notify_all();
  }

  bool resource_alive() const noexcept {
    return resource_alive_.load(std::memory_order_acquire);
  }

  int close_calls() const {
    const std::lock_guard<std::mutex> lock(coordination_mutex_);
    return close_calls_;
  }

  int cleanup_calls() const {
    const std::lock_guard<std::mutex> lock(coordination_mutex_);
    return cleanup_calls_;
  }

  bool cleanup_saw_callback_returned() const {
    const std::lock_guard<std::mutex> lock(coordination_mutex_);
    return cleanup_saw_callback_returned_;
  }

  bool callback_returned() const {
    const std::lock_guard<std::mutex> lock(coordination_mutex_);
    return callback_returned_;
  }

  bool callback_wait_timed_out() const {
    const std::lock_guard<std::mutex> lock(coordination_mutex_);
    return callback_wait_timed_out_;
  }

  bool close_wait_timed_out() const {
    const std::lock_guard<std::mutex> lock(coordination_mutex_);
    return close_wait_timed_out_;
  }

  std::vector<std::string> events() const {
    const std::lock_guard<std::mutex> lock(coordination_mutex_);
    return events_;
  }

 protected:
  Status prepare_impl() override {
    resource_alive_.store(true, std::memory_order_release);
    return Status::success();
  }

  Status subscribe_impl(WujiSub** output) override {
    *output = fake_subscription(8);
    return Status::success();
  }

  void unsubscribe_impl(WujiSub* subscription) noexcept override {
    static_cast<void>(subscription);
    std::unique_lock<std::mutex> lock(coordination_mutex_);
    ++close_calls_;
    close_entered_ = true;
    condition_.notify_all();
    if (!condition_.wait_for(lock, kCoordinationTimeout, [this]() { return callback_returned_; })) {
      close_wait_timed_out_ = true;
    }
    events_.push_back("close");
  }

  void cleanup_impl() noexcept override {
    const std::lock_guard<std::mutex> lock(coordination_mutex_);
    ++cleanup_calls_;
    cleanup_saw_callback_returned_ = callback_returned_;
    resource_alive_.store(false, std::memory_order_release);
    events_.push_back("cleanup");
  }

 private:
  mutable std::mutex coordination_mutex_;
  std::condition_variable condition_;
  std::atomic<bool> resource_alive_{false};
  bool callback_entered_{false};
  bool release_callback_{false};
  bool callback_returned_{false};
  bool callback_wait_timed_out_{false};
  bool close_entered_{false};
  bool close_wait_timed_out_{false};
  bool cleanup_saw_callback_returned_{false};
  int close_calls_{0};
  int cleanup_calls_{0};
  std::vector<std::string> events_;
  TestFrame frame_{17};
};

class CoordinatedThreadsGuard {
 public:
  CoordinatedThreadsGuard(CoordinatedCloseStream& stream, std::thread& callback_thread,
                          std::thread& stop_thread)
      : stream_(stream), callback_thread_(callback_thread), stop_thread_(stop_thread) {}

  ~CoordinatedThreadsGuard() noexcept {
    finish();
  }

  void finish() noexcept {
    if (finished_) {
      return;
    }
    const bool stop_started = stop_thread_.joinable();
    stream_.release_callback();
    if (callback_thread_.joinable()) {
      callback_thread_.join();
    }
    if (stop_started) {
      stop_thread_.join();
    } else {
      stream_.stop();
    }
    finished_ = true;
  }

 private:
  CoordinatedCloseStream& stream_;
  std::thread& callback_thread_;
  std::thread& stop_thread_;
  bool finished_{false};
};

TEST(StreamController, RunsPrepareBeforeSubscribeWithExactLifecycleStates) {
  TestStream stream;

  ASSERT_TRUE(stream.start());

  EXPECT_EQ(stream.prepare_calls(), 1);
  EXPECT_EQ(stream.subscribe_calls(), 1);
  EXPECT_EQ(stream.prepare_state(), StreamState::Preparing);
  EXPECT_EQ(stream.subscribe_state(), StreamState::Subscribing);
  EXPECT_EQ(stream.snapshot().state, StreamState::Active);
  stream.stop();
}

TEST(StreamController, PrepareFailureSkipsSubscribeAndCleanup) {
  TestStream stream;
  stream.set_prepare_status(Status::failure("prepare failed"));

  const Status result = stream.start();

  EXPECT_FALSE(result);
  EXPECT_EQ(result.message, "prepare failed");
  EXPECT_EQ(stream.subscribe_calls(), 0);
  EXPECT_EQ(stream.cleanup_calls(), 0);
  EXPECT_EQ(stream.snapshot().fault.kind, TerminalKind::Error);
  EXPECT_EQ(stream.snapshot().fault.reason, "prepare failed");
}

TEST(StreamController, CleansPreparedResourcesWhenSubscribeFails) {
  TestStream stream(Status::failure("subscribe failed"), nullptr);

  EXPECT_FALSE(stream.start());
  EXPECT_EQ(stream.cleanup_calls(), 1);
  EXPECT_EQ(stream.unsubscribe_calls(), 0);
  EXPECT_EQ(stream.snapshot().fault.kind, TerminalKind::Error);
}

TEST(StreamController, ClosesNonNullHandleOnFailedStatus) {
  WujiSub* const handle = fake_subscription();
  TestStream stream(Status::failure("subscribe failed"), handle);

  EXPECT_FALSE(stream.start());
  EXPECT_EQ(stream.unsubscribe_calls(), 1);
  EXPECT_EQ(stream.unsubscribed_handle(), handle);
  EXPECT_EQ(stream.cleanup_calls(), 1);
}

TEST(StreamController, PreservesSubscribeFailureDetailsInResultAndFault) {
  TestStream stream(
      Status::failure("wuji_subscribe failed with status -4: resource imu_raw/palm not found"),
      nullptr);

  const Status result = stream.start();
  const StreamSnapshot snapshot = stream.snapshot();

  EXPECT_FALSE(result);
  EXPECT_EQ(result.message,
            "wuji_subscribe failed with status -4: resource imu_raw/palm not found");
  EXPECT_EQ(snapshot.fault.kind, TerminalKind::Error);
  EXPECT_EQ(snapshot.fault.reason,
            "wuji_subscribe failed with status -4: resource imu_raw/palm not found");
}

TEST(StreamController, RejectsSuccessfulSubscribeWithoutAHandle) {
  TestStream stream(Status::success(), nullptr);

  const Status result = stream.start();

  EXPECT_FALSE(result);
  EXPECT_EQ(result.message, "subscribe returned a null handle");
  EXPECT_EQ(stream.cleanup_calls(), 1);
}

TEST(StreamController, PreservesSynchronousEndBeforeSubscribeReturns) {
  WujiSub* const handle = fake_subscription();
  TestStream stream(Status::success(), handle);
  stream.set_synchronous_callback(WUJI_FRAME_KIND_END, false);

  EXPECT_FALSE(stream.start());
  const StreamSnapshot snapshot = stream.snapshot();

  EXPECT_EQ(snapshot.state, StreamState::Terminal);
  EXPECT_EQ(snapshot.fault.kind, TerminalKind::End);
  EXPECT_EQ(stream.unsubscribe_calls(), 1);
  EXPECT_EQ(stream.unsubscribed_handle(), handle);
  EXPECT_EQ(stream.cleanup_calls(), 1);
}

TEST(StreamController, TerminalStateIsStickyAndBlocksLaterOkFrames) {
  TestStream stream;
  TestFrame first{1};
  TestFrame later{2};
  ASSERT_TRUE(stream.start());
  stream.emit(WUJI_FRAME_KIND_OK, &first);
  stream.emit(WUJI_FRAME_KIND_END, nullptr);
  stream.emit(WUJI_FRAME_KIND_ERROR, nullptr);
  stream.emit(WUJI_FRAME_KIND_OK, &later);
  const StreamSnapshot snapshot = stream.snapshot();

  EXPECT_EQ(snapshot.state, StreamState::Terminal);
  EXPECT_EQ(snapshot.fault.kind, TerminalKind::End);
  EXPECT_EQ(snapshot.fault.reason, "stream ended");
  EXPECT_EQ(stream.on_ok_calls(), 1);
  EXPECT_EQ(stream.converted_value(), 1);
  stream.stop();
}

TEST(StreamController, LatchesErrorWhenOkCallbackThrows) {
  TestStream stream;
  TestFrame frame{1};
  ASSERT_TRUE(stream.start());

  stream.emit_throwing_ok(&frame);

  const StreamSnapshot snapshot = stream.snapshot();
  EXPECT_EQ(snapshot.state, StreamState::Terminal);
  EXPECT_EQ(snapshot.fault.kind, TerminalKind::Error);
  EXPECT_EQ(snapshot.fault.reason, "stream callback threw an exception");
  stream.stop();
}

TEST(StreamController, StopMarksStoppingBeforeCloseAndIsIdempotent) {
  TestStream stream;
  stream.set_emit_during_unsubscribe(true);
  ASSERT_TRUE(stream.start());

  stream.stop();
  stream.stop();
  const StreamSnapshot snapshot = stream.snapshot();

  EXPECT_EQ(snapshot.state, StreamState::Stopped);
  EXPECT_EQ(snapshot.fault.kind, TerminalKind::None);
  EXPECT_EQ(stream.on_ok_calls(), 0);
  EXPECT_EQ(stream.unsubscribe_calls(), 1);
  EXPECT_EQ(stream.cleanup_calls(), 1);
}

TEST(StreamController, StopWaitsForActiveCallbackBeforeCleanup) {
  CoordinatedCloseStream stream;
  ASSERT_TRUE(stream.start());

  std::thread callback_thread([&stream]() { stream.run_blocking_callback(); });
  std::thread stop_thread;
  CoordinatedThreadsGuard threads_guard(stream, callback_thread, stop_thread);

  EXPECT_TRUE(stream.wait_for_callback_entry());
  stop_thread = std::thread([&stream]() { stream.stop(); });
  EXPECT_TRUE(stream.wait_for_close_entry());

  for (int index = 0; index < 64; ++index) {
    const StreamSnapshot snapshot = stream.snapshot();
    EXPECT_EQ(snapshot.state, StreamState::Stopping);
    EXPECT_EQ(snapshot.fault.kind, TerminalKind::None);
  }
  EXPECT_TRUE(stream.resource_alive());
  EXPECT_EQ(stream.cleanup_calls(), 0);

  threads_guard.finish();

  EXPECT_EQ(stream.close_calls(), 1);
  EXPECT_EQ(stream.cleanup_calls(), 1);
  EXPECT_TRUE(stream.cleanup_saw_callback_returned());
  EXPECT_FALSE(stream.callback_wait_timed_out());
  EXPECT_FALSE(stream.close_wait_timed_out());
  EXPECT_FALSE(stream.resource_alive());
  EXPECT_EQ(stream.events(), (std::vector<std::string>{"close", "cleanup"}));

  const StreamSnapshot stopped = stream.snapshot();
  EXPECT_EQ(stopped.state, StreamState::Stopped);
  EXPECT_EQ(stopped.fault.kind, TerminalKind::None);
}

}  // namespace
}  // namespace wuji_driver_common::stream
