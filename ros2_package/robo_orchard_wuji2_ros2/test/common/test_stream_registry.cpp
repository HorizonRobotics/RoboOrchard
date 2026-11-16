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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <wuji_driver_common/stream/stream_registry.hpp>

namespace wuji_driver_common::stream {
namespace {

WujiSub* fake_subscription(StreamKey key) {
  return reinterpret_cast<WujiSub*>(static_cast<std::uintptr_t>(key.value) + 1U);
}

class RegistryTestStream final : public StreamController {
 public:
  RegistryTestStream(StreamKey key, StreamPolicy policy, WujiStatus start_status,
                     std::vector<std::uint16_t>* stop_order,
                     std::vector<bool>* destroyed_cleanly = nullptr)
      : StreamController(key, nullptr, policy),
        key_(key),
        start_status_(start_status),
        stop_order_(stop_order),
        destroyed_cleanly_(destroyed_cleanly) {}

  ~RegistryTestStream() noexcept override {
    if (destroyed_cleanly_ != nullptr) {
      destroyed_cleanly_->push_back(!started_ || unsubscribe_calls_ == 1);
    }
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

 protected:
  Status prepare_impl() override {
    ++prepare_calls_;
    return Status::success();
  }

  Status subscribe_impl(WujiSub** output) override {
    ++subscribe_calls_;
    if (start_status_ != WUJI_STATUS_OK) {
      return Status::failure("subscribe failed");
    }
    *output = fake_subscription(key_);
    started_ = true;
    return Status::success();
  }

  void unsubscribe_impl(WujiSub* subscription) noexcept override {
    static_cast<void>(subscription);
    ++unsubscribe_calls_;
    started_ = false;
    if (stop_order_ != nullptr) {
      stop_order_->push_back(key_.value);
    }
  }

  void cleanup_impl() noexcept override {
    ++cleanup_calls_;
  }

 private:
  StreamKey key_;
  WujiStatus start_status_;
  std::vector<std::uint16_t>* stop_order_;
  std::vector<bool>* destroyed_cleanly_;
  bool started_{false};
  int prepare_calls_{0};
  int subscribe_calls_{0};
  int unsubscribe_calls_{0};
  int cleanup_calls_{0};
};

std::unique_ptr<RegistryTestStream> make_stream(std::uint16_t key, StreamPolicy policy,
                                                WujiStatus status,
                                                std::vector<std::uint16_t>* stop_order = nullptr,
                                                std::vector<bool>* destroyed_cleanly = nullptr) {
  return std::make_unique<RegistryTestStream>(StreamKey{key}, policy, status, stop_order,
                                              destroyed_cleanly);
}

TEST(StreamRegistry, RejectsDuplicateKey) {
  StreamRegistry registry;

  ASSERT_TRUE(registry.add(make_stream(1, StreamPolicy::Required, WUJI_STATUS_OK)));
  EXPECT_FALSE(registry.add(make_stream(1, StreamPolicy::Required, WUJI_STATUS_OK)));
}

TEST(StreamRegistry, FreezesStructureWhenStarting) {
  StreamRegistry registry;
  ASSERT_TRUE(registry.add(make_stream(1, StreamPolicy::Required, WUJI_STATUS_OK)));
  ASSERT_TRUE(registry.start_required());
  ASSERT_TRUE(registry.start_optional());

  const Status result = registry.add(make_stream(2, StreamPolicy::Required, WUJI_STATUS_OK));

  EXPECT_FALSE(result);
  EXPECT_EQ(result.message, "stream registry is not building");
  registry.stop_all();
}

TEST(StreamRegistry, StartsRequiredBeforeOptionalAcrossExplicitPhases) {
  StreamRegistry registry;
  auto required = make_stream(1, StreamPolicy::Required, WUJI_STATUS_OK);
  auto optional = make_stream(2, StreamPolicy::Optional, WUJI_STATUS_OK);
  RegistryTestStream* const required_ptr = required.get();
  RegistryTestStream* const optional_ptr = optional.get();
  ASSERT_TRUE(registry.add(std::move(required)));
  ASSERT_TRUE(registry.add(std::move(optional)));

  ASSERT_TRUE(registry.start_required());
  EXPECT_EQ(required_ptr->subscribe_calls(), 1);
  EXPECT_EQ(optional_ptr->subscribe_calls(), 0);
  EXPECT_FALSE(registry.add(make_stream(3, StreamPolicy::Optional, WUJI_STATUS_OK)));

  ASSERT_TRUE(registry.start_optional());
  EXPECT_EQ(optional_ptr->subscribe_calls(), 1);
  registry.stop_all();
}

TEST(StreamRegistry, CancelledOptionalStartSkipsTheRemainingStreams) {
  StreamRegistry registry;
  auto first = make_stream(1, StreamPolicy::Optional, WUJI_STATUS_OK);
  auto second = make_stream(2, StreamPolicy::Optional, WUJI_STATUS_OK);
  RegistryTestStream* const first_ptr = first.get();
  RegistryTestStream* const second_ptr = second.get();
  ASSERT_TRUE(registry.add(std::move(first)));
  ASSERT_TRUE(registry.add(std::move(second)));
  ASSERT_TRUE(registry.start_required());

  ASSERT_TRUE(registry.start_optional([first_ptr]() { return first_ptr->subscribe_calls() > 0; }));

  EXPECT_EQ(first_ptr->subscribe_calls(), 1);
  EXPECT_EQ(second_ptr->prepare_calls(), 0);
  registry.stop_all();
}

TEST(StreamRegistry, RequiredFailureRollsBackEarlierStreamsInReverse) {
  std::vector<std::uint16_t> stop_order;
  StreamRegistry registry;
  auto first = make_stream(1, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order);
  auto second = make_stream(2, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order);
  auto failing = make_stream(3, StreamPolicy::Required, WUJI_STATUS_ERR_INTERNAL, &stop_order);
  RegistryTestStream* const first_ptr = first.get();
  RegistryTestStream* const second_ptr = second.get();
  RegistryTestStream* const failing_ptr = failing.get();
  ASSERT_TRUE(registry.add(std::move(first)));
  ASSERT_TRUE(registry.add(std::move(second)));
  ASSERT_TRUE(registry.add(std::move(failing)));

  const Status result = registry.start_required();

  EXPECT_FALSE(result);
  EXPECT_EQ(stop_order, (std::vector<std::uint16_t>{2, 1}));
  EXPECT_EQ(first_ptr->unsubscribe_calls(), 1);
  EXPECT_EQ(second_ptr->unsubscribe_calls(), 1);
  EXPECT_EQ(failing_ptr->unsubscribe_calls(), 0);
  EXPECT_EQ(failing_ptr->cleanup_calls(), 1);
  EXPECT_EQ(registry.snapshots()[2].fault.kind, TerminalKind::Error);
}

TEST(StreamRegistry, OptionalStartFailureRetainsErrorAndContinues) {
  std::vector<std::uint16_t> stop_order;
  StreamRegistry registry;
  auto first = make_stream(1, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order);
  auto optional = make_stream(2, StreamPolicy::Optional, WUJI_STATUS_ERR_INTERNAL, &stop_order);
  auto third = make_stream(3, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order);
  RegistryTestStream* const optional_ptr = optional.get();
  RegistryTestStream* const third_ptr = third.get();
  ASSERT_TRUE(registry.add(std::move(first)));
  ASSERT_TRUE(registry.add(std::move(optional)));
  ASSERT_TRUE(registry.add(std::move(third)));

  ASSERT_TRUE(registry.start_required());

  ASSERT_TRUE(registry.start_optional());
  const std::vector<StreamSnapshot> snapshots = registry.snapshots();

  ASSERT_EQ(snapshots.size(), 3U);
  EXPECT_EQ(snapshots[1].state, StreamState::Terminal);
  EXPECT_EQ(snapshots[1].fault.kind, TerminalKind::Error);
  EXPECT_EQ(optional_ptr->cleanup_calls(), 1);
  EXPECT_EQ(third_ptr->subscribe_calls(), 1);
  registry.stop_all();
  EXPECT_EQ(stop_order, (std::vector<std::uint16_t>{3, 1}));
}

TEST(StreamRegistry, StopAllUsesReverseOrderAndIsIdempotent) {
  std::vector<std::uint16_t> stop_order;
  StreamRegistry registry;
  ASSERT_TRUE(registry.add(make_stream(1, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order)));
  ASSERT_TRUE(registry.add(make_stream(2, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order)));
  ASSERT_TRUE(registry.add(make_stream(3, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order)));
  ASSERT_TRUE(registry.start_required());
  ASSERT_TRUE(registry.start_optional());

  registry.stop_all();
  registry.stop_all();

  EXPECT_EQ(stop_order, (std::vector<std::uint16_t>{3, 2, 1}));
}

TEST(StreamRegistry, StopByKeyIsIdempotentAndStopAllSkipsItsClosedHandle) {
  std::vector<std::uint16_t> stop_order;
  StreamRegistry registry;
  ASSERT_TRUE(registry.add(make_stream(1, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order)));
  ASSERT_TRUE(registry.add(make_stream(2, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order)));
  ASSERT_TRUE(registry.add(make_stream(3, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order)));
  ASSERT_TRUE(registry.start_required());
  ASSERT_TRUE(registry.start_optional());

  registry.stop(StreamKey{2});
  registry.stop(StreamKey{2});
  registry.stop_all();

  EXPECT_EQ(stop_order, (std::vector<std::uint16_t>{2, 3, 1}));
}

TEST(StreamRegistry, DestructorStopsAllBeforeDeletingControllers) {
  std::vector<std::uint16_t> stop_order;
  std::vector<bool> destroyed_cleanly;
  {
    StreamRegistry registry;
    ASSERT_TRUE(registry.add(
        make_stream(1, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order, &destroyed_cleanly)));
    ASSERT_TRUE(registry.add(
        make_stream(2, StreamPolicy::Required, WUJI_STATUS_OK, &stop_order, &destroyed_cleanly)));
    ASSERT_TRUE(registry.start_required());
    ASSERT_TRUE(registry.start_optional());
  }

  EXPECT_EQ(stop_order, (std::vector<std::uint16_t>{2, 1}));
  EXPECT_EQ(destroyed_cleanly, (std::vector<bool>{true, true}));
}

}  // namespace
}  // namespace wuji_driver_common::stream
