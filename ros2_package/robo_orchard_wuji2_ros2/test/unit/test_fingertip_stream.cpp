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
#include <wuji_sdk.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <wuji_msgs/msg/fingertip_tactile.hpp>

#include "fake_sdk/fake_hand2_sdk.hpp"
#include "wuji_driver_common/stream/stream_types.hpp"
#include "wuji_hand2_driver/stream/fingertip_stream.hpp"

namespace wuji_hand2_driver::stream {
namespace {

using namespace std::chrono_literals;
using wuji_driver_common::stream::StreamKey;
using wuji_driver_common::stream::StreamState;
using wuji_driver_common::stream::TerminalKind;

constexpr std::array<std::uint8_t, 248> kThumbPayload{
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x64, 0x00, 0x38, 0xff, 0x2c, 0x01, 0xfa, 0x00}};

constexpr std::array<std::uint8_t, 212> kStandardPayload{
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x38, 0xff,
     0x2c, 0x01, 0xfa, 0x00}};

struct FingerCase {
  Fingertip finger;
  std::uint8_t index;
  test::FakeHand2Sdk::TypedStream fake_stream;
  const char* name;
  const char* subscribe_call;
  std::size_t point_count;
};

constexpr std::array<FingerCase, 5> kFingers{{
    {Fingertip::Thumb, 0, test::FakeHand2Sdk::TypedStream::FingertipThumb, "thumb",
     "subscribe_fingertip_thumb", 40},
    {Fingertip::Index, 1, test::FakeHand2Sdk::TypedStream::FingertipIndex, "index",
     "subscribe_fingertip_index", 34},
    {Fingertip::Middle, 2, test::FakeHand2Sdk::TypedStream::FingertipMiddle, "middle",
     "subscribe_fingertip_middle", 34},
    {Fingertip::Ring, 3, test::FakeHand2Sdk::TypedStream::FingertipRing, "ring",
     "subscribe_fingertip_ring", 34},
    {Fingertip::Pinky, 4, test::FakeHand2Sdk::TypedStream::FingertipPinky, "pinky",
     "subscribe_fingertip_pinky", 34},
}};

WujiDevice* fake_device() {
  return reinterpret_cast<WujiDevice*>(std::uintptr_t{1});
}

void set_header(WujiFrameHeader& header, std::uint64_t timestamp_us, const std::string& frame_id) {
  header = {};
  header.timestamp_us = timestamp_us;
  std::strncpy(header.frame_id, frame_id.c_str(), sizeof(header.frame_id) - 1);
}

WujiFingertipSensorData frame_for(const FingerCase& finger, std::uint32_t digest,
                                  std::uint8_t* data = nullptr, std::size_t data_len = 0) {
  WujiFingertipSensorData frame{};
  set_header(frame.header, 1234567, std::string(finger.name) + "_tip_sensor_frame");
  frame.info_digest = digest;
  if (data != nullptr) {
    frame.data = data;
    frame.data_len = data_len;
  } else if (finger.index == 0) {
    frame.data = const_cast<std::uint8_t*>(kThumbPayload.data());
    frame.data_len = kThumbPayload.size();
  } else {
    frame.data = const_cast<std::uint8_t*>(kStandardPayload.data());
    frame.data_len = kStandardPayload.size();
  }
  return frame;
}

template <typename StreamT>
struct StopDeleter {
  void operator()(StreamT* stream) const noexcept {
    if (stream != nullptr) {
      stream->stop();
      delete stream;
    }
  }
};

using StreamPtr = std::unique_ptr<FingertipStream, StopDeleter<FingertipStream>>;

class FingertipStreamTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    const auto id = next_namespace_.fetch_add(1, std::memory_order_relaxed);
    namespace_ = "/fingertip_stream_test_" + std::to_string(id);
    source_ = std::make_shared<rclcpp::Node>("source", namespace_);
    observer_ = std::make_shared<rclcpp::Node>("observer", namespace_);
    executor_.add_node(source_);
    executor_.add_node(observer_);
  }

  void TearDown() override {
    executor_.remove_node(observer_);
    executor_.remove_node(source_);
  }

  StreamPtr make_stream(const FingerCase& finger) {
    return StreamPtr(new FingertipStream(StreamKey{static_cast<std::uint16_t>(20 + finger.index)},
                                         fake_device(), fake_.api(), *source_, finger.finger,
                                         fake_.api().subscribe_fingertip[finger.index]));
  }

  rclcpp::Subscription<wuji_msgs::msg::FingertipTactile>::SharedPtr capture(
      const FingerCase& finger) {
    return observer_->create_subscription<wuji_msgs::msg::FingertipTactile>(
        topic(finger), rclcpp::SensorDataQoS().keep_last(5),
        [this, index = finger.index](wuji_msgs::msg::FingertipTactile::ConstSharedPtr message) {
          const std::lock_guard<std::mutex> lock(messages_mutex_);
          messages_[index].push_back(*message);
        });
  }

  bool spin_until(const std::function<bool()>& predicate, std::chrono::milliseconds timeout = 2s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    executor_.spin_some();
    return predicate();
  }

  bool wait_for_match(
      const FingerCase& finger,
      const rclcpp::Subscription<wuji_msgs::msg::FingertipTactile>::SharedPtr& subscription) {
    return spin_until([this, &finger, &subscription]() {
      return subscription->get_publisher_count() == 1U &&
             source_->count_subscribers(topic(finger)) == 1U;
    });
  }

  std::size_t message_count(std::uint8_t finger) const {
    const std::lock_guard<std::mutex> lock(messages_mutex_);
    return messages_[finger].size();
  }

  wuji_msgs::msg::FingertipTactile last_message(std::uint8_t finger) const {
    const std::lock_guard<std::mutex> lock(messages_mutex_);
    return messages_[finger].back();
  }

  std::string topic(const FingerCase& finger) const {
    return namespace_ + "/fingertip_tactile/" + finger.name;
  }

  void expect_sensor_publisher(const FingerCase& finger) {
    const auto publishers = source_->get_publishers_info_by_topic(topic(finger));
    ASSERT_EQ(publishers.size(), 1U);
    EXPECT_EQ(publishers.front().topic_type(), "wuji_msgs/msg/FingertipTactile");
    const auto& qos = publishers.front().qos_profile();
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  }

  test::FakeHand2Sdk fake_;
  std::shared_ptr<rclcpp::Node> source_;
  std::shared_ptr<rclcpp::Node> observer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::string namespace_;
  mutable std::mutex messages_mutex_;
  std::array<std::vector<wuji_msgs::msg::FingertipTactile>, 5> messages_;
  static std::atomic<std::uint64_t> next_namespace_;
};

std::atomic<std::uint64_t> FingertipStreamTest::next_namespace_{1};

TEST_F(FingertipStreamTest, StartsAndPublishesAllFiveParameterizedFingerStreams) {
  for (const auto& finger : kFingers) {
    auto subscription = capture(finger);
    auto stream = make_stream(finger);

    const auto start_status = stream->start();

    ASSERT_TRUE(start_status) << finger.name << ": " << start_status.message;
    ASSERT_FALSE(fake_.fingertip_info_requests().empty());
    EXPECT_EQ(fake_.fingertip_info_requests().back(), finger.index);
    EXPECT_EQ(fake_.count(finger.subscribe_call), 1U);
    expect_sensor_publisher(finger);
    ASSERT_TRUE(wait_for_match(finger, subscription));

    auto frame = frame_for(finger, 0xA1000000U + finger.index);
    fake_.emit_fingertip(finger.index, WUJI_FRAME_KIND_OK, &frame);
    ASSERT_TRUE(spin_until([this, &finger]() { return message_count(finger.index) == 1U; }));

    const auto message = last_message(finger.index);
    ASSERT_EQ(message.point_forces.size(), finger.point_count);
    EXPECT_EQ(message.info_digest, 0xA1000000U + finger.index);

    stream->stop();
    EXPECT_TRUE(spin_until([this, &finger]() {
      return source_->get_publishers_info_by_topic(topic(finger)).empty();
    }));
    subscription.reset();
  }
}

TEST_F(FingertipStreamTest, MetadataFailureCopiesLastErrorAndCreatesNoPublisher) {
  const auto& finger = kFingers[2];
  fake_.set_fingertip_info_failure(finger.index, WUJI_STATUS_ERR_TIMEOUT,
                                   "metadata read timed out");
  auto stream = make_stream(finger);

  const auto status = stream->start();

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("metadata read timed out"), std::string::npos);
  EXPECT_EQ(fake_.count(finger.subscribe_call), 0U);
  const auto& calls = fake_.calls();
  const auto get = std::find(calls.begin(), calls.end(), "get_fingertip_info");
  ASSERT_NE(get, calls.end());
  ASSERT_NE(std::next(get), calls.end());
  EXPECT_EQ(*std::next(get), "last_error");
  EXPECT_TRUE(source_->get_publishers_info_by_topic(topic(finger)).empty());
}

TEST_F(FingertipStreamTest, DigestMismatchIsStickyTerminalAndWaitsForExecutorStop) {
  const auto& finger = kFingers[1];
  auto subscription = capture(finger);
  auto stream = make_stream(finger);
  ASSERT_TRUE(stream->start());
  ASSERT_TRUE(wait_for_match(finger, subscription));

  auto valid = frame_for(finger, 0xA1000001U);
  fake_.emit_fingertip(finger.index, WUJI_FRAME_KIND_OK, &valid);
  ASSERT_TRUE(spin_until([this, &finger]() { return message_count(finger.index) == 1U; }));

  auto mismatch = valid;
  mismatch.info_digest = 0xDEADBEEFU;
  fake_.emit_fingertip(finger.index, WUJI_FRAME_KIND_OK, &mismatch);
  EXPECT_EQ(stream->snapshot().fault.kind, TerminalKind::Error);
  EXPECT_NE(stream->snapshot().fault.reason.find("digest"), std::string::npos);
  EXPECT_EQ(fake_.count("sub_close"), 0U);

  fake_.emit_fingertip(finger.index, WUJI_FRAME_KIND_OK, &valid);
  EXPECT_FALSE(spin_until([this, &finger]() { return message_count(finger.index) > 1U; }, 100ms));
  EXPECT_EQ(stream->snapshot().fault.kind, TerminalKind::Error);

  stream->stop();
  EXPECT_EQ(fake_.count("sub_close"), 1U);
  EXPECT_TRUE(spin_until(
      [this, &finger]() { return source_->get_publishers_info_by_topic(topic(finger)).empty(); }));
}

TEST_F(FingertipStreamTest, OrdinaryInvalidPayloadDoesNotTerminateAndNextFramePublishes) {
  const auto& finger = kFingers[3];
  auto subscription = capture(finger);
  auto stream = make_stream(finger);
  ASSERT_TRUE(stream->start());
  ASSERT_TRUE(wait_for_match(finger, subscription));

  auto short_frame = frame_for(finger, 0xA1000003U);
  --short_frame.data_len;
  fake_.emit_fingertip(finger.index, WUJI_FRAME_KIND_OK, &short_frame);
  EXPECT_EQ(stream->snapshot().fault.kind, TerminalKind::None);
  EXPECT_EQ(message_count(finger.index), 0U);

  auto valid = frame_for(finger, 0xA1000003U);
  fake_.emit_fingertip(finger.index, WUJI_FRAME_KIND_OK, &valid);
  ASSERT_TRUE(spin_until([this, &finger]() { return message_count(finger.index) == 1U; }));
  EXPECT_EQ(stream->snapshot().state, StreamState::Active);
  EXPECT_EQ(stream->snapshot().fault.kind, TerminalKind::None);
}

}  // namespace
}  // namespace wuji_hand2_driver::stream
