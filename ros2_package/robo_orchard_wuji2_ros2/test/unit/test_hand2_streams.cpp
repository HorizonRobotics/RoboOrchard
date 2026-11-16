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

#include <gtest/gtest.h>
#include <wuji_sdk.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wuji_msgs/msg/joint_diagnostics_array.hpp>

#include "fake_sdk/fake_hand2_sdk.hpp"
#include "wuji_driver_common/stream/stream_types.hpp"
#include "wuji_hand2_driver/conversion/joint_mapping.hpp"
#include "wuji_hand2_driver/stream/imu_stream.hpp"
#include "wuji_hand2_driver/stream/joint_diagnostics_stream.hpp"
#include "wuji_hand2_driver/stream/joint_state_stream.hpp"

namespace wuji_hand2_driver::stream {
namespace {

using namespace std::chrono_literals;
using wuji_driver_common::stream::StreamKey;
using wuji_driver_common::stream::StreamPolicy;
using wuji_driver_common::stream::StreamState;
using wuji_driver_common::stream::TerminalKind;

void set_header(WujiFrameHeader& header, std::uint64_t timestamp_us, const char* frame_id) {
  header = {};
  header.timestamp_us = timestamp_us;
  std::strncpy(header.frame_id, frame_id, sizeof(header.frame_id) - 1);
}

WujiDevice* fake_device() {
  return reinterpret_cast<WujiDevice*>(std::uintptr_t{1});
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

template <typename StreamT>
using StreamPtr = std::unique_ptr<StreamT, StopDeleter<StreamT>>;

class Hand2StreamsTest : public testing::Test {
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
    namespace_ = "/hand2_stream_test_" + std::to_string(id);
    source_ = std::make_shared<rclcpp::Node>("source", namespace_);
    observer_ = std::make_shared<rclcpp::Node>("observer", namespace_);
    executor_.add_node(source_);
    executor_.add_node(observer_);
  }

  void TearDown() override {
    executor_.remove_node(observer_);
    executor_.remove_node(source_);
  }

  StreamPtr<JointStateStream> make_joint_state_stream(StreamKey key = StreamKey{11}) {
    return StreamPtr<JointStateStream>(
        new JointStateStream(key, fake_device(), fake_.api(), *source_, mapping_));
  }

  StreamPtr<JointDiagnosticsStream> make_joint_diagnostics_stream(StreamKey key = StreamKey{12}) {
    return StreamPtr<JointDiagnosticsStream>(
        new JointDiagnosticsStream(key, fake_device(), fake_.api(), *source_, mapping_));
  }

  StreamPtr<ImuStream> make_imu_stream(StreamKey key = StreamKey{13}) {
    return StreamPtr<ImuStream>(new ImuStream(key, fake_device(), fake_.api(), *source_));
  }

  void capture_joint_states() {
    joint_state_subscription_ = observer_->create_subscription<sensor_msgs::msg::JointState>(
        topic("joint_states"), rclcpp::SensorDataQoS().keep_last(5),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          const std::lock_guard<std::mutex> lock(messages_mutex_);
          joint_state_messages_.push_back(*message);
        });
  }

  void capture_joint_diagnostics() {
    joint_diagnostics_subscription_ =
        observer_->create_subscription<wuji_msgs::msg::JointDiagnosticsArray>(
            topic("joint_diagnostics"), rclcpp::SensorDataQoS().keep_last(5),
            [this](wuji_msgs::msg::JointDiagnosticsArray::ConstSharedPtr message) {
              const std::lock_guard<std::mutex> lock(messages_mutex_);
              joint_diagnostics_messages_.push_back(*message);
            });
  }

  void capture_imu() {
    imu_subscription_ = observer_->create_subscription<sensor_msgs::msg::Imu>(
        topic("imu"), rclcpp::SensorDataQoS().keep_last(5),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          const std::lock_guard<std::mutex> lock(messages_mutex_);
          imu_messages_.push_back(*message);
        });
  }

  bool spin_until(const std::function<bool()>& predicate, std::chrono::milliseconds timeout = 1s) {
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

  void spin_for(std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(5ms);
    }
  }

  template <typename MessageT>
  bool wait_for_match(const typename rclcpp::Subscription<MessageT>::SharedPtr& subscription,
                      const char* relative_name) {
    return spin_until(
        [this, &subscription, relative_name]() {
          return subscription->get_publisher_count() == 1U &&
                 source_->count_subscribers(topic(relative_name)) == 1U;
        },
        2s);
  }

  std::size_t joint_state_message_count() const {
    const std::lock_guard<std::mutex> lock(messages_mutex_);
    return joint_state_messages_.size();
  }

  std::size_t joint_diagnostics_message_count() const {
    const std::lock_guard<std::mutex> lock(messages_mutex_);
    return joint_diagnostics_messages_.size();
  }

  std::size_t imu_message_count() const {
    const std::lock_guard<std::mutex> lock(messages_mutex_);
    return imu_messages_.size();
  }

  sensor_msgs::msg::JointState last_joint_state() const {
    const std::lock_guard<std::mutex> lock(messages_mutex_);
    return joint_state_messages_.back();
  }

  wuji_msgs::msg::JointDiagnosticsArray last_joint_diagnostics() const {
    const std::lock_guard<std::mutex> lock(messages_mutex_);
    return joint_diagnostics_messages_.back();
  }

  sensor_msgs::msg::Imu last_imu() const {
    const std::lock_guard<std::mutex> lock(messages_mutex_);
    return imu_messages_.back();
  }

  std::string topic(const char* relative_name) const {
    return namespace_ + "/" + relative_name;
  }

  void expect_sensor_publisher(const char* relative_name, const char* type_name) {
    const auto publishers = source_->get_publishers_info_by_topic(topic(relative_name));
    ASSERT_EQ(publishers.size(), 1U);
    EXPECT_EQ(publishers.front().topic_type(), type_name);
    const auto& qos = publishers.front().qos_profile();
    EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
    EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  }

  static WujiImuData valid_imu_frame() {
    WujiImuData frame{};
    set_header(frame.header, 9876543, "l_wrist_imu");
    frame.orientation = WujiQuaternionF64{0.1, 0.2, 0.3, 0.4};
    frame.angular_velocity = WujiVector3F64{1.1, 2.2, 3.3};
    frame.linear_acceleration = WujiVector3F64{4.4, 5.5, 6.6};
    for (std::size_t index = 0; index < 9; ++index) {
      frame.orientation_covariance[index] = static_cast<double>(index) + 0.25;
      frame.angular_velocity_covariance[index] = static_cast<double>(index) + 10.25;
      frame.linear_acceleration_covariance[index] = static_cast<double>(index) + 20.25;
    }
    return frame;
  }

  static std::atomic<std::uint64_t> next_namespace_;
  test::FakeHand2Sdk fake_;
  conversion::JointMapping mapping_{conversion::HandSide::kLeft};
  std::string namespace_;
  rclcpp::Node::SharedPtr source_;
  rclcpp::Node::SharedPtr observer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<wuji_msgs::msg::JointDiagnosticsArray>::SharedPtr
      joint_diagnostics_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  mutable std::mutex messages_mutex_;
  std::vector<sensor_msgs::msg::JointState> joint_state_messages_;
  std::vector<wuji_msgs::msg::JointDiagnosticsArray> joint_diagnostics_messages_;
  std::vector<sensor_msgs::msg::Imu> imu_messages_;
};

std::atomic<std::uint64_t> Hand2StreamsTest::next_namespace_{1};

TEST_F(Hand2StreamsTest, PublishesRepresentativeJointStateFromSdkCallback) {
  capture_joint_states();
  auto stream = make_joint_state_stream(StreamKey{101});
  ASSERT_TRUE(stream->start());
  ASSERT_TRUE(
      wait_for_match<sensor_msgs::msg::JointState>(joint_state_subscription_, "joint_states"));
  std::array<WujiJointStateEntry, 3> entries{{
      {24, 0.25F, 0.5F, 0.75F},
      {1, -1.0F, -2.0F, -3.0F},
      {7, 4.0F, 5.0F, 6.0F},
  }};
  WujiJointStateFrame frame{};
  set_header(frame.header, 1234567, "l_wrist");
  frame.num_joints = 3;
  frame.joints = entries.data();
  frame.joints_len = entries.size();

  fake_.emit_joint_state(WUJI_FRAME_KIND_OK, &frame);

  ASSERT_TRUE(spin_until([this]() { return joint_state_message_count() == 1; }));
  const auto message = last_joint_state();
  ASSERT_EQ(message.name.size(), 3U);
  EXPECT_EQ(message.header.frame_id, "l_wrist");
  EXPECT_EQ(message.name.front(), "l_pinky_dip");
  EXPECT_DOUBLE_EQ(message.position.front(), 0.25);
  const auto snapshot = stream->snapshot();
  EXPECT_EQ(snapshot.key, StreamKey{101});
  EXPECT_EQ(snapshot.policy, StreamPolicy::Required);
  EXPECT_EQ(snapshot.fault.kind, TerminalKind::None);
}

TEST_F(Hand2StreamsTest, PublishesRepresentativeJointDiagnosticsFromSdkCallback) {
  capture_joint_diagnostics();
  auto stream = make_joint_diagnostics_stream();
  ASSERT_TRUE(stream->start());
  ASSERT_TRUE(wait_for_match<wuji_msgs::msg::JointDiagnosticsArray>(joint_diagnostics_subscription_,
                                                                    "joint_diagnostics"));
  WujiJointDiagnosticsEntry entry{6, 0x10203040U, 1.25F, 24.5F, 42.75F, 0x3456U, 97U, 1234U};
  WujiJointDiagnosticsFrame frame{};
  set_header(frame.header, 2345678, "l_wrist_diag");
  frame.num_joints = 1;
  frame.joints = &entry;
  frame.joints_len = 1;

  fake_.emit_joint_diagnostics(WUJI_FRAME_KIND_OK, &frame);

  ASSERT_TRUE(spin_until([this]() { return joint_diagnostics_message_count() == 1; }));
  const auto message = last_joint_diagnostics();
  ASSERT_EQ(message.joints.size(), 1U);
  EXPECT_EQ(message.header.frame_id, "l_wrist_diag");
  EXPECT_EQ(message.joints.front().nid, 6U);
  EXPECT_EQ(message.joints.front().name, "l_index_finger_mcp_flex");
  EXPECT_EQ(message.joints.front().error_code, 0x3456U);
}

TEST_F(Hand2StreamsTest, PublishesRepresentativeImuFromSdkCallback) {
  capture_imu();
  auto stream = make_imu_stream();
  ASSERT_TRUE(stream->start());
  ASSERT_TRUE(wait_for_match<sensor_msgs::msg::Imu>(imu_subscription_, "imu"));
  const auto frame = valid_imu_frame();

  fake_.emit_imu(WUJI_FRAME_KIND_OK, &frame);

  ASSERT_TRUE(spin_until([this]() { return imu_message_count() == 1; }));
  const auto message = last_imu();
  EXPECT_EQ(message.header.frame_id, "l_wrist_imu");
  EXPECT_DOUBLE_EQ(message.orientation_covariance[0], -1.0);
  EXPECT_DOUBLE_EQ(message.angular_velocity.x, 1.1);
  EXPECT_DOUBLE_EQ(message.linear_acceleration.z, 6.6);
  EXPECT_EQ(stream->snapshot().policy, StreamPolicy::Optional);
}

TEST_F(Hand2StreamsTest, CreatesExactRelativeTopicsAndFixedSensorQos) {
  auto joint_state = make_joint_state_stream();
  auto diagnostics = make_joint_diagnostics_stream();
  auto imu = make_imu_stream();
  ASSERT_TRUE(joint_state->start());
  ASSERT_TRUE(diagnostics->start());
  ASSERT_TRUE(imu->start());

  expect_sensor_publisher("joint_states", "sensor_msgs/msg/JointState");
  expect_sensor_publisher("joint_diagnostics", "wuji_msgs/msg/JointDiagnosticsArray");
  expect_sensor_publisher("imu", "sensor_msgs/msg/Imu");
  EXPECT_EQ(source_->count_publishers("/joint_states"), 0U);
  EXPECT_EQ(source_->count_publishers("/joint_diagnostics"), 0U);
  EXPECT_EQ(source_->count_publishers("/imu"), 0U);
}

TEST_F(Hand2StreamsTest, DropsRepresentativeMalformedFramesWithoutFault) {
  capture_joint_states();
  capture_joint_diagnostics();
  capture_imu();
  auto joint_state = make_joint_state_stream();
  auto diagnostics = make_joint_diagnostics_stream();
  auto imu = make_imu_stream();
  ASSERT_TRUE(joint_state->start());
  ASSERT_TRUE(diagnostics->start());
  ASSERT_TRUE(imu->start());
  ASSERT_TRUE(
      wait_for_match<sensor_msgs::msg::JointState>(joint_state_subscription_, "joint_states"));
  ASSERT_TRUE(wait_for_match<wuji_msgs::msg::JointDiagnosticsArray>(joint_diagnostics_subscription_,
                                                                    "joint_diagnostics"));
  ASSERT_TRUE(wait_for_match<sensor_msgs::msg::Imu>(imu_subscription_, "imu"));

  WujiJointStateEntry state_entry{1, 1.0F, 2.0F, 3.0F};
  WujiJointStateFrame state_frame{};
  set_header(state_frame.header, 1, "bad_state");
  state_frame.num_joints = 2;
  state_frame.joints = &state_entry;
  state_frame.joints_len = 1;
  fake_.emit_joint_state(WUJI_FRAME_KIND_OK, &state_frame);

  WujiJointDiagnosticsEntry diagnostics_entry{
      1, 0U, 1.0F, 2.0F, std::numeric_limits<float>::infinity(), 0U, 100U, 0U};
  WujiJointDiagnosticsFrame diagnostics_frame{};
  set_header(diagnostics_frame.header, 2, "bad_diagnostics");
  diagnostics_frame.num_joints = 1;
  diagnostics_frame.joints = &diagnostics_entry;
  diagnostics_frame.joints_len = 1;
  fake_.emit_joint_diagnostics(WUJI_FRAME_KIND_OK, &diagnostics_frame);

  auto imu_frame = valid_imu_frame();
  imu_frame.angular_velocity.z = std::numeric_limits<double>::quiet_NaN();
  fake_.emit_imu(WUJI_FRAME_KIND_OK, &imu_frame);
  spin_for(100ms);

  EXPECT_EQ(joint_state_message_count(), 0U);
  EXPECT_EQ(joint_diagnostics_message_count(), 0U);
  EXPECT_EQ(imu_message_count(), 0U);
  EXPECT_EQ(joint_state->snapshot().fault.kind, TerminalKind::None);
  EXPECT_EQ(diagnostics->snapshot().fault.kind, TerminalKind::None);
  EXPECT_EQ(imu->snapshot().fault.kind, TerminalKind::None);
}

TEST_F(Hand2StreamsTest, IgnoresLagAndLatchesTerminalWithoutClosingInCallback) {
  auto joint_state = make_joint_state_stream();
  auto imu = make_imu_stream();
  ASSERT_TRUE(joint_state->start());
  ASSERT_TRUE(imu->start());

  fake_.emit_joint_state(WUJI_FRAME_KIND_LAG, nullptr);
  fake_.emit_joint_state(WUJI_FRAME_KIND_END, nullptr);
  fake_.emit_joint_state(WUJI_FRAME_KIND_ERROR, nullptr);
  fake_.emit_imu(WUJI_FRAME_KIND_ERROR, nullptr);

  const auto required_snapshot = joint_state->snapshot();
  EXPECT_EQ(required_snapshot.fault.kind, TerminalKind::End);
  EXPECT_EQ(required_snapshot.state, StreamState::Terminal);
  EXPECT_EQ(required_snapshot.policy, StreamPolicy::Required);
  const auto optional_snapshot = imu->snapshot();
  EXPECT_EQ(optional_snapshot.fault.kind, TerminalKind::Error);
  EXPECT_EQ(optional_snapshot.policy, StreamPolicy::Optional);
  EXPECT_EQ(fake_.count("sub_close"), 0U);
  EXPECT_EQ(source_->count_publishers(topic("joint_states")), 1U);
  EXPECT_EQ(source_->count_publishers(topic("imu")), 1U);
}

}  // namespace
}  // namespace wuji_hand2_driver::stream
