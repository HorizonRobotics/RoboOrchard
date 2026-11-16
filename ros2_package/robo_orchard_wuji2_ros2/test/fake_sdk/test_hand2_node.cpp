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
#include <rcl/timer.h>

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <future>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "fake_sdk/fake_hand2_sdk.hpp"
#include "wuji_hand2_driver/hand2_node.hpp"
#include "wuji_hand2_driver/lifecycle/exit_state.hpp"

namespace wuji_hand2_driver {
namespace {

using namespace std::chrono_literals;

using test::FakeHand2Sdk;

class ManualClock {
 public:
  using Clock = std::chrono::steady_clock;

  Clock::time_point now() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return now_;
  }

  void advance(Clock::duration duration) {
    const std::lock_guard<std::mutex> lock(mutex_);
    now_ += duration;
  }

  void set(Clock::time_point value) {
    const std::lock_guard<std::mutex> lock(mutex_);
    now_ = value;
  }

 private:
  mutable std::mutex mutex_;
  Clock::time_point now_{Clock::now()};
};

class BlockingCallCounter {
 public:
  void enter() {
    std::unique_lock<std::mutex> lock(mutex_);
    ++entered_;
    changed_.notify_all();
    changed_.wait(lock, [this]() { return released_; });
  }

  bool wait_for_entries(std::size_t count, std::chrono::milliseconds timeout = 2s) {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, timeout, [this, count]() { return entered_ >= count; });
  }

  void release() {
    const std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    changed_.notify_all();
  }

 private:
  std::mutex mutex_;
  std::condition_variable changed_;
  std::size_t entered_{0};
  bool released_{false};
};

class BlockingCallReleaseGuard {
 public:
  explicit BlockingCallReleaseGuard(std::shared_ptr<BlockingCallCounter> counter)
      : counter_(std::move(counter)) {}

  ~BlockingCallReleaseGuard() noexcept {
    counter_->release();
  }

 private:
  std::shared_ptr<BlockingCallCounter> counter_;
};

class ExecutorSpinGuard {
 public:
  explicit ExecutorSpinGuard(rclcpp::executors::MultiThreadedExecutor& executor)
      : executor_(executor), thread_([this]() { executor_.spin(); }) {}

  ~ExecutorSpinGuard() noexcept {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

 private:
  rclcpp::executors::MultiThreadedExecutor& executor_;
  std::thread thread_;
};

class Hand2NodeTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  std::shared_ptr<Hand2Node> make_node(std::vector<rclcpp::Parameter> overrides = {},
                                       bool add_default_serial = true) {
    if (add_default_serial) {
      const auto has_target =
          std::find_if(overrides.begin(), overrides.end(), [](const rclcpp::Parameter& parameter) {
            return parameter.get_name() == "serial_number" || parameter.get_name() == "endpoint";
          });
      if (has_target == overrides.end()) {
        overrides.emplace_back("serial_number", "WH2J000000000001");
      }
    }

    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    Hand2NodeHooks hooks;
    hooks.now = [this]() { return clock_.now(); };
    hooks.sleep_for = [this](std::chrono::steady_clock::duration duration) {
      clock_.advance(duration);
    };
    hooks.cancelled = [this]() { return cancelled_.load(std::memory_order_acquire); };
    hooks.request_shutdown = [this]() {
      shutdown_requested_.store(true, std::memory_order_release);
    };
    return std::make_shared<Hand2Node>(fake_.api(), std::move(hooks), options);
  }

  bool spin_until(const std::shared_ptr<Hand2Node>& node, const std::function<bool()>& predicate,
                  std::chrono::milliseconds timeout = 500ms) {
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2U);
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(2ms);
    }
    executor.remove_node(node);
    return predicate();
  }

  static WujiJointStateFrame valid_joint_state() {
    static WujiJointStateEntry entry{1U, 0.1F, 0.2F, 0.3F};
    WujiJointStateFrame frame{};
    frame.header.timestamp_us = 1234567U;
    std::snprintf(frame.header.frame_id, sizeof(frame.header.frame_id), "hand");
    frame.num_joints = 1U;
    frame.joints = &entry;
    frame.joints_len = 1U;
    return frame;
  }

  static WujiJointStateFrame complete_joint_state(std::array<WujiJointStateEntry, 20>& entries,
                                                  float position) {
    for (std::size_t offset = 0; offset < entries.size(); ++offset) {
      const auto index = 19 - offset;
      entries[offset].nid = static_cast<std::uint8_t>(index / 4 * 5 + index % 4 + 1);
      entries[offset].position = position;
    }
    WujiJointStateFrame frame{};
    frame.header.timestamp_us = 1'234'567;
    frame.num_joints = 20;
    frame.joints = entries.data();
    frame.joints_len = entries.size();
    return frame;
  }
  FakeHand2Sdk fake_;
  ManualClock clock_;
  std::atomic<bool> cancelled_{false};
  std::atomic<bool> shutdown_requested_{false};
};

TEST_F(Hand2NodeTest, DeclaresExactParameterDefaultsWithoutSdkResources) {
  auto node = make_node({}, false);

  EXPECT_EQ(node->get_parameter("endpoint").as_string(), "");
  EXPECT_EQ(node->get_parameter("serial_number").as_string(), "");
  EXPECT_EQ(node->get_parameter("hand_side").as_string(), "");
  EXPECT_DOUBLE_EQ(node->get_parameter("connect_timeout").as_double(), 10.0);
  EXPECT_FALSE(node->get_parameter("auto_enable").as_bool());
  EXPECT_TRUE(node->get_parameter("publish_joint_diagnostics").as_bool());
  EXPECT_TRUE(node->get_parameter("publish_imu").as_bool());
  EXPECT_FALSE(node->get_parameter("publish_fingertip_tactile").as_bool());
  EXPECT_EQ(node->get_parameter("reset_joint_position").as_double_array(),
            std::vector<double>(20, 0.0));
  EXPECT_DOUBLE_EQ(node->get_parameter("reset_duration_s").as_double(), 2.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("reset_timeout_s").as_double(), 5.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("reset_feedback_timeout_s").as_double(), 0.2);
  EXPECT_DOUBLE_EQ(node->get_parameter("reset_position_tolerance").as_double(), 0.2);
  EXPECT_TRUE(fake_.calls_snapshot().empty());
}

TEST_F(Hand2NodeTest, RejectsInvalidSideAndTimeoutBeforeSdkUse) {
  auto invalid_side = make_node({rclcpp::Parameter("hand_side", "up")});
  EXPECT_FALSE(invalid_side->start());
  EXPECT_NE(invalid_side->exit_reason().find("hand_side"), std::string::npos);
  invalid_side->stop();

  auto negative_timeout = make_node({rclcpp::Parameter("connect_timeout", -0.1)});
  EXPECT_FALSE(negative_timeout->start());
  EXPECT_NE(negative_timeout->exit_reason().find("connect_timeout"), std::string::npos);
  negative_timeout->stop();

  auto nonfinite_timeout =
      make_node({rclcpp::Parameter("connect_timeout", std::numeric_limits<double>::quiet_NaN())});
  EXPECT_FALSE(nonfinite_timeout->start());
  EXPECT_NE(nonfinite_timeout->exit_reason().find("connect_timeout"), std::string::npos);
  nonfinite_timeout->stop();
  EXPECT_TRUE(fake_.calls_snapshot().empty());
}

TEST_F(Hand2NodeTest, RejectsInvalidResetConfigurationBeforeSdkUse) {
  const std::vector<rclcpp::Parameter> invalid{
      rclcpp::Parameter("reset_joint_position", std::vector<double>(19, 0.0)),
      rclcpp::Parameter("reset_joint_position",
                         std::vector<double>(20, std::numeric_limits<double>::quiet_NaN())),
      rclcpp::Parameter("reset_joint_position",
                         std::vector<double>(20, std::numeric_limits<double>::max())),
      rclcpp::Parameter("reset_duration_s", 0.0),
      rclcpp::Parameter("reset_duration_s", -1.0),
      rclcpp::Parameter("reset_duration_s", std::numeric_limits<double>::infinity()),
      rclcpp::Parameter("reset_timeout_s", 2.0),
      rclcpp::Parameter("reset_timeout_s", std::numeric_limits<double>::quiet_NaN()),
      rclcpp::Parameter("reset_feedback_timeout_s", 0.0),
      rclcpp::Parameter("reset_feedback_timeout_s", std::numeric_limits<double>::infinity()),
      rclcpp::Parameter("reset_position_tolerance", -0.01),
      rclcpp::Parameter("reset_position_tolerance", std::numeric_limits<double>::quiet_NaN()),
  };
  for (const auto& parameter : invalid) {
    SCOPED_TRACE(parameter.get_name());
    auto node = make_node({parameter});
    EXPECT_FALSE(node->start());
    EXPECT_NE(node->exit_reason().find("reset"), std::string::npos);
    node->stop();
  }
  EXPECT_TRUE(fake_.calls_snapshot().empty());
}

TEST_F(Hand2NodeTest, EndpointPrecedesSerialAndDoesNotScan) {
  auto node = make_node({rclcpp::Parameter("endpoint", "udp/192.0.2.1"),
                         rclcpp::Parameter("serial_number", "WH2J000000000001"),
                         rclcpp::Parameter("hand_side", "left")});

  ASSERT_TRUE(node->start());
  ASSERT_EQ(fake_.connect_calls().size(), 1U);
  EXPECT_EQ(fake_.connect_calls().front().target_kind, WUJI_CONNECT_TARGET_KIND_ADDR);
  EXPECT_EQ(fake_.connect_calls().front().target_value, "udp/192.0.2.1");
  EXPECT_EQ(fake_.count("scan"), 0U);
  node->stop();
}

TEST_F(Hand2NodeTest, DiscoveryRetriesOnlyUntilExactTimeout) {
  fake_.set_scan_results_sequence({{}, {}, {}});
  auto node = make_node({rclcpp::Parameter("connect_timeout", 0.2)}, false);

  EXPECT_FALSE(node->start());
  EXPECT_EQ(fake_.count("scan"), 2U);
  EXPECT_EQ(node->exit_code(), EXIT_FAILURE);
  node->stop();
}

TEST_F(Hand2NodeTest, HandSideProbesDeterministicallyAndClosesRejectedCandidate) {
  fake_.set_identity_from_connect_target(true);
  fake_.set_scan_results_sequence({{{"WH2K000000000002"}, {"WH2J000000000001"}}});
  auto node = make_node({rclcpp::Parameter("hand_side", "right")}, false);

  ASSERT_TRUE(node->start());
  ASSERT_EQ(fake_.connect_calls().size(), 3U);
  EXPECT_EQ(fake_.connect_calls()[0].target_value, "WH2J000000000001");
  EXPECT_EQ(fake_.connect_calls()[1].target_value, "WH2K000000000002");
  EXPECT_EQ(fake_.connect_calls()[2].target_value, "WH2K000000000002");
  EXPECT_EQ(fake_.count("open_joint_command_publisher"), 1U);
  EXPECT_EQ(fake_.count("disconnect"), fake_.count("release"));
  EXPECT_GE(fake_.count("disconnect"), 2U);
  node->stop();
  EXPECT_EQ(fake_.count("init"), fake_.count("shutdown"));
  EXPECT_EQ(fake_.count("disconnect"), fake_.count("release"));
}

TEST_F(Hand2NodeTest, RequiredStartFailureRollsBackConnection) {
  fake_.set_subscribe_failure(FakeHand2Sdk::TypedStream::JointState, WUJI_STATUS_ERR_INTERNAL,
                              "required failed", true);
  auto node = make_node();

  EXPECT_FALSE(node->start());
  EXPECT_EQ(fake_.count("last_error"), 1U);
  EXPECT_NE(node->exit_reason().find("wuji_hand_2_subscribe_joint_states"), std::string::npos);
  EXPECT_NE(node->exit_reason().find("required failed"), std::string::npos);
  EXPECT_EQ(fake_.count("open_joint_command_publisher"),
            fake_.count("close_joint_command_publisher"));
  EXPECT_EQ(fake_.count("disconnect"), fake_.count("release"));
  EXPECT_EQ(fake_.count("init"), fake_.count("shutdown"));
  EXPECT_EQ(node->exit_code(), EXIT_FAILURE);
  node->stop();
}

TEST_F(Hand2NodeTest, OptionalStartFailureContinuesWithoutExit) {
  fake_.set_subscribe_failure(FakeHand2Sdk::TypedStream::Imu, WUJI_STATUS_ERR_INTERNAL,
                              "imu failed", true);
  auto node = make_node({rclcpp::Parameter("publish_fingertip_tactile", true)});

  ASSERT_TRUE(node->start());
  EXPECT_EQ(fake_.count("last_error"), 1U);
  EXPECT_FALSE(node->exit_requested());
  EXPECT_TRUE(
      spin_until(node, [this]() { return fake_.count("subscribe_fingertip_pinky") == 1U; }));
  node->stop();
}

TEST_F(Hand2NodeTest, FingertipStreamsStartOnTheExecutorAfterStartReturns) {
  auto node = make_node({rclcpp::Parameter("publish_fingertip_tactile", true)});

  ASSERT_TRUE(node->start());
  EXPECT_EQ(fake_.count("fingertip_online"), 0U);
  EXPECT_EQ(fake_.count("get_fingertip_info"), 0U);

  ASSERT_TRUE(
      spin_until(node, [this]() { return fake_.count("subscribe_fingertip_pinky") == 1U; }));
  EXPECT_EQ(fake_.count("fingertip_online"), 1U);
  EXPECT_EQ(fake_.count("get_fingertip_info"), 5U);
  node->stop();
}

TEST_F(Hand2NodeTest, OfflineFingertipModulesAreSkipped) {
  fake_.set_fingertip_online({true, false, true, false, false});
  auto node = make_node({rclcpp::Parameter("publish_fingertip_tactile", true)});
  ASSERT_TRUE(node->start());

  ASSERT_TRUE(
      spin_until(node, [this]() { return fake_.count("subscribe_fingertip_middle") == 1U; }));
  EXPECT_EQ(fake_.count("get_fingertip_info"), 2U);
  EXPECT_EQ(fake_.count("subscribe_fingertip_thumb"), 1U);
  EXPECT_EQ(fake_.count("subscribe_fingertip_index"), 0U);
  EXPECT_EQ(fake_.count("subscribe_fingertip_ring"), 0U);
  EXPECT_EQ(fake_.count("subscribe_fingertip_pinky"), 0U);
  EXPECT_FALSE(node->exit_requested());
  node->stop();
}

TEST_F(Hand2NodeTest, FingertipOnlineFailureSkipsAllFingertipStreams) {
  fake_.set_fingertip_online_failure(WUJI_STATUS_ERR_TIMEOUT, "no reply");
  auto node = make_node({rclcpp::Parameter("publish_fingertip_tactile", true)});
  ASSERT_TRUE(node->start());

  ASSERT_TRUE(spin_until(node, [this]() { return fake_.count("fingertip_online") == 1U; }));
  EXPECT_EQ(fake_.count("get_fingertip_info"), 0U);
  EXPECT_FALSE(node->exit_requested());
  node->stop();
}

TEST_F(Hand2NodeTest, ShutdownRequestDuringFingertipStartupSkipsRemainingFingers) {
  fake_.set_fingertip_info_hook([this]() { cancelled_.store(true, std::memory_order_release); });
  auto node = make_node({rclcpp::Parameter("publish_fingertip_tactile", true)});
  ASSERT_TRUE(node->start());

  ASSERT_TRUE(spin_until(node, [this]() { return fake_.count("get_fingertip_info") == 1U; }));
  EXPECT_FALSE(
      spin_until(node, [this]() { return fake_.count("get_fingertip_info") > 1U; }, 100ms));
  node->stop();
}

TEST_F(Hand2NodeTest, AutoEnableFailureRollsBackEveryStartedResource) {
  fake_.set_enable_failure(WUJI_STATUS_ERR_INTERNAL, "enable failed");
  auto node = make_node({rclcpp::Parameter("auto_enable", true)});

  EXPECT_FALSE(node->start());
  EXPECT_GT(fake_.count("sub_close"), 0U);
  EXPECT_EQ(fake_.count("open_joint_command_publisher"),
            fake_.count("close_joint_command_publisher"));
  EXPECT_EQ(fake_.count("disable"), 1U);
  EXPECT_EQ(fake_.count("disconnect"), fake_.count("release"));
  EXPECT_EQ(fake_.count("init"), fake_.count("shutdown"));
  node->stop();
}

TEST_F(Hand2NodeTest, StopUsesReverseSubscriptionsThenPublisherDisableAndDevice) {
  auto node = make_node({rclcpp::Parameter("publish_fingertip_tactile", true)});
  ASSERT_TRUE(node->start());
  ASSERT_TRUE(
      spin_until(node, [this]() { return fake_.count("subscribe_fingertip_pinky") == 1U; }));

  node->stop();
  node->stop();

  EXPECT_EQ(
      fake_.closed_streams_snapshot(),
      (std::vector<FakeHand2Sdk::TypedStream>{
          FakeHand2Sdk::TypedStream::FingertipPinky, FakeHand2Sdk::TypedStream::FingertipRing,
          FakeHand2Sdk::TypedStream::FingertipMiddle, FakeHand2Sdk::TypedStream::FingertipIndex,
          FakeHand2Sdk::TypedStream::FingertipThumb, FakeHand2Sdk::TypedStream::Imu,
          FakeHand2Sdk::TypedStream::JointDiagnostics, FakeHand2Sdk::TypedStream::JointState}));
  const auto calls = fake_.calls_snapshot();
  const auto close = std::find(calls.begin(), calls.end(), "close_joint_command_publisher");
  const auto disable = std::find(calls.begin(), calls.end(), "disable");
  const auto disconnect = std::find(calls.begin(), calls.end(), "disconnect");
  const auto release = std::find(calls.begin(), calls.end(), "release");
  ASSERT_NE(close, calls.end());
  ASSERT_NE(disable, calls.end());
  ASSERT_NE(disconnect, calls.end());
  ASSERT_NE(release, calls.end());
  EXPECT_LT(close, disable);
  EXPECT_LT(disable, disconnect);
  EXPECT_LT(disconnect, release);
  EXPECT_EQ(fake_.count("disable"), 1U);
}

TEST_F(Hand2NodeTest, OptionalTerminalStopsOnlyThatStreamAndDoesNotExit) {
  auto node = make_node();
  ASSERT_TRUE(node->start());
  fake_.emit_imu(WUJI_FRAME_KIND_END, nullptr);

  ASSERT_TRUE(spin_until(node, [this]() { return fake_.count("sub_close") == 1U; }));
  EXPECT_FALSE(shutdown_requested_.load(std::memory_order_acquire));
  EXPECT_FALSE(node->exit_requested());
  EXPECT_EQ(fake_.closed_streams_snapshot(),
            (std::vector<FakeHand2Sdk::TypedStream>{FakeHand2Sdk::TypedStream::Imu}));
  node->stop();
}

TEST_F(Hand2NodeTest, DoesNotPublishStandardDiagnostics) {
  auto node = make_node();
  ASSERT_TRUE(node->start());

  EXPECT_TRUE(node->get_publishers_info_by_topic("/diagnostics").empty());

  node->stop();
}

TEST_F(Hand2NodeTest, RequiredTerminalRequestsExit) {
  auto node = make_node();
  ASSERT_TRUE(node->start());
  fake_.emit_joint_state(WUJI_FRAME_KIND_ERROR, nullptr);

  ASSERT_TRUE(spin_until(
      node, [this]() { return shutdown_requested_.load(std::memory_order_acquire); }, 500ms));
  EXPECT_EQ(node->exit_code(), EXIT_FAILURE);
  EXPECT_NE(node->exit_reason().find("joint_states ERROR: stream error"), std::string::npos);
  node->stop();
}

TEST_F(Hand2NodeTest, ResetReceivesRequiredFeedbackEvenBeforeServicesStart) {
  std::array<WujiJointStateEntry, 20> entries{};
  auto frame = complete_joint_state(entries, 0.0F);
  fake_.set_synchronous_joint_state(WUJI_FRAME_KIND_OK, &frame);
  auto node = make_node();
  ASSERT_TRUE(node->start());
  auto peer = std::make_shared<rclcpp::Node>("reset_feedback_peer");
  auto client = peer->create_client<std_srvs::srv::Trigger>("/reset_ctrl");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(node);
  executor.add_node(peer);
  {
    ExecutorSpinGuard spinning(executor);
    ASSERT_TRUE(client->wait_for_service(1s));
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
    EXPECT_TRUE(future.get()->success);
    EXPECT_EQ(fake_.count("send_joint_commands"), 0U);
    EXPECT_EQ(fake_.count("enable"), 0U);
  }
  executor.remove_node(peer);
  executor.remove_node(node);
  node->stop();
}

TEST_F(Hand2NodeTest, RequiredStreamFailureInterruptsActiveResetWithoutRealShutdown) {
  std::array<WujiJointStateEntry, 20> entries{};
  auto frame = complete_joint_state(entries, 1.0F);
  fake_.set_synchronous_joint_state(WUJI_FRAME_KIND_OK, &frame);
  auto node = make_node({rclcpp::Parameter("reset_duration_s", 0.2),
                         rclcpp::Parameter("reset_timeout_s", 0.5)});
  ASSERT_TRUE(node->start());
  auto peer = std::make_shared<rclcpp::Node>("reset_failure_peer");
  auto client = peer->create_client<std_srvs::srv::Trigger>("/reset_ctrl");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(node);
  executor.add_node(peer);
  {
    ExecutorSpinGuard spinning(executor);
    ASSERT_TRUE(client->wait_for_service(1s));
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (fake_.count("send_joint_commands") == 0U &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(2ms);
    }
    ASSERT_GT(fake_.count("send_joint_commands"), 0U);
    fake_.emit_joint_state(WUJI_FRAME_KIND_ERROR, nullptr);
    ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
    const auto response = future.get();
    EXPECT_FALSE(response->success);
    EXPECT_NE(response->message.find("interrupted"), std::string::npos);
    EXPECT_TRUE(shutdown_requested_.load(std::memory_order_acquire));
    const auto count = fake_.count("send_joint_commands");
    std::this_thread::sleep_for(20ms);
    EXPECT_EQ(fake_.count("send_joint_commands"), count);
  }
  executor.remove_node(peer);
  executor.remove_node(node);
  node->stop();
}

TEST_F(Hand2NodeTest, CreatesFourMutuallyExclusiveAndOneReentrantGroupWithExactMembership) {
  auto node = make_node();
  ASSERT_TRUE(node->start());

  const auto default_group = node->get_node_base_interface()->get_default_callback_group();
  std::vector<rclcpp::CallbackGroup::SharedPtr> groups;
  node->get_node_base_interface()->for_each_callback_group(
      [&groups, &default_group](const rclcpp::CallbackGroup::SharedPtr& group) {
        if (group != default_group) {
          groups.push_back(group);
        }
      });
  ASSERT_EQ(groups.size(), 5U);

  std::set<std::string> command_topics;
  std::set<std::string> control_services;
  std::set<std::string> singleton_services;
  std::vector<std::int64_t> timer_periods;
  for (const auto& group : groups) {
    std::set<std::string> topics;
    std::set<std::string> services;
    std::vector<std::int64_t> periods;
    group->collect_all_ptrs(
        [&topics](const rclcpp::SubscriptionBase::SharedPtr& subscription) {
          topics.insert(subscription->get_topic_name());
        },
        [&services](const rclcpp::ServiceBase::SharedPtr& service) {
          services.insert(service->get_service_name());
        },
        [](const rclcpp::ClientBase::SharedPtr&) {},
        [&periods](const rclcpp::TimerBase::SharedPtr& timer) {
          std::int64_t period = 0;
          EXPECT_EQ(rcl_timer_get_period(timer->get_timer_handle().get(), &period), RCL_RET_OK);
          periods.push_back(period);
        },
        [](const rclcpp::Waitable::SharedPtr&) {});

    const bool is_reentrant = services == (std::set<std::string>{"/emergency_stop", "/reset_ctrl"});
    EXPECT_EQ(group->type(), is_reentrant ? rclcpp::CallbackGroupType::Reentrant
                                                : rclcpp::CallbackGroupType::MutuallyExclusive);

    if (!topics.empty()) {
      command_topics.insert(topics.begin(), topics.end());
    } else if (services.size() == 6U) {
      control_services = services;
    } else if (services.size() == 1U || is_reentrant) {
      singleton_services.insert(services.begin(), services.end());
    } else if (!periods.empty()) {
      timer_periods = periods;
    }
  }

  EXPECT_EQ(command_topics, (std::set<std::string>{"/joint_commands"}));
  EXPECT_EQ(control_services,
            (std::set<std::string>{"/clear_faults", "/enable", "/set_effort_limit",
                                   "/set_effort_limits", "/set_enabled", "/set_mit_params"}));
  EXPECT_EQ(singleton_services,
            (std::set<std::string>{"/calibrate_tactile", "/emergency_stop", "/reset_ctrl"}));
  std::sort(timer_periods.begin(), timer_periods.end());
  const auto fault_bridge_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(50ms).count();
  EXPECT_EQ(timer_periods, (std::vector<std::int64_t>{fault_bridge_period}));
  node->stop();
}

TEST_F(Hand2NodeTest, SecondEmergencyRequestEntersWhileFirstRequestIsBlocked) {
  auto node = make_node();
  ASSERT_TRUE(node->start());
  auto client_node = std::make_shared<rclcpp::Node>("emergency_reentrant_client");
  auto client = client_node->create_client<std_srvs::srv::Trigger>("/emergency_stop");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2U);
  executor.add_node(node);
  executor.add_node(client_node);
  ExecutorSpinGuard spin_guard(executor);
  auto blocker = std::make_shared<BlockingCallCounter>();
  BlockingCallReleaseGuard release_guard(blocker);
  fake_.set_emergency_stop_hook([blocker]() { blocker->enter(); });
  ASSERT_TRUE(client->wait_for_service(2s));

  auto first_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto second_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto first = client->async_send_request(first_request);
  auto second = client->async_send_request(second_request);

  EXPECT_TRUE(blocker->wait_for_entries(2U));
  blocker->release();
  ASSERT_EQ(first.wait_for(1s), std::future_status::ready);
  ASSERT_EQ(second.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(first.get()->success);
  EXPECT_TRUE(second.get()->success);
  EXPECT_EQ(fake_.count("emergency_stop"), 2U);
  EXPECT_FALSE(node->exit_requested());
}

TEST_F(Hand2NodeTest, StopWaitsForFingertipStartupOnAnotherThread) {
  auto blocker = std::make_shared<BlockingCallCounter>();
  BlockingCallReleaseGuard release_guard(blocker);
  fake_.set_fingertip_info_hook([blocker]() { blocker->enter(); });
  auto node = make_node({rclcpp::Parameter("publish_fingertip_tactile", true)});
  ASSERT_TRUE(node->start());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 2U);
  executor.add_node(node);
  ExecutorSpinGuard spin_guard(executor);
  ASSERT_TRUE(blocker->wait_for_entries(1U));

  std::atomic<bool> stop_started{false};
  auto stopped = std::async(std::launch::async, [node, &stop_started]() {
    stop_started.store(true, std::memory_order_release);
    node->stop();
  });
  while (!stop_started.load(std::memory_order_acquire)) {
    std::this_thread::yield();
  }
  EXPECT_EQ(stopped.wait_for(100ms), std::future_status::timeout);
  EXPECT_EQ(fake_.count("disconnect"), 0U);

  blocker->release();
  ASSERT_EQ(stopped.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(fake_.count("disconnect"), 1U);
}

TEST_F(Hand2NodeTest, TwoMatchingPhysicalDevicesFailAsAmbiguousAfterAllProbes) {
  fake_.set_identity_from_connect_target(true);
  fake_.set_scan_results_sequence({{{"WH2J000000000001"}, {"WH2J000000000002"}}});
  auto node = make_node({rclcpp::Parameter("hand_side", "left")}, false);

  EXPECT_FALSE(node->start());

  EXPECT_EQ(fake_.count("connect"), 2U);
  EXPECT_EQ(fake_.count("open_joint_command_publisher"), 0U);
  EXPECT_NE(node->exit_reason().find("multiple"), std::string::npos);
  node->stop();
  EXPECT_EQ(fake_.count("disconnect"), fake_.count("release"));
  EXPECT_EQ(fake_.count("init"), fake_.count("shutdown"));
}

TEST_F(Hand2NodeTest, DirectSelectorRetriesTimeoutAndNotFoundWithinOneDeadline) {
  fake_.set_connect_replies({{WUJI_STATUS_ERR_TIMEOUT, "timeout", false, 40ms},
                             {WUJI_STATUS_ERR_NOT_FOUND, "not found", false, 40ms},
                             {WUJI_STATUS_OK, {}, false, 10ms}},
                            [this](std::chrono::milliseconds elapsed) { clock_.advance(elapsed); });
  auto node = make_node(
      {rclcpp::Parameter("endpoint", "udp/192.0.2.1"), rclcpp::Parameter("connect_timeout", 0.5)},
      false);

  ASSERT_TRUE(node->start());

  ASSERT_EQ(fake_.connect_calls().size(), 3U);
  EXPECT_GT(fake_.connect_calls()[0].options.timeout_ms, 0U);
  EXPECT_GT(fake_.connect_calls()[1].options.timeout_ms, 0U);
  EXPECT_GT(fake_.connect_calls()[2].options.timeout_ms, 0U);
  EXPECT_GT(fake_.connect_calls()[0].options.timeout_ms,
            fake_.connect_calls()[1].options.timeout_ms);
  EXPECT_GT(fake_.connect_calls()[1].options.timeout_ms,
            fake_.connect_calls()[2].options.timeout_ms);
  node->stop();
  EXPECT_EQ(fake_.count("init"), fake_.count("shutdown"));
  EXPECT_EQ(fake_.count("disconnect"), fake_.count("release"));
}

TEST_F(Hand2NodeTest, NoMatchingSideRetriesDiscoveryBeforeSelectingUniqueMatch) {
  fake_.set_identity_from_connect_target(true);
  fake_.set_scan_results_sequence(
      {{{"WH2J000000000001"}, {"WH2J000000000002"}}, {{"WH2K000000000003"}}});
  auto node = make_node(
      {rclcpp::Parameter("hand_side", "right"), rclcpp::Parameter("connect_timeout", 1.0)}, false);

  ASSERT_TRUE(node->start());

  EXPECT_EQ(fake_.count("scan"), 2U);
  ASSERT_FALSE(fake_.connect_calls().empty());
  EXPECT_EQ(fake_.connect_calls().back().target_value, "WH2K000000000003");
  EXPECT_EQ(fake_.count("open_joint_command_publisher"), 1U);
  node->stop();
}

}  // namespace
}  // namespace wuji_hand2_driver
