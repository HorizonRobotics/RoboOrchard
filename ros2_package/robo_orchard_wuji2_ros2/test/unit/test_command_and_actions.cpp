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
#include <rcutils/logging.h>
#include <wuji_sdk.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <future>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wuji_msgs/srv/clear_faults.hpp>
#include <wuji_msgs/srv/set_enabled.hpp>

#include "fake_sdk/fake_hand2_sdk.hpp"
#include "wuji_hand2_driver/control/hand_action_services.hpp"
#include "wuji_hand2_driver/control/joint_command_controller.hpp"
#include "wuji_hand2_driver/control/operation_context.hpp"
#include "wuji_hand2_driver/conversion/joint_mapping.hpp"
#include "wuji_hand2_driver/sdk/hand2_session.hpp"

namespace wuji_hand2_driver::control {
namespace {

using namespace std::chrono_literals;
using JointMapping = conversion::JointMapping;

std::size_t service_count(const rclcpp::Node& node, const std::string& name) {
  return node.get_service_names_and_types_by_node(
      node.get_name(), node.get_namespace()).count(name);
}

std::mutex log_handler_mutex;
std::mutex captured_logs_mutex;
std::vector<std::string> captured_logs;

void capture_ros_log(const rcutils_log_location_t*, int, const char*, rcutils_time_point_value_t,
                     const char* format, va_list* arguments) {
  if (format == nullptr || arguments == nullptr) {
    return;
  }
  std::array<char, 1024> buffer{};
  va_list copied_arguments;
  va_copy(copied_arguments, *arguments);
  static_cast<void>(std::vsnprintf(buffer.data(), buffer.size(), format, copied_arguments));
  va_end(copied_arguments);

  const std::lock_guard<std::mutex> lock(captured_logs_mutex);
  captured_logs.emplace_back(buffer.data());
}

class ScopedRosLogCapture {
 public:
  ScopedRosLogCapture()
      : handler_lock_(log_handler_mutex), previous_handler_(rcutils_logging_get_output_handler()) {
    {
      const std::lock_guard<std::mutex> lock(captured_logs_mutex);
      captured_logs.clear();
    }
    rcutils_logging_set_output_handler(&capture_ros_log);
  }

  ~ScopedRosLogCapture() noexcept {
    rcutils_logging_set_output_handler(previous_handler_);
  }

  ScopedRosLogCapture(const ScopedRosLogCapture&) = delete;
  ScopedRosLogCapture& operator=(const ScopedRosLogCapture&) = delete;

 private:
  std::unique_lock<std::mutex> handler_lock_;
  rcutils_logging_output_handler_t previous_handler_;
};

std::size_t captured_rejection_count() {
  const std::lock_guard<std::mutex> lock(captured_logs_mutex);
  return static_cast<std::size_t>(
      std::count_if(captured_logs.cbegin(), captured_logs.cend(), [](const std::string& message) {
        return message.find("joint command rejected") != std::string::npos;
      }));
}

bool captured_log_contains(const std::string& expected) {
  const std::lock_guard<std::mutex> lock(captured_logs_mutex);
  return std::any_of(captured_logs.cbegin(), captured_logs.cend(),
                     [&expected](const std::string& message) {
                       return message.find(expected) != std::string::npos;
                     });
}

class BlockingHook {
 public:
  void operator()() {
    std::unique_lock<std::mutex> lock(mutex_);
    entered_ = true;
    changed_.notify_all();
    changed_.wait(lock, [this]() { return released_; });
  }

  bool wait_until_entered(std::chrono::milliseconds timeout = 2s) {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, timeout, [this]() { return entered_; });
  }

  void release() {
    const std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    changed_.notify_all();
  }

  ~BlockingHook() noexcept {
    release();
  }

 private:
  std::mutex mutex_;
  std::condition_variable changed_;
  bool entered_{false};
  bool released_{false};
};

class BlockingHookReleaseGuard {
 public:
  explicit BlockingHookReleaseGuard(std::shared_ptr<BlockingHook> hook) : hook_(std::move(hook)) {}

  ~BlockingHookReleaseGuard() noexcept {
    hook_->release();
  }

 private:
  std::shared_ptr<BlockingHook> hook_;
};

constexpr std::array<bool, JointMapping::kJointCount> kAllFalseMask{{
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
}};

constexpr std::array<bool, JointMapping::kJointCount> kMixedMask{{
    false, true,  false, false, true,  false, false, true,  false, false,
    true,  false, false, true,  false, false, true,  false, false, true,
}};

constexpr std::array<bool, JointMapping::kJointCount> kAllTrueMask{{
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
}};

constexpr std::array<std::uint8_t, JointMapping::kJointCount> kAllFalseBytes{{
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
}};

constexpr std::array<std::uint8_t, JointMapping::kJointCount> kMixedBytes{{
    0U, 1U, 0U, 0U, 1U, 0U, 0U, 1U, 0U, 0U, 1U, 0U, 0U, 1U, 0U, 0U, 1U, 0U, 0U, 1U,
}};

constexpr std::array<std::uint8_t, JointMapping::kJointCount> kAllTrueBytes{{
    1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U, 1U,
}};

class CommandAndActionsTest : public testing::Test {
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
    namespace_ = "/command_actions_test_" + std::to_string(id);
    driver_ = std::make_shared<rclcpp::Node>("driver", namespace_);
    client_node_ = std::make_shared<rclcpp::Node>("client", namespace_);

    sdk::Hand2ConnectConfig config;
    config.serial_number = "WH2J000000000001";
    config.hand_side = "left";
    ASSERT_TRUE(session_.connect(config));

    context_ = std::make_unique<OperationContext>(
        OperationContext{fake_.api(), session_, mapping_, gate_, *driver_, driver_->get_logger()});
    command_group_ = driver_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ = driver_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    emergency_group_ = driver_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    command_ = std::make_unique<JointCommandController>(*context_, command_config());
    actions_ = std::make_unique<HandActionServices>(*context_);
    ASSERT_TRUE(command_->start(command_group_, emergency_group_));
    ASSERT_TRUE(actions_->start(control_group_, emergency_group_));

    command_publisher_ = client_node_->create_publisher<sensor_msgs::msg::JointState>(
        "joint_commands", rclcpp::SensorDataQoS().keep_last(1));
    set_enabled_client_ = client_node_->create_client<wuji_msgs::srv::SetEnabled>("set_enabled");
    enable_client_ = client_node_->create_client<std_srvs::srv::Trigger>("enable");
    clear_faults_client_ = client_node_->create_client<wuji_msgs::srv::ClearFaults>("clear_faults");
    emergency_client_ = client_node_->create_client<std_srvs::srv::Trigger>("emergency_stop");

    executor_ =
        std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2U);
    executor_->add_node(driver_);
    executor_->add_node(client_node_);

    auto executor_started_signal = std::make_shared<std::promise<void>>();
    auto executor_started_future = executor_started_signal->get_future();
    auto executor_started = std::make_shared<std::atomic<bool>>(false);
    auto executor_started_timer =
        driver_->create_wall_timer(1ms, [executor_started_signal, executor_started]() {
          if (!executor_started->exchange(true, std::memory_order_relaxed)) {
            executor_started_signal->set_value();
          }
        });
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    ASSERT_EQ(executor_started_future.wait_for(2s), std::future_status::ready);
    executor_started_timer->cancel();

    ASSERT_TRUE(
        wait_until([this]() { return command_publisher_->get_subscription_count() == 1U; }, 2s));
    ASSERT_TRUE(set_enabled_client_->wait_for_service(2s));
    ASSERT_TRUE(enable_client_->wait_for_service(2s));
    ASSERT_TRUE(clear_faults_client_->wait_for_service(2s));
    ASSERT_TRUE(emergency_client_->wait_for_service(2s));
  }

  void TearDown() override {
    if (executor_ != nullptr) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (actions_ != nullptr) {
      actions_->stop();
    }
    if (command_ != nullptr) {
      command_->stop();
    }
    if (executor_ != nullptr && client_node_ != nullptr && driver_ != nullptr) {
      executor_->remove_node(client_node_);
      executor_->remove_node(driver_);
    }
    session_.disconnect();
  }

  virtual DriverConfig command_config() const { return {}; }

  template <typename PredicateT>
  bool wait_until(PredicateT predicate, std::chrono::milliseconds timeout = 1s) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return predicate();
  }

  std::string name(const char* relative) const {
    return namespace_ + "/" + relative;
  }

  sensor_msgs::msg::JointState named_command() const {
    sensor_msgs::msg::JointState message;
    message.name = {"l_pinky_dip", "l_thumb_cmc_flex"};
    message.position = {2.5, -1.25};
    message.velocity = {3.5, -2.25};
    return message;
  }

  sensor_msgs::msg::JointState anonymous_command() const {
    sensor_msgs::msg::JointState message;
    message.position = {0.25,  -1.5,  2.75,  -3.0,  4.125,  -5.25,  6.5,  -7.75,  8.0,  -9.125,
                        10.25, -11.5, 12.75, -13.0, 14.125, -15.25, 16.5, -17.75, 18.0, -19.125};
    message.effort = {20.5, -19.25, 18.0, -17.75, 16.5, -15.25, 14.0, -13.75, 12.5, -11.25,
                      10.0, -9.75,  8.5,  -7.25,  6.0,  -5.75,  4.5,  -3.25,  2.0,  -1.75};
    return message;
  }

  void publish(const sensor_msgs::msg::JointState& message) {
    command_publisher_->publish(message);
  }

  std::shared_ptr<wuji_msgs::srv::SetEnabled::Response> call_set_enabled(
      bool enabled, const std::array<bool, JointMapping::kJointCount>& mask,
      std::chrono::milliseconds timeout = 2s) {
    auto request = std::make_shared<wuji_msgs::srv::SetEnabled::Request>();
    request->enabled = enabled;
    request->mask = mask;
    auto future = set_enabled_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<wuji_msgs::srv::ClearFaults::Response> call_clear_faults(
      const std::array<bool, JointMapping::kJointCount>& mask,
      std::chrono::milliseconds timeout = 2s) {
    auto request = std::make_shared<wuji_msgs::srv::ClearFaults::Request>();
    request->mask = mask;
    auto future = clear_faults_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<std_srvs::srv::Trigger::Response> call_enable() {
    auto future = enable_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<std_srvs::srv::Trigger::Response> call_emergency(
      std::chrono::milliseconds timeout = 2s) {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = emergency_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  bool graph_has_service(const char* relative, const char* type) const {
    const auto services = client_node_->get_service_names_and_types();
    const auto found = services.find(name(relative));
    if (found == services.end()) {
      return false;
    }
    return std::find(found->second.begin(), found->second.end(), type) != found->second.end();
  }

  static std::atomic<std::uint64_t> next_namespace_;
  test::FakeHand2Sdk fake_;
  sdk::Hand2Session session_{fake_.api()};
  conversion::JointMapping mapping_{conversion::HandSide::kLeft};
  OperationGate gate_;
  std::string namespace_;
  rclcpp::Node::SharedPtr driver_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<OperationContext> context_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr emergency_group_;
  std::unique_ptr<JointCommandController> command_;
  std::unique_ptr<HandActionServices> actions_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  rclcpp::Client<wuji_msgs::srv::SetEnabled>::SharedPtr set_enabled_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enable_client_;
  rclcpp::Client<wuji_msgs::srv::ClearFaults>::SharedPtr clear_faults_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_client_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

std::atomic<std::uint64_t> CommandAndActionsTest::next_namespace_{1};

TEST_F(CommandAndActionsTest, CreatesExactRelativeEndpointsTypesAndCommandQos) {
  // An unrelated node may legitimately subscribe to the global topic.
  const auto unrelated_subscription = client_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_commands", rclcpp::SensorDataQoS(),
      [](sensor_msgs::msg::JointState::ConstSharedPtr) {});
  ASSERT_NE(unrelated_subscription, nullptr);
  ASSERT_TRUE(wait_until([this]() { return driver_->count_subscribers("/joint_commands") > 0U; }));

  const auto subscriptions = client_node_->get_subscriptions_info_by_topic(name("joint_commands"));
  ASSERT_EQ(subscriptions.size(), 1U);
  EXPECT_EQ(subscriptions.front().topic_type(), "sensor_msgs/msg/JointState");
  const auto& qos = subscriptions.front().qos_profile();
  EXPECT_EQ(qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  const auto driver_subscriptions =
      driver_->get_node_graph_interface()->get_subscriber_names_and_types_by_node(
          driver_->get_name(), driver_->get_namespace());
  EXPECT_EQ(driver_subscriptions.count(name("joint_commands")), 1U);
  EXPECT_EQ(driver_subscriptions.count("/joint_commands"), 0U);

  EXPECT_TRUE(graph_has_service("set_enabled", "wuji_msgs/srv/SetEnabled"));
  EXPECT_TRUE(graph_has_service("enable", "std_srvs/srv/Trigger"));
  EXPECT_TRUE(graph_has_service("clear_faults", "wuji_msgs/srv/ClearFaults"));
  EXPECT_TRUE(graph_has_service("emergency_stop", "std_srvs/srv/Trigger"));
}

TEST_F(CommandAndActionsTest, SendsNamedAndAnonymousCommandsAcrossRosBoundary) {
  publish(named_command());
  ASSERT_TRUE(wait_until([this]() { return fake_.sent_commands_snapshot().size() == 1U; }));
  auto sent = fake_.sent_commands_snapshot();
  ASSERT_EQ(sent.size(), 1U);
  const std::array<WujiJointCommand, JointMapping::kJointCount> expected_named{{
      {-1.25F, -2.25F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F},
      {0.0F, 0.0F, 0.0F},     {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F},
      {0.0F, 0.0F, 0.0F},     {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F},
      {0.0F, 0.0F, 0.0F},     {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F},
      {0.0F, 0.0F, 0.0F},     {0.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F}, {2.5F, 3.5F, 0.0F},
  }};
  for (std::size_t index = 0; index < expected_named.size(); ++index) {
    EXPECT_FLOAT_EQ(sent[0][index].position, expected_named[index].position) << index;
    EXPECT_FLOAT_EQ(sent[0][index].velocity, expected_named[index].velocity) << index;
    EXPECT_FLOAT_EQ(sent[0][index].effort, expected_named[index].effort) << index;
  }

  publish(anonymous_command());
  ASSERT_TRUE(wait_until([this]() { return fake_.sent_commands_snapshot().size() == 2U; }));
  sent = fake_.sent_commands_snapshot();
  ASSERT_EQ(sent.size(), 2U);
  const std::array<WujiJointCommand, JointMapping::kJointCount> expected_anonymous{{
      {0.25F, 0.0F, 20.5F},     {-1.5F, 0.0F, -19.25F},   {2.75F, 0.0F, 18.0F},
      {-3.0F, 0.0F, -17.75F},   {4.125F, 0.0F, 16.5F},    {-5.25F, 0.0F, -15.25F},
      {6.5F, 0.0F, 14.0F},      {-7.75F, 0.0F, -13.75F},  {8.0F, 0.0F, 12.5F},
      {-9.125F, 0.0F, -11.25F}, {10.25F, 0.0F, 10.0F},    {-11.5F, 0.0F, -9.75F},
      {12.75F, 0.0F, 8.5F},     {-13.0F, 0.0F, -7.25F},   {14.125F, 0.0F, 6.0F},
      {-15.25F, 0.0F, -5.75F},  {16.5F, 0.0F, 4.5F},      {-17.75F, 0.0F, -3.25F},
      {18.0F, 0.0F, 2.0F},      {-19.125F, 0.0F, -1.75F},
  }};
  for (std::size_t index = 0; index < expected_anonymous.size(); ++index) {
    EXPECT_FLOAT_EQ(sent[1][index].position, expected_anonymous[index].position) << index;
    EXPECT_FLOAT_EQ(sent[1][index].velocity, expected_anonymous[index].velocity) << index;
    EXPECT_FLOAT_EQ(sent[1][index].effort, expected_anonymous[index].effort) << index;
  }
}

TEST_F(CommandAndActionsTest, ConversionAndSendFailuresUseOneSecondThrottledErrorLog) {
  ScopedRosLogCapture logs;
  auto invalid = named_command();
  invalid.position[0] = std::numeric_limits<double>::infinity();

  publish(invalid);
  ASSERT_TRUE(wait_until([]() {
    return captured_log_contains(
        "joint command rejected: joint command contains an unconvertible position");
  }));
  EXPECT_EQ(captured_rejection_count(), 1U);
  EXPECT_EQ(fake_.count("send_joint_commands"), 0U);

  fake_.set_send_failure(WUJI_STATUS_ERR_TIMEOUT, "command transport timed out");
  publish(named_command());
  ASSERT_TRUE(wait_until([this]() { return fake_.count("send_joint_commands") == 1U; }));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(captured_rejection_count(), 1U);

  std::this_thread::sleep_for(1s);
  publish(anonymous_command());
  ASSERT_TRUE(wait_until([this]() {
    return fake_.count("send_joint_commands") == 2U && captured_rejection_count() == 2U;
  }));
  EXPECT_TRUE(
      captured_log_contains("joint command rejected: wuji_joint_command_publisher_send failed "
                            "(status -4): command transport timed out"));
}

TEST_F(CommandAndActionsTest, GateClosedRejectsCommandAndAllServices) {
  gate_.stop();
  publish(named_command());
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(fake_.count("send_joint_commands"), 0U);

  const auto enabled = call_set_enabled(true, kAllFalseMask);
  const auto trigger_enabled = call_enable();
  const auto cleared = call_clear_faults(kAllFalseMask);
  const auto emergency = call_emergency();
  ASSERT_NE(enabled, nullptr);
  ASSERT_NE(trigger_enabled, nullptr);
  ASSERT_NE(cleared, nullptr);
  ASSERT_NE(emergency, nullptr);
  EXPECT_FALSE(enabled->success);
  EXPECT_FALSE(trigger_enabled->success);
  EXPECT_EQ(trigger_enabled->message, "control operations are stopping");
  EXPECT_FALSE(cleared->success);
  EXPECT_FALSE(emergency->success);
  EXPECT_FALSE(enabled->message.empty());
  EXPECT_FALSE(cleared->message.empty());
  EXPECT_FALSE(emergency->message.empty());
  EXPECT_EQ(fake_.count("enable"), 0U);
  EXPECT_EQ(fake_.count("clear_fault"), 0U);
  EXPECT_EQ(fake_.count("emergency_stop"), 0U);
}

TEST_F(CommandAndActionsTest, SetEnabledPassesEveryMaskAsNonnullExactBytes) {
  const std::array<std::array<bool, JointMapping::kJointCount>, 3> masks{kAllFalseMask, kMixedMask,
                                                                         kAllTrueMask};
  const std::array<std::array<std::uint8_t, JointMapping::kJointCount>, 3> expected{
      kAllFalseBytes, kMixedBytes, kAllTrueBytes};
  WujiDevice* const device = session_.device();

  for (const auto& mask : masks) {
    const auto enabled = call_set_enabled(true, mask);
    const auto disabled = call_set_enabled(false, mask);
    ASSERT_NE(enabled, nullptr);
    ASSERT_NE(disabled, nullptr);
    EXPECT_TRUE(enabled->success);
    EXPECT_TRUE(disabled->success);
  }

  const auto enable_calls = fake_.enable_calls();
  const auto disable_calls = fake_.disable_calls();
  ASSERT_EQ(enable_calls.size(), masks.size());
  ASSERT_EQ(disable_calls.size(), masks.size());
  for (std::size_t index = 0; index < masks.size(); ++index) {
    EXPECT_FALSE(enable_calls[index].used_null);
    EXPECT_FALSE(disable_calls[index].used_null);
    EXPECT_EQ(enable_calls[index].device, device);
    EXPECT_EQ(disable_calls[index].device, device);
    EXPECT_EQ(enable_calls[index].mask, expected[index]);
    EXPECT_EQ(disable_calls[index].mask, expected[index]);
  }
}

TEST_F(CommandAndActionsTest, EnableTriggerEnablesAllJointsOnlyWhenRequested) {
  EXPECT_EQ(fake_.count("enable"), 0U);
  const auto response = call_enable();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_TRUE(response->message.empty());
  const auto calls = fake_.enable_calls();
  ASSERT_EQ(calls.size(), 1U);
  EXPECT_EQ(calls[0].device, session_.device());
  EXPECT_FALSE(calls[0].used_null);
  EXPECT_EQ(calls[0].mask, kAllTrueBytes);
  EXPECT_EQ(fake_.count("clear_fault"), 0U);
  EXPECT_EQ(fake_.count("disable"), 0U);
}

TEST_F(CommandAndActionsTest, EnableTriggerPropagatesSdkFailure) {
  fake_.set_enable_failure(WUJI_STATUS_ERR_TIMEOUT, "enable transport timed out");
  const auto response = call_enable();
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message,
            "wuji_hand_2_enable failed (status -4): enable transport timed out");
  ASSERT_EQ(fake_.enable_calls().size(), 1U);
  EXPECT_EQ(fake_.enable_calls()[0].mask, kAllTrueBytes);
  EXPECT_EQ(fake_.count("clear_fault"), 0U);
}

TEST_F(CommandAndActionsTest, EmergencyGroupRunsWhileNormalControlGroupIsBlocked) {
  auto enable_blocker = std::make_shared<BlockingHook>();
  BlockingHookReleaseGuard enable_release(enable_blocker);
  fake_.set_enable_hook([enable_blocker]() { (*enable_blocker)(); });
  auto enable_request = std::make_shared<wuji_msgs::srv::SetEnabled::Request>();
  enable_request->enabled = true;
  enable_request->mask = kMixedMask;
  auto enable_future = set_enabled_client_->async_send_request(enable_request);
  ASSERT_TRUE(enable_blocker->wait_until_entered());

  const auto emergency = call_emergency(1s);
  ASSERT_NE(emergency, nullptr);
  EXPECT_TRUE(emergency->success);
  EXPECT_EQ(fake_.count("emergency_stop"), 1U);

  enable_blocker->release();
  ASSERT_EQ(enable_future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(enable_future.get()->success);

  const auto cleared = call_clear_faults(kMixedMask, 1s);
  ASSERT_NE(cleared, nullptr);
  EXPECT_TRUE(cleared->success);
  EXPECT_EQ(fake_.count("clear_fault"), 1U);
}

TEST_F(CommandAndActionsTest, InFlightActionSurvivesStopAndComponentDestruction) {
  auto blocker = std::make_shared<BlockingHook>();
  BlockingHookReleaseGuard release(blocker);
  fake_.set_enable_hook([blocker]() { (*blocker)(); });
  WujiDevice* const device = session_.device();

  auto request = std::make_shared<wuji_msgs::srv::SetEnabled::Request>();
  request->enabled = true;
  request->mask = kMixedMask;
  auto future = set_enabled_client_->async_send_request(request);
  ASSERT_TRUE(blocker->wait_until_entered());
  const auto calls = fake_.enable_calls();
  ASSERT_EQ(calls.size(), 1U);
  EXPECT_EQ(calls.front().device, device);

  actions_->stop();
  actions_.reset();
  blocker->release();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);
  EXPECT_TRUE(wait_until(
      [this]() {
        return service_count(*driver_, name("set_enabled")) == 0U &&
               service_count(*driver_, name("enable")) == 0U &&
               service_count(*driver_, name("clear_faults")) == 0U &&
               service_count(*driver_, name("emergency_stop")) == 0U;
      },
      2s));
}

class ResetTest : public CommandAndActionsTest {
 protected:
  DriverConfig command_config() const override {
    DriverConfig config;
    for (std::size_t index = 0; index < 20; ++index) {
      config.reset_joint_position[index] = 0.25 + 0.01 * index;
    }
    config.reset_duration_s = 0.06;
    config.reset_timeout_s = 0.25;
    config.reset_feedback_timeout_s = 0.08;
    config.reset_position_tolerance = 0.001;
    return config;
  }

  void SetUp() override {
    CommandAndActionsTest::SetUp();
    reset_client_ = client_node_->create_client<std_srvs::srv::Trigger>("reset_ctrl");
    ASSERT_TRUE(reset_client_->wait_for_service(2s));
  }

  void feedback(const std::vector<double>& positions, std::size_t count = 20,
                std::uint64_t timestamp = 0) {
    std::array<WujiJointStateEntry, 20> joints{};
    // Deliberately reverse SDK order. NIDs, not array offsets, identify joints.
    for (std::size_t index = 0; index < count; ++index) {
      const auto dense = 19 - index;
      joints[index].nid = *mapping_.nid_from_index(dense);
      joints[index].position = static_cast<float>(positions[dense]);
    }
    WujiJointStateFrame frame{};
    frame.header.timestamp_us = timestamp ? timestamp : ++timestamp_;
    frame.num_joints = static_cast<std::uint8_t>(count);
    frame.joints = joints.data();
    frame.joints_len = count;
    command_->feedback_observer()(frame);
  }

  auto begin_reset() {
    return reset_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  template <typename Future>
  void follow(Future& future, bool follow_commands = true) {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (future.wait_for(0ms) != std::future_status::ready &&
           std::chrono::steady_clock::now() < deadline) {
      std::vector<double> positions(20, 1.0);
      const auto sent = fake_.sent_commands_snapshot();
      if (follow_commands && !sent.empty()) {
        for (std::size_t index = 0; index < 20; ++index) {
          positions[index] = sent.back()[index].position;
        }
      }
      feedback(positions);
      std::this_thread::sleep_for(2ms);
    }
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
  std::uint64_t timestamp_{100};
};

TEST_F(ResetTest, InterpolatesConfiguredTargetAndRequiresFreshPermutedArrival) {
  feedback(std::vector<double>(20, 1.0));
  const auto started = std::chrono::steady_clock::now();
  auto future = begin_reset();
  ASSERT_TRUE(wait_until([this]() { return fake_.count("send_joint_commands") >= 1; }));
  // Inputs sent during reset must never be replayed after its subscription is restored.
  auto normal = anonymous_command();
  normal.position.assign(20, 900.0);
  publish(normal);
  follow(future);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);
  EXPECT_GE(std::chrono::steady_clock::now() - started, 60ms);
  const auto sent = fake_.sent_commands_snapshot();
  ASSERT_GE(sent.size(), 7U);
  const auto target = command_config().reset_joint_position;
  for (std::size_t joint = 0; joint < 20; ++joint) {
    EXPECT_NEAR(sent.front()[joint].position, 1.0, 0.01);
    EXPECT_FLOAT_EQ(sent.back()[joint].position, static_cast<float>(target[joint]));
    for (std::size_t step = 1; step < sent.size(); ++step) {
      EXPECT_LE(sent[step][joint].position, sent[step - 1][joint].position);
      EXPECT_GE(sent[step][joint].position, target[joint] - 1e-6);
      EXPECT_FLOAT_EQ(sent[step][joint].velocity, 0.0F);
      EXPECT_FLOAT_EQ(sent[step][joint].effort, 0.0F);
    }
  }
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(fake_.sent_commands_snapshot().size(), sent.size());
  EXPECT_EQ(fake_.count("enable"), 0U);
  EXPECT_EQ(fake_.count("clear_fault"), 0U);
  ASSERT_TRUE(wait_until([this]() { return command_publisher_->get_subscription_count() == 1U; }));
  publish(named_command());
  EXPECT_TRUE(wait_until([this, &sent]() { return fake_.sent_commands_snapshot().size() > sent.size(); }));
}

TEST_F(ResetTest, FreshAlreadyAtTargetDoesNotSendOrEnable) {
  feedback(command_config().reset_joint_position);
  auto future = begin_reset();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);
  EXPECT_EQ(fake_.count("send_joint_commands"), 0U);
  EXPECT_EQ(fake_.count("enable"), 0U);
}

TEST_F(ResetTest, RejectsAbsentSparseAndStaleFeedbackWithoutSending) {
  auto absent = begin_reset();
  ASSERT_EQ(absent.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(absent.get()->success);
  feedback(std::vector<double>(20, 1.0), 19);
  auto sparse = begin_reset();
  ASSERT_EQ(sparse.wait_for(1s), std::future_status::ready);
  EXPECT_NE(sparse.get()->message.find("complete 20-joint"), std::string::npos);
  feedback(std::vector<double>(20, 1.0));
  std::this_thread::sleep_for(100ms);
  auto stale = begin_reset();
  ASSERT_EQ(stale.wait_for(1s), std::future_status::ready);
  EXPECT_NE(stale.get()->message.find("stale"), std::string::npos);
  EXPECT_EQ(fake_.count("send_joint_commands"), 0U);
}

TEST_F(ResetTest, TrajectoryCompletionAloneDoesNotReportArrival) {
  feedback(std::vector<double>(20, 1.0));
  auto future = begin_reset();
  follow(future, false);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("arrival timed out"), std::string::npos);
  EXPECT_NE(response->message.find("enable"), std::string::npos);
}

TEST_F(ResetTest, FeedbackLossAndRepeatedDeviceTimestampAbortActiveReset) {
  feedback(std::vector<double>(20, 1.0), 20, 10'000);
  auto future = begin_reset();
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (future.wait_for(0ms) != std::future_status::ready &&
         std::chrono::steady_clock::now() < deadline) {
    feedback(std::vector<double>(20, 1.0), 20, 10'000);
    std::this_thread::sleep_for(2ms);
  }
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_NE(future.get()->message.find("stale"), std::string::npos);
}

TEST_F(ResetTest, TargetFeedbackBeforeFinalCommandCannotProveArrival) {
  feedback(std::vector<double>(20, 1.0));
  auto future = begin_reset();
  ASSERT_TRUE(wait_until([this]() { return fake_.count("send_joint_commands") >= 1; }));
  feedback(command_config().reset_joint_position);
  EXPECT_EQ(future.wait_for(20ms), std::future_status::timeout);
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("stale"), std::string::npos);
}

TEST_F(ResetTest, SendFailureAndDisconnectPropagate) {
  feedback(std::vector<double>(20, 1.0));
  fake_.set_send_failure(WUJI_STATUS_ERR_TIMEOUT, "reset send failed");
  auto failed = begin_reset();
  ASSERT_EQ(failed.wait_for(1s), std::future_status::ready);
  EXPECT_NE(failed.get()->message.find("reset send failed"), std::string::npos);
  fake_.set_connected(false);
  auto disconnected = begin_reset();
  ASSERT_EQ(disconnected.wait_for(1s), std::future_status::ready);
  EXPECT_NE(disconnected.get()->message.find("disconnected"), std::string::npos);
}

TEST_F(ResetTest, ConcurrentRequestIsRejectedRatherThanRestartingTrajectory) {
  feedback(std::vector<double>(20, 1.0));
  auto first = begin_reset();
  ASSERT_TRUE(wait_until([this]() { return fake_.count("send_joint_commands") >= 1; }));
  auto second = begin_reset();
  ASSERT_EQ(second.wait_for(50ms), std::future_status::ready);
  const auto rejected = second.get();
  EXPECT_FALSE(rejected->success);
  EXPECT_EQ(rejected->message, "reset is already in progress");
  follow(first);
  ASSERT_EQ(first.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(first.get()->success);
}

TEST_F(ResetTest, EmergencyInterruptsWithoutAnySubsequentResetSend) {
  feedback(std::vector<double>(20, 1.0));
  auto future = begin_reset();
  ASSERT_TRUE(wait_until([this]() { return fake_.count("send_joint_commands") >= 1; }));
  const auto stopped = call_emergency();
  ASSERT_NE(stopped, nullptr);
  EXPECT_TRUE(stopped->success);
  const auto count = fake_.count("send_joint_commands");
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_NE(future.get()->message.find("interrupted"), std::string::npos);
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(fake_.count("send_joint_commands"), count);
}

TEST_F(ResetTest, ShutdownWaitsForResetBeforePublisherCanBeClosed) {
  feedback(std::vector<double>(20, 1.0));
  auto future = begin_reset();
  ASSERT_TRUE(wait_until([this]() { return fake_.count("send_joint_commands") >= 1; }));
  command_->stop();
  const auto count = fake_.count("send_joint_commands");
  session_.close_command_publisher();
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_NE(future.get()->message.find("interrupted"), std::string::npos);
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(fake_.count("send_joint_commands"), count);
  EXPECT_EQ(service_count(*driver_, name("reset_ctrl")), 0U);
}

TEST_F(ResetTest, NonemptyDisableInterruptsButEmptyMaskDoesNot) {
  feedback(std::vector<double>(20, 1.0));
  auto future = begin_reset();
  ASSERT_TRUE(wait_until([this]() { return fake_.count("send_joint_commands") >= 1; }));
  const auto no_op = call_set_enabled(false, kAllFalseMask);
  ASSERT_NE(no_op, nullptr);
  EXPECT_TRUE(no_op->success);
  EXPECT_EQ(future.wait_for(0ms), std::future_status::timeout);
  const auto disabled = call_set_enabled(false, kMixedMask);
  ASSERT_NE(disabled, nullptr);
  EXPECT_TRUE(disabled->success);
  const auto count = fake_.count("send_joint_commands");
  ASSERT_EQ(future.wait_for(1s), std::future_status::ready);
  EXPECT_FALSE(future.get()->success);
  std::this_thread::sleep_for(30ms);
  EXPECT_EQ(fake_.count("send_joint_commands"), count);
}

class ControlLifecycleTest : public testing::Test {
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
    node_ = std::make_shared<rclcpp::Node>("control_lifecycle", "/control_lifecycle_test");
    command_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    emergency_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sdk::Hand2ConnectConfig config;
    config.serial_number = "WH2J000000000001";
    config.hand_side = "left";
    const auto status = session_.connect(config);
    ASSERT_TRUE(status) << status.message;
  }

  void TearDown() override {
    session_.disconnect();
  }

  std::string name(const char* relative) const {
    return std::string{"/control_lifecycle_test/"} + relative;
  }

  test::FakeHand2Sdk fake_;
  sdk::Hand2Session session_{fake_.api()};
  conversion::JointMapping mapping_{conversion::HandSide::kLeft};
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr emergency_group_;
};

TEST_F(ControlLifecycleTest, ConstructorsFailedStartAndRepeatedStopLeaveNoEndpoints) {
  OperationGate command_gate;
  OperationContext command_context{fake_.api(),  session_, mapping_,
                                   command_gate, *node_,   node_->get_logger()};
  JointCommandController command(command_context);
  EXPECT_EQ(node_->count_subscribers(name("joint_commands")), 0U);
  EXPECT_FALSE(command.start(nullptr, emergency_group_));
  EXPECT_EQ(node_->count_subscribers(name("joint_commands")), 0U);
  ASSERT_TRUE(command.start(command_group_, emergency_group_));
  ASSERT_TRUE(command.start(command_group_, emergency_group_));
  EXPECT_EQ(node_->count_subscribers(name("joint_commands")), 1U);
  command.stop();
  command.stop();
  EXPECT_EQ(node_->count_subscribers(name("joint_commands")), 0U);

  OperationGate action_gate;
  OperationContext action_context{fake_.api(), session_, mapping_,
                                  action_gate, *node_,   node_->get_logger()};
  HandActionServices actions(action_context);
  EXPECT_EQ(service_count(*node_, name("enable")), 0U);
  EXPECT_EQ(service_count(*node_, name("set_enabled")), 0U);
  EXPECT_FALSE(actions.start(control_group_, nullptr));
  EXPECT_EQ(service_count(*node_, name("enable")), 0U);
  EXPECT_EQ(service_count(*node_, name("set_enabled")), 0U);
  ASSERT_TRUE(actions.start(control_group_, emergency_group_));
  ASSERT_TRUE(actions.start(control_group_, emergency_group_));
  EXPECT_EQ(service_count(*node_, name("set_enabled")), 1U);
  EXPECT_EQ(service_count(*node_, name("enable")), 1U);
  EXPECT_EQ(service_count(*node_, name("clear_faults")), 1U);
  EXPECT_EQ(service_count(*node_, name("emergency_stop")), 1U);
  actions.stop();
  actions.stop();
  EXPECT_EQ(service_count(*node_, name("set_enabled")), 0U);
  EXPECT_EQ(service_count(*node_, name("enable")), 0U);
  EXPECT_EQ(service_count(*node_, name("clear_faults")), 0U);
  EXPECT_EQ(service_count(*node_, name("emergency_stop")), 0U);
}

TEST_F(ControlLifecycleTest, RejectsIncompleteActionSdkBindingsBeforeCreatingEndpoints) {
  const auto complete_api = fake_.api();
  const auto expect_rejected = [this, &complete_api](const char* binding, auto clear_binding) {
    SCOPED_TRACE(binding);
    clear_binding(fake_.api());

    OperationGate gate;
    OperationContext context{fake_.api(), session_, mapping_, gate, *node_, node_->get_logger()};
    HandActionServices actions(context);
    const auto status = actions.start(control_group_, emergency_group_);

    EXPECT_FALSE(status);
    EXPECT_EQ(status.message, "Hand action SDK bindings are incomplete");
    EXPECT_EQ(service_count(*node_, name("set_enabled")), 0U);
    EXPECT_EQ(service_count(*node_, name("enable")), 0U);
    EXPECT_EQ(service_count(*node_, name("clear_faults")), 0U);
    EXPECT_EQ(service_count(*node_, name("emergency_stop")), 0U);
    fake_.api() = complete_api;
  };

  expect_rejected("enable", [](auto& api) { api.enable = nullptr; });
  expect_rejected("disable", [](auto& api) { api.disable = nullptr; });
  expect_rejected("clear_fault", [](auto& api) { api.clear_fault = nullptr; });
  expect_rejected("emergency_stop", [](auto& api) { api.emergency_stop = nullptr; });
}

}  // namespace
}  // namespace wuji_hand2_driver::control
