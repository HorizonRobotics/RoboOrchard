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

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <wuji_msgs/srv/set_effort_limit.hpp>
#include <wuji_msgs/srv/set_enabled.hpp>
#include <wuji_msgs/srv/set_effort_limits.hpp>
#include <wuji_msgs/srv/set_mit_params.hpp>

#include "fake_sdk/fake_hand2_sdk.hpp"
#include "wuji_hand2_driver/control/control_plane.hpp"
#include "wuji_hand2_driver/control/hand_action_services.hpp"
#include "wuji_hand2_driver/control/hand_configuration_services.hpp"
#include "wuji_hand2_driver/control/operation_context.hpp"
#include "wuji_hand2_driver/control/tactile_calibration_service.hpp"
#include "wuji_hand2_driver/conversion/joint_mapping.hpp"
#include "wuji_hand2_driver/sdk/hand2_session.hpp"

namespace wuji_hand2_driver::control {
namespace {

using namespace std::chrono_literals;
using JointMapping = conversion::JointMapping;
using TactileReply = test::FakeHand2Sdk::TactileStatusReply;

std::size_t service_count(const rclcpp::Node& node, const std::string& name) {
  return node.get_service_names_and_types_by_node(
      node.get_name(), node.get_namespace()).count(name);
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

class ManualClock {
 public:
  using Clock = std::chrono::steady_clock;

  Clock::time_point now() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return Clock::time_point{elapsed_};
  }

  void sleep_for(Clock::duration duration) {
    const std::lock_guard<std::mutex> lock(mutex_);
    elapsed_ += duration;
    ++sleep_count_;
  }

  void advance(Clock::duration duration) {
    const std::lock_guard<std::mutex> lock(mutex_);
    elapsed_ += duration;
  }

  std::size_t sleep_count() const {
    const std::lock_guard<std::mutex> lock(mutex_);
    return sleep_count_;
  }

 private:
  mutable std::mutex mutex_;
  Clock::duration elapsed_{};
  std::size_t sleep_count_{0};
};

TactileReply ready_reply() {
  return TactileReply{
      WUJI_STATUS_OK, WUJI_FINGERTIP_SENSOR_MODEL_STANDARD, WUJI_FINGERTIP_SENSOR_STATE_READY, {}};
}

TactileReply calibrating_reply() {
  return TactileReply{WUJI_STATUS_OK,
                      WUJI_FINGERTIP_SENSOR_MODEL_STANDARD,
                      WUJI_FINGERTIP_SENSOR_STATE_CALIBRATING,
                      {}};
}

std::vector<TactileReply> repeat_reply(const TactileReply& reply, std::size_t count) {
  return std::vector<TactileReply>(count, reply);
}

std::size_t callback_group_service_count(const rclcpp::CallbackGroup::SharedPtr& group) {
  std::size_t count = 0;
  group->find_service_ptrs_if([&count](const auto&) {
    ++count;
    return false;
  });
  return count;
}

bool callback_group_has_service(const rclcpp::CallbackGroup::SharedPtr& group,
                                const std::string& service_name) {
  return group->find_service_ptrs_if(
             [&service_name](const rclcpp::ServiceBase::SharedPtr& candidate) {
               return candidate->get_service_name() == service_name;
             }) != nullptr;
}

bool callback_group_has_subscription(const rclcpp::CallbackGroup::SharedPtr& group,
                                     const std::string& topic_name) {
  return group->find_subscription_ptrs_if(
             [&topic_name](const rclcpp::SubscriptionBase::SharedPtr& candidate) {
               return candidate->get_topic_name() == topic_name;
             }) != nullptr;
}

class ConfigurationAndCalibrationTest : public testing::Test {
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
    namespace_ = "/configuration_calibration_test_" + std::to_string(id);
    driver_ = std::make_shared<rclcpp::Node>("driver", namespace_);
    client_node_ = std::make_shared<rclcpp::Node>("client", namespace_);
    command_group_ = driver_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ = driver_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    emergency_group_ = driver_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    tactile_group_ = driver_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    sdk::Hand2ConnectConfig config;
    config.serial_number = "WH2J000000000001";
    config.hand_side = "left";
    ASSERT_TRUE(session_.connect(config));
    context_ = std::make_unique<OperationContext>(
        OperationContext{fake_.api(), session_, mapping_, gate_, *driver_, driver_->get_logger()});

    effort_limit_client_ =
        client_node_->create_client<wuji_msgs::srv::SetEffortLimit>("set_effort_limit");
    effort_limits_client_ =
        client_node_->create_client<wuji_msgs::srv::SetEffortLimits>("set_effort_limits");
    mit_client_ = client_node_->create_client<wuji_msgs::srv::SetMitParams>("set_mit_params");
    calibration_client_ = client_node_->create_client<std_srvs::srv::Trigger>("calibrate_tactile");
    emergency_client_ = client_node_->create_client<std_srvs::srv::Trigger>("emergency_stop");

    executor_ =
        std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2U);
    executor_->add_node(driver_);
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override {
    if (executor_ != nullptr) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    if (plane_ != nullptr) {
      plane_->stop();
    }
    if (calibration_ != nullptr) {
      calibration_->stop();
    }
    if (configuration_ != nullptr) {
      configuration_->stop();
    }
    if (actions_ != nullptr) {
      actions_->stop();
    }
    if (executor_ != nullptr && client_node_ != nullptr && driver_ != nullptr) {
      executor_->remove_node(client_node_);
      executor_->remove_node(driver_);
    }
    session_.disconnect();
  }

  TactileCalibrationTiming timing(std::chrono::steady_clock::duration interval = 1ms,
                                  std::chrono::steady_clock::duration timeout = 30s) {
    return TactileCalibrationTiming{[clock = clock_]() { return clock->now(); },
                                    [clock = clock_](std::chrono::steady_clock::duration duration) {
                                      clock->sleep_for(duration);
                                    },
                                    interval, timeout};
  }

  void start_configuration() {
    configuration_ = std::make_unique<HandConfigurationServices>(*context_);
    const auto status = configuration_->start(control_group_);
    ASSERT_TRUE(status) << status.message;
    ASSERT_TRUE(effort_limit_client_->wait_for_service(2s));
    ASSERT_TRUE(effort_limits_client_->wait_for_service(2s));
    ASSERT_TRUE(mit_client_->wait_for_service(2s));
  }

  void start_calibration(const TactileCalibrationTiming& calibration_timing) {
    calibration_ = std::make_unique<TactileCalibrationService>(*context_, calibration_timing);
    const auto status = calibration_->start(tactile_group_);
    ASSERT_TRUE(status) << status.message;
    ASSERT_TRUE(calibration_client_->wait_for_service(2s));
  }

  template <typename ServiceT>
  std::shared_ptr<typename ServiceT::Response> call(
      const typename rclcpp::Client<ServiceT>::SharedPtr& client,
      const std::shared_ptr<typename ServiceT::Request>& request,
      std::chrono::milliseconds timeout = 2s) {
    auto future = client->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<wuji_msgs::srv::SetEffortLimit::Response> set_effort_limit(float value) {
    auto request = std::make_shared<wuji_msgs::srv::SetEffortLimit::Request>();
    request->effort_limit = value;
    return call<wuji_msgs::srv::SetEffortLimit>(effort_limit_client_, request);
  }

  std::shared_ptr<wuji_msgs::srv::SetEffortLimits::Response> set_effort_limits(
      const std::array<float, JointMapping::kJointCount>& values) {
    auto request = std::make_shared<wuji_msgs::srv::SetEffortLimits::Request>();
    request->effort_limits = values;
    return call<wuji_msgs::srv::SetEffortLimits>(effort_limits_client_, request);
  }

  std::shared_ptr<wuji_msgs::srv::SetMitParams::Response> set_mit_params(
      const std::array<float, JointMapping::kJointCount>& kp,
      const std::array<float, JointMapping::kJointCount>& kd) {
    auto request = std::make_shared<wuji_msgs::srv::SetMitParams::Request>();
    request->kp = kp;
    request->kd = kd;
    return call<wuji_msgs::srv::SetMitParams>(mit_client_, request);
  }

  std::shared_ptr<std_srvs::srv::Trigger::Response> trigger(
      const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
      std::chrono::milliseconds timeout = 2s) {
    return call<std_srvs::srv::Trigger>(client, std::make_shared<std_srvs::srv::Trigger::Request>(),
                                        timeout);
  }

  std::string name(const char* relative) const {
    return namespace_ + "/" + relative;
  }

  bool graph_has_service(const char* relative, const char* type) const {
    const auto services = client_node_->get_service_names_and_types();
    const auto found = services.find(name(relative));
    return found != services.end() &&
           std::find(found->second.begin(), found->second.end(), type) != found->second.end();
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
  rclcpp::CallbackGroup::SharedPtr tactile_group_;
  std::shared_ptr<ManualClock> clock_{std::make_shared<ManualClock>()};
  std::unique_ptr<HandConfigurationServices> configuration_;
  std::unique_ptr<TactileCalibrationService> calibration_;
  std::unique_ptr<HandActionServices> actions_;
  std::unique_ptr<ControlPlane> plane_;
  rclcpp::Client<wuji_msgs::srv::SetEffortLimit>::SharedPtr effort_limit_client_;
  rclcpp::Client<wuji_msgs::srv::SetEffortLimits>::SharedPtr effort_limits_client_;
  rclcpp::Client<wuji_msgs::srv::SetMitParams>::SharedPtr mit_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr calibration_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr emergency_client_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
};

std::atomic<std::uint64_t> ConfigurationAndCalibrationTest::next_namespace_{1};

TEST_F(ConfigurationAndCalibrationTest, ConfigurationUsesExactRelativeEndpointsGroupAndSdkArrays) {
  start_configuration();
  EXPECT_TRUE(graph_has_service("set_effort_limit", "wuji_msgs/srv/SetEffortLimit"));
  EXPECT_TRUE(graph_has_service("set_effort_limits", "wuji_msgs/srv/SetEffortLimits"));
  EXPECT_TRUE(graph_has_service("set_mit_params", "wuji_msgs/srv/SetMitParams"));
  EXPECT_EQ(service_count(*driver_, "/set_effort_limit"), 0U);
  EXPECT_EQ(callback_group_service_count(control_group_), 3U);
  EXPECT_TRUE(callback_group_has_service(control_group_, name("set_effort_limit")));
  EXPECT_TRUE(callback_group_has_service(control_group_, name("set_effort_limits")));
  EXPECT_TRUE(callback_group_has_service(control_group_, name("set_mit_params")));

  const std::array<float, 20> limits{{0.125F,  1.25F,  2.375F,  3.5F,    4.625F,  5.75F,   6.875F,
                                      7.0F,    8.125F, 9.25F,   10.375F, 11.5F,   12.625F, 13.75F,
                                      14.875F, 15.0F,  16.125F, 17.25F,  18.375F, 19.5F}};
  const std::array<float, 20> kp{{20.5F,   19.375F, 18.25F,  17.125F, 16.0F,  15.875F, 14.75F,
                                  13.625F, 12.5F,   11.375F, 10.25F,  9.125F, 8.0F,    7.875F,
                                  6.75F,   5.625F,  4.5F,    3.375F,  2.25F,  1.125F}};
  const std::array<float, 20> kd{{0.0625F, 0.125F,  0.1875F, 0.25F,   0.3125F, 0.375F,  0.4375F,
                                  0.5F,    0.5625F, 0.625F,  0.6875F, 0.75F,   0.8125F, 0.875F,
                                  0.9375F, 1.0F,    1.0625F, 1.125F,  1.1875F, 1.25F}};

  ASSERT_TRUE(set_effort_limit(7.375F)->success);
  ASSERT_TRUE(set_effort_limits(limits)->success);
  ASSERT_TRUE(set_mit_params(kp, kd)->success);
  const auto scalar_calls = fake_.effort_limit_calls();
  const auto array_calls = fake_.effort_limits_calls();
  const auto mit_calls = fake_.mit_params_calls();
  ASSERT_EQ(scalar_calls.size(), 1U);
  ASSERT_EQ(array_calls.size(), 1U);
  ASSERT_EQ(mit_calls.size(), 1U);
  EXPECT_EQ(scalar_calls[0].device, session_.device());
  EXPECT_FLOAT_EQ(scalar_calls[0].limit, 7.375F);
  EXPECT_EQ(array_calls[0].device, session_.device());
  EXPECT_EQ(array_calls[0].limits, limits);
  EXPECT_EQ(mit_calls[0].device, session_.device());
  EXPECT_EQ(mit_calls[0].kp, kp);
  EXPECT_EQ(mit_calls[0].kd, kd);
}

TEST_F(ConfigurationAndCalibrationTest, ConfigurationRejectsEveryInvalidFloatBeforeSdk) {
  start_configuration();
  const std::array<float, 4> invalid{{-0.125F, std::numeric_limits<float>::quiet_NaN(),
                                      std::numeric_limits<float>::infinity(),
                                      -std::numeric_limits<float>::infinity()}};
  std::array<float, 20> values{};
  std::array<float, 20> other{};

  for (std::size_t index = 0; index < invalid.size(); ++index) {
    auto scalar = set_effort_limit(invalid[index]);
    ASSERT_NE(scalar, nullptr);
    EXPECT_FALSE(scalar->success);
    EXPECT_FALSE(scalar->message.empty());

    values.fill(1.25F);
    values[index + 3U] = invalid[index];
    auto limits = set_effort_limits(values);
    ASSERT_NE(limits, nullptr);
    EXPECT_FALSE(limits->success);
    EXPECT_FALSE(limits->message.empty());

    values.fill(2.5F);
    other.fill(0.75F);
    values[index + 7U] = invalid[index];
    auto kp = set_mit_params(values, other);
    ASSERT_NE(kp, nullptr);
    EXPECT_FALSE(kp->success);

    values.fill(2.5F);
    other.fill(0.75F);
    other[index + 11U] = invalid[index];
    auto kd = set_mit_params(values, other);
    ASSERT_NE(kd, nullptr);
    EXPECT_FALSE(kd->success);
  }

  EXPECT_EQ(fake_.count("set_effort_limit"), 0U);
  EXPECT_EQ(fake_.count("set_effort_limits"), 0U);
  EXPECT_EQ(fake_.count("set_mit_params"), 0U);
}

TEST_F(ConfigurationAndCalibrationTest, CalibrationUsesDistinctGroupAndQueriesAllFingersInOrder) {
  start_calibration(timing());
  EXPECT_TRUE(graph_has_service("calibrate_tactile", "std_srvs/srv/Trigger"));
  EXPECT_EQ(callback_group_service_count(tactile_group_), 1U);
  EXPECT_TRUE(callback_group_has_service(tactile_group_, name("calibrate_tactile")));
  EXPECT_FALSE(callback_group_has_service(control_group_, name("calibrate_tactile")));

  fake_.set_tactile_status_replies(repeat_reply(ready_reply(), 5U));
  const auto response = trigger(calibration_client_);
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_NE(response->message.find("all five"), std::string::npos);
  EXPECT_EQ(fake_.count("calibrate_fingertip"), 1U);
  const auto devices = fake_.tactile_calibrate_devices();
  ASSERT_EQ(devices.size(), 1U);
  EXPECT_EQ(devices[0], session_.device());
  const auto calls = fake_.tactile_status_calls();
  ASSERT_EQ(calls.size(), 5U);
  for (std::uint8_t finger = 0; finger < 5U; ++finger) {
    EXPECT_EQ(calls[finger].device, session_.device());
    EXPECT_EQ(calls[finger].finger, finger);
  }
}

TEST_F(ConfigurationAndCalibrationTest,
       CalibrationRetriesAStatusTimeoutThenSucceedsWhenAllFingersReady) {
  start_calibration(timing());
  fake_.set_tactile_status_replies(
      {TactileReply{WUJI_STATUS_ERR_TIMEOUT, WUJI_FINGERTIP_SENSOR_MODEL_STANDARD,
                    WUJI_FINGERTIP_SENSOR_STATE_CALIBRATING, "temporary status timeout"},
       ready_reply(), ready_reply(), ready_reply(), ready_reply(), ready_reply()});

  const auto response = trigger(calibration_client_);

  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(fake_.count("fingertip_status"), 6U);
  EXPECT_EQ(clock_->sleep_count(), 1U);
}

TEST_F(ConfigurationAndCalibrationTest, CalibrationFailsWhenTheOverallDeadlineExpires) {
  start_calibration(timing(1ms, 3ms));
  fake_.set_tactile_status_replies(repeat_reply(calibrating_reply(), 15U));

  const auto response = trigger(calibration_client_);

  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message,
            "tactile calibration did not complete before the 30 second deadline");
  EXPECT_EQ(fake_.count("fingertip_status"), 15U);
  EXPECT_EQ(clock_->sleep_count(), 3U);
}

TEST_F(ConfigurationAndCalibrationTest, CalibrationFailsImmediatelyOnStartStatusAndUnknownState) {
  start_calibration(timing());
  fake_.set_tactile_calibrate_failure(WUJI_STATUS_ERR_INTERNAL, "calibration enqueue fault");
  auto response = trigger(calibration_client_);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message,
            "wuji_hand_2_calibrate_fingertip failed (status -99): calibration enqueue fault");
  EXPECT_EQ(fake_.count("fingertip_status"), 0U);

  fake_.set_tactile_calibrate_failure(WUJI_STATUS_OK, {});
  fake_.set_tactile_status_replies(
      {TactileReply{WUJI_STATUS_ERR_PROTOCOL, WUJI_FINGERTIP_SENSOR_MODEL_STANDARD,
                    WUJI_FINGERTIP_SENSOR_STATE_READY, "finger status protocol error"}});
  response = trigger(calibration_client_);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("finger status protocol error"), std::string::npos);
  EXPECT_EQ(fake_.count("fingertip_status"), 1U);
  EXPECT_EQ(clock_->sleep_count(), 0U);

  fake_.set_tactile_status_replies({TactileReply{WUJI_STATUS_OK,
                                                 WUJI_FINGERTIP_SENSOR_MODEL_STANDARD,
                                                 static_cast<WujiFingertipSensorState>(77),
                                                 {}}});
  response = trigger(calibration_client_);
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("unknown fingertip state 77"), std::string::npos);
  EXPECT_EQ(fake_.count("fingertip_status"), 2U);
}

TEST_F(ConfigurationAndCalibrationTest, ControlPlaneStartsAndStopsConcreteComponentsInFixedOrder) {
  auto node = std::make_shared<rclcpp::Node>("plane", "/control_plane_lifecycle");
  auto command_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto control_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto emergency_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto tactile_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  OperationGate gate;
  OperationContext context{fake_.api(), session_, mapping_, gate, *node, node->get_logger()};
  ControlPlane plane(context, timing());
  EXPECT_EQ(node->count_subscribers("/control_plane_lifecycle/joint_commands"), 0U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/set_enabled"), 0U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/set_effort_limit"), 0U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/calibrate_tactile"), 0U);

  auto status = plane.start(command_group, control_group, emergency_group, tactile_group);
  ASSERT_TRUE(status) << status.message;
  ASSERT_TRUE(plane.start(command_group, control_group, emergency_group, tactile_group));
  EXPECT_TRUE(
      callback_group_has_subscription(command_group, "/control_plane_lifecycle/joint_commands"));
  EXPECT_FALSE(
      callback_group_has_subscription(control_group, "/control_plane_lifecycle/joint_commands"));
  EXPECT_EQ(callback_group_service_count(control_group), 6U);
  EXPECT_EQ(callback_group_service_count(emergency_group), 2U);
  EXPECT_EQ(callback_group_service_count(tactile_group), 1U);
  EXPECT_EQ(node->count_subscribers("/control_plane_lifecycle/joint_commands"), 1U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/set_enabled"), 1U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/clear_faults"), 1U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/emergency_stop"), 1U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/reset_ctrl"), 1U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/set_effort_limit"), 1U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/set_effort_limits"), 1U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/set_mit_params"), 1U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/calibrate_tactile"), 1U);

  plane.stop();
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/reset_ctrl"), 0U);
  EXPECT_EQ(node->count_subscribers("/control_plane_lifecycle/joint_commands"), 0U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/set_enabled"), 0U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/set_effort_limit"), 0U);
  EXPECT_EQ(service_count(*node, "/control_plane_lifecycle/calibrate_tactile"), 0U);

  plane.stop();
  status = plane.start(command_group, control_group, emergency_group, tactile_group);
  EXPECT_FALSE(status);
  EXPECT_FALSE(gate.accepting());
}

}  // namespace
}  // namespace wuji_hand2_driver::control
