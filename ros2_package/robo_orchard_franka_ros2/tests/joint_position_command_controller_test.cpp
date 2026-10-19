/*
 * Project RoboOrchard
 *
 * Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include "robo_orchard_franka_ros2/joint_position_command_controller.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <franka/rate_limiting.h>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>

namespace {

using namespace std::chrono_literals;
using Controller = robo_orchard_franka_ros2::JointPositionCommandController;
using Positions = std::array<double, 7>;
using JointState = sensor_msgs::msg::JointState;
using Trigger = std_srvs::srv::Trigger;
using Return = controller_interface::return_type;
using CallbackReturn = Controller::CallbackReturn;

constexpr double kPeriod = 0.001;
const Positions kInitial{0.0, -0.4, 0.0, -1.5, 0.0, 1.0, 0.0};
const Positions kHome{0.4, -0.4, 0.0, -1.5, 0.0, 1.0, 0.0};

class JointPositionControllerTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    static std::atomic_uint counter{0};
    const auto ns =
        "/franka_controller_test/case_" + std::to_string(counter.fetch_add(1));
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    controller_ = std::make_unique<Controller>();
    ASSERT_EQ(controller_->init("controller", ns, options), Return::OK);
    setParameters({
        rclcpp::Parameter("reset_joint_position",
                          std::vector<double>(kHome.begin(), kHome.end())),
        rclcpp::Parameter("max_velocity", std::vector<double>(7, 1.0)),
        rclcpp::Parameter("reset_timeout", 0.5),
    });
    ASSERT_EQ(controller_->configure().id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    assignInterfaces();

    peer_ = std::make_shared<rclcpp::Node>("peer", ns, options);
    publisher_ = peer_->create_publisher<JointState>(
        ns + "/controller/command", rclcpp::QoS(1).reliable());
    reset_client_ =
        peer_->create_client<Trigger>(ns + "/controller/reset_ctrl");
    executor_.add_node(controller_->get_node()->get_node_base_interface());
    executor_.add_node(peer_);
    ASSERT_TRUE(reset_client_->wait_for_service(2s));
    ASSERT_TRUE(waitUntil(
        [this] { return publisher_->get_subscription_count() == 1; }));
  }

  void TearDown() override {
    if (controller_) {
      controller_->on_deactivate(rclcpp_lifecycle::State{});
    }
    stopExecutor();
    if (controller_) {
      controller_->release_interfaces();
    }
  }

  void setParameters(const std::vector<rclcpp::Parameter> &parameters) {
    for (const auto &result :
         controller_->get_node()->set_parameters(parameters)) {
      ASSERT_TRUE(result.successful) << result.reason;
    }
  }

  void assignInterfaces(std::size_t command_count = 7,
                        std::size_t state_count = 7) {
    controller_->release_interfaces();
    commands_storage_.clear();
    states_storage_.clear();
    commands_storage_.reserve(7);
    states_storage_.reserve(7);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (std::size_t i = 0; i < command_count; ++i) {
      commands_storage_.emplace_back("fr3_joint" + std::to_string(i + 1),
                                     "position", &commands_[i]);
      commands.emplace_back(commands_storage_.back());
    }
    for (std::size_t i = 0; i < state_count; ++i) {
      states_storage_.emplace_back("fr3_joint" + std::to_string(i + 1),
                                   "position", &feedback_[i]);
      states.emplace_back(states_storage_.back());
    }
    controller_->assign_interfaces(std::move(commands), std::move(states));
  }

  void activate() {
    ASSERT_EQ(controller_->get_node()->activate().id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_EQ(tick(), Return::OK);
  }

  Return tick() {
    time_ns_ += 1000000;
    return controller_->update(rclcpp::Time(time_ns_),
                               rclcpp::Duration::from_seconds(kPeriod));
  }

  void runTicks(int count = 3000) {
    for (int i = 0; i < count; ++i) {
      ASSERT_EQ(tick(), Return::OK);
    }
  }

  JointState messageFor(const Positions &positions) {
    JointState message;
    message.position.assign(positions.begin(), positions.end());
    for (int i = 1; i <= 7; ++i) {
      message.name.push_back("fr3_joint" + std::to_string(i));
    }
    return message;
  }

  void send(const JointState &message) {
    // Intra-process delivery lets spin_some execute the real subscription
    // synchronously. Never call this while the service executor is spinning.
    ASSERT_FALSE(executor_thread_.joinable());
    publisher_->publish(message);
    executor_.spin_some();
  }

  void expectPositions(const Positions &expected) {
    for (std::size_t i = 0; i < expected.size(); ++i) {
      EXPECT_NEAR(commands_[i], expected[i], 1e-9) << "joint " << i;
    }
  }

  template <typename Predicate> bool waitUntil(Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    do {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  void startExecutor() {
    executor_thread_ = std::thread([this] { executor_.spin(); });
  }

  void stopExecutor() {
    if (executor_thread_.joinable()) {
      executor_.cancel();
      executor_thread_.join();
    }
  }

  std::unique_ptr<Controller> controller_;
  Positions feedback_{kInitial};
  Positions commands_{};
  std::vector<hardware_interface::CommandInterface> commands_storage_;
  std::vector<hardware_interface::StateInterface> states_storage_;
  rclcpp::Node::SharedPtr peer_;
  rclcpp::Publisher<JointState>::SharedPtr publisher_;
  rclcpp::Client<Trigger>::SharedPtr reset_client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread executor_thread_;
  int64_t time_ns_{0};
};

TEST_F(JointPositionControllerTest,
       ClaimsSevenNamedInterfacesAndHoldsFeedback) {
  const auto command = controller_->command_interface_configuration();
  const auto state = controller_->state_interface_configuration();
  EXPECT_EQ(command.type,
            controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(state.type, command.type);
  ASSERT_EQ(command.names.size(), 7u);
  EXPECT_EQ(command.names, state.names);
  for (std::size_t i = 0; i < command.names.size(); ++i) {
    EXPECT_EQ(command.names[i],
              "fr3_joint" + std::to_string(i + 1) + "/position");
  }
  activate();
  runTicks();
  expectPositions(kInitial);
}

TEST_F(JointPositionControllerTest,
       ReordersTargetsAndRespectsTrajectoryLimits) {
  activate();
  auto target = kInitial;
  for (std::size_t i = 0; i < target.size(); ++i) {
    target[i] += 0.1;
  }
  auto message = messageFor(target);
  std::reverse(message.name.begin(), message.name.end());
  std::reverse(message.position.begin(), message.position.end());
  send(message);
  Positions previous = commands_;
  Positions previous_velocity{};
  for (int step = 0; step < 3000; ++step) {
    ASSERT_EQ(tick(), Return::OK);
    for (std::size_t i = 0; i < target.size(); ++i) {
      const double velocity = (commands_[i] - previous[i]) / kPeriod;
      const double acceleration = (velocity - previous_velocity[i]) / kPeriod;
      EXPECT_TRUE(std::isfinite(commands_[i]));
      EXPECT_GE(commands_[i], kInitial[i] - 1e-9);
      EXPECT_LE(commands_[i], target[i] + 1e-9);
      EXPECT_LE(std::abs(velocity), 0.5 + 1e-6);
      EXPECT_LE(std::abs(acceleration),
                franka::kMaxJointAcceleration[i] * 0.5 + 1e-5);
      previous_velocity[i] = velocity;
    }
    previous = commands_;
  }
  expectPositions(target);
}

TEST_F(JointPositionControllerTest, RejectsMalformedAndUnsafeTargets) {
  activate();
  auto target = kInitial;
  target[0] = 0.1;
  const auto valid = messageFor(target);
  std::vector<JointState> invalid;
  auto message = valid;
  message.name.pop_back();
  invalid.push_back(message);
  message = valid;
  message.position.pop_back();
  invalid.push_back(message);
  message = valid;
  message.name[0] = "unknown_joint";
  invalid.push_back(message);
  message = valid;
  message.name[0] = message.name[1];
  invalid.push_back(message);
  for (const double value :
       {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), 4.0, -4.0}) {
    message = valid;
    message.position[0] = value;
    invalid.push_back(message);
  }
  for (std::size_t i = 0; i < invalid.size(); ++i) {
    SCOPED_TRACE(i);
    send(invalid[i]);
    runTicks();
    expectPositions(kInitial);
  }
  // Positive control: the same publisher and callback path accept valid data.
  send(valid);
  runTicks();
  expectPositions(target);
}

TEST_F(JointPositionControllerTest, RejectsLargeTargetStep) {
  activate();
  send(messageFor(kHome));
  runTicks();
  expectPositions(kInitial);
  auto target = kInitial;
  target[0] += 0.1;
  send(messageFor(target));
  runTicks();
  expectPositions(target);
}

TEST_F(JointPositionControllerTest,
       ReactivationDiscardsOldAndInactiveCommands) {
  activate();
  auto target = kInitial;
  target[0] += 0.1;
  send(messageFor(target));
  runTicks(50);
  ASSERT_GT(commands_[0], kInitial[0]);
  ASSERT_EQ(controller_->get_node()->deactivate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  send(messageFor(target));
  feedback_[0] = -0.1;
  activate();
  runTicks();
  expectPositions(feedback_);
}

TEST_F(JointPositionControllerTest, RejectsIncompleteInterfaces) {
  for (const auto &sizes :
       {std::pair<std::size_t, std::size_t>{6, 7}, {7, 6}}) {
    assignInterfaces(sizes.first, sizes.second);
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}),
              CallbackReturn::SUCCESS);
    EXPECT_EQ(tick(), Return::ERROR);
    controller_->on_deactivate(rclcpp_lifecycle::State{});
  }
}

TEST_F(JointPositionControllerTest, RejectsNonFiniteInitialFeedback) {
  for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
    feedback_[0] = value;
    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}),
              CallbackReturn::SUCCESS);
    EXPECT_EQ(tick(), Return::ERROR);
    controller_->on_deactivate(rclcpp_lifecycle::State{});
  }
}

TEST_F(JointPositionControllerTest, RejectsInvalidConfiguration) {
  const std::vector<rclcpp::Parameter> invalid{
      rclcpp::Parameter("velocity_limit_scale", 0.0),
      rclcpp::Parameter("velocity_limit_scale",
                        std::numeric_limits<double>::quiet_NaN()),
      rclcpp::Parameter("acceleration_limit_scale", 1.1),
      rclcpp::Parameter("jerk_limit_scale", -0.1),
      rclcpp::Parameter("reset_timeout", 0.0),
      rclcpp::Parameter("max_target_step", 0.0),
      rclcpp::Parameter("max_velocity", std::vector<double>(6, 1.0)),
      rclcpp::Parameter("max_velocity", std::vector<double>(7, 0.0)),
      rclcpp::Parameter("lower_limits", std::vector<double>(7, 10.0)),
      rclcpp::Parameter("reset_joint_position", std::vector<double>(7, 10.0)),
  };
  for (const auto &parameter : invalid) {
    SCOPED_TRACE(parameter.get_name());
    const auto original =
        controller_->get_node()->get_parameter(parameter.get_name());
    setParameters({parameter});
    EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}),
              CallbackReturn::ERROR);
    setParameters({original});
  }
}

TEST_F(JointPositionControllerTest, ResetRejectsInactiveController) {
  startExecutor();
  auto future =
      reset_client_->async_send_request(std::make_shared<Trigger::Request>());
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("not active"), std::string::npos);
}

TEST_F(JointPositionControllerTest, ResetReachesHomeAndCanBeRequestedAgain) {
  activate();
  startExecutor();
  for (int request = 0; request < 2; ++request) {
    auto future = reset_client_->async_send_request(
        std::make_shared<Trigger::Request>());
    ASSERT_TRUE(waitUntil([&] {
      for (int i = 0; i < 50; ++i) {
        EXPECT_EQ(tick(), Return::OK);
      }
      return future.wait_for(0s) == std::future_status::ready;
    }));
    EXPECT_TRUE(future.get()->success);
    // Home exceeds max_target_step, which must not reject a reset command.
    expectPositions(kHome);
  }
  stopExecutor();
  auto target = kHome;
  target[0] -= 0.1;
  send(messageFor(target));
  runTicks();
  expectPositions(target);
}

TEST_F(JointPositionControllerTest,
       ResetDiscardsQueuedTargetsAndAcceptsFreshCommands) {
  activate();
  // Make both the service and the old intra-process waitable ready before
  // spinning. Humble executes services before intra-process waitables.
  publisher_->publish(messageFor(kInitial));
  auto future =
      reset_client_->async_send_request(std::make_shared<Trigger::Request>());
  ASSERT_EQ(future.wait_for(100ms), std::future_status::timeout);
  startExecutor();
  ASSERT_TRUE(waitUntil([this] {
    EXPECT_EQ(tick(), Return::OK);
    return commands_[0] > kInitial[0] + 1e-8;
  }));
  auto target = kHome;
  target[0] -= 0.1;
  publisher_->publish(messageFor(target));
  ASSERT_TRUE(waitUntil([&] {
    for (int i = 0; i < 50; ++i) {
      EXPECT_EQ(tick(), Return::OK);
    }
    return future.wait_for(0s) == std::future_status::ready;
  }));
  ASSERT_TRUE(future.get()->success);
  stopExecutor();
  executor_.spin_some();
  runTicks();
  expectPositions(kHome);

  ASSERT_TRUE(waitUntil(
      [this] { return publisher_->get_subscription_count() == 1; }));
  send(messageFor(target));
  runTicks();
  expectPositions(target);
}

TEST_F(JointPositionControllerTest,
       ResetTimeoutCancelsTargetBeforeUpdatesResume) {
  activate();
  startExecutor();
  auto future =
      reset_client_->async_send_request(std::make_shared<Trigger::Request>());
  ASSERT_EQ(future.wait_for(100ms), std::future_status::timeout);
  auto target = kInitial;
  target[0] += 0.1;
  publisher_->publish(messageFor(target));
  // No updates: the reset cannot reach home, and cancellation must time out
  // without deadlocking the service. The next update acknowledges
  // cancellation.
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  const auto response = future.get();
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("timed out"), std::string::npos);
  stopExecutor();
  executor_.spin_some();
  runTicks();
  expectPositions(kInitial);
  ASSERT_TRUE(waitUntil(
      [this] { return publisher_->get_subscription_count() == 1; }));
  send(messageFor(target));
  runTicks();
  expectPositions(target);
}

TEST_F(JointPositionControllerTest,
       DeactivationFailsPendingResetAndAllowsReactivation) {
  activate();
  startExecutor();
  auto future =
      reset_client_->async_send_request(std::make_shared<Trigger::Request>());
  ASSERT_TRUE(waitUntil([this] {
    EXPECT_EQ(tick(), Return::OK);
    return commands_[0] > kInitial[0] + 1e-8;
  }));
  auto target = kInitial;
  target[0] += 0.1;
  publisher_->publish(messageFor(target));
  ASSERT_EQ(controller_->get_node()->deactivate().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(future.wait_for(3s), std::future_status::ready);
  EXPECT_FALSE(future.get()->success);
  stopExecutor();
  executor_.spin_some();
  feedback_[0] = -0.1;
  activate();
  runTicks();
  expectPositions(feedback_);
}

} // namespace
