// Project RoboOrchard
//
// Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
// implied. See the License for the specific language governing
// permissions and limitations under the License.

#define ROBO_ORCHARD_ROBOTIQ_NO_MAIN
#include "robotiq_driver_node.cpp"

#include <gtest/gtest.h>

#include <atomic>

class RobotiqNode : public testing::TestWithParam<bool>
{
};

TEST_P(RobotiqNode, FakeServiceCommandAndStatusContract)
{
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_fake", true), rclcpp::Parameter("status_timeout_ms", 300)});
  if (GetParam())
  {
    options.append_parameter_override("auto_enable", true);
  }
  auto driver = std::make_shared<robo_orchard_robotiq_ros2::RobotiqDriver>(options);
  auto client = std::make_shared<rclcpp::Node>("robotiq_test_client");
  std::atomic<double> opening{-1.0};
  std::atomic<bool> online{false};
  std::atomic<bool> enabled{false};
  auto states = client->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [&opening](sensor_msgs::msg::JointState::ConstSharedPtr state)
      {
        EXPECT_EQ(state->name, std::vector<std::string>{"robotiq_gripper"});
        EXPECT_TRUE(state->velocity.empty());
        EXPECT_TRUE(state->effort.empty());
        opening = state->position.at(0);
      });
  auto status = client->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "status", 10,
      [&online, &enabled](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
      {
        for (const auto &item : message->status.at(0).values)
        {
          if (item.key == "online")
          {
            online = item.value == "true";
          }
          if (item.key == "motion_enabled")
          {
            enabled = item.value == "true";
          }
        }
      });
  auto commands = client->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 1);
  auto activate = client->create_client<std_srvs::srv::Trigger>("activate");
  auto stop = client->create_client<std_srvs::srv::Trigger>("stop");
  auto recover = client->create_client<std_srvs::srv::Trigger>("recover");
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(driver);
  executor.add_node(client);
  std::thread spin([&executor]() { executor.spin(); });
  const auto wait = [](auto condition)
  {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!condition() && std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(10ms);
    }
    return condition();
  };
  EXPECT_TRUE(activate->wait_for_service(3s));
  EXPECT_TRUE(wait([&]() { return online.load() && commands->get_subscription_count() > 0; }));
  EXPECT_FALSE(driver->set_parameter(rclcpp::Parameter("force_fraction", 0.2)).successful);
  const auto call = [](auto service)
  {
    auto future = service->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if (future.wait_for(3s) != std::future_status::ready)
    {
      return false;
    }
    return future.get()->success;
  };
  if (!GetParam())
  {
    EXPECT_FALSE(enabled.load());
    EXPECT_TRUE(call(activate));
  }
  EXPECT_TRUE(wait([&]() { return enabled.load(); }));
  sensor_msgs::msg::JointState command;
  command.name = {"robotiq_gripper"};
  command.position = {0.04};
  commands->publish(command);
  EXPECT_TRUE(wait([&]() { return std::abs(opening.load() - 0.04) < 0.0004; }));
  EXPECT_TRUE(call(stop));
  EXPECT_TRUE(wait([&]() { return !enabled.load(); }));
  command.position = {0.08};
  commands->publish(command);
  std::this_thread::sleep_for(100ms);
  EXPECT_FALSE(enabled.load());
  EXPECT_NEAR(opening.load(), 0.04, 0.0004);
  EXPECT_TRUE(call(recover));
  EXPECT_TRUE(call(stop));
  executor.cancel();
  spin.join();
  executor.remove_node(driver);
  driver.reset();
  rclcpp::shutdown();
}

INSTANTIATE_TEST_SUITE_P(StartupEnable, RobotiqNode, testing::Bool());

TEST(RobotiqStartup, FailedAutoEnableStaysDisabledWithoutRetry)
{
  using namespace std::chrono_literals;
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  // The activation deadline cannot cover the next 100 ms SDK exchange.
  options.parameter_overrides({rclcpp::Parameter("use_fake", true),
                               rclcpp::Parameter("auto_enable", true),
                               rclcpp::Parameter("exchange_rate_hz", 10.0),
                               rclcpp::Parameter("activation_timeout_ms", 1)});
  auto driver = std::make_shared<robo_orchard_robotiq_ros2::RobotiqDriver>(options);
  auto client = std::make_shared<rclcpp::Node>("robotiq_startup_test_client");
  bool completed = false;
  int disabled_samples = 0;
  auto status = client->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "status", 10,
      [&](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
      {
        bool busy = false;
        for (const auto &item : message->status.at(0).values)
        {
          if (item.key == "operation_busy")
          {
            busy = item.value == "true";
          }
          if (item.key == "motion_enabled")
          {
            EXPECT_EQ(item.value, "false");
          }
        }
        if (completed)
        {
          EXPECT_FALSE(busy);
        }
        if (!busy)
        {
          completed = true;
          ++disabled_samples;
        }
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(driver);
  executor.add_node(client);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(completed);
  EXPECT_GT(disabled_samples, 10);
  executor.remove_node(driver);
  driver.reset();
  rclcpp::shutdown();
}
