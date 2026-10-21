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

#include "command.hpp"

#include <Robotiq/gripper/fake/gripper_factory.hpp>
#include <gtest/gtest.h>

#include <limits>

namespace
{
using namespace std::chrono_literals;
using robo_orchard_robotiq_ros2::command_position;
using robo_orchard_robotiq_ros2::exchange_fresh;
using robo_orchard_robotiq_ros2::stop_command;
} // namespace

TEST(RobotiqDriver, OpeningDirectionAndCommandValidation)
{
  sensor_msgs::msg::JointState message;
  message.name = {"gripper"};
  message.position = {0.085};
  ASSERT_TRUE(command_position(message, "gripper"));
  EXPECT_EQ(*command_position(message, "gripper"), 3);
  message.position = {0.0};
  EXPECT_EQ(*command_position(message, "gripper"), 230);
  message.position = {0.1};
  EXPECT_FALSE(command_position(message, "gripper"));
  message.position = {-0.01};
  EXPECT_FALSE(command_position(message, "gripper"));
  message.position = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_FALSE(command_position(message, "gripper"));
  message.position = {0.04};
  EXPECT_FALSE(command_position(message, "other"));
  message.velocity = {0.0};
  EXPECT_FALSE(command_position(message, "gripper"));
  message.velocity.clear();
  message.effort = {0.1};
  EXPECT_FALSE(command_position(message, "gripper"));
  message.effort.clear();
  message.name.push_back("other");
  EXPECT_FALSE(command_position(message, "gripper"));
}

TEST(RobotiqDriver, RejectsCachedAndFaultedStatus)
{
  Robotiq::StampedExchange exchange;
  const auto now = std::chrono::steady_clock::now();
  exchange.metadata.timestamp = now;
  EXPECT_FALSE(exchange_fresh(exchange, Robotiq::ConnectionState::Operational, now, 1s));
  exchange.metadata.exchangeCount = 1;
  EXPECT_TRUE(exchange_fresh(exchange, Robotiq::ConnectionState::Operational, now, 1s));
  EXPECT_FALSE(exchange_fresh(exchange, Robotiq::ConnectionState::Faulted, now, 1s));
  EXPECT_FALSE(exchange_fresh(exchange, Robotiq::ConnectionState::Operational, now + 2s, 1s));
}

TEST(RobotiqDriver, FakeActivationMoveStopAndRecoveryHandshake)
{
  auto gripper = Robotiq::makeFakeGripper({}, std::make_shared<Robotiq::NullLogger>());
  ASSERT_EQ(Robotiq::activate(*gripper, 1s), Robotiq::ActivationResult::Activated);
  auto command = gripper->getCommand();
  command.action.set(Robotiq::ActionRequestBit::GoTo);
  command.positionRequest =
      Robotiq::units::openingToRegister(0.04, Robotiq::profiles::k2F85).value();
  command.speed = 40;
  command.force = 20;
  const auto moved = Robotiq::setCommandAndWaitForExchange(*gripper, command, 1s);
  ASSERT_TRUE(moved);
  EXPECT_EQ(moved->status.position, command.positionRequest);
  const auto stop = stop_command(command);
  const auto stopped = Robotiq::setCommandAndWaitForExchange(*gripper, stop, 1s);
  ASSERT_TRUE(stopped);
  EXPECT_TRUE(stopped->status.gripperStatus.activated());
  EXPECT_FALSE(stopped->status.gripperStatus.goToEnabled());
  EXPECT_EQ(stop.positionRequest, command.positionRequest);
  EXPECT_EQ(Robotiq::recoverFromFault(*gripper, 1s), Robotiq::ActivationResult::Activated);
  EXPECT_EQ(Robotiq::activate(*gripper, 1s), Robotiq::ActivationResult::AlreadyActive);
}

TEST(RobotiqDriver, ZeroTimeoutOperationsLeaveStoppedImage)
{
  Robotiq::ConnectionConfig config;
  // A slower fake exchange keeps the confirmation boundary observable.
  config.connectionFrequency = 10.0;
  auto gripper = Robotiq::makeFakeGripper(config, std::make_shared<Robotiq::NullLogger>());
  ASSERT_EQ(Robotiq::activate(*gripper, 2s), Robotiq::ActivationResult::Activated);
  auto command = gripper->getCommand();
  command.action.set(Robotiq::ActionRequestBit::GoTo);
  command.positionRequest = 100;
  ASSERT_TRUE(Robotiq::setCommandAndWaitForExchange(*gripper, command, 2s));
  // Stop exchange succeeds, then recovery's zero timeout fails before its
  // handshake. This covers failure cleanup, not a mid-handshake disconnect
  // or the SDK restore-previous-command branch (the fake has no fault injection).
  EXPECT_FALSE(robo_orchard_robotiq_ros2::activate_stopped(*gripper, true, 2s, 0ms));
  const auto stopped = gripper->getCommand();
  EXPECT_EQ(stopped, robo_orchard_robotiq_ros2::stop_command(stopped));
  const auto recovered_exchange = gripper->waitForExchange(2s);
  ASSERT_TRUE(recovered_exchange);
  EXPECT_EQ(recovered_exchange->command, stopped);
  EXPECT_FALSE(recovered_exchange->status.gripperStatus.goToEnabled());

  // A stop confirmation itself timing out still replaces the moving image.
  gripper->setCommand(command);
  EXPECT_FALSE(robo_orchard_robotiq_ros2::activate_stopped(*gripper, false, 0ms, 0ms));
  EXPECT_EQ(gripper->getCommand(), robo_orchard_robotiq_ros2::stop_command(command));
  const auto next = gripper->waitForExchange(2s);
  ASSERT_TRUE(next);
  EXPECT_EQ(next->command, robo_orchard_robotiq_ros2::stop_command(command));
}
