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

#ifndef ROBO_ORCHARD_ROBOTIQ_ROS2__COMMAND_HPP_
#define ROBO_ORCHARD_ROBOTIQ_ROS2__COMMAND_HPP_

#include <Robotiq/gripper.hpp>
#include <Robotiq/gripper/units.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cmath>
#include <optional>
#include <string>

namespace robo_orchard_robotiq_ros2
{

// A single virtual joint reports the full parallel-finger opening in metres.
// Reject out-of-range commands rather than silently saturating the SDK mapping.
inline std::optional<uint8_t> command_position(const sensor_msgs::msg::JointState &message,
                                               const std::string &joint_name)
{
  if (message.name.size() != 1 || message.name[0] != joint_name || message.position.size() != 1 ||
      !message.velocity.empty() || !message.effort.empty())
  {
    return std::nullopt;
  }
  const double opening = message.position[0];
  if (!std::isfinite(opening) || opening < 0.0 || opening > 0.085)
  {
    return std::nullopt;
  }
  return Robotiq::units::openingToRegister(opening, Robotiq::profiles::k2F85);
}

inline Robotiq::GripperCommand stop_command(Robotiq::GripperCommand command)
{
  command.action.set(Robotiq::ActionRequestBit::GoTo, false);
  command.action.set(Robotiq::ActionRequestBit::AutoRelease, false);
  return command;
}

// Both before and after a handshake keep a stop image on the SDK. In
// particular recoverFromFault restores its previous image on timeout.
inline bool activate_stopped(Robotiq::Gripper &gripper, bool recover,
                             std::chrono::milliseconds exchange_timeout,
                             std::chrono::milliseconds activation_timeout)
{
  bool activated = false;
  try
  {
    if (Robotiq::setCommandAndWaitForExchange(gripper, stop_command(gripper.getCommand()),
                                              exchange_timeout))
    {
      const auto result = recover ? Robotiq::recoverFromFault(gripper, activation_timeout)
                                  : Robotiq::activate(gripper, activation_timeout);
      activated = result == Robotiq::ActivationResult::Activated ||
                  result == Robotiq::ActivationResult::AlreadyActive;
    }
  }
  catch (...)
  {
    // Clear any image left by an interrupted SDK handshake before propagating.
    gripper.setCommand(stop_command(gripper.getCommand()));
    throw;
  }
  // This is unconditional even if the handshake timed out. A failed wait
  // still leaves the stop block installed for the next successful exchange.
  const bool stopped = Robotiq::setCommandAndWaitForExchange(
                           gripper, stop_command(gripper.getCommand()), exchange_timeout)
                           .has_value();
  return activated && stopped;
}

inline bool exchange_fresh(const Robotiq::StampedExchange &exchange,
                           Robotiq::ConnectionState connection,
                           std::chrono::steady_clock::time_point now,
                           std::chrono::milliseconds timeout)
{
  return connection == Robotiq::ConnectionState::Operational &&
         exchange.metadata.exchangeCount != 0 && now - exchange.metadata.timestamp <= timeout;
}

} // namespace robo_orchard_robotiq_ros2
#endif // ROBO_ORCHARD_ROBOTIQ_ROS2__COMMAND_HPP_
