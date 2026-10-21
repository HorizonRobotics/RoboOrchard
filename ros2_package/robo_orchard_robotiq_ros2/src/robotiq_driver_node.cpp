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
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <thread>

namespace robo_orchard_robotiq_ros2
{

class RosLogger final : public Robotiq::Logger
{
public:
  explicit RosLogger(rclcpp::Logger logger) : logger_(logger) {}
  void log(Level level, std::string_view message) override
  {
    const std::string text(message);
    switch (level)
    {
    case Level::Debug:
      RCLCPP_DEBUG(logger_, "%s", text.c_str());
      break;
    case Level::Info:
      RCLCPP_INFO(logger_, "%s", text.c_str());
      break;
    case Level::Warn:
      RCLCPP_WARN(logger_, "%s", text.c_str());
      break;
    case Level::Error:
      RCLCPP_ERROR(logger_, "%s", text.c_str());
      break;
    }
  }

private:
  rclcpp::Logger logger_;
};

class RobotiqDriver final : public rclcpp::Node
{
public:
  explicit RobotiqDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("robotiq_driver", options)
  {
    Robotiq::ConnectionConfig config;
    config.serial.port = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    const auto baud = declare_parameter<int>("baud_rate", 115200);
    const auto slave = declare_parameter<int>("slave_address", 9);
    const auto serial_timeout = declare_parameter<int>("serial_timeout_ms", 500);
    config.connectionFrequency = declare_parameter<double>("exchange_rate_hz", 100.0);
    joint_name_ = declare_parameter<std::string>("joint_name", "robotiq_gripper");
    const auto speed = declare_parameter<double>("speed_mps", 0.05);
    const auto force = declare_parameter<double>("force_fraction", 0.1);
    const auto rate = declare_parameter<double>("state_rate_hz", 30.0);
    const auto stale = declare_parameter<int>("status_timeout_ms", 1000);
    const auto activation_timeout = declare_parameter<int>("activation_timeout_ms", 15000);
    const bool fake = declare_parameter<bool>("use_fake", false);
    const bool auto_enable = declare_parameter<bool>("auto_enable", false);
    if (baud <= 0 || slave < 1 || slave > 247 || serial_timeout <= 0 || joint_name_.empty() ||
        !std::isfinite(speed) || speed < 0.02 || speed > 0.15 || !std::isfinite(force) ||
        force < 0.0 || force > 1.0 || !std::isfinite(rate) || rate <= 0.0 || rate > 1000.0 ||
        !std::isfinite(config.connectionFrequency) || config.connectionFrequency < 0.1 ||
        config.connectionFrequency > 1000.0 || stale <= 0 || activation_timeout <= 0 ||
        activation_timeout > 60000)
    {
      throw std::invalid_argument("Invalid Robotiq startup parameters");
    }
    config.serial.baudrate = static_cast<uint32_t>(baud);
    config.modbusSlaveAddress = static_cast<uint8_t>(slave);
    config.serial.timeout = std::chrono::milliseconds(serial_timeout);
    status_timeout_ = std::chrono::milliseconds(stale);
    activation_timeout_ = std::chrono::milliseconds(activation_timeout);
    speed_ = Robotiq::units::speedToRegister(speed, Robotiq::profiles::k2F85).value();
    force_ = Robotiq::units::effortToRegister(force).value();
    parameter_guard_ = add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> &)
        {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason = "Driver parameters are startup-only";
          return result;
        });
    auto logger = std::make_shared<RosLogger>(get_logger());
    gripper_ = fake ? Robotiq::makeFakeGripper(config, logger)
                    : std::make_unique<Robotiq::Gripper>(config, logger);

    // The SDK starts exchanging an image seeded from the device on
    // construction. Stop it immediately and confirm before exposing services.
    // This constructor is the sole writer until ownership passes to run().
    if (!Robotiq::setCommandAndWaitForExchange(*gripper_, stop_command(gripper_->getCommand()),
                                               status_timeout_))
    {
      throw std::runtime_error("Initial stop was not confirmed; hardware state unknown");
    }

    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("status", 10);
    command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands", rclcpp::QoS(1),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message)
        {
          const auto position = command_position(*message, joint_name_);
          std::lock_guard<std::mutex> lock(mutex_);
          if (!position || !enabled_ || busy_ || !fresh())
          {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Rejected command: invalid, disabled, busy or offline");
            return;
          }
          pending_position_ = position;
          wake_.notify_one();
        });
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    activate_service_ = service("activate", Operation::Activate);
    recover_service_ = service("recover", Operation::Recover);
    stop_service_ = service("stop", Operation::Stop);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   std::chrono::duration<double>(1.0 / rate)),
                               [this]() { publish_status(); });
    if (auto_enable)
    {
      RCLCPP_WARN(get_logger(), "Auto-enable requested; activation may fully open/close the fingers");
      busy_ = true;
      operation_ = Operation::Activate;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Connected%s; motion disabled until activate is requested",
                  fake ? " to fake device" : "");
    }
    worker_ = std::thread([this]() { run(); });
  }

  ~RobotiqDriver() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      quitting_ = true;
      enabled_ = false;
      pending_position_.reset();
    }
    wake_.notify_one();
    if (worker_.joinable())
    {
      worker_.join();
    }
  }

private:
  enum class Operation
  {
    Activate,
    Recover,
    Stop
  };
  using Trigger = std_srvs::srv::Trigger;

  bool fresh() const
  {
    return exchange_fresh(gripper_->getMostRecentStampedExchange(), gripper_->connectionState(),
                          std::chrono::steady_clock::now(), status_timeout_);
  }

  rclcpp::Service<Trigger>::SharedPtr service(const std::string &name, Operation operation)
  {
    return create_service<Trigger>(
        name,
        [this, operation](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
        {
          std::future<bool> completion;
          {
            std::lock_guard<std::mutex> lock(mutex_);
            if (busy_ || quitting_ || (operation != Operation::Stop && !fresh()))
            {
              response->success = false;
              response->message = "Busy, shutting down or offline";
              return;
            }
            enabled_ = false;
            busy_ = true;
            pending_position_.reset();
            operation_ = operation;
            completion = result_.get_future();
          }
          wake_.notify_one();
          response->success = completion.get();
          response->message = response->success
                                  ? "Operation completed"
                                  : "Operation failed; motion remains disabled; inspect status";
        },
        rmw_qos_profile_services_default, service_group_);
  }

  // Only this worker calls SDK command-mutating APIs, including their waits.
  // A handshake never shares the SDK command image with a ROS command callback.
  void run()
  {
    bool stopped_for_offline = false;
    while (true)
    {
      std::optional<Operation> operation;
      std::optional<uint8_t> position;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait_for(lock, std::chrono::milliseconds(10),
                       [this]() {
                         return quitting_ || operation_.has_value() ||
                                pending_position_.has_value();
                       });
        if (quitting_)
        {
          break;
        }
        operation = operation_;
        operation_.reset();
        position = pending_position_;
        pending_position_.reset();
      }
      if (!fresh())
      {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          enabled_ = false;
        }
        if (!stopped_for_offline)
        {
          gripper_->setCommand(stop_command(gripper_->getCommand()));
          stopped_for_offline = true;
        }
        position.reset();
      }
      else
      {
        stopped_for_offline = false;
      }
      if (operation)
      {
        bool success = false;
        try
        {
          if (*operation == Operation::Stop)
          {
            success = Robotiq::setCommandAndWaitForExchange(
                          *gripper_, stop_command(gripper_->getCommand()), status_timeout_)
                          .has_value();
          }
          else
          {
            success = activate_stopped(*gripper_, *operation == Operation::Recover, status_timeout_,
                                       activation_timeout_) &&
                      fresh();
          }
        }
        catch (const std::exception &error)
        {
          gripper_->setCommand(stop_command(gripper_->getCommand()));
          RCLCPP_ERROR(get_logger(), "Operation failed: %s", error.what());
        }
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = success && *operation != Operation::Stop && !quitting_;
        if (!success)
        {
          RCLCPP_ERROR(get_logger(), "Operation failed; motion remains disabled; inspect status");
        }
        else
        {
          RCLCPP_INFO(get_logger(), "Operation completed; motion %s", enabled_ ? "enabled" : "disabled");
        }
        busy_ = false;
        result_.set_value(success);
        result_ = std::promise<bool>();
      }
      else if (position)
      {
        // Re-check under the lock: a service may have disabled the node after
        // the worker took a pending position.
        std::lock_guard<std::mutex> lock(mutex_);
        const auto status = gripper_->getStatus();
        if (enabled_ && !busy_ && fresh() &&
            status.gripperStatus.activationState() == Robotiq::ActivationState::Complete &&
            status.faultStatus.gripperFault() == Robotiq::GripperFault::None &&
            status.faultStatus.controllerFault() == Robotiq::ControllerFault::None)
        {
          auto command = stop_command(gripper_->getCommand());
          command.action.set(Robotiq::ActionRequestBit::GoTo);
          command.positionRequest = *position;
          command.speed = speed_;
          command.force = force_;
          gripper_->setCommand(command);
        }
      }
    }
    // Keep rACT set and never auto-release on exit. Best effort only when
    // unplugged: the stop image cannot guarantee a hardware stop without a reply.
    const auto stopped = Robotiq::setCommandAndWaitForExchange(
        *gripper_, stop_command(gripper_->getCommand()), status_timeout_);
    if (!stopped)
    {
      RCLCPP_WARN(get_logger(), "Shutdown stop unconfirmed; hardware state is unknown");
    }
  }

  void publish_status()
  {
    const auto exchange = gripper_->getMostRecentStampedExchange();
    const bool online = exchange_fresh(exchange, gripper_->connectionState(),
                                       std::chrono::steady_clock::now(), status_timeout_);
    bool enabled;
    bool busy;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      enabled = enabled_;
      busy = busy_;
    }
    const auto &status = exchange.status;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus diagnostic;
    diagnostic.name = std::string(get_namespace()) + "/robotiq";
    diagnostic.hardware_id = joint_name_;
    diagnostic.level = online ? diagnostic_msgs::msg::DiagnosticStatus::OK
                              : diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnostic.message = online ? "Online" : "Offline or stale; cached fields are not live";
    if (online && (status.faultStatus.gripperFault() != Robotiq::GripperFault::None ||
                   status.faultStatus.controllerFault() != Robotiq::ControllerFault::None))
    {
      diagnostic.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnostic.message = "Device fault";
    }
    const auto add = [&diagnostic](const std::string &key, const std::string &value)
    {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      diagnostic.values.push_back(item);
    };
    add("online", online ? "true" : "false");
    add("motion_enabled", online && enabled ? "true" : "false");
    add("operation_busy", busy ? "true" : "false");
    add("exchange_count", std::to_string(exchange.metadata.exchangeCount));
    if (online)
    {
      add("activated", status.gripperStatus.activated() ? "true" : "false");
      add("activation_state",
          std::to_string(static_cast<int>(status.gripperStatus.activationState())));
      add("gripper_fault", std::to_string(static_cast<int>(status.faultStatus.gripperFault())));
      add("controller_fault",
          std::to_string(static_cast<int>(status.faultStatus.controllerFault())));
      add("current_a", std::to_string(Robotiq::units::motorCurrentFromRegister(status.current)));
      add("go_to", status.gripperStatus.goToEnabled() ? "true" : "false");
      add("object_detection",
          status.gripperStatus.goToEnabled()
              ? std::to_string(static_cast<int>(status.gripperStatus.objectDetection()))
              : "unknown");
      // Only publish position from a newly completed live exchange. No fabricated
      // zero velocity or current-as-effort; JointState stamp estimates reception.
      if (exchange.metadata.exchangeCount != published_exchange_)
      {
        sensor_msgs::msg::JointState state;
        const auto age = std::chrono::steady_clock::now() - exchange.metadata.timestamp;
        state.header.stamp =
            now() - rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(age));
        state.name = {joint_name_};
        state.position = {
            Robotiq::units::openingFromRegister(status.position, Robotiq::profiles::k2F85).value()};
        state_pub_->publish(state);
        published_exchange_ = exchange.metadata.exchangeCount;
      }
    }
    diagnostics.status.push_back(diagnostic);
    diagnostic_pub_->publish(diagnostics);
  }

  std::unique_ptr<Robotiq::Gripper> gripper_;
  std::string joint_name_;
  uint8_t speed_{};
  uint8_t force_{};
  std::chrono::milliseconds status_timeout_{1000};
  std::chrono::milliseconds activation_timeout_{15000};
  uint64_t published_exchange_{0};
  std::mutex mutex_;
  std::condition_variable wake_;
  bool quitting_{false};
  bool enabled_{false};
  bool busy_{false};
  std::optional<uint8_t> pending_position_;
  std::optional<Operation> operation_;
  std::promise<bool> result_;
  std::thread worker_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
  rclcpp::Service<Trigger>::SharedPtr activate_service_;
  rclcpp::Service<Trigger>::SharedPtr recover_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
};

} // namespace robo_orchard_robotiq_ros2

#ifndef ROBO_ORCHARD_ROBOTIQ_NO_MAIN
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<robo_orchard_robotiq_ros2::RobotiqDriver>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("robotiq_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
