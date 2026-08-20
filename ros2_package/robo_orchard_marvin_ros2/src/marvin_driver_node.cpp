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

#include "marvin_forward_kinematics.hpp"

#include "MarvinSDK.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <robo_orchard_marvin_msg_ros2/srv/set_control_mode.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace
{

using namespace std::chrono_literals;

constexpr std::size_t kArmCount = 2;
constexpr std::size_t kJointCount = 7;
constexpr std::size_t kToolKinematicsCount = 6;
constexpr std::size_t kToolDynamicsCount = 10;
constexpr double kPi = 3.14159265358979323846;
constexpr double kToolParameterTolerance = 1e-4;
constexpr std::array<unsigned char, 4> kMarvinControllerIp{192, 168, 1, 190};
constexpr char kEndEffectorPoseFrame[] = "robot_stand";
constexpr std::int64_t kVersionFamilyDivisor = 1000;
constexpr int kControllerVersionReadAttempts = 5;

using JointArray = std::array<double, kJointCount>;
using ToolKinematicsArray = std::array<double, kToolKinematicsCount>;
using ToolDynamicsArray = std::array<double, kToolDynamicsCount>;
using SetControlMode =
  robo_orchard_marvin_msg_ros2::srv::SetControlMode;
using Trigger = std_srvs::srv::Trigger;

enum class ControlMode : std::uint8_t
{
  kIdle = SetControlMode::Request::IDLE,
  kPosition = SetControlMode::Request::POSITION,
  kJointImpedance = SetControlMode::Request::JOINT_IMPEDANCE,
  kJointDrag = SetControlMode::Request::JOINT_DRAG,
  kUnknown = SetControlMode::Request::UNKNOWN,
};

double degrees_to_radians(double value)
{
  return value * kPi / 180.0;
}

double radians_to_degrees(double value)
{
  return value * 180.0 / kPi;
}

bool versions_are_compatible(std::int64_t controller_version, std::int64_t sdk_version)
{
  return controller_version > 0 && sdk_version > 0 &&
         controller_version / kVersionFamilyDivisor ==
         sdk_version / kVersionFamilyDivisor;
}

std::optional<std::int64_t> read_controller_version()
{
  std::array<char, 30> parameter_name{};
  constexpr char kVersionParameter[] = "VERSION";
  std::copy_n(
    kVersionParameter, sizeof(kVersionParameter) - 1, parameter_name.begin());

  for (int attempt = 0; attempt < kControllerVersionReadAttempts; ++attempt) {
    // long, not std::int64_t: OnGetIntPara takes a long* out-parameter.
    long controller_version = 0;  // NOLINT(runtime/int)
    if (
      OnGetIntPara(parameter_name.data(), &controller_version) == 0 &&
      controller_version > 0)
    {
      return controller_version;
    }
    std::this_thread::sleep_for(10ms);
  }
  return std::nullopt;
}

std::string mode_name(ControlMode mode)
{
  switch (mode) {
    case ControlMode::kIdle:
      return "idle";
    case ControlMode::kPosition:
      return "position";
    case ControlMode::kJointImpedance:
      return "joint_impedance";
    case ControlMode::kJointDrag:
      return "joint_drag";
    case ControlMode::kUnknown:
      return "unknown";
  }
  return "unknown";
}

std::optional<ControlMode> mode_from_value(std::uint8_t value)
{
  switch (value) {
    case SetControlMode::Request::IDLE:
      return ControlMode::kIdle;
    case SetControlMode::Request::POSITION:
      return ControlMode::kPosition;
    case SetControlMode::Request::JOINT_IMPEDANCE:
      return ControlMode::kJointImpedance;
    case SetControlMode::Request::JOINT_DRAG:
      return ControlMode::kJointDrag;
    default:
      return std::nullopt;
  }
}

std::optional<ControlMode> mode_from_parameter(const std::string & value)
{
  if (value == "position") {
    return ControlMode::kPosition;
  }
  if (value == "joint_impedance") {
    return ControlMode::kJointImpedance;
  }
  if (value == "joint_drag") {
    return ControlMode::kJointDrag;
  }
  return std::nullopt;
}

JointArray vector_to_joint_array(
  const std::vector<double> & values, const std::string & parameter_name)
{
  if (values.size() != kJointCount) {
    throw std::invalid_argument(parameter_name + " must contain seven values");
  }
  JointArray result{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    if (!std::isfinite(values[index])) {
      throw std::invalid_argument(parameter_name + " must contain finite values");
    }
    result[index] = values[index];
  }
  return result;
}

ToolDynamicsArray vector_to_tool_dynamics_array(
  const std::vector<double> & values, const std::string & parameter_name)
{
  if (values.size() != kToolDynamicsCount) {
    throw std::invalid_argument(parameter_name + " must contain ten values");
  }
  ToolDynamicsArray result{};
  for (std::size_t index = 0; index < kToolDynamicsCount; ++index) {
    if (!std::isfinite(values[index])) {
      throw std::invalid_argument(parameter_name + " must contain finite values");
    }
    result[index] = values[index];
  }
  return result;
}

template<std::size_t Size>
std::string format_parameters(const std::array<double, Size> & values)
{
  std::ostringstream stream;
  stream << '[';
  for (std::size_t index = 0; index < Size; ++index) {
    if (index != 0) {
      stream << ", ";
    }
    stream << values[index];
  }
  stream << ']';
  return stream.str();
}

template<std::size_t Size>
bool parameters_match(
  const std::array<double, Size> & actual,
  const std::array<double, Size> & expected)
{
  for (std::size_t index = 0; index < Size; ++index) {
    if (std::abs(actual[index] - expected[index]) > kToolParameterTolerance) {
      return false;
    }
  }
  return true;
}

double smoothstep(double value)
{
  return value * value * (3.0 - 2.0 * value);
}

double maximum_joint_error(const JointArray & first, const JointArray & second)
{
  double maximum = 0.0;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    maximum = std::max(maximum, std::abs(first[index] - second[index]));
  }
  return maximum;
}

}  // namespace

class MarvinDriverNode final : public rclcpp::Node
{
public:
  MarvinDriverNode()
  : Node("marvin_driver_node")
  {
    auto_enable_side_ =
      declare_parameter<std::string>("auto_enable_side", "none");
    auto_enable_mode_name_ =
      declare_parameter<std::string>("auto_enable_mode", "");
    control_frequency_hz_ =
      declare_parameter<double>("control_frequency_hz", 200.0);
    feedback_stale_timeout_s_ =
      declare_parameter<double>("feedback_stale_timeout_s", 0.1);
    mode_switch_timeout_s_ =
      declare_parameter<double>("mode_switch_timeout_s", 2.0);
    sdk_log_enabled_ = declare_parameter<bool>("sdk_log_enabled", false);
    velocity_ratio_ = declare_parameter<int>("velocity_ratio", 10);
    acceleration_ratio_ = declare_parameter<int>("acceleration_ratio", 10);
    stiffness_ = vector_to_joint_array(
      declare_parameter<std::vector<double>>(
        "joint_stiffness", {5.0, 5.0, 5.0, 4.0, 3.0, 3.0, 2.0}),
      "joint_stiffness");
    damping_ = vector_to_joint_array(
      declare_parameter<std::vector<double>>(
        "joint_damping", {0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2}),
      "joint_damping");
    apply_tool_dynamics_on_startup_ =
      declare_parameter<bool>("apply_tool_dynamics_on_startup", false);
    tool_dynamics_[0] = vector_to_tool_dynamics_array(
      declare_parameter<std::vector<double>>(
        "left_tool_dynamics", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "left_tool_dynamics");
    tool_dynamics_[1] = vector_to_tool_dynamics_array(
      declare_parameter<std::vector<double>>(
        "right_tool_dynamics", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "right_tool_dynamics");
    reset_targets_[0] = vector_to_joint_array(
      declare_parameter<std::vector<double>>(
        "left_reset_joint_position", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "left_reset_joint_position");
    reset_targets_[1] = vector_to_joint_array(
      declare_parameter<std::vector<double>>(
        "right_reset_joint_position", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "right_reset_joint_position");
    reset_duration_s_ = declare_parameter<double>("reset_duration_s", 10.0);
    reset_goal_tolerance_rad_ =
      declare_parameter<double>("reset_goal_tolerance_rad", degrees_to_radians(1.0));
    reset_timeout_s_ = declare_parameter<double>("reset_timeout_s", 15.0);

    validate_parameters();
    configure_arms();
    const auto package_share =
      ament_index_cpp::get_package_share_directory("robo_orchard_marvin_ros2");
    forward_kinematics_ =
      std::make_unique<robo_orchard_marvin_ros2::MarvinForwardKinematics>(
      package_share + "/config/ccs_m6_40.MvKDCfg");
    control_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    reset_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    configure_ros_interfaces();

    const auto period = std::chrono::duration<double>(1.0 / control_frequency_hz_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MarvinDriverNode::control_tick, this), control_callback_group_);

    std::lock_guard<std::mutex> lock(sdk_mutex_);
    std::string message;
    if (!connect_robot_unlocked(message)) {
      RCLCPP_ERROR(get_logger(), "Marvin startup connection failed: %s", message.c_str());
      throw std::runtime_error(message);
    }
    if (!auto_enable_unlocked(message)) {
      RCLCPP_ERROR(get_logger(), "Marvin auto-enable failed: %s", message.c_str());
    }
  }

  ~MarvinDriverNode() override
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    shutdown_robot_unlocked();
  }

private:
  struct ArmContext
  {
    char sdk_arm{'A'};
    std::size_t sdk_index{0};
    std::string side;
    std::vector<std::string> joint_names;

    JointArray position{};
    JointArray velocity{};
    JointArray effort{};
    JointArray latest_command{};
    ToolKinematicsArray tool_kinematics{};
    ToolDynamicsArray tool_dynamics{};

    int controller_state{ARM_STATE_IDLE};
    int impedance_type{0};
    int drag_space_type{0};
    int error_code{0};
    bool low_speed{false};
    bool feedback_valid{false};
    bool frame_seen{false};
    int last_frame_serial{0};
    std::chrono::steady_clock::time_point last_frame_change{};

    bool mode_switching{false};
    bool latest_command_valid{false};
    std::uint64_t latest_sequence{0};
    std::uint64_t sent_sequence{0};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr control_mode_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr controller_state_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr impedance_type_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_code_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub;
    rclcpp::Service<SetControlMode>::SharedPtr set_mode_service;
    rclcpp::Service<Trigger>::SharedPtr reset_service;
    rclcpp::Service<Trigger>::SharedPtr clear_error_service;
    rclcpp::Service<Trigger>::SharedPtr emergency_stop_service;
  };

  struct ResetExecution
  {
    std::size_t arm_index{0};
    JointArray start{};
    JointArray target{};
    std::chrono::steady_clock::time_point started_at{};
    bool final_target_sent{false};
    bool finished{false};
    bool success{false};
    std::string message;
  };

  void validate_parameters()
  {
    if (control_frequency_hz_ <= 0.0 || !std::isfinite(control_frequency_hz_)) {
      throw std::invalid_argument("control_frequency_hz must be positive");
    }
    if (
      feedback_stale_timeout_s_ <= 0.0 ||
      !std::isfinite(feedback_stale_timeout_s_))
    {
      throw std::invalid_argument("feedback_stale_timeout_s must be positive");
    }
    if (mode_switch_timeout_s_ <= 0.0 || !std::isfinite(mode_switch_timeout_s_)) {
      throw std::invalid_argument("mode_switch_timeout_s must be positive");
    }
    if (reset_duration_s_ <= 0.0 || !std::isfinite(reset_duration_s_)) {
      throw std::invalid_argument("reset_duration_s must be positive");
    }
    if (
      reset_goal_tolerance_rad_ <= 0.0 ||
      !std::isfinite(reset_goal_tolerance_rad_))
    {
      throw std::invalid_argument("reset_goal_tolerance_rad must be positive");
    }
    if (
      reset_timeout_s_ <= reset_duration_s_ ||
      !std::isfinite(reset_timeout_s_))
    {
      throw std::invalid_argument("reset_timeout_s must exceed reset_duration_s");
    }
    if (velocity_ratio_ < 0 || velocity_ratio_ > 100) {
      throw std::invalid_argument("velocity_ratio must be in [0, 100]");
    }
    if (acceleration_ratio_ < 0 || acceleration_ratio_ > 100) {
      throw std::invalid_argument("acceleration_ratio must be in [0, 100]");
    }
    sdk_velocity_ratio_ = std::max(1, velocity_ratio_);
    sdk_acceleration_ratio_ = std::max(1, acceleration_ratio_);
    if (velocity_ratio_ == 0 || acceleration_ratio_ == 0) {
      RCLCPP_WARN(
        get_logger(),
        "Marvin SDK clamps zero speed or acceleration ratios to one percent");
    }
    for (double value : stiffness_) {
      if (value < 0.0) {
        throw std::invalid_argument("joint_stiffness values must be non-negative");
      }
    }
    for (double value : damping_) {
      if (value < 0.0 || value > 1.0) {
        throw std::invalid_argument("joint_damping values must be in [0, 1]");
      }
    }
    if (apply_tool_dynamics_on_startup_) {
      for (std::size_t index = 0; index < tool_dynamics_.size(); ++index) {
        if (tool_dynamics_[index][0] <= 0.0) {
          throw std::invalid_argument(
                  std::string(index == 0 ? "left" : "right") +
                  "_tool_dynamics mass must be positive");
        }
      }
    }
    if (
      auto_enable_side_ != "none" && auto_enable_side_ != "left" &&
      auto_enable_side_ != "right" && auto_enable_side_ != "both")
    {
      throw std::invalid_argument(
              "auto_enable_side must be one of: none, left, right, both");
    }
    if (auto_enable_side_ != "none") {
      auto_enable_mode_ = mode_from_parameter(auto_enable_mode_name_);
      if (!auto_enable_mode_.has_value()) {
        throw std::invalid_argument(
                "auto_enable_mode must be position, joint_impedance, or joint_drag "
                "when auto_enable_side is set");
      }
    } else if (!auto_enable_mode_name_.empty()) {
      RCLCPP_WARN(
        get_logger(), "auto_enable_mode is ignored because auto_enable_side is none");
    }
  }

  void configure_arms()
  {
    arms_[0].sdk_arm = 'A';
    arms_[0].sdk_index = 0;
    arms_[0].side = "left";
    arms_[0].joint_names = declare_parameter<std::vector<std::string>>(
      "left_joint_names",
      {"Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L", "Joint5_L",
        "Joint6_L", "Joint7_L"});

    arms_[1].sdk_arm = 'B';
    arms_[1].sdk_index = 1;
    arms_[1].side = "right";
    arms_[1].joint_names = declare_parameter<std::vector<std::string>>(
      "right_joint_names",
      {"Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R", "Joint5_R",
        "Joint6_R", "Joint7_R"});

    for (const auto & arm : arms_) {
      if (arm.joint_names.size() != kJointCount) {
        throw std::invalid_argument(arm.side + "_joint_names must contain seven names");
      }
    }
  }

  void configure_ros_interfaces()
  {
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      auto & arm = arms_[index];
      const std::string prefix = "/robot/" + arm.side;
      arm.joint_state_pub = create_publisher<sensor_msgs::msg::JointState>(
        prefix + "/joint_state", 10);
      arm.ee_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
        prefix + "/ee_pose", 10);
      arm.control_mode_pub = create_publisher<std_msgs::msg::UInt8>(
        prefix + "/control_mode", 10);
      arm.controller_state_pub = create_publisher<std_msgs::msg::Int32>(
        prefix + "/controller_state", 10);
      arm.impedance_type_pub = create_publisher<std_msgs::msg::Int32>(
        prefix + "/impedance_type", 10);
      arm.error_code_pub = create_publisher<std_msgs::msg::Int32>(
        prefix + "/error_code", 10);
      arm.joint_command_sub = create_subscription<sensor_msgs::msg::JointState>(
        prefix + "/joint_cmd", rclcpp::QoS(1).reliable(),
        [this, index](sensor_msgs::msg::JointState::ConstSharedPtr message) {
          receive_joint_command(index, *message);
        });
      arm.set_mode_service = create_service<SetControlMode>(
        prefix + "/set_mode",
        [this, index](
          const SetControlMode::Request::SharedPtr request,
          SetControlMode::Response::SharedPtr response)
        {
          set_mode_service_callback(index, *request, *response);
        });
      arm.reset_service = create_service<Trigger>(
        prefix + "/reset_ctrl",
        [this, index](
          const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
        {
          reset_service_callback(index, *response);
        },
        rmw_qos_profile_services_default, reset_callback_group_);
      arm.clear_error_service = create_service<Trigger>(
        prefix + "/clear_error",
        [this, index](
          const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
        {
          clear_error_service_callback(index, *response);
        });
      arm.emergency_stop_service = create_service<Trigger>(
        prefix + "/emergency_stop",
        [this, index](
          const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
        {
          emergency_stop_service_callback(index, *response);
        });
    }
    reset_all_service_ = create_service<Trigger>(
      "/robot/reset_ctrl",
      [this](
        const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response)
      {
        reset_all_service_callback(*response);
      },
      rmw_qos_profile_services_default, reset_callback_group_);
  }

  ControlMode inferred_mode(const ArmContext & arm) const
  {
    if (arm.controller_state == ARM_STATE_IDLE) {
      return ControlMode::kIdle;
    }
    if (arm.controller_state == ARM_STATE_POSITION) {
      return ControlMode::kPosition;
    }
    if (arm.controller_state == ARM_STATE_TORQ && arm.impedance_type == 1) {
      if (arm.drag_space_type == 1) {
        return ControlMode::kJointDrag;
      }
      if (arm.drag_space_type == 0) {
        return ControlMode::kJointImpedance;
      }
    }
    return ControlMode::kUnknown;
  }

  bool reset_active_for_arm_unlocked(std::size_t arm_index) const
  {
    return reset_reserved_[arm_index];
  }

  void finish_reset_unlocked(
    const std::shared_ptr<ResetExecution> & reset, bool success,
    const std::string & message)
  {
    if (!reset || reset->finished) {
      return;
    }
    reset->finished = true;
    reset->success = success;
    reset->message = message;
    auto & arm = arms_[reset->arm_index];
    arm.latest_command_valid = false;
    arm.sent_sequence = arm.latest_sequence;
    auto & active_reset = active_resets_[reset->arm_index];
    if (active_reset == reset) {
      active_reset.reset();
    }
    reset_condition_.notify_all();
    if (success) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    }
  }

  bool validate_reset_unlocked(
    std::size_t arm_index, std::string & message) const
  {
    const auto & arm = arms_[arm_index];
    if (reset_active_for_arm_unlocked(arm_index)) {
      message = arm.side + " arm reset is already in progress";
      return false;
    }
    if (!arm.feedback_valid) {
      message = arm.side + " feedback is unavailable";
      return false;
    }
    if (arm.error_code != 0 || arm.controller_state == ARM_STATE_ERROR) {
      message = arm.side + " arm has an error; call clear_error explicitly";
      return false;
    }
    const ControlMode mode = inferred_mode(arm);
    if (mode != ControlMode::kPosition && mode != ControlMode::kJointImpedance) {
      message = arm.side + " reset requires position or joint_impedance mode";
      return false;
    }
    if (!arm.low_speed) {
      message = arm.side + " arm is moving; reset requires a stationary arm";
      return false;
    }
    return true;
  }

  std::shared_ptr<ResetExecution> start_reset_unlocked(
    std::size_t arm_index,
    const std::chrono::steady_clock::time_point & started_at)
  {
    auto & arm = arms_[arm_index];
    auto reset = std::make_shared<ResetExecution>();
    reset->arm_index = arm_index;
    reset->start = arm.position;
    reset->target = reset_targets_[arm_index];
    reset->started_at = started_at;
    active_resets_[arm_index] = reset;
    reset_reserved_[arm_index] = true;
    arm.latest_command_valid = false;
    arm.sent_sequence = arm.latest_sequence;
    RCLCPP_INFO(
      get_logger(), "Resetting %s arm in %s mode", arm.side.c_str(),
      mode_name(inferred_mode(arm)).c_str());
    return reset;
  }

  void wait_for_reset_unlocked(
    std::unique_lock<std::mutex> & lock,
    const std::shared_ptr<ResetExecution> & reset)
  {
    while (!reset->finished && rclcpp::ok()) {
      reset_condition_.wait_for(lock, 100ms);
    }
    if (!reset->finished) {
      const auto & arm = arms_[reset->arm_index];
      finish_reset_unlocked(
        reset, false, arm.side + " reset interrupted by shutdown");
    }
  }

  void reset_service_callback(
    std::size_t arm_index, Trigger::Response & response)
  {
    std::unique_lock<std::mutex> lock(sdk_mutex_);
    if (!connected_) {
      response.success = false;
      response.message = "Marvin SDK is not connected";
      return;
    }
    if (!read_feedback_unlocked(false)) {
      response.success = false;
      response.message = arms_[arm_index].side + " feedback is unavailable";
      return;
    }
    if (!validate_reset_unlocked(arm_index, response.message)) {
      response.success = false;
      return;
    }
    const auto reset = start_reset_unlocked(
      arm_index, std::chrono::steady_clock::now());
    wait_for_reset_unlocked(lock, reset);
    reset_reserved_[arm_index] = false;
    response.success = reset->success;
    response.message = reset->message;
  }

  void reset_all_service_callback(Trigger::Response & response)
  {
    std::unique_lock<std::mutex> lock(sdk_mutex_);
    if (!connected_) {
      response.success = false;
      response.message = "Marvin SDK is not connected";
      return;
    }
    if (!read_feedback_unlocked(false)) {
      response.success = false;
      response.message = "dual-arm feedback is unavailable";
      return;
    }
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      if (!validate_reset_unlocked(index, response.message)) {
        response.success = false;
        return;
      }
    }

    const auto started_at = std::chrono::steady_clock::now();
    std::array<std::shared_ptr<ResetExecution>, kArmCount> resets{};
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      resets[index] = start_reset_unlocked(index, started_at);
    }

    while (rclcpp::ok()) {
      const bool all_finished = std::all_of(
        resets.begin(), resets.end(),
        [](const auto & reset) {return reset->finished;});
      if (all_finished) {
        break;
      }
      reset_condition_.wait_for(lock, 100ms);
      for (std::size_t index = 0; index < resets.size(); ++index) {
        const auto & reset = resets[index];
        if (!reset->finished || reset->success) {
          continue;
        }
        for (std::size_t peer_index = 0; peer_index < resets.size(); ++peer_index) {
          if (peer_index == index || resets[peer_index]->finished) {
            continue;
          }
          finish_reset_unlocked(
            resets[peer_index], false,
            arms_[peer_index].side + " reset aborted because " +
            arms_[index].side + " reset failed");
        }
      }
    }
    for (const auto & reset : resets) {
      if (!reset->finished) {
        finish_reset_unlocked(
          reset, false,
          arms_[reset->arm_index].side + " reset interrupted by shutdown");
      }
    }
    reset_reserved_.fill(false);

    response.success = std::all_of(
      resets.begin(), resets.end(),
      [](const auto & reset) {return reset->success;});
    response.message =
      "left: " + resets[0]->message + "; right: " + resets[1]->message;
  }

  std::optional<JointArray> ordered_positions(
    const sensor_msgs::msg::JointState & message,
    const std::vector<std::string> & joint_names) const
  {
    JointArray positions{};
    if (message.name.empty()) {
      if (message.position.size() != kJointCount) {
        return std::nullopt;
      }
      for (std::size_t index = 0; index < kJointCount; ++index) {
        positions[index] = message.position[index];
      }
    } else {
      if (message.name.size() != message.position.size()) {
        return std::nullopt;
      }
      std::unordered_map<std::string, double> positions_by_name;
      for (std::size_t index = 0; index < message.name.size(); ++index) {
        positions_by_name[message.name[index]] = message.position[index];
      }
      for (std::size_t index = 0; index < kJointCount; ++index) {
        const auto iterator = positions_by_name.find(joint_names[index]);
        if (iterator == positions_by_name.end()) {
          return std::nullopt;
        }
        positions[index] = iterator->second;
      }
    }
    for (double value : positions) {
      if (!std::isfinite(value)) {
        return std::nullopt;
      }
    }
    return positions;
  }

  void receive_joint_command(
    std::size_t arm_index, const sensor_msgs::msg::JointState & message)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    auto & arm = arms_[arm_index];
    const ControlMode mode = inferred_mode(arm);
    if (
      !connected_ || !arm.feedback_valid || arm.mode_switching ||
      reset_active_for_arm_unlocked(arm_index) ||
      arm.error_code != 0 || arm.controller_state == ARM_STATE_ERROR ||
      (mode != ControlMode::kPosition && mode != ControlMode::kJointImpedance))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected %s joint command in mode %s", arm.side.c_str(),
        mode_name(mode).c_str());
      return;
    }
    const auto positions = ordered_positions(message, arm.joint_names);
    if (!positions.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected %s joint command: expected seven finite named or positional joints",
        arm.side.c_str());
      return;
    }
    arm.latest_command = *positions;
    arm.latest_command_valid = true;
    ++arm.latest_sequence;
  }

  void control_tick()
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    if (!connected_) {
      return;
    }
    if (!read_feedback_unlocked(true)) {
      for (std::size_t index = 0; index < active_resets_.size(); ++index) {
        if (active_resets_[index]) {
          finish_reset_unlocked(
            active_resets_[index], false,
            arms_[index].side + " reset aborted: feedback unavailable");
        }
      }
      return;
    }

    std::array<bool, kArmCount> should_send{};
    std::array<bool, kArmCount> reset_command{};
    std::array<JointArray, kArmCount> commands{};
    std::array<bool, kArmCount> final_reset_target_command{};
    auto resets = active_resets_;
    for (std::size_t index = 0; index < resets.size(); ++index) {
      auto & reset = resets[index];
      if (!reset) {
        continue;
      }
      auto & arm = arms_[index];
      const ControlMode mode = inferred_mode(arm);
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - reset->started_at).count();
      if (!arm.feedback_valid) {
        finish_reset_unlocked(
          reset, false, arm.side + " reset aborted: feedback became stale");
        reset.reset();
        continue;
      }
      if (arm.error_code != 0 || arm.controller_state == ARM_STATE_ERROR) {
        finish_reset_unlocked(
          reset, false, arm.side + " reset aborted: controller entered an error state");
        reset.reset();
        continue;
      }
      if (mode != ControlMode::kPosition && mode != ControlMode::kJointImpedance) {
        finish_reset_unlocked(
          reset, false, arm.side + " reset aborted: control mode changed to " +
          mode_name(mode));
        reset.reset();
        continue;
      }
      if (elapsed >= reset_timeout_s_) {
        finish_reset_unlocked(reset, false, arm.side + " reset timed out");
        reset.reset();
        continue;
      }
      if (
        reset->final_target_sent && arm.low_speed &&
        maximum_joint_error(arm.position, reset->target) <= reset_goal_tolerance_rad_)
      {
        finish_reset_unlocked(reset, true, arm.side + " arm reset successfully");
        reset.reset();
        continue;
      }
      const double progress = std::clamp(elapsed / reset_duration_s_, 0.0, 1.0);
      const double interpolation = smoothstep(progress);
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        commands[index][joint] =
          reset->start[joint] +
          (reset->target[joint] - reset->start[joint]) * interpolation;
      }
      final_reset_target_command[index] = progress >= 1.0;
      should_send[index] = true;
      reset_command[index] = true;
    }
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      auto & arm = arms_[index];
      if (reset_command[index]) {
        continue;
      }
      const ControlMode mode = inferred_mode(arm);
      should_send[index] =
        arm.latest_command_valid && arm.latest_sequence != arm.sent_sequence &&
        arm.feedback_valid && !arm.mode_switching && arm.error_code == 0 &&
        arm.controller_state != ARM_STATE_ERROR &&
        (mode == ControlMode::kPosition || mode == ControlMode::kJointImpedance);
      if (should_send[index]) {
        commands[index] = arm.latest_command;
      }
      if (!arm.feedback_valid || arm.error_code != 0) {
        arm.latest_command_valid = false;
      }
    }
    if (!should_send[0] && !should_send[1]) {
      return;
    }
    if (!OnClearSet()) {
      for (std::size_t index = 0; index < resets.size(); ++index) {
        if (reset_command[index] && resets[index]) {
          finish_reset_unlocked(
            resets[index], false,
            arms_[index].side + " reset aborted: SDK send buffer is busy");
        }
      }
      return;
    }
    bool staged = true;
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      if (!should_send[index]) {
        continue;
      }
      JointArray command_degrees{};
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        command_degrees[joint] = radians_to_degrees(commands[index][joint]);
      }
      staged = set_joint_command_sdk(index, command_degrees) && staged;
    }
    if (!staged || !OnSetSend()) {
      for (std::size_t index = 0; index < resets.size(); ++index) {
        if (reset_command[index] && resets[index]) {
          finish_reset_unlocked(
            resets[index], false,
            arms_[index].side + " reset aborted: SDK command send failed");
        }
      }
      return;
    }
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      if (reset_command[index] && resets[index]) {
        if (final_reset_target_command[index]) {
          resets[index]->final_target_sent = true;
        }
      } else if (should_send[index]) {
        arms_[index].sent_sequence = arms_[index].latest_sequence;
      }
    }
  }

  bool read_feedback_unlocked(bool publish)
  {
    DCSS state{};
    if (!OnGetBuf(&state)) {
      for (auto & arm : arms_) {
        arm.feedback_valid = false;
      }
      return false;
    }

    const auto steady_now = std::chrono::steady_clock::now();
    for (auto & arm : arms_) {
      const auto sdk_index = arm.sdk_index;
      arm.controller_state = state.m_State[sdk_index].m_CurState;
      arm.error_code = state.m_State[sdk_index].m_ERRCode;
      arm.impedance_type = state.m_In[sdk_index].m_ImpType;
      arm.drag_space_type = state.m_In[sdk_index].m_DragSpType;
      arm.low_speed = state.m_Out[sdk_index].m_LowSpdFlag == 1;
      for (std::size_t parameter = 0; parameter < kToolKinematicsCount; ++parameter) {
        arm.tool_kinematics[parameter] = state.m_In[sdk_index].m_ToolKine[parameter];
      }
      for (std::size_t parameter = 0; parameter < kToolDynamicsCount; ++parameter) {
        arm.tool_dynamics[parameter] = state.m_In[sdk_index].m_ToolDyn[parameter];
      }

      const int frame_serial = state.m_Out[sdk_index].m_OutFrameSerial;
      if (!arm.frame_seen) {
        if (frame_serial != 0) {
          arm.frame_seen = true;
          arm.last_frame_serial = frame_serial;
          arm.last_frame_change = steady_now;
        }
      } else if (frame_serial != arm.last_frame_serial) {
        arm.last_frame_serial = frame_serial;
        arm.last_frame_change = steady_now;
      }
      arm.feedback_valid =
        arm.frame_seen &&
        std::chrono::duration<double>(steady_now - arm.last_frame_change).count() <=
        feedback_stale_timeout_s_;

      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        arm.position[joint] = degrees_to_radians(
          state.m_Out[sdk_index].m_FB_Joint_Pos[joint]);
        arm.velocity[joint] = degrees_to_radians(
          state.m_Out[sdk_index].m_FB_Joint_Vel[joint]);
        arm.effort[joint] = state.m_Out[sdk_index].m_FB_Joint_SToq[joint];
      }
      if (arm.error_code != 0 || arm.controller_state == ARM_STATE_ERROR) {
        arm.latest_command_valid = false;
      }
    }
    if (publish) {
      publish_feedback_unlocked();
    }
    return true;
  }

  void log_tool_dynamics_unlocked(const std::string & description) const
  {
    for (const auto & arm : arms_) {
      RCLCPP_INFO(
        get_logger(), "%s %s ToolDyn=%s", description.c_str(), arm.side.c_str(),
        format_parameters(arm.tool_dynamics).c_str());
    }
  }

  bool apply_tool_dynamics_unlocked(std::string & message)
  {
    if (!apply_tool_dynamics_on_startup_) {
      return true;
    }
    for (std::size_t index = 0; index < arms_.size(); ++index) {
      auto & arm = arms_[index];
      if (parameters_match(arm.tool_dynamics, tool_dynamics_[index])) {
        RCLCPP_INFO(
          get_logger(), "%s ToolDyn already matches configured values", arm.side.c_str());
        continue;
      }

      auto preserved_kinematics = arm.tool_kinematics;
      auto configured_dynamics = tool_dynamics_[index];
      if (!SetTool(
          arm.sdk_arm, preserved_kinematics.data(), configured_dynamics.data()))
      {
        message = arm.side + " ToolDyn configuration failed";
        return false;
      }
      if (!read_feedback_unlocked(false) || !arm.feedback_valid) {
        message = arm.side + " feedback unavailable after ToolDyn configuration";
        return false;
      }
      if (!parameters_match(arm.tool_kinematics, preserved_kinematics)) {
        message = arm.side + " ToolKine changed while configuring ToolDyn";
        return false;
      }
      if (!parameters_match(arm.tool_dynamics, configured_dynamics)) {
        message = arm.side + " ToolDyn readback does not match configured values";
        return false;
      }
      RCLCPP_INFO(
        get_logger(), "Configured %s ToolDyn=%s while preserving ToolKine=%s",
        arm.side.c_str(), format_parameters(arm.tool_dynamics).c_str(),
        format_parameters(arm.tool_kinematics).c_str());
    }
    return true;
  }

  void publish_feedback_unlocked()
  {
    const auto stamp = get_clock()->now();
    for (auto & arm : arms_) {
      sensor_msgs::msg::JointState joint_state;
      joint_state.header.stamp = stamp;
      joint_state.name = arm.joint_names;
      joint_state.position.assign(arm.position.begin(), arm.position.end());
      joint_state.velocity.assign(arm.velocity.begin(), arm.velocity.end());
      joint_state.effort.assign(arm.effort.begin(), arm.effort.end());
      arm.joint_state_pub->publish(joint_state);

      if (arm.feedback_valid) {
        const auto pose = forward_kinematics_->forward(arm.sdk_index, arm.position);
        if (pose.has_value()) {
          geometry_msgs::msg::PoseStamped ee_pose;
          ee_pose.header.stamp = stamp;
          ee_pose.header.frame_id = kEndEffectorPoseFrame;
          ee_pose.pose.position.x = pose->position_m[0];
          ee_pose.pose.position.y = pose->position_m[1];
          ee_pose.pose.position.z = pose->position_m[2];
          ee_pose.pose.orientation.x = pose->orientation_xyzw[0];
          ee_pose.pose.orientation.y = pose->orientation_xyzw[1];
          ee_pose.pose.orientation.z = pose->orientation_xyzw[2];
          ee_pose.pose.orientation.w = pose->orientation_xyzw[3];
          arm.ee_pose_pub->publish(ee_pose);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Marvin FK failed for %s feedback", arm.side.c_str());
        }
      }

      std_msgs::msg::UInt8 control_mode;
      control_mode.data = static_cast<std::uint8_t>(inferred_mode(arm));
      arm.control_mode_pub->publish(control_mode);

      std_msgs::msg::Int32 controller_state;
      controller_state.data = arm.controller_state;
      arm.controller_state_pub->publish(controller_state);

      std_msgs::msg::Int32 impedance_type;
      impedance_type.data = arm.impedance_type;
      arm.impedance_type_pub->publish(impedance_type);

      std_msgs::msg::Int32 error_code;
      error_code.data = arm.error_code;
      arm.error_code_pub->publish(error_code);
    }
  }

  bool connect_robot_unlocked(std::string & message)
  {
    if (connected_) {
      message = "Marvin SDK is already connected";
      return true;
    }
    if (!OnLinkTo(
        kMarvinControllerIp[0], kMarvinControllerIp[1],
        kMarvinControllerIp[2], kMarvinControllerIp[3]))
    {
      message = "OnLinkTo failed; another process may own the fixed SDK ports";
      return false;
    }
    connected_ = true;
    if (sdk_log_enabled_) {
      OnLogOn();
      OnLocalLogOn();
    } else {
      OnLogOff();
      OnLocalLogOff();
    }

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (
        read_feedback_unlocked(false) && arms_[0].feedback_valid &&
        arms_[1].feedback_valid)
      {
        const std::int64_t sdk_version = OnGetSDKVersion();
        const auto controller_version = read_controller_version();
        if (!controller_version.has_value()) {
          OnRelease();
          connected_ = false;
          message = "failed to read controller VERSION parameter";
          return false;
        }
        if (!versions_are_compatible(*controller_version, sdk_version)) {
          OnRelease();
          connected_ = false;
          message =
            "controller version " + std::to_string(*controller_version) +
            " is incompatible with SDK version " + std::to_string(sdk_version);
          return false;
        }
        log_tool_dynamics_unlocked("Initial controller");
        if (!apply_tool_dynamics_unlocked(message)) {
          OnRelease();
          connected_ = false;
          return false;
        }
        publish_feedback_unlocked();
        message = "connected; no control mode selected";
        RCLCPP_INFO(
          get_logger(),
          "Marvin connected with SDK version %ld, controller version %ld; "
          "no mode selected",
          sdk_version, *controller_version);
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    OnRelease();
    connected_ = false;
    message = "connected socket but received no valid dual-arm feedback";
    return false;
  }

  bool auto_enable_unlocked(std::string & message)
  {
    if (auto_enable_side_ == "none") {
      message = "auto-enable disabled";
      return true;
    }

    std::vector<std::size_t> targets;
    if (auto_enable_side_ == "left" || auto_enable_side_ == "both") {
      targets.push_back(0);
    }
    if (auto_enable_side_ == "right" || auto_enable_side_ == "both") {
      targets.push_back(1);
    }

    std::vector<std::size_t> changed;
    for (std::size_t index : targets) {
      if (!set_mode_unlocked(index, *auto_enable_mode_, message)) {
        for (auto iterator = changed.rbegin(); iterator != changed.rend(); ++iterator) {
          std::string rollback_message;
          set_mode_unlocked(*iterator, ControlMode::kIdle, rollback_message);
        }
        return false;
      }
      changed.push_back(index);
    }
    message = "auto-enabled " + auto_enable_side_ + " in " +
      mode_name(*auto_enable_mode_) + " mode";
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return true;
  }

  void set_mode_service_callback(
    std::size_t arm_index, const SetControlMode::Request & request,
    SetControlMode::Response & response)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    const auto target_mode = mode_from_value(request.mode);
    if (!target_mode.has_value()) {
      response.success = false;
      response.current_mode = static_cast<std::uint8_t>(inferred_mode(arms_[arm_index]));
      response.message = "unsupported control mode";
      return;
    }
    response.success = set_mode_unlocked(arm_index, *target_mode, response.message);
    read_feedback_unlocked(true);
    response.current_mode = static_cast<std::uint8_t>(inferred_mode(arms_[arm_index]));
  }

  bool set_mode_unlocked(
    std::size_t arm_index, ControlMode target_mode, std::string & message)
  {
    auto & arm = arms_[arm_index];
    if (!connected_) {
      message = "Marvin SDK is not connected";
      return false;
    }
    if (reset_active_for_arm_unlocked(arm_index)) {
      message = arm.side + " arm reset is in progress";
      return false;
    }
    if (!read_feedback_unlocked(false) || !arm.feedback_valid) {
      message = arm.side + " feedback is unavailable";
      return false;
    }
    if (arm.error_code != 0 || arm.controller_state == ARM_STATE_ERROR) {
      message = arm.side + " arm has an error; call clear_error explicitly";
      return false;
    }
    if (!arm.low_speed) {
      message = arm.side + " arm is moving; mode switching requires a stationary arm";
      return false;
    }

    const ControlMode current_mode = inferred_mode(arm);
    arm.mode_switching = true;
    bool success = true;

    if (
      arm.drag_space_type != 0 &&
      !(target_mode == ControlMode::kJointDrag && current_mode == ControlMode::kJointDrag))
    {
      success = exit_drag_unlocked(arm_index, message);
    }

    if (success) {
      switch (target_mode) {
        case ControlMode::kIdle:
          success = transition_to_idle_unlocked(arm_index, message);
          break;
        case ControlMode::kPosition:
          success = transition_to_position_unlocked(arm_index, message);
          break;
        case ControlMode::kJointImpedance:
          success = transition_to_joint_impedance_unlocked(arm_index, message);
          break;
        case ControlMode::kJointDrag:
          if (current_mode == ControlMode::kJointDrag) {
            message = arm.side + " arm is already in joint_drag mode";
          } else {
            success = transition_to_joint_drag_unlocked(arm_index, message);
          }
          break;
        case ControlMode::kUnknown:
          success = false;
          message = "unknown mode cannot be requested";
          break;
      }
    }

    arm.mode_switching = false;
    arm.latest_command_valid = false;
    arm.sent_sequence = arm.latest_sequence;
    if (success) {
      RCLCPP_INFO(
        get_logger(), "%s arm entered %s mode", arm.side.c_str(),
        mode_name(target_mode).c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "%s arm failed to enter %s mode: %s", arm.side.c_str(),
        mode_name(target_mode).c_str(), message.c_str());
    }
    return success;
  }

  bool transition_to_idle_unlocked(std::size_t arm_index, std::string & message)
  {
    auto & arm = arms_[arm_index];
    if (arm.controller_state != ARM_STATE_IDLE) {
      if (!send_transaction_unlocked(
          arm.side + " idle transition",
          [this, arm_index]() {return set_target_state_sdk(arm_index, ARM_STATE_IDLE);},
          message))
      {
        return false;
      }
    }
    return wait_for_arm_unlocked(
      arm_index,
      [](const ArmContext & value) {
        return value.controller_state == ARM_STATE_IDLE;
      },
      "idle", message);
  }

  bool transition_to_position_unlocked(
    std::size_t arm_index, std::string & message)
  {
    auto & arm = arms_[arm_index];
    JointArray current_degrees{};
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      current_degrees[joint] = radians_to_degrees(arm.position[joint]);
    }
    if (!send_transaction_unlocked(
        arm.side + " position parameters",
        [this, arm_index, &current_degrees]() {
          return set_joint_limits_sdk(arm_index) &&
          set_joint_command_sdk(arm_index, current_degrees);
        },
        message))
    {
      return false;
    }
    std::this_thread::sleep_for(20ms);
    if (arm.controller_state != ARM_STATE_POSITION) {
      if (!send_transaction_unlocked(
          arm.side + " position mode",
          [this, arm_index]() {
            return set_target_state_sdk(arm_index, ARM_STATE_POSITION);
          },
          message))
      {
        return false;
      }
    }
    return wait_for_arm_unlocked(
      arm_index,
      [](const ArmContext & value) {
        return value.controller_state == ARM_STATE_POSITION;
      },
      "position", message);
  }

  bool transition_to_joint_impedance_unlocked(
    std::size_t arm_index, std::string & message)
  {
    auto & arm = arms_[arm_index];
    JointArray current_degrees{};
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      current_degrees[joint] = radians_to_degrees(arm.position[joint]);
    }
    if (!send_transaction_unlocked(
        arm.side + " joint impedance parameters",
        [this, arm_index, &current_degrees]() {
          return set_joint_limits_sdk(arm_index) &&
          set_joint_impedance_sdk(arm_index) &&
          set_joint_command_sdk(arm_index, current_degrees);
        },
        message))
    {
      return false;
    }
    std::this_thread::sleep_for(20ms);
    if (
      arm.controller_state != ARM_STATE_TORQ || arm.impedance_type != 1 ||
      arm.drag_space_type != 0)
    {
      if (!send_transaction_unlocked(
          arm.side + " joint impedance mode",
          [this, arm_index]() {
            return set_target_state_sdk(arm_index, ARM_STATE_TORQ) &&
            set_impedance_type_sdk(arm_index, 1);
          },
          message))
      {
        return false;
      }
    }
    return wait_for_arm_unlocked(
      arm_index,
      [](const ArmContext & value) {
        return value.controller_state == ARM_STATE_TORQ &&
        value.impedance_type == 1 && value.drag_space_type == 0;
      },
      "joint_impedance", message);
  }

  bool transition_to_joint_drag_unlocked(
    std::size_t arm_index, std::string & message)
  {
    if (!transition_to_joint_impedance_unlocked(arm_index, message)) {
      return false;
    }
    if (!send_transaction_unlocked(
        arms_[arm_index].side + " joint drag",
        [this, arm_index]() {return set_drag_space_sdk(arm_index, 1);},
        message))
    {
      return false;
    }
    return wait_for_arm_unlocked(
      arm_index,
      [](const ArmContext & value) {
        return value.controller_state == ARM_STATE_TORQ &&
        value.impedance_type == 1 && value.drag_space_type == 1;
      },
      "joint_drag", message);
  }

  bool exit_drag_unlocked(std::size_t arm_index, std::string & message)
  {
    if (!send_transaction_unlocked(
        arms_[arm_index].side + " exit drag",
        [this, arm_index]() {return set_drag_space_sdk(arm_index, 0);},
        message))
    {
      return false;
    }
    return wait_for_arm_unlocked(
      arm_index,
      [](const ArmContext & value) {return value.drag_space_type == 0;},
      "drag exit", message);
  }

  bool send_transaction_unlocked(
    const std::string & description, const std::function<bool()> & stage,
    std::string & message)
  {
    if (!OnClearSet()) {
      message = description + ": SDK send buffer is busy";
      return false;
    }
    if (!stage()) {
      message = description + ": failed to stage SDK command";
      return false;
    }
    if (!OnSetSend()) {
      message = description + ": failed to submit SDK command";
      return false;
    }
    std::this_thread::sleep_for(5ms);
    return true;
  }

  bool wait_for_arm_unlocked(
    std::size_t arm_index,
    const std::function<bool(const ArmContext &)> & predicate,
    const std::string & target_description, std::string & message,
    bool reject_error = true)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(mode_switch_timeout_s_));
    while (std::chrono::steady_clock::now() < deadline) {
      read_feedback_unlocked(false);
      const auto & arm = arms_[arm_index];
      if (
        reject_error &&
        (arm.error_code != 0 || arm.controller_state == ARM_STATE_ERROR))
      {
        message = arm.side + " arm entered an error state while waiting for " +
          target_description;
        return false;
      }
      if (arm.feedback_valid && predicate(arm)) {
        message = arm.side + " arm entered " + target_description;
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    const auto & arm = arms_[arm_index];
    message = arm.side + " arm timed out waiting for " + target_description +
      "; state=" + std::to_string(arm.controller_state) +
      ", impedance=" + std::to_string(arm.impedance_type) +
      ", drag=" + std::to_string(arm.drag_space_type);
    return false;
  }

  void clear_error_service_callback(
    std::size_t arm_index, Trigger::Response & response)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    auto & arm = arms_[arm_index];
    if (!connected_) {
      response.success = false;
      response.message = "Marvin SDK is not connected";
      return;
    }
    if (reset_active_for_arm_unlocked(arm_index)) {
      response.success = false;
      response.message = arm.side + " arm reset is in progress";
      return;
    }
    response.success = send_transaction_unlocked(
      arm.side + " clear error",
      [this, arm_index]() {
        if (arm_index == 0) {
          OnClearErr_A();
        } else {
          OnClearErr_B();
        }
        return true;
      },
      response.message);
    if (response.success) {
      response.success = wait_for_arm_unlocked(
        arm_index,
        [](const ArmContext & value) {
          return value.error_code == 0 && value.controller_state != ARM_STATE_ERROR;
        },
        "error clear", response.message, false);
    }
    arm.latest_command_valid = false;
    read_feedback_unlocked(true);
  }

  void emergency_stop_service_callback(
    std::size_t arm_index, Trigger::Response & response)
  {
    std::lock_guard<std::mutex> lock(sdk_mutex_);
    auto & arm = arms_[arm_index];
    if (!connected_) {
      response.success = false;
      response.message = "Marvin SDK is not connected";
      return;
    }
    if (reset_active_for_arm_unlocked(arm_index)) {
      finish_reset_unlocked(
        active_resets_[arm_index], false,
        arm.side + " reset interrupted by software emergency stop");
    }
    if (arm_index == 0) {
      OnEMG_A();
    } else {
      OnEMG_B();
    }
    arm.latest_command_valid = false;
    response.success = true;
    response.message = arm.side + " software emergency stop requested";
    std::this_thread::sleep_for(5ms);
    read_feedback_unlocked(true);
  }

  bool set_joint_limits_sdk(std::size_t arm_index)
  {
    return arm_index == 0 ?
           OnSetJointLmt_A(sdk_velocity_ratio_, sdk_acceleration_ratio_) :
           OnSetJointLmt_B(sdk_velocity_ratio_, sdk_acceleration_ratio_);
  }

  bool set_joint_impedance_sdk(std::size_t arm_index)
  {
    return arm_index == 0 ?
           OnSetJointKD_A(stiffness_.data(), damping_.data()) :
           OnSetJointKD_B(stiffness_.data(), damping_.data());
  }

  bool set_target_state_sdk(std::size_t arm_index, int state)
  {
    return arm_index == 0 ? OnSetTargetState_A(state) : OnSetTargetState_B(state);
  }

  bool set_impedance_type_sdk(std::size_t arm_index, int type)
  {
    return arm_index == 0 ? OnSetImpType_A(type) : OnSetImpType_B(type);
  }

  bool set_drag_space_sdk(std::size_t arm_index, int type)
  {
    return arm_index == 0 ? OnSetDragSpace_A(type) : OnSetDragSpace_B(type);
  }

  bool set_joint_command_sdk(std::size_t arm_index, JointArray & command_degrees)
  {
    return arm_index == 0 ?
           OnSetJointCmdPos_A(command_degrees.data()) :
           OnSetJointCmdPos_B(command_degrees.data());
  }

  void shutdown_robot_unlocked()
  {
    for (std::size_t index = 0; index < active_resets_.size(); ++index) {
      if (active_resets_[index]) {
        finish_reset_unlocked(
          active_resets_[index], false,
          arms_[index].side + " reset interrupted by shutdown");
      }
    }
    if (!connected_) {
      return;
    }
    read_feedback_unlocked(false);

    bool has_drag = false;
    for (const auto & arm : arms_) {
      has_drag = has_drag || arm.drag_space_type != 0;
    }
    if (has_drag && OnClearSet()) {
      bool staged = true;
      for (std::size_t index = 0; index < arms_.size(); ++index) {
        if (arms_[index].drag_space_type != 0) {
          staged = set_drag_space_sdk(index, 0) && staged;
        }
      }
      if (staged) {
        OnSetSend();
        std::this_thread::sleep_for(20ms);
      }
    }

    if (OnClearSet()) {
      bool staged = true;
      for (std::size_t index = 0; index < arms_.size(); ++index) {
        staged = set_target_state_sdk(index, ARM_STATE_IDLE) && staged;
      }
      if (staged) {
        OnSetSend();
        std::this_thread::sleep_for(20ms);
      }
    }
    OnRelease();
    connected_ = false;
  }

  std::array<ArmContext, kArmCount> arms_{};
  std::mutex sdk_mutex_;
  bool connected_{false};

  std::string auto_enable_side_;
  std::string auto_enable_mode_name_;
  std::optional<ControlMode> auto_enable_mode_;
  double control_frequency_hz_{200.0};
  double feedback_stale_timeout_s_{0.1};
  double mode_switch_timeout_s_{2.0};
  bool sdk_log_enabled_{false};
  int velocity_ratio_{10};
  int acceleration_ratio_{10};
  int sdk_velocity_ratio_{10};
  int sdk_acceleration_ratio_{10};
  JointArray stiffness_{};
  JointArray damping_{};
  bool apply_tool_dynamics_on_startup_{false};
  std::array<ToolDynamicsArray, kArmCount> tool_dynamics_{};
  std::array<JointArray, kArmCount> reset_targets_{};
  double reset_duration_s_{10.0};
  double reset_goal_tolerance_rad_{degrees_to_radians(1.0)};
  double reset_timeout_s_{15.0};
  std::array<std::shared_ptr<ResetExecution>, kArmCount> active_resets_{};
  std::array<bool, kArmCount> reset_reserved_{};
  std::condition_variable reset_condition_;
  std::unique_ptr<robo_orchard_marvin_ros2::MarvinForwardKinematics>
  forward_kinematics_;

  rclcpp::CallbackGroup::SharedPtr control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr reset_callback_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Service<Trigger>::SharedPtr reset_all_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MarvinDriverNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("marvin_driver_node"), "Marvin driver startup failed: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
