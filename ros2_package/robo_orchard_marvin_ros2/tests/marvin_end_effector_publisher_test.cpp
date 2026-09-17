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

#include "marvin_end_effector_publisher.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace
{

using namespace std::chrono_literals;
using robo_orchard_marvin_ros2::EndEffectorPose;
using robo_orchard_marvin_ros2::MarvinEndEffectorPublisher;
using robo_orchard_marvin_ros2::MarvinForwardKinematics;
using robo_orchard_marvin_ros2::MarvinJointArray;

constexpr std::array<EndEffectorPose, 2> kFeedbackPoses = {{
  {{0.123, -0.45, 0.67}, {0.0, 0.0, 0.6, 0.8}},
  {{-0.321, 0.54, -0.76}, {0.5, -0.5, 0.5, 0.5}},
}};

void expect_pose_matches(
  const geometry_msgs::msg::PoseStamped & message, const EndEffectorPose & pose,
  const std::string & base_frame_id, const rclcpp::Time & stamp)
{
  EXPECT_EQ(message.header.frame_id, base_frame_id);
  EXPECT_EQ(rclcpp::Time(message.header.stamp), stamp);
  EXPECT_DOUBLE_EQ(message.pose.position.x, pose.position_m[0]);
  EXPECT_DOUBLE_EQ(message.pose.position.y, pose.position_m[1]);
  EXPECT_DOUBLE_EQ(message.pose.position.z, pose.position_m[2]);
  EXPECT_DOUBLE_EQ(message.pose.orientation.x, pose.orientation_xyzw[0]);
  EXPECT_DOUBLE_EQ(message.pose.orientation.y, pose.orientation_xyzw[1]);
  EXPECT_DOUBLE_EQ(message.pose.orientation.z, pose.orientation_xyzw[2]);
  EXPECT_DOUBLE_EQ(message.pose.orientation.w, pose.orientation_xyzw[3]);
}

class MarvinEndEffectorPublisherTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    receiver_ = std::make_shared<rclcpp::Node>("marvin_ee_receiver", options);
    executor_.add_node(receiver_);
    const std::array<std::string, 2> sides{"left", "right"};
    for (std::size_t arm_index = 0; arm_index < sides.size(); ++arm_index) {
      pose_subscriptions_[arm_index] =
        receiver_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot/" + sides[arm_index] + "/ee_pose", 10,
        [this, arm_index](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
          poses_[arm_index].push_back(*message);
        });
    }
    tf_subscription_ = receiver_->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        transforms_.insert(
          transforms_.end(), message->transforms.begin(), message->transforms.end());
      });
  }

  bool configure(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    options.parameter_overrides(parameters);
    source_ = std::make_shared<rclcpp::Node>(
      "marvin_ee_source", "/driver_namespace", options);
    publisher_ = std::make_unique<MarvinEndEffectorPublisher>(*source_);
    const std::size_t tf_count =
      source_->get_parameter("publish_ee_tf").as_bool() ? 1 : 0;
    return wait_until(
      [this, tf_count]() {
        return pose_subscriptions_[0]->get_publisher_count() == 1 &&
        pose_subscriptions_[1]->get_publisher_count() == 1 &&
        tf_subscription_->get_publisher_count() == tf_count;
      });
  }

  bool wait_until(
    const std::function<bool()> & condition,
    std::chrono::milliseconds timeout = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    do {
      executor_.spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  void expect_arm_feedback(
    std::size_t arm_index, const std::string & base_frame_id,
    const std::string & ee_frame_id, const rclcpp::Time & stamp)
  {
    ASSERT_EQ(poses_[arm_index].size(), 1u);
    const auto & pose = poses_[arm_index].front();
    expect_pose_matches(pose, kFeedbackPoses[arm_index], base_frame_id, stamp);
    const auto transform = std::find_if(
      transforms_.begin(), transforms_.end(),
      [&ee_frame_id](const geometry_msgs::msg::TransformStamped & candidate) {
        return candidate.child_frame_id == ee_frame_id;
      });
    ASSERT_NE(transform, transforms_.end());
    EXPECT_EQ(transform->header, pose.header);
    EXPECT_DOUBLE_EQ(transform->transform.translation.x, pose.pose.position.x);
    EXPECT_DOUBLE_EQ(transform->transform.translation.y, pose.pose.position.y);
    EXPECT_DOUBLE_EQ(transform->transform.translation.z, pose.pose.position.z);
    EXPECT_EQ(transform->transform.rotation, pose.pose.orientation);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr receiver_;
  rclcpp::Node::SharedPtr source_;
  std::unique_ptr<MarvinEndEffectorPublisher> publisher_;
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 2>
  pose_subscriptions_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_subscription_;
  std::array<std::vector<geometry_msgs::msg::PoseStamped>, 2> poses_;
  std::vector<geometry_msgs::msg::TransformStamped> transforms_;
};

TEST_F(MarvinEndEffectorPublisherTest, DefaultDualArmTransformsMatchPoseAndStamp) {
  ASSERT_TRUE(configure());
  const rclcpp::Time stamp(123, 456789, RCL_ROS_TIME);
  for (std::size_t arm_index = 0; arm_index < kFeedbackPoses.size(); ++arm_index) {
    publisher_->publish(arm_index, kFeedbackPoses[arm_index], stamp);
  }
  ASSERT_TRUE(
    wait_until(
      [this]() {
        return poses_[0].size() == 1 && poses_[1].size() == 1 &&
        transforms_.size() == 2;
      }));
  expect_arm_feedback(0, "robot_stand", "TCP_Link_L", stamp);
  expect_arm_feedback(1, "robot_stand", "TCP_Link_R", stamp);
}

TEST_F(MarvinEndEffectorPublisherTest, ConfiguredFramesOnlyRelabelFeedback) {
  ASSERT_TRUE(
    configure(
    {
      rclcpp::Parameter("left_base_frame_id", "rig/left_reference"),
      rclcpp::Parameter("left_ee_frame_id", "rig/left_flange"),
      rclcpp::Parameter("right_base_frame_id", "rig/right_reference"),
      rclcpp::Parameter("right_ee_frame_id", "rig/right_flange"),
    }));
  const rclcpp::Time stamp(234, 987654321, RCL_ROS_TIME);
  for (std::size_t arm_index = 0; arm_index < kFeedbackPoses.size(); ++arm_index) {
    publisher_->publish(arm_index, kFeedbackPoses[arm_index], stamp);
  }
  ASSERT_TRUE(
    wait_until(
      [this]() {
        return poses_[0].size() == 1 && poses_[1].size() == 1 &&
        transforms_.size() == 2;
      }));
  expect_arm_feedback(0, "rig/left_reference", "rig/left_flange", stamp);
  expect_arm_feedback(1, "rig/right_reference", "rig/right_flange", stamp);
}

TEST_F(MarvinEndEffectorPublisherTest, DisablingTfPreservesBothPoseTopicsAndFrames) {
  ASSERT_TRUE(
    configure(
    {
      rclcpp::Parameter("publish_ee_tf", false),
      rclcpp::Parameter("left_base_frame_id", "left_reference"),
      rclcpp::Parameter("right_base_frame_id", "right_reference"),
    }));
  const rclcpp::Time stamp(345, 123456789, RCL_ROS_TIME);
  for (std::size_t arm_index = 0; arm_index < kFeedbackPoses.size(); ++arm_index) {
    publisher_->publish(arm_index, kFeedbackPoses[arm_index], stamp);
  }
  ASSERT_TRUE(
    wait_until(
      [this]() {
        return poses_[0].size() == 1 && poses_[1].size() == 1;
      }));
  expect_pose_matches(poses_[0].front(), kFeedbackPoses[0], "left_reference", stamp);
  expect_pose_matches(poses_[1].front(), kFeedbackPoses[1], "right_reference", stamp);
  EXPECT_EQ(tf_subscription_->get_publisher_count(), 0u);
  EXPECT_FALSE(wait_until([this]() {return !transforms_.empty();}, 100ms));
}

TEST_F(MarvinEndEffectorPublisherTest, MissingOrInvalidFkPublishesNeitherPoseNorTf) {
  ASSERT_TRUE(configure());
  const MarvinForwardKinematics kinematics(MARVIN_KINEMATICS_CONFIG);
  MarvinJointArray invalid_joints{};
  invalid_joints[3] = std::numeric_limits<double>::quiet_NaN();
  const auto invalid_pose = kinematics.forward(1, invalid_joints);
  ASSERT_FALSE(invalid_pose.has_value());

  const rclcpp::Time stamp(456, 7654321, RCL_ROS_TIME);
  publisher_->publish(0, std::nullopt, stamp);
  publisher_->publish(1, invalid_pose, stamp);
  EXPECT_FALSE(
    wait_until(
      [this]() {
        return !poses_[0].empty() || !poses_[1].empty() || !transforms_.empty();
      }, 100ms));

  publisher_->publish(1, kFeedbackPoses[1], stamp);
  ASSERT_TRUE(
    wait_until(
      [this]() {
        return poses_[1].size() == 1 && transforms_.size() == 1;
      }));
  EXPECT_TRUE(poses_[0].empty());
  expect_arm_feedback(1, "robot_stand", "TCP_Link_R", stamp);
}

TEST_F(MarvinEndEffectorPublisherTest, RejectsEmptyFramesEvenWithTfDisabled) {
  for (const bool publish_ee_tf : {true, false}) {
    for (const std::string parameter_name : {
        "left_base_frame_id", "left_ee_frame_id",
        "right_base_frame_id", "right_ee_frame_id"})
    {
      for (const std::string frame_id : {"", " \t\r\n"}) {
        SCOPED_TRACE(parameter_name);
        rclcpp::NodeOptions options;
        options.parameter_overrides(
          {
            rclcpp::Parameter("publish_ee_tf", publish_ee_tf),
            rclcpp::Parameter(parameter_name, frame_id),
          });
        rclcpp::Node node("invalid_marvin_frames", options);
        EXPECT_THROW(MarvinEndEffectorPublisher{node}, std::invalid_argument);
      }
    }
  }
}

TEST_F(MarvinEndEffectorPublisherTest, RejectsSelfTransformsEvenWithTfDisabled) {
  for (const bool publish_ee_tf : {true, false}) {
    for (const std::string side : {"left", "right"}) {
      SCOPED_TRACE(side);
      rclcpp::NodeOptions options;
      options.parameter_overrides(
        {
          rclcpp::Parameter("publish_ee_tf", publish_ee_tf),
          rclcpp::Parameter(side + "_base_frame_id", "same_frame"),
          rclcpp::Parameter(side + "_ee_frame_id", "same_frame"),
        });
      rclcpp::Node node("invalid_marvin_frames", options);
      EXPECT_THROW(MarvinEndEffectorPublisher{node}, std::invalid_argument);
    }
  }
}

TEST_F(MarvinEndEffectorPublisherTest, RejectsDuplicateChildrenEvenWithTfDisabled) {
  for (const bool publish_ee_tf : {true, false}) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
      {
        rclcpp::Parameter("publish_ee_tf", publish_ee_tf),
        rclcpp::Parameter("left_ee_frame_id", "same_child"),
        rclcpp::Parameter("right_ee_frame_id", "same_child"),
      });
    rclcpp::Node node("invalid_marvin_frames", options);
    EXPECT_THROW(MarvinEndEffectorPublisher{node}, std::invalid_argument);
  }
}

}  // namespace
