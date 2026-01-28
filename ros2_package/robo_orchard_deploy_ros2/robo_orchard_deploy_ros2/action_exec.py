# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.

from rclpy.node import Node
from sensor_msgs.msg import JointState

from robo_orchard_deploy_ros2.config import DeployConfig


class ActionExecutor:
    def __init__(self, node: Node, config: DeployConfig):
        self._node = node
        self._config = config
        self._initialize()

    def _initialize(self):
        self._left_arm_publisher = self._node.create_publisher(
            JointState,
            self._config.control_config.left_arm_control_topic,
            1,
        )
        self._right_arm_publisher = self._node.create_publisher(
            JointState,
            self._config.control_config.right_arm_control_topic,
            1,
        )

    def _send_joint_action(self, joint_position, publisher):
        if joint_position is None:
            self._node.get_logger().warn(
                "Joint position is None, skipping action send."
            )

        goal_msg = JointState()
        goal_msg.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.name = self._config.robot_config.joint_names
        goal_msg.position = joint_position
        goal_msg.velocity = self._config.robot_config.joint_velocities
        goal_msg.effort = self._config.robot_config.joint_efforts
        publisher.publish(goal_msg)

    def send_action(self, action, action_index):
        self._send_joint_action(
            action.get("left_arm_actions", [])[action_index],
            self._left_arm_publisher,
        )
        self._send_joint_action(
            action.get("right_arm_actions", [])[action_index],
            self._right_arm_publisher,
        )
