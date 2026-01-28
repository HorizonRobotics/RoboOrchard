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


import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robo_orchard_piper_msg_ros2.msg import PiperStatusMsg
from robo_orchard_piper_ros2.ros_bridge import (
    PiperLossError,
    create_piper,
    get_arm_ctrl_state,
    get_arm_ee_pose,
    get_arm_state,
    get_arm_status,
)


class PiperAlohaRawNode(Node):
    def __init__(self) -> None:
        """Initializes the PiperAlohaNode."""
        super().__init__("piper_aloha_raw")

        self.declare_parameter("can_port", "can0")

        self.can_port = (
            self.get_parameter("can_port").get_parameter_value().string_value
        )

        self.get_logger().info(f"can_port = {self.can_port}")

        # Create piper class and open CAN interface
        self.piper = create_piper(self.can_port)

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "joint_state", 1)
        self.master_joint_pub = self.create_publisher(
            JointState, "master/joint_state", 1
        )
        self.arm_status_pub = self.create_publisher(
            PiperStatusMsg, "status", 1
        )
        self.end_pose_pub = self.create_publisher(PoseStamped, "ee_pose", 1)

        self.timer = self.create_timer(1 / 200.0, self.publish_callback)

    def _check(self):
        if not self.piper.isOK():
            self.get_logger().error(f"{self.can_port} is loss, exit...")
            raise PiperLossError(f"{self.can_port} is loss, exit...")

    def publish_callback(self):
        self._check()

        arm_status = get_arm_status(self.piper)
        self.arm_status_pub.publish(arm_status)

        joint_state = get_arm_state(self.piper)
        joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_state)

        joint_state = get_arm_ctrl_state(self.piper)
        joint_state.header.stamp = self.get_clock().now().to_msg()
        self.master_joint_pub.publish(joint_state)

        ee_pose = get_arm_ee_pose(self.piper)
        ee_pose.header.stamp = self.get_clock().now().to_msg()
        self.end_pose_pub.publish(ee_pose)


def main(args=None):
    rclpy.init(args=args)
    node = PiperAlohaRawNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
