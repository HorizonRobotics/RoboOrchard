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
from piper_sdk import C_PiperInterface
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robo_orchard_piper_msg_ros2.msg import PiperStatusMsg
from robo_orchard_piper_ros2.ros_bridge import (
    create_piper,
    enable_arm_ctrl,
    get_arm_ee_pose,
    get_arm_state,
    get_arm_status,
    joint_control,
    set_ctrl_method,
)


class RobotStatusError(Exception):
    pass


class PiperAlohaNode(Node):
    """A ROS 2 node for controlling a master-slave (Aloha style) robot arm setup.

    This node controls a master arm based on external commands and makes a slave
    arm mirror the master arm's movements in real-time. It publishes the status
    of both arms.
    """  # noqa: E501

    def __init__(self) -> None:
        """Initializes the PiperAlohaNode."""
        super().__init__("piper_aloha")

        # ROS parameters for master and slave arms
        self.declare_parameter("master_can_port", "can0")
        self.declare_parameter("slave_can_port", "can1")
        self.declare_parameter("gripper_exist", True)
        self.declare_parameter("gripper_val_mutiple", 1)
        self.declare_parameter("sync_frequency", 200.0)
        self.declare_parameter("enable_mit_ctrl", False)

        self.declare_parameter("enable_master_ctrl", False)
        self.declare_parameter("enable_master_mit_ctrl", False)

        self.master_can_port = (
            self.get_parameter("master_can_port")
            .get_parameter_value()
            .string_value
        )
        self.slave_can_port = (
            self.get_parameter("slave_can_port")
            .get_parameter_value()
            .string_value
        )
        self.gripper_exist = (
            self.get_parameter("gripper_exist")
            .get_parameter_value()
            .bool_value
        )
        self.gripper_val_mutiple = (
            self.get_parameter("gripper_val_mutiple")
            .get_parameter_value()
            .integer_value
        )
        self.enable_mit_ctrl = (
            self.get_parameter("enable_mit_ctrl")
            .get_parameter_value()
            .bool_value
        )
        self.enable_master_ctrl = (
            self.get_parameter("enable_master_ctrl")
            .get_parameter_value()
            .bool_value
        )
        self.enable_master_mit_ctrl = (
            self.get_parameter("enable_master_mit_ctrl")
            .get_parameter_value()
            .bool_value
        )
        self.sync_frequency = (
            self.get_parameter("sync_frequency")
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info(
            f"master can port = {self.master_can_port}, "
            f"slave can port = {self.slave_can_port}, "
            f"sync frequency = {self.sync_frequency} Hz, "
            f"enable_mit_ctrl = {self.enable_mit_ctrl}"
        )

        # Create two Piper interface instances, one for master, one for slave
        self.master_piper = create_piper(self.master_can_port)
        self.slave_piper = create_piper(self.slave_can_port)

        self.master_arm_enabled = False

        self._prepare()

        # Publishers for Master Arm
        self.master_joint_pub = self.create_publisher(
            JointState, "master/joint_state", 1
        )
        self.master_status_pub = self.create_publisher(
            PiperStatusMsg, "master/status", 1
        )
        self.master_pose_pub = self.create_publisher(
            PoseStamped, "master/ee_pose", 1
        )

        # Publishers for Slave Arm
        self.slave_joint_pub = self.create_publisher(
            JointState, "joint_state", 1
        )
        self.slave_status_pub = self.create_publisher(
            PiperStatusMsg, "status", 1
        )
        self.slave_pose_pub = self.create_publisher(PoseStamped, "ee_pose", 1)

        if self.enable_master_ctrl:
            self.create_subscription(
                JointState, "joint_cmd", self.joint_cmd_callback, 1
            )

        # Main timer for synchronization and publishing
        self.sync_timer = self.create_timer(1.0 / 200, self.sync_callback)
        self.pub_timer = self.create_timer(
            1.0 / self.sync_frequency, self.publish_callback
        )

    def _prepare(self):
        if self.enable_master_ctrl:
            self.get_logger().info("Attempting to auto-enable master arm...")
            try:
                enable_arm_ctrl(self.master_piper)
                set_ctrl_method(
                    self.master_piper, is_mit=self.enable_master_mit_ctrl
                )
                self.master_arm_enabled = True
                self.get_logger().info("Successful auto-enable master arm!")
            except TimeoutError:
                self.master_arm_enabled = False
                self.get_logger().warn("Failed auto-enable master arm!")

        self.get_logger().info("Attempting to auto-enable slave arm...")
        try:
            enable_arm_ctrl(self.slave_piper)
            set_ctrl_method(self.slave_piper, is_mit=self.enable_mit_ctrl)
            self.get_logger().info("Successful auto-enable slave arm!")
        except TimeoutError:
            self.get_logger().warn("Failed auto-enable slave arm!")
            raise

        self._check_robot_status()

    def _check_robot_status(self):
        master_ctrl_mode = (
            self.master_piper.GetArmStatus().arm_status.ctrl_mode
        )
        if not self.enable_master_ctrl and master_ctrl_mode != 0x02:
            raise RobotStatusError(
                "Master arm not in teach mode (0x02)! "
                f"Current mode: {master_ctrl_mode}"
            )

        slave_ctrl_mode = self.slave_piper.GetArmStatus().arm_status.ctrl_mode
        if slave_ctrl_mode != 0x01:
            raise RobotStatusError(
                "Slave arm not in control mode (0x01)! "
                f"Current mode: {slave_ctrl_mode}"
            )

    def can_ctrl_master(self) -> bool:
        master_ctrl_mode = (
            self.master_piper.GetArmStatus().arm_status.ctrl_mode
        )
        return self.master_arm_enabled and master_ctrl_mode == 0x01

    def sync_callback(self):
        """Periodically reads master state, commands the slave, and publishes all states."""  # noqa: E501
        # Sync master to slave if enabled
        master_joints_raw = self.master_piper.GetArmJointMsgs()

        self.slave_piper.JointCtrl(
            master_joints_raw.joint_state.joint_1,
            master_joints_raw.joint_state.joint_2,
            master_joints_raw.joint_state.joint_3,
            master_joints_raw.joint_state.joint_4,
            master_joints_raw.joint_state.joint_5,
            master_joints_raw.joint_state.joint_6,
        )

        if self.gripper_exist:
            master_gripper_raw = self.master_piper.GetArmGripperMsgs()
            self.slave_piper.GripperCtrl(
                abs(master_gripper_raw.gripper_state.grippers_angle),
                1000,
                0x01,
                0,
            )

    def publish_callback(self):
        # Publish states for both arms
        self._publish_arm_data(
            self.master_piper,
            self.master_joint_pub,
            self.master_status_pub,
            self.master_pose_pub,
        )
        self._publish_arm_data(
            self.slave_piper,
            self.slave_joint_pub,
            self.slave_status_pub,
            self.slave_pose_pub,
        )

    def _publish_arm_data(
        self,
        piper: C_PiperInterface,
        joint_pub: rclpy.publisher.Publisher,
        status_pub: rclpy.publisher.Publisher,
        pose_pub: rclpy.publisher.Publisher,
    ):
        """Helper function to read data from a piper instance and publish it.

        Args:
            piper: The C_PiperInterface instance for the arm.
            joint_pub: The publisher for joint states.
            status_pub: The publisher for arm status.
            pose_pub: The publisher for end-effector pose.
        """  # noqa: E501

        # publish arm status
        arm_status = get_arm_status(piper)
        status_pub.publish(arm_status)

        # publish joint states
        joint_states_msg = get_arm_state(piper)
        joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        joint_pub.publish(joint_states_msg)

        # Publish end effector pose
        end_pose_msg = get_arm_ee_pose(piper)
        end_pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_pub.publish(end_pose_msg)

    def joint_cmd_callback(self, msg: JointState):
        if self.can_ctrl_master():
            joint_control(
                self.master_piper,
                joint_data=msg,
                has_gripper=self.gripper_exist,
                gripper_val_mutiple=self.gripper_val_mutiple,
            )


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    node = PiperAlohaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Piper Aloha Node.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
