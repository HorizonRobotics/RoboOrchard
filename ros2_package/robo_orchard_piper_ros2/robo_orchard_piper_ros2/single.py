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


import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robo_orchard_piper_msg_ros2.msg import PiperStatusMsg
from robo_orchard_piper_ros2.ros_bridge import (
    create_piper,
    disable_arm_ctrl,
    enable_arm_ctrl,
    get_arm_ee_pose,
    get_arm_state,
    get_arm_status,
    joint_control,
    reset_piper_ctrl_mode,
    set_ctrl_method,
)


class PiperSingleControlNode(Node):
    def __init__(self) -> None:
        super().__init__("piper_single_ctrl")

        # ROS parameters
        self.declare_parameter("can_port", "can0")
        self.declare_parameter("gripper_exist", True)
        self.declare_parameter("gripper_val_mutiple", 1)
        self.declare_parameter("auto_enable_arm_ctrl", False)
        self.declare_parameter("enable_mit_ctrl", False)

        self.can_port = (
            self.get_parameter("can_port").get_parameter_value().string_value
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
        self.gripper_val_mutiple = max(0, min(self.gripper_val_mutiple, 10))
        self.auto_enable_arm_ctrl = (
            self.get_parameter("auto_enable_arm_ctrl")
            .get_parameter_value()
            .bool_value
        )
        self.enable_mit_ctrl = (
            self.get_parameter("enable_mit_ctrl")
            .get_parameter_value()
            .bool_value
        )

        self.get_logger().info(
            f"can_port = {self.can_port}, "
            f"auto_enable_arm_ctrl = {self.auto_enable_arm_ctrl}, "  # noqa: E501
            f"gripper_exist = {self.gripper_exist}, "
            f"gripper_val_mutiple = {self.gripper_val_mutiple}, "
            f"enable_mit_ctrl = {self.enable_mit_ctrl}"
        )

        self.piper = create_piper(self.can_port)

        # Enable flag
        self._enable_flag = False
        if self.auto_enable_arm_ctrl:
            if self.enable_arm_ctrl():
                self.get_logger().info("Auto enable successed!")
            else:
                self.get_logger().warn(
                    "Auto enable failed! Maybe in teach mode?"
                )

        # Publishers
        self.joint_pub = self.create_publisher(JointState, "joint_state", 1)
        self.arm_status_pub = self.create_publisher(
            PiperStatusMsg, "status", 1
        )
        self.end_pose_pub = self.create_publisher(PoseStamped, "ee_pose", 1)

        # Service
        self.create_service(
            Trigger, "enable_ctrl", self._enable_ctrl_service_callback
        )
        self.create_service(
            Trigger, "disable_ctrl", self._disable_ctrl_service_callback
        )
        self.create_service(
            Trigger, "reset_ctrl", self._reset_ctrl_service_callback
        )

        # Start subscription thread
        self.create_subscription(
            JointState, "joint_cmd", self.joint_callback, 1
        )

        self.timer = self.create_timer(1 / 200.0, self.publish_callback)

    def is_controlable(self):
        ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        return self._enable_flag and ctrl_mode == 0x01

    def enable_arm_ctrl(self, force_reset: bool = False) -> bool:
        ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        if ctrl_mode == 0x02:
            self.get_logger().warn(
                f"ctrl_mode is {ctrl_mode}, try to reset..."
            )
            if force_reset:
                if not reset_piper_ctrl_mode(self.piper, 0x01):
                    return False
            else:
                return False
        else:
            enable_arm_ctrl(self.piper)
        set_ctrl_method(piper=self.piper, is_mit=self.enable_mit_ctrl)
        self._enable_flag = True
        return True

    def disable_arm_ctrl(self, force_reset: bool = False) -> bool:
        ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        # 1. reset to ctrl mode
        if ctrl_mode == 0x02:
            self.get_logger().warn(
                f"ctrl_mode is {ctrl_mode}, try to reset..."
            )
            if force_reset:
                if not reset_piper_ctrl_mode(self.piper, 0x01):
                    return False
            else:
                return False
        # 2. disable
        disable_arm_ctrl(self.piper)
        self._enable_flag = False
        return True

    def publish_callback(self):
        arm_status = get_arm_status(self.piper)
        self.arm_status_pub.publish(arm_status)

        joint_state = get_arm_state(self.piper)
        joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_state)

        ee_pose = get_arm_ee_pose(self.piper)
        ee_pose.header.stamp = self.get_clock().now().to_msg()
        self.end_pose_pub.publish(ee_pose)

    def joint_callback(self, joint_data):
        """Callback function for joint angles."""
        if self.is_controlable():
            joint_control(
                self.piper,
                joint_data=joint_data,
                has_gripper=self.gripper_exist,
                gripper_val_mutiple=self.gripper_val_mutiple,
            )

    def _enable_ctrl_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Service callback to enable the arm control."""
        self.get_logger().info("Received request to enable arm.")

        ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        if ctrl_mode != 0x02 and self._enable_flag:
            response.success = True
            response.message = "Arm is already enabled."
            return response

        try:
            if self.enable_arm_ctrl(force_reset=True):
                response.success = True
                response.message = "Arm enabled successfully."
            else:
                response.success = False
                response.message = "Failed to enable arm. It might be in an unrecoverable state."  # noqa: E501
        except Exception as e:
            self.get_logger().error(f"Error while enabling arm: {e}")
            response.success = False
            response.message = f"An unexpected error occurred: {e}"
        return response

    def _disable_ctrl_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Service callback to disable the arm control."""
        self.get_logger().info("Received request to disable arm.")

        ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        if ctrl_mode != 0x02 and not self._enable_flag:
            response.success = True
            response.message = "Arm is already disabled."
            return response

        try:
            if self.disable_arm_ctrl(force_reset=True):
                response.success = True
                response.message = "Arm disabled successfully."
            else:
                response.success = False
                response.message = "Failed to disable arm. It might be in an unrecoverable state."  # noqa: E501
        except Exception as e:
            self.get_logger().error(f"Error while disabling arm: {e}")
            response.success = False
            response.message = f"An unexpected error occurred: {e}"
        return response

    def _reset_ctrl_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Service callback to reset the arm control."""
        self.get_logger().info("Received request to reset arm.")
        if not self.is_controlable():
            error_msg = "Arm is not controllable, reset failed."
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        control_freq = 200.0  # Hz

        def _gen_reset_traj():
            cur_joint_state = get_arm_state(self.piper)
            target_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            elapsed_time = 3.0  # seconds
            num_steps = int(elapsed_time * control_freq)
            traj = []
            for step in range(num_steps):
                interp_state = [
                    cur_joint_state.position[i]
                    + (target_joint_state[i] - cur_joint_state.position[i])
                    * (step + 1)
                    / num_steps
                    for i in range(7)
                ]
                msg = JointState()
                msg.position = interp_state
                msg.name = cur_joint_state.name
                msg.velocity = cur_joint_state.velocity
                msg.effort = cur_joint_state.effort
                traj.append(msg)
            return traj

        try:
            traj = _gen_reset_traj()
            for position in traj:
                joint_control(
                    self.piper,
                    joint_data=position,
                    has_gripper=self.gripper_exist,
                    gripper_val_mutiple=self.gripper_val_mutiple,
                )
                time.sleep(1.0 / control_freq)
            response.success = True
            response.message = "Arm reset successfully."
        except Exception as e:
            self.get_logger().error(f"Error while resetting arm: {e}")
            response.success = False
            response.message = f"An unexpected error occurred: {e}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PiperSingleControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
