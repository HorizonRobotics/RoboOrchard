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
from enum import IntEnum

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robo_orchard_piper_msg_ros2.msg import PiperStatusMsg
from robo_orchard_piper_ros2.ros_bridge import (
    create_piper,
    enable_arm_ctrl,
    get_arm_ee_pose,
    get_arm_state,
    get_arm_status,
    joint_control,
    set_ctrl_method,
    switch_piper_ctrl_mode,
)


class CanControlMode(IntEnum):
    """CAN control mode values reported by Piper SDK."""

    IDLE_MODE = 0x00
    CAN_MODE = 0x01
    TEACH_MODE = 0x02


class TeachStatusMode(IntEnum):
    """Teach status values reported by Piper SDK."""

    TEACHING_CLOSED = 0x00
    IN_TEACHING = 0x01
    POST_TEACHING = 0x02


class PiperSingleControlNode(Node):
    def __init__(self) -> None:
        super().__init__("piper_single_ctrl")

        # ROS parameters
        self.declare_parameter("can_port", "can0")
        self.declare_parameter("gripper_exist", True)
        self.declare_parameter("gripper_val_mutiple", 1)
        self.declare_parameter("auto_enable_arm_ctrl", False)
        self.declare_parameter("enable_mit_ctrl", False)
        self.declare_parameter(
            "reset_joint_position",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

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
        self.reset_joint_position = list(
            self.get_parameter("reset_joint_position")
            .get_parameter_value()
            .double_array_value
        )
        if len(self.reset_joint_position) != 7:
            self.get_logger().error(
                "reset_joint_position must have 7 values "
                "(6 joints + gripper), got "
                f"{len(self.reset_joint_position)}. Falling back to zeros."
            )
            self.reset_joint_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.get_logger().info(
            f"can_port = {self.can_port}, "
            f"auto_enable_arm_ctrl = {self.auto_enable_arm_ctrl}, "  # noqa: E501
            f"gripper_exist = {self.gripper_exist}, "
            f"gripper_val_mutiple = {self.gripper_val_mutiple}, "
            f"enable_mit_ctrl = {self.enable_mit_ctrl}, "
            f"reset_joint_position = {self.reset_joint_position}"
        )

        self.piper = create_piper(self.can_port)

        # Enable flag
        self._enable_flag = False
        if self.auto_enable_arm_ctrl:
            try:
                if self.enable_arm_ctrl():
                    self.get_logger().info("Auto enable successed!")
                else:
                    self.get_logger().warning("Auto enable failed!")
            except TimeoutError as e:
                msg = "Auto enable timed out."
                if str(e):
                    msg = f"{msg} {e}"
                self.get_logger().warning(msg)

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
            Trigger, "reset_ctrl", self._reset_ctrl_service_callback
        )

        # Start subscription thread
        self.create_subscription(
            JointState, "joint_cmd", self.joint_callback, 1
        )

        self.timer = self.create_timer(1 / 200.0, self.publish_callback)

    def is_controlable(self):
        ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        return self._enable_flag and ctrl_mode == CanControlMode.CAN_MODE

    def enable_arm_ctrl(self) -> bool:
        arm_status = self.piper.GetArmStatus().arm_status
        ctrl_mode = arm_status.ctrl_mode
        if ctrl_mode == CanControlMode.TEACH_MODE:
            # TEACH_MODE + IN_TEACHING means the robot is in active teach mode.
            if arm_status.teach_status == TeachStatusMode.IN_TEACHING:
                self.get_logger().warning(
                    "Skip enable: arm is in active teach mode "
                    "(ctrl_mode=TEACH_MODE, teach_status=IN_TEACHING). "
                    "Press the teach button again to exit teach mode, "
                    "then retry."
                )
                return False
            # TEACH_MODE + POST_TEACHING means the robot has exited teach mode.
            elif arm_status.teach_status == TeachStatusMode.POST_TEACHING:
                self.get_logger().warning("Switch to CAN control mode.")
                switch_piper_ctrl_mode(
                    self.piper,
                    CanControlMode.CAN_MODE,
                    is_mit=self.enable_mit_ctrl,
                )
            else:
                self.get_logger().warning(
                    f"Invalid teach status: {arm_status.teach_status}"
                )
                return False
        # Non-TEACH_MODE status is treated as cold start or normal enable.
        else:
            enable_arm_ctrl(self.piper)
            set_ctrl_method(piper=self.piper, is_mit=self.enable_mit_ctrl)
        self._enable_flag = True
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

        if self.is_controlable():
            response.success = True
            response.message = "Arm is already enabled."
            return response

        try:
            if self.enable_arm_ctrl():
                response.success = True
                response.message = "Arm enabled successfully."
            else:
                response.success = False
                response.message = (
                    "Failed to enable arm. Check the arm state and retry."  # noqa: E501
                )
        except Exception as e:
            self.get_logger().error(f"Error while enabling arm: {e}")
            response.success = False
            response.message = f"An unexpected error occurred: {e}"
        return response

    def _reset_ctrl_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Service callback to reset the arm control."""
        self.get_logger().info("Received request to reset arm.")
        if not self.is_controlable():
            ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
            if ctrl_mode == CanControlMode.TEACH_MODE:
                error_msg = (
                    "Reset failed: arm is in teach mode. "
                    "Exit teach mode first."
                )
            elif not self._enable_flag:
                error_msg = (
                    "Reset failed: arm control is not enabled. "
                    "Enable the arm first."
                )
            else:
                error_msg = (
                    "Reset failed: arm is not in CAN control mode "
                    f"(ctrl_mode=0x{ctrl_mode:02x})."
                )
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        control_freq = 200.0  # Hz

        def _gen_reset_traj():
            cur_joint_state = get_arm_state(self.piper)
            target_joint_state = self.reset_joint_position
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
