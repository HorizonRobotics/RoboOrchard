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

import os

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from robo_orchard_pico_msg_ros2.msg import (
    VRState,
)
from robo_orchard_teleop_ros2.bridge.pico.intent import (
    GripperIntent,
    ResetIntent,
)
from robo_orchard_teleop_ros2.bridge.pico.teleop import (
    Action,
    TeleOpResult,
    VRTeleOp,
)


class PiperPicoVRTeleOpNode(Node):
    def __init__(self):
        super().__init__("piper_pico_vr_teleop")

        self.declare_parameter(
            "urdf_path",
            "",
            descriptor=ParameterDescriptor(description="URDF Path"),
        )
        self.urdf_path: str = (
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )
        if not os.path.exists(self.urdf_path):
            raise FileNotFoundError(
                f"urdf file {self.urdf_path} does not exists!"
            )

        self.declare_parameter(
            "enable_pose_control",
            True,
            descriptor=ParameterDescriptor(
                description="Enable or not enable pose control."
            ),
        )
        self.enable_pose_control: str = (
            self.get_parameter("enable_pose_control")
            .get_parameter_value()
            .bool_value
        )

        self.left_teleop = VRTeleOp(
            source_type="left",
            urdf_path=self.urdf_path,
            base_link_name="base_link",
            ee_link_name="gripper_base",
            trigger_intent=GripperIntent(
                source_type="left",
                value_thresh=0.5,
                thresh=1.0,
            ),
            reset_intent=ResetIntent(source_type="X", thresh=1.0),
            reset_callback=self.reset_left,
            logger=self.get_logger(),
        )
        self.right_teleop = VRTeleOp(
            source_type="right",
            urdf_path=self.urdf_path,
            base_link_name="base_link",
            ee_link_name="gripper_base",
            trigger_intent=GripperIntent(
                source_type="right",
                value_thresh=0.5,
                thresh=1.0,
            ),
            reset_intent=ResetIntent(source_type="A", thresh=1.0),
            reset_callback=self.reset_right,
            logger=self.get_logger(),
        )

        # sub
        self.vr_state_sub = self.create_subscription(
            VRState, "/pico_bridge/vr_state", self.sub_vr_state_callback, 1
        )
        self.left_ee_pose_sub = self.create_subscription(
            Pose,
            "/robot/left/ee_pose",
            self.sub_left_ee_pose_callback,
            1,
        )
        self.left_joint_state_sub = self.create_subscription(
            JointState,
            "/robot/left/joint_state",
            self.sub_left_joint_state_callback,
            1,
        )
        self.right_ee_pose_sub = self.create_subscription(
            Pose,
            "/robot/right/ee_pose",
            self.sub_right_ee_pose_callback,
            1,
        )
        self.right_joint_state_sub = self.create_subscription(
            JointState,
            "/robot/right/joint_state",
            self.sub_right_joint_state_callback,
            1,
        )

        # pub
        self.left_cmd_pub = self.create_publisher(
            JointState,
            "/robot/left/joint_cmd",
            1,
        )
        self.left_target_pub = self.create_publisher(
            PoseStamped, "robot/left/ee_pose_target", 1
        )

        self.right_cmd_pub = self.create_publisher(
            JointState,
            "/robot/right/joint_cmd",
            1,
        )
        self.right_target_pub = self.create_publisher(
            PoseStamped, "robot/right/ee_pose_target", 1
        )

        # timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def sub_vr_state_callback(self, msg: VRState):
        action = self.left_teleop.update_vr_state(msg)
        if action == Action.RESET:
            self.get_logger().info("Reset left robot")

        action = self.right_teleop.update_vr_state(msg)
        if action == Action.RESET:
            self.get_logger().info("Reset right robot")

    def sub_left_ee_pose_callback(self, msg: PoseStamped):
        self.left_teleop.update_robot_ee_pose(msg)

    def sub_left_joint_state_callback(self, msg: JointState):
        self.left_teleop.update_robot_joint_state(msg.position[:-1])

    def sub_right_ee_pose_callback(self, msg: PoseStamped):
        self.right_teleop.update_robot_ee_pose(msg)

    def sub_right_joint_state_callback(self, msg: JointState):
        self.right_teleop.update_robot_joint_state(msg.position[:-1])

    def reset_left(self):
        msg = JointState(
            header=Header(
                frame_id="/robot/left", stamp=self.get_clock().now().to_msg()
            ),
            position=[0.0] * 7,
        )
        self.left_cmd_pub.publish(msg)

    def reset_right(self):
        msg = JointState(
            header=Header(
                frame_id="/robot/right", stamp=self.get_clock().now().to_msg()
            ),
            position=[0.0] * 7,
        )
        self.right_cmd_pub.publish(msg)

    def _handle_teleop_result(
        self,
        ret: TeleOpResult,
        gripper: float,
        header: Header,
        joint_state_cmd_publisher,
        target_pose_publisher,
    ):
        if self.enable_pose_control and ret.solution is not None:
            positions = ret.solution
            positions.append(gripper)
            joint_state_msg = JointState(header=header, position=positions)
            joint_state_cmd_publisher.publish(joint_state_msg)

        pose_msg = PoseStamped(header=header, pose=ret.target_ee_pose)
        target_pose_publisher.publish(pose_msg)

    def timer_callback(self):
        current_stamp = self.get_clock().now().to_msg()

        left_ret = self.left_teleop()
        right_ret = self.right_teleop()

        if left_ret is not None:
            self._handle_teleop_result(
                ret=left_ret,
                gripper=self.left_teleop.latest_vr_state.left_controller.trigger,
                header=Header(frame_id="/robot/left", stamp=current_stamp),
                joint_state_cmd_publisher=self.left_cmd_pub,
                target_pose_publisher=self.left_target_pub,
            )

        if right_ret is not None:
            self._handle_teleop_result(
                ret=right_ret,
                gripper=self.right_teleop.latest_vr_state.right_controller.trigger,
                header=Header(frame_id="/robot/right", stamp=current_stamp),
                joint_state_cmd_publisher=self.right_cmd_pub,
                target_pose_publisher=self.right_target_pub,
            )


def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    node = PiperPicoVRTeleOpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
