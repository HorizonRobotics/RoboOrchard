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

import logging
import math
import time

from geometry_msgs.msg import PoseStamped
from piper_sdk import C_PiperInterface
from scipy.spatial.transform import (
    Rotation as R,  # noqa: N817
)
from sensor_msgs.msg import JointState

from robo_orchard_piper_msg_ros2.msg import PiperStatusMsg

__all__ = [
    "PiperLossError",
    "create_piper",
    "get_arm_status",
    "get_arm_ctrl_state",
    "get_arm_state",
    "get_arm_ee_pose",
    "joint_control",
    "get_enable_flag",
    "enable_arm_ctrl",
    "disable_arm_ctrl",
    "reset_piper_ctrl_mode",
    "set_ctrl_method",
]


global_logger = logging.getLogger(__name__)


class PiperLossError(Exception):
    pass


def create_piper(can_port: str) -> C_PiperInterface:
    piper = C_PiperInterface(can_name=can_port)
    piper.ConnectPort()

    # NOTE: refresh piper message, without this stage,
    # you may get error message
    _ = piper.GetArmStatus()
    _ = get_arm_ctrl_state(piper)
    _ = get_arm_ee_pose(piper)
    _ = get_enable_flag(piper)
    _ = get_disable_flag(piper)
    _ = get_arm_status(piper)

    return piper


def get_arm_status(piper: C_PiperInterface) -> PiperStatusMsg:
    status_msg = piper.GetArmStatus()

    arm_status = PiperStatusMsg()
    arm_status.ctrl_mode = status_msg.arm_status.ctrl_mode
    arm_status.arm_status = status_msg.arm_status.arm_status
    arm_status.mode_feedback = status_msg.arm_status.mode_feed
    arm_status.teach_status = status_msg.arm_status.teach_status
    arm_status.motion_status = status_msg.arm_status.motion_status
    arm_status.trajectory_num = status_msg.arm_status.trajectory_num
    arm_status.err_code = status_msg.arm_status.err_code
    arm_status.joint_1_angle_limit = (
        status_msg.arm_status.err_status.joint_1_angle_limit
    )
    arm_status.joint_2_angle_limit = (
        status_msg.arm_status.err_status.joint_2_angle_limit
    )
    arm_status.joint_3_angle_limit = (
        status_msg.arm_status.err_status.joint_3_angle_limit
    )
    arm_status.joint_4_angle_limit = (
        status_msg.arm_status.err_status.joint_4_angle_limit
    )
    arm_status.joint_5_angle_limit = (
        status_msg.arm_status.err_status.joint_5_angle_limit
    )
    arm_status.joint_6_angle_limit = (
        status_msg.arm_status.err_status.joint_6_angle_limit
    )
    arm_status.communication_status_joint_1 = (
        status_msg.arm_status.err_status.communication_status_joint_1
    )  # noqa: E501
    arm_status.communication_status_joint_2 = (
        status_msg.arm_status.err_status.communication_status_joint_2
    )  # noqa: E501
    arm_status.communication_status_joint_3 = (
        status_msg.arm_status.err_status.communication_status_joint_3
    )  # noqa: E501
    arm_status.communication_status_joint_4 = (
        status_msg.arm_status.err_status.communication_status_joint_4
    )  # noqa: E501
    arm_status.communication_status_joint_5 = (
        status_msg.arm_status.err_status.communication_status_joint_5
    )  # noqa: E501
    arm_status.communication_status_joint_6 = (
        status_msg.arm_status.err_status.communication_status_joint_6
    )  # noqa: E501
    return arm_status


def get_arm_ctrl_state(piper: C_PiperInterface) -> JointState:
    joint_states = JointState()
    joint_states.name = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper",
    ]
    joint_states.position = [0.0] * 7
    joint_states.velocity = [0.0] * 7
    joint_states.effort = [0.0] * 7

    joint_state_factor = 1.0 / 1000 * 0.017444
    gripper_state_factor = 1.0 / 1000000

    joint_msg = piper.GetArmJointCtrl()
    gripper_msg = piper.GetArmGripperCtrl()

    joint_states.position = [
        joint_msg.joint_ctrl.joint_1 * joint_state_factor,
        joint_msg.joint_ctrl.joint_2 * joint_state_factor,
        joint_msg.joint_ctrl.joint_3 * joint_state_factor,
        joint_msg.joint_ctrl.joint_4 * joint_state_factor,
        joint_msg.joint_ctrl.joint_5 * joint_state_factor,
        joint_msg.joint_ctrl.joint_6 * joint_state_factor,
        gripper_msg.gripper_ctrl.grippers_angle * gripper_state_factor,
    ]

    return joint_states


def get_arm_state(piper: C_PiperInterface) -> JointState:
    joint_states = JointState()
    joint_states.name = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper",
    ]
    joint_states.position = [0.0] * 7
    joint_states.velocity = [0.0] * 7
    joint_states.effort = [0.0] * 7

    joint_msg = piper.GetArmJointMsgs()
    spd_info_msg = piper.GetArmHighSpdInfoMsgs()
    gripper_msg = piper.GetArmGripperMsgs()

    # Here, you can set the joint positions to any value you want
    # The raw data obtained is in degrees multiplied by 1000.
    # To convert to radians, divide by 1000, multiply by π/180,
    # and limit to 5 decimal places
    joint_0: float = (joint_msg.joint_state.joint_1 / 1000) * 0.017444
    joint_1: float = (joint_msg.joint_state.joint_2 / 1000) * 0.017444
    joint_2: float = (joint_msg.joint_state.joint_3 / 1000) * 0.017444
    joint_3: float = (joint_msg.joint_state.joint_4 / 1000) * 0.017444
    joint_4: float = (joint_msg.joint_state.joint_5 / 1000) * 0.017444
    joint_5: float = (joint_msg.joint_state.joint_6 / 1000) * 0.017444
    joint_6: float = gripper_msg.gripper_state.grippers_angle / 1000000

    vel_0: float = spd_info_msg.motor_1.motor_speed / 1000
    vel_1: float = spd_info_msg.motor_2.motor_speed / 1000
    vel_2: float = spd_info_msg.motor_3.motor_speed / 1000
    vel_3: float = spd_info_msg.motor_4.motor_speed / 1000
    vel_4: float = spd_info_msg.motor_5.motor_speed / 1000
    vel_5: float = spd_info_msg.motor_6.motor_speed / 1000

    effort_0: float = spd_info_msg.motor_1.effort / 1000
    effort_1: float = spd_info_msg.motor_2.effort / 1000
    effort_2: float = spd_info_msg.motor_3.effort / 1000
    effort_3: float = spd_info_msg.motor_4.effort / 1000
    effort_4: float = spd_info_msg.motor_5.effort / 1000
    effort_5: float = spd_info_msg.motor_6.effort / 1000
    effort_6: float = gripper_msg.gripper_state.grippers_effort / 1000

    joint_states.position = [
        joint_0,
        joint_1,
        joint_2,
        joint_3,
        joint_4,
        joint_5,
        joint_6,
    ]
    joint_states.velocity = [vel_0, vel_1, vel_2, vel_3, vel_4, vel_5]
    joint_states.effort = [
        effort_0,
        effort_1,
        effort_2,
        effort_3,
        effort_4,
        effort_5,
        effort_6,
    ]

    return joint_states


def get_arm_ee_pose(piper: C_PiperInterface) -> PoseStamped:
    endpos = PoseStamped()

    pose_msg = piper.GetArmEndPoseMsgs()

    endpos.pose.position.x = pose_msg.end_pose.X_axis / 1000000
    endpos.pose.position.y = pose_msg.end_pose.Y_axis / 1000000
    endpos.pose.position.z = pose_msg.end_pose.Z_axis / 1000000
    roll = pose_msg.end_pose.RX_axis / 1000
    pitch = pose_msg.end_pose.RY_axis / 1000
    yaw = pose_msg.end_pose.RZ_axis / 1000
    roll = math.radians(roll)
    pitch = math.radians(pitch)
    yaw = math.radians(yaw)
    quaternion = R.from_euler("xyz", [roll, pitch, yaw]).as_quat()
    endpos.pose.orientation.x = quaternion[0]
    endpos.pose.orientation.y = quaternion[1]
    endpos.pose.orientation.z = quaternion[2]
    endpos.pose.orientation.w = quaternion[3]

    return endpos


def joint_control(
    piper: C_PiperInterface,
    joint_data: JointState,
    has_gripper: bool = True,
    gripper_val_mutiple: float = 1.0,
):
    factor = 57324.840764  # 1000 * 180 / 3.14

    joint_positions = {}

    gripper = 0

    for idx, joint_name in enumerate(joint_data.name):
        joint_positions[joint_name] = round(joint_data.position[idx] * factor)

    if len(joint_data.position) >= 7:
        gripper = round(joint_data.position[6] * 1000 * 1000)
        gripper = gripper * gripper_val_mutiple

    # control joints
    piper.JointCtrl(
        joint_positions.get("joint1", 0),
        joint_positions.get("joint2", 0),
        joint_positions.get("joint3", 0),
        joint_positions.get("joint4", 0),
        joint_positions.get("joint5", 0),
        joint_positions.get("joint6", 0),
    )

    # control gripper
    if has_gripper:
        piper.GripperCtrl(abs(gripper), 1000, 0x01, 0)


def get_enable_flag(piper: C_PiperInterface):
    msg = piper.GetArmLowSpdInfoMsgs()

    enable_flag = (
        msg.motor_1.foc_status.driver_enable_status
        and msg.motor_2.foc_status.driver_enable_status
        and msg.motor_3.foc_status.driver_enable_status
        and msg.motor_4.foc_status.driver_enable_status
        and msg.motor_5.foc_status.driver_enable_status
        and msg.motor_6.foc_status.driver_enable_status
    )
    return enable_flag


def enable_arm_ctrl(
    piper: C_PiperInterface,
    timeout: float = 5,
):
    timeout = 5
    start_time = time.time()

    while True:
        elapsed_time = time.time() - start_time

        piper.EnableArm(7)
        piper.GripperCtrl(0, 1000, 0x01, 0)

        flag = get_enable_flag(piper)

        if flag:
            return

        if elapsed_time > timeout:
            break

        time.sleep(1)

    raise TimeoutError


def get_disable_flag(piper: C_PiperInterface):
    msg = piper.GetArmLowSpdInfoMsgs()

    enable_flag = (
        not msg.motor_1.foc_status.driver_enable_status
        and not msg.motor_2.foc_status.driver_enable_status
        and not msg.motor_3.foc_status.driver_enable_status
        and not msg.motor_4.foc_status.driver_enable_status
        and not msg.motor_5.foc_status.driver_enable_status
        and not msg.motor_6.foc_status.driver_enable_status
    )
    return enable_flag


def disable_arm_ctrl(piper: C_PiperInterface, timeout: float = 5):
    timeout = 5
    start_time = time.time()

    while True:
        elapsed_time = time.time() - start_time

        piper.DisableArm(7)
        piper.GripperCtrl(0, 1000, 0x02, 0)

        flag = get_disable_flag(piper)

        if flag:
            return

        if elapsed_time > timeout:
            break

        time.sleep(1)

    raise TimeoutError


def reset_piper_ctrl_mode(
    piper: C_PiperInterface, target_mode: int, max_retry: int = 5
) -> bool:
    if piper.GetArmStatus().arm_status.ctrl_mode == target_mode:  # noqa: E501
        return True

    for _ in range(5):
        disable_arm_ctrl(piper, timeout=5)
        enable_arm_ctrl(piper, timeout=5)

        piper.MotionCtrl_2(target_mode, 0x01, 100, 0x00)

        if piper.GetArmStatus().arm_status.ctrl_mode == target_mode:  # noqa: E501
            return True

    return False


def set_ctrl_method(piper: C_PiperInterface, is_mit: bool = False):
    if is_mit:
        piper.MotionCtrl_2(0x01, 0x01, 100, is_mit_mode=0xAD)
        for idx in range(6):
            piper.JointMitCtrl(idx + 1, 0, 45, 10, 0.8, 0)
    else:
        piper.MotionCtrl_2(0x01, 0x01, 100, is_mit_mode=0x00)
