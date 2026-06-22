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
from pathlib import Path
from typing import Sequence

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
    "joint_mit_control",
    "PinocchioGravityCompensator",
    "get_enable_flag",
    "enable_arm_ctrl",
    "switch_piper_ctrl_mode",
    "set_ctrl_method",
]


class GravityCompensationError(RuntimeError):
    pass


class PinocchioGravityCompensator:
    def __init__(self) -> None:
        self.enabled = False
        self.urdf_path = ""
        self.joint_names = [f"joint{i}" for i in range(1, 7)]
        self.scale = 1.0
        # Per-joint multiplier applied on top of ``scale``. Needed because the
        # Piper firmware executes commanded MIT torque at ~4x on joints 1-3 and
        # ~1x on 4-6, so a single global scale cannot compensate gravity
        # correctly for both motor groups. An empty/short vector is padded with
        # 1.0, which reproduces the original single-scale behaviour.
        self.per_joint_scale = [1.0] * 6
        self.max_abs_t_ref = 8.0
        self._pin = None
        self._np = None
        self._model = None
        self._data = None
        self._neutral_q = None
        self._zero_v = None
        self._zero_a = None
        self._joint_indices: list[tuple[int, int]] = []
        self._model_key: tuple[str, tuple[str, ...]] | None = None
        # Per-timestep recording of the gravity-compensation computation.
        # When a path is set, every compute() call appends one CSV row with
        # the raw pin.rnea gravity torque and the final applied torque.
        self._record_path = ""
        self._record_file = None
        self._record_count = 0

    def set_record_path(self, path: str) -> None:
        """Enable/disable per-timestep CSV recording of the computation.

        An empty path disables recording. Each row records the joint
        positions, the raw ``pin.rnea`` gravity torque, the per-joint scale,
        and the final (scaled + clamped) torque actually added to the MIT
        feedforward.
        """
        path = str(path or "").strip()
        if path == self._record_path and self._record_file is not None:
            return
        self.close_record()
        self._record_path = path
        if not path:
            return
        record_path = Path(path).expanduser()
        record_path.parent.mkdir(parents=True, exist_ok=True)
        # Line-buffered so the file can be tailed live while recording.
        self._record_file = record_path.open("w", buffering=1)
        n = len(self.joint_names) or 6
        header = ["t_unix", "scale", "max_t_ref"]
        header += [f"q{i}" for i in range(1, n + 1)]
        header += [f"rnea{i}" for i in range(1, n + 1)]
        header += [f"per_joint{i}" for i in range(1, n + 1)]
        header += [f"applied_tau{i}" for i in range(1, n + 1)]
        self._record_file.write(",".join(header) + "\n")
        self._record_count = 0

    def close_record(self) -> None:
        if self._record_file is not None:
            try:
                self._record_file.close()
            finally:
                self._record_file = None

    def _record_row(
        self,
        positions: Sequence[float],
        raw_taus: Sequence[float],
        joint_scales: Sequence[float],
        torques: Sequence[float],
    ) -> None:
        if self._record_file is None:
            return
        n = len(self.joint_names) or 6

        def _cells(values: Sequence[float]) -> list[str]:
            cells = [f"{float(v):.6g}" for v in list(values)[:n]]
            cells += [""] * (n - len(cells))
            return cells

        row = [
            f"{time.time():.6f}",
            f"{self.scale:.6g}",
            f"{self.max_abs_t_ref:.6g}",
        ]
        row += _cells(positions)
        row += _cells(raw_taus)
        row += _cells(joint_scales)
        row += _cells(torques)
        self._record_file.write(",".join(row) + "\n")
        self._record_count += 1

    def configure(
        self,
        *,
        enabled: bool,
        urdf_path: str,
        joint_names: Sequence[str],
        scale: float,
        max_abs_t_ref: float,
        per_joint_scale: Sequence[float] | None = None,
    ) -> None:
        old_state = (
            self.enabled,
            self.urdf_path,
            list(self.joint_names),
            self.scale,
            list(self.per_joint_scale),
            self.max_abs_t_ref,
            self._model,
            self._data,
            self._neutral_q,
            self._zero_v,
            self._zero_a,
            list(self._joint_indices),
            self._model_key,
        )
        try:
            self.enabled = bool(enabled)
            self.urdf_path = str(urdf_path)
            self.joint_names = list(joint_names)
            self.scale = float(scale)
            self.per_joint_scale = self._normalize_per_joint_scale(
                per_joint_scale, len(self.joint_names)
            )
            self.max_abs_t_ref = float(max_abs_t_ref)
            model_key = (self.urdf_path, tuple(self.joint_names))
            if model_key != self._model_key:
                self._model = None
                self._data = None
                self._neutral_q = None
                self._zero_v = None
                self._zero_a = None
                self._joint_indices = []
                self._model_key = None
            if self.enabled:
                self._ensure_model()
        except Exception:
            (
                self.enabled,
                self.urdf_path,
                self.joint_names,
                self.scale,
                self.per_joint_scale,
                self.max_abs_t_ref,
                self._model,
                self._data,
                self._neutral_q,
                self._zero_v,
                self._zero_a,
                self._joint_indices,
                self._model_key,
            ) = old_state
            raise

    @staticmethod
    def _normalize_per_joint_scale(
        per_joint_scale: Sequence[float] | None, joint_count: int
    ) -> list[float]:
        if not per_joint_scale:
            return [1.0] * joint_count
        values = [float(v) for v in per_joint_scale]
        if len(values) < joint_count:
            # Pad with 1.0 so a short vector only overrides the joints given.
            values = values + [1.0] * (joint_count - len(values))
        elif len(values) > joint_count:
            raise GravityCompensationError(
                f"mit_gravity_compensation_scale_per_joint has "
                f"{len(values)} values, expected at most {joint_count}."
            )
        return values

    def _ensure_model(self) -> None:
        if self._model is not None:
            return
        if not self.urdf_path:
            raise GravityCompensationError(
                "mit_gravity_compensation_urdf_path must be set before "
                "enabling gravity compensation."
            )

        try:
            import numpy as np
            import pinocchio as pin
        except ImportError as exc:
            raise GravityCompensationError(
                "Pinocchio is required for MIT gravity compensation. "
                "Install python3-pinocchio or the pin/pinocchio Python "
                "package in the ROS environment."
            ) from exc

        urdf_path = Path(self.urdf_path).expanduser()
        if not urdf_path.exists():
            raise GravityCompensationError(
                f"Gravity compensation URDF does not exist: {urdf_path}"
            )

        model = pin.buildModelFromUrdf(str(urdf_path))
        model.gravity.linear = np.array([0.0, 0.0, -9.81])
        joint_indices: list[tuple[int, int]] = []
        for joint_name in self.joint_names:
            joint_id = model.getJointId(joint_name)
            if joint_id >= model.njoints:
                raise GravityCompensationError(
                    f"Joint {joint_name!r} not found in URDF {urdf_path}"
                )
            joint_model = model.joints[joint_id]
            if joint_model.nq != 1 or joint_model.nv != 1:
                raise GravityCompensationError(
                    f"Joint {joint_name!r} must be a single-DoF joint for "
                    "Piper MIT gravity compensation."
                )
            joint_indices.append((joint_model.idx_q, joint_model.idx_v))

        self._pin = pin
        self._np = np
        self._model = model
        self._data = model.createData()
        self._neutral_q = pin.neutral(model)
        self._zero_v = np.zeros(model.nv)
        self._zero_a = np.zeros(model.nv)
        self._joint_indices = joint_indices
        self._model_key = (str(self.urdf_path), tuple(self.joint_names))

    def compute(self, positions: Sequence[float]) -> list[float]:
        if not self.enabled:
            return [0.0] * min(6, len(positions))
        self._ensure_model()
        assert self._pin is not None
        assert self._model is not None
        assert self._data is not None
        assert self._neutral_q is not None
        assert self._zero_v is not None
        assert self._zero_a is not None

        q = self._neutral_q.copy()
        for (idx_q, _), position in zip(
            self._joint_indices, positions, strict=False
        ):
            q[idx_q] = float(position)

        tau = self._pin.rnea(
            self._model, self._data, q, self._zero_v, self._zero_a
        )
        torques: list[float] = []
        raw_taus: list[float] = []
        joint_scales: list[float] = []
        for joint_pos, (_, idx_v) in enumerate(
            self._joint_indices[: len(positions)]
        ):
            joint_scale = (
                self.per_joint_scale[joint_pos]
                if joint_pos < len(self.per_joint_scale)
                else 1.0
            )
            raw_tau = float(tau[idx_v])
            torque = raw_tau * self.scale * joint_scale
            if self.max_abs_t_ref >= 0.0:
                torque = max(
                    -self.max_abs_t_ref, min(self.max_abs_t_ref, torque)
                )
            torques.append(torque)
            raw_taus.append(raw_tau)
            joint_scales.append(joint_scale)
        if self._record_file is not None:
            self._record_row(positions, raw_taus, joint_scales, torques)
        return torques


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


def joint_mit_control(
    piper: C_PiperInterface,
    joint_data: JointState,
    mit_kp: float,
    mit_kd: float,
    mit_torque_ref: float = 0.0,
    gravity_compensator: PinocchioGravityCompensator | None = None,
    has_gripper: bool = False,
    gripper_val_mutiple: float = 1.0,
    velocity_ref: Sequence[float] | None = None,
):
    joint_count = min(6, len(joint_data.position))
    gravity_torques = [0.0] * joint_count
    if gravity_compensator is not None and gravity_compensator.enabled:
        gravity_torques = gravity_compensator.compute(
            joint_data.position[:joint_count]
        )

    for idx in range(joint_count):
        # v_des feedforward: kd then damps the velocity tracking error
        # (v_des - q_dot) instead of absolute velocity, removing the drag the
        # kd term otherwise applies while following a moving command.
        v_des = (
            float(velocity_ref[idx])
            if velocity_ref is not None and idx < len(velocity_ref)
            else 0.0
        )
        piper.JointMitCtrl(
            idx + 1,
            float(joint_data.position[idx]),
            v_des,
            mit_kp,
            mit_kd,
            float(mit_torque_ref) + gravity_torques[idx],
        )

    # The gripper is not a MIT joint; keep driving it the same way as
    # joint_control does.
    if has_gripper:
        gripper = 0
        if len(joint_data.position) >= 7:
            gripper = round(joint_data.position[6] * 1000 * 1000)
            gripper = gripper * gripper_val_mutiple
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


def set_ctrl_method(
    piper: C_PiperInterface,
    is_mit: bool = False,
    mit_kp: float = 10.0,
    mit_kd: float = 0.8,
    mit_vel_ref: float = 45.0,
    mit_torque_ref: float = 0.0,
):
    if is_mit:
        piper.MotionCtrl_2(0x01, 0x04, 0, 0xAD)
    else:
        piper.MotionCtrl_2(0x01, 0x01, 100, is_mit_mode=0x00)
