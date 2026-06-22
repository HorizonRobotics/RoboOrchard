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
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robo_orchard_piper_msg_ros2.msg import PiperStatusMsg
from robo_orchard_piper_ros2.ros_bridge import (
    PinocchioGravityCompensator,
    create_piper,
    enable_arm_ctrl,
    get_arm_ee_pose,
    get_arm_state,
    get_arm_status,
    joint_control,
    joint_mit_control,
    reset_piper_ctrl_mode,
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
        self.declare_parameter("mit_kp", 10.0)
        self.declare_parameter("mit_kd", 0.8)
        self.declare_parameter("mit_vel_ref", 45.0)
        self.declare_parameter("mit_torque_ref", 0.0)
        # When true, feed forward the commanded joint velocity (estimated by
        # finite-differencing the position commands, since the command stream
        # carries no velocity) as v_des, so the kd term damps velocity error
        # instead of absolute velocity. Reduces tracking lag and overshoot.
        self.declare_parameter("mit_velocity_feedforward", False)
        # EMA smoothing for the finite-differenced v_des (lower = smoother but
        # more lag). Differentiating quantized/jittery commands is noisy, and
        # kd amplifies that into torque chatter, so this trades noise vs lag.
        self.declare_parameter("mit_velocity_feedforward_alpha", 0.2)
        # Velocities with magnitude below this (rad/s) are zeroed, to stop
        # feedforward chatter while holding position.
        self.declare_parameter(
            "mit_velocity_feedforward_deadband", 0.02
        )
        self.declare_parameter("mit_gravity_compensation_enabled", False)
        self.declare_parameter("mit_gravity_compensation_urdf_path", "")
        self.declare_parameter("mit_gravity_compensation_scale", 1.0)
        # Per-joint multiplier on top of the global scale. Piper firmware
        # executes commanded MIT torque at ~4x on joints 1-3 and ~1x on 4-6,
        # so correct gravity compensation needs e.g. [0.25,0.25,0.25,1,1,1].
        # Empty (default) reproduces the original single-scale behaviour.
        self.declare_parameter(
            "mit_gravity_compensation_scale_per_joint", [1.0] * 6
        )
        self.declare_parameter("mit_gravity_compensation_max_t_ref", 8.0)
        self.declare_parameter(
            "mit_gravity_compensation_joint_names",
            [f"joint{i}" for i in range(1, 7)],
        )
        # When non-empty, every gravity-compensation timestep is appended as a
        # CSV row (raw pin.rnea torque, scale, per-joint, applied torque) for
        # offline verification. Empty disables recording.
        self.declare_parameter("mit_gravity_compensation_record_path", "")
        self.declare_parameter("reset_position", [0.0] * 7)

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
        self.mit_kp = (
            self.get_parameter("mit_kp").get_parameter_value().double_value
        )
        self.mit_kd = (
            self.get_parameter("mit_kd").get_parameter_value().double_value
        )
        self.mit_vel_ref = (
            self.get_parameter("mit_vel_ref")
            .get_parameter_value()
            .double_value
        )
        self.mit_torque_ref = (
            self.get_parameter("mit_torque_ref")
            .get_parameter_value()
            .double_value
        )
        self.mit_velocity_feedforward = (
            self.get_parameter("mit_velocity_feedforward")
            .get_parameter_value()
            .bool_value
        )
        self.mit_velocity_feedforward_alpha = (
            self.get_parameter("mit_velocity_feedforward_alpha")
            .get_parameter_value()
            .double_value
        )
        self.mit_velocity_feedforward_deadband = (
            self.get_parameter("mit_velocity_feedforward_deadband")
            .get_parameter_value()
            .double_value
        )
        # Finite-difference state for commanded-velocity feedforward.
        self._prev_cmd_pos: list[float] | None = None
        self._prev_cmd_vel: list[float] | None = None
        self._prev_cmd_time: float | None = None
        self.mit_gravity_compensation_enabled = (
            self.get_parameter("mit_gravity_compensation_enabled")
            .get_parameter_value()
            .bool_value
        )
        self.mit_gravity_compensation_urdf_path = (
            self.get_parameter("mit_gravity_compensation_urdf_path")
            .get_parameter_value()
            .string_value
        )
        self.mit_gravity_compensation_scale = (
            self.get_parameter("mit_gravity_compensation_scale")
            .get_parameter_value()
            .double_value
        )
        self.mit_gravity_compensation_scale_per_joint = list(
            self.get_parameter("mit_gravity_compensation_scale_per_joint")
            .get_parameter_value()
            .double_array_value
        )
        self.mit_gravity_compensation_max_t_ref = (
            self.get_parameter("mit_gravity_compensation_max_t_ref")
            .get_parameter_value()
            .double_value
        )
        self.mit_gravity_compensation_joint_names = list(
            self.get_parameter("mit_gravity_compensation_joint_names")
            .get_parameter_value()
            .string_array_value
        )
        self.mit_gravity_compensation_record_path = (
            self.get_parameter("mit_gravity_compensation_record_path")
            .get_parameter_value()
            .string_value
        )
        self.gravity_compensator = PinocchioGravityCompensator()
        self.gravity_compensator.configure(
            enabled=self.mit_gravity_compensation_enabled,
            urdf_path=self.mit_gravity_compensation_urdf_path,
            joint_names=self.mit_gravity_compensation_joint_names,
            scale=self.mit_gravity_compensation_scale,
            max_abs_t_ref=self.mit_gravity_compensation_max_t_ref,
            per_joint_scale=self.mit_gravity_compensation_scale_per_joint,
        )
        self.gravity_compensator.set_record_path(
            self.mit_gravity_compensation_record_path
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"can_port = {self.can_port}, "
            f"auto_enable_arm_ctrl = {self.auto_enable_arm_ctrl}, "  # noqa: E501
            f"gripper_exist = {self.gripper_exist}, "
            f"gripper_val_mutiple = {self.gripper_val_mutiple}, "
            f"enable_mit_ctrl = {self.enable_mit_ctrl}, "
            f"mit_kp = {self.mit_kp}, mit_kd = {self.mit_kd}, "
            f"mit_vel_ref = {self.mit_vel_ref}, "
            f"mit_torque_ref = {self.mit_torque_ref}, "
            "mit_gravity_compensation_enabled = "
            f"{self.mit_gravity_compensation_enabled}, "
            "mit_gravity_compensation_urdf_path = "
            f"{self.mit_gravity_compensation_urdf_path}"
        )

        self.piper = create_piper(self.can_port)

        # Enable flag
        self._enable_flag = False
        self._resetting = False
        if self.auto_enable_arm_ctrl:
            try:
                if self.enable_arm_ctrl():
                    self.get_logger().info("Auto enable successed!")
                else:
                    self.get_logger().warn("Auto enable failed!")
            except TimeoutError as e:
                msg = "Auto enable timed out."
                if str(e):
                    msg = f"{msg} {e}"
                self.get_logger().warn(msg)

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

    def _on_set_parameters(self, params):
        next_enable_mit_ctrl = self.enable_mit_ctrl
        next_mit_kp = self.mit_kp
        next_mit_kd = self.mit_kd
        next_mit_vel_ref = self.mit_vel_ref
        next_mit_torque_ref = self.mit_torque_ref
        next_velocity_ff = self.mit_velocity_feedforward
        next_velocity_ff_alpha = self.mit_velocity_feedforward_alpha
        next_velocity_ff_deadband = self.mit_velocity_feedforward_deadband
        next_gravity_enabled = self.mit_gravity_compensation_enabled
        next_gravity_urdf_path = self.mit_gravity_compensation_urdf_path
        next_gravity_scale = self.mit_gravity_compensation_scale
        next_gravity_max_t_ref = self.mit_gravity_compensation_max_t_ref
        next_gravity_joint_names = list(
            self.mit_gravity_compensation_joint_names
        )
        next_gravity_per_joint_scale = list(
            self.mit_gravity_compensation_scale_per_joint
        )
        next_gravity_record_path = self.mit_gravity_compensation_record_path

        for param in params:
            if param.name == "reset_position":
                if len(param.value) != 7:
                    return SetParametersResult(
                        successful=False,
                        reason="reset_position must have 7 joint values.",
                    )
            elif param.name == "enable_mit_ctrl":
                next_enable_mit_ctrl = bool(param.value)
            elif param.name == "mit_kp":
                next_mit_kp = float(param.value)
            elif param.name == "mit_kd":
                next_mit_kd = float(param.value)
            elif param.name == "mit_vel_ref":
                next_mit_vel_ref = float(param.value)
            elif param.name == "mit_torque_ref":
                next_mit_torque_ref = float(param.value)
            elif param.name == "mit_velocity_feedforward":
                next_velocity_ff = bool(param.value)
            elif param.name == "mit_velocity_feedforward_alpha":
                next_velocity_ff_alpha = float(param.value)
            elif param.name == "mit_velocity_feedforward_deadband":
                next_velocity_ff_deadband = float(param.value)
            elif param.name == "mit_gravity_compensation_enabled":
                next_gravity_enabled = bool(param.value)
            elif param.name == "mit_gravity_compensation_urdf_path":
                next_gravity_urdf_path = str(param.value)
            elif param.name == "mit_gravity_compensation_scale":
                next_gravity_scale = float(param.value)
            elif param.name == "mit_gravity_compensation_scale_per_joint":
                next_gravity_per_joint_scale = list(param.value)
            elif param.name == "mit_gravity_compensation_max_t_ref":
                next_gravity_max_t_ref = float(param.value)
            elif param.name == "mit_gravity_compensation_joint_names":
                next_gravity_joint_names = list(param.value)
            elif param.name == "mit_gravity_compensation_record_path":
                next_gravity_record_path = str(param.value)

        if next_mit_kp < 0 or next_mit_kd < 0:
            return SetParametersResult(
                successful=False,
                reason="MIT kp and kd must be non-negative.",
            )

        if next_gravity_max_t_ref < 0:
            return SetParametersResult(
                successful=False,
                reason=(
                    "mit_gravity_compensation_max_t_ref must be "
                    "non-negative."
                ),
            )
        if len(next_gravity_joint_names) != 6:
            return SetParametersResult(
                successful=False,
                reason=(
                    "mit_gravity_compensation_joint_names must contain "
                    "6 joint names."
                ),
            )
        if len(next_gravity_per_joint_scale) > len(next_gravity_joint_names):
            return SetParametersResult(
                successful=False,
                reason=(
                    "mit_gravity_compensation_scale_per_joint must have at "
                    "most as many values as there are joints."
                ),
            )
        try:
            self.gravity_compensator.configure(
                enabled=next_gravity_enabled,
                urdf_path=next_gravity_urdf_path,
                joint_names=next_gravity_joint_names,
                scale=next_gravity_scale,
                max_abs_t_ref=next_gravity_max_t_ref,
                per_joint_scale=next_gravity_per_joint_scale,
            )
        except Exception as e:
            return SetParametersResult(
                successful=False,
                reason=f"Failed to configure MIT gravity compensation: {e}",
            )

        try:
            self.gravity_compensator.set_record_path(next_gravity_record_path)
        except Exception as e:
            return SetParametersResult(
                successful=False,
                reason=f"Failed to set gravity compensation record path: {e}",
            )

        self.enable_mit_ctrl = next_enable_mit_ctrl
        self.mit_kp = next_mit_kp
        self.mit_kd = next_mit_kd
        self.mit_vel_ref = next_mit_vel_ref
        self.mit_torque_ref = next_mit_torque_ref
        self.mit_velocity_feedforward = next_velocity_ff
        self.mit_velocity_feedforward_alpha = next_velocity_ff_alpha
        self.mit_velocity_feedforward_deadband = next_velocity_ff_deadband
        self.mit_gravity_compensation_enabled = next_gravity_enabled
        self.mit_gravity_compensation_urdf_path = next_gravity_urdf_path
        self.mit_gravity_compensation_scale = next_gravity_scale
        self.mit_gravity_compensation_scale_per_joint = (
            next_gravity_per_joint_scale
        )
        self.mit_gravity_compensation_record_path = next_gravity_record_path
        self.mit_gravity_compensation_max_t_ref = next_gravity_max_t_ref
        self.mit_gravity_compensation_joint_names = next_gravity_joint_names

        if self.is_controlable():
            try:
                set_ctrl_method(
                    piper=self.piper,
                    is_mit=self.enable_mit_ctrl,
                    mit_kp=self.mit_kp,
                    mit_kd=self.mit_kd,
                    mit_vel_ref=self.mit_vel_ref,
                    mit_torque_ref=self.mit_torque_ref,
                )
            except Exception as e:
                return SetParametersResult(
                    successful=False,
                    reason=f"Failed to apply MIT control settings: {e}",
                )

        self.get_logger().info(
            "MIT control settings updated: "
            f"enabled = {self.enable_mit_ctrl}, "
            f"kp = {self.mit_kp}, kd = {self.mit_kd}, "
            f"vel_ref = {self.mit_vel_ref}, "
            f"torque_ref = {self.mit_torque_ref}, "
            "gravity_compensation_enabled = "
            f"{self.mit_gravity_compensation_enabled}, "
            f"gravity_scale = {self.mit_gravity_compensation_scale}, "
            "gravity_scale_per_joint = "
            f"{self.mit_gravity_compensation_scale_per_joint}"
        )
        return SetParametersResult(successful=True)

    def enable_arm_ctrl(self, force_reset: bool = False) -> bool:
        ctrl_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
        if ctrl_mode == 0x02:
            self.get_logger().warn(
                f"ctrl_mode is {ctrl_mode}, try to reset..."
            )
            if force_reset:
                if not reset_piper_ctrl_mode(self.piper, 0x01):
                    return False
    def enable_arm_ctrl(self) -> bool:
        arm_status = self.piper.GetArmStatus().arm_status
        ctrl_mode = arm_status.ctrl_mode
        if ctrl_mode == CanControlMode.TEACH_MODE:
            # TEACH_MODE + IN_TEACHING means the robot is in active teach mode.
            if arm_status.teach_status == TeachStatusMode.IN_TEACHING:
                self.get_logger().warn(
                    "Skip enable: arm is in active teach mode "
                    "(ctrl_mode=TEACH_MODE, teach_status=IN_TEACHING). "
                    "Press the teach button again to exit teach mode, "
                    "then retry."
                )
                return False
            # TEACH_MODE + POST_TEACHING means the robot has exited teach mode.
            elif arm_status.teach_status == TeachStatusMode.POST_TEACHING:
                self.get_logger().warn("Switch to CAN control mode.")
                switch_piper_ctrl_mode(
                    self.piper,
                    CanControlMode.CAN_MODE,
                    is_mit=self.enable_mit_ctrl,
                )
            else:
                self.get_logger().warn(
                    f"Invalid teach status: {arm_status.teach_status}"
                )
                return False
        # Non-TEACH_MODE status is treated as cold start or normal enable.
        else:
            enable_arm_ctrl(self.piper)
        set_ctrl_method(
            piper=self.piper,
            is_mit=self.enable_mit_ctrl,
            mit_kp=self.mit_kp,
            mit_kd=self.mit_kd,
            mit_vel_ref=self.mit_vel_ref,
            mit_torque_ref=self.mit_torque_ref,
        )
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

    def _estimate_cmd_velocity(self, joint_data) -> list[float]:
        """Estimate commanded joint velocity by finite-differencing commands.

        The command stream carries no velocity, so v_des is derived from the
        change in commanded position over wall-clock time, lightly smoothed and
        clamped to the MIT velocity range.
        """
        n = min(6, len(joint_data.position))
        now = time.time()
        pos = [float(joint_data.position[i]) for i in range(n)]
        vel = [0.0] * n
        if (
            self._prev_cmd_pos is not None
            and len(self._prev_cmd_pos) == n
            and self._prev_cmd_time is not None
        ):
            dt = now - self._prev_cmd_time
            # Ignore degenerate / stale gaps to avoid huge spurious velocities.
            if 1e-4 < dt < 0.5:
                prev_vel = self._prev_cmd_vel or [0.0] * n
                alpha = self.mit_velocity_feedforward_alpha
                deadband = self.mit_velocity_feedforward_deadband
                for i in range(n):
                    raw = (pos[i] - self._prev_cmd_pos[i]) / dt
                    # EMA low-pass to tame finite-difference noise.
                    smoothed = alpha * raw + (1.0 - alpha) * (
                        prev_vel[i] if i < len(prev_vel) else 0.0
                    )
                    # Deadband suppresses chatter while holding position.
                    if abs(smoothed) < deadband:
                        smoothed = 0.0
                    vel[i] = max(-45.0, min(45.0, smoothed))
        self._prev_cmd_pos = pos
        self._prev_cmd_vel = vel
        self._prev_cmd_time = now
        return vel

    def _command_velocity(self, joint_data) -> list[float]:
        """Velocity feedforward source for v_des.

        Prefers the velocity carried in the command itself -- the master's
        hardware-measured motor speed (rad/s), forwarded unchanged by the
        teleop muxer -- so there is no differentiation and no derivative
        noise. Only falls back to finite-differencing the position commands
        when the command carries no velocity (e.g. a position-only source).
        """
        n = min(6, len(joint_data.position))
        if len(joint_data.velocity) >= n:
            deadband = self.mit_velocity_feedforward_deadband
            vel: list[float] = []
            for i in range(n):
                v = float(joint_data.velocity[i])
                # Deadband suppresses chatter while holding position.
                if abs(v) < deadband:
                    v = 0.0
                vel.append(max(-45.0, min(45.0, v)))
            return vel
        return self._estimate_cmd_velocity(joint_data)

    def joint_callback(self, joint_data):
        """Callback function for joint angles."""
        if self._resetting:
            return

        if self.is_controlable():
            if self.enable_mit_ctrl:
                velocity_ref = (
                    self._command_velocity(joint_data)
                    if self.mit_velocity_feedforward
                    else None
                )
                joint_mit_control(
                    self.piper,
                    joint_data=joint_data,
                    mit_kp=self.mit_kp,
                    mit_kd=self.mit_kd,
                    mit_torque_ref=self.mit_torque_ref,
                    gravity_compensator=self.gravity_compensator,
                    has_gripper=self.gripper_exist,
                    gripper_val_mutiple=self.gripper_val_mutiple,
                    velocity_ref=velocity_ref,
                )
            else:
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
        reset_position = list(
            self.get_parameter("reset_position")
            .get_parameter_value()
            .double_array_value
        )
        if len(reset_position) != 7:
            error_msg = (
                f"reset_position must have 7 joint values, "
                f"got {len(reset_position)}."
            )
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        def _gen_reset_traj():
            cur_joint_state = get_arm_state(self.piper)
            target_joint_state = reset_position
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

        self._resetting = True
        traj = []
        try:
            traj = _gen_reset_traj()
            # Reset always runs through the firmware position planner
            # (JointCtrl) so the motion never depends on the currently
            # configured MIT gains (e.g. kp=0 would not move at all).
            if self.enable_mit_ctrl:
                set_ctrl_method(piper=self.piper, is_mit=False)
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
        finally:
            if self.enable_mit_ctrl:
                try:
                    set_ctrl_method(
                        piper=self.piper,
                        is_mit=True,
                        mit_kp=self.mit_kp,
                        mit_kd=self.mit_kd,
                        mit_vel_ref=self.mit_vel_ref,
                        mit_torque_ref=self.mit_torque_ref,
                    )
                    # Re-seat the MIT setpoint at the reset pose so control
                    # resumes without a jump.
                    if traj:
                        final_position = traj[-1]
                        for _ in range(int(0.5 * control_freq)):
                            joint_mit_control(
                                self.piper,
                                joint_data=final_position,
                                mit_kp=self.mit_kp,
                                mit_kd=self.mit_kd,
                                mit_torque_ref=self.mit_torque_ref,
                                gravity_compensator=self.gravity_compensator,
                                has_gripper=self.gripper_exist,
                                gripper_val_mutiple=self.gripper_val_mutiple,
                            )
                            time.sleep(1.0 / control_freq)
                except Exception as e:
                    error_msg = f"Failed to restore MIT control mode: {e}"
                    self.get_logger().error(error_msg)
                    response.success = False
                    response.message = error_msg
            self._resetting = False
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
