# Project RoboOrchard
#
# Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
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
import tempfile
from functools import partial
from time import monotonic
from typing import Literal

import numpy as np
import rclpy
import xacro
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from robo_orchard_pico_msg_ros2.msg import VRState
from robo_orchard_teleop_msg_ros2.msg import ControlMode
from robo_orchard_teleop_ros2.bridge.pico.intent import (
    GripperIntent,
    ResetIntent,
)
from robo_orchard_teleop_ros2.bridge.pico.teleop import Action, VRTeleOp
from robo_orchard_teleop_ros2.msg_adaptor import (
    matrix_to_pose_msg,
    pose_msg_to_matrix,
)

FRANKA_JOINT_NAMES = [f"fr3_joint{index}" for index in range(1, 8)]


class FrankaPicoVRTeleOpNode(Node):
    """Convert Pico controller motion into Franka joint targets."""

    def __init__(self, **kwargs):
        super().__init__("franka_pico_vr_teleop", **kwargs)

        self.declare_parameter(
            "urdf_path",
            "",
            descriptor=ParameterDescriptor(description="Expanded FR3 URDF"),
        )
        self.declare_parameter("source_type", "right")
        self.declare_parameter("base_link_name", "fr3_link0")
        self.declare_parameter("ee_link_name", "fr3_link8")
        self.declare_parameter("joint_state_topic", "/franka/joint_states")
        self.declare_parameter(
            "ee_pose_topic",
            "/franka_robot_state_broadcaster/current_pose",
        )
        self.declare_parameter(
            "robot_state_topic",
            "/franka_robot_state_broadcaster/robot_state",
        )
        self.declare_parameter(
            "joint_command_topic",
            "/robot/franka/joint_cmd",
        )
        self.declare_parameter("reset_service", "/robot/control/reset")
        self.declare_parameter("control_frequency_hz", 90.0)
        self.declare_parameter("stale_timeout_s", 0.3)
        self.declare_parameter("translation_scale_factor", 1.2)
        self.declare_parameter("pose_low_pass_alpha", 0.5)
        self.declare_parameter("ik_regularization_parameter", 0.005)

        source_urdf_path = self.get_parameter("urdf_path").value
        if not os.path.isfile(source_urdf_path):
            raise FileNotFoundError(
                f"Franka URDF or xacro does not exist: {source_urdf_path}"
            )
        self._generated_urdf_path: str | None = None
        self.urdf_path = self._prepare_urdf(source_urdf_path)

        self.source_type: Literal["left", "right"] = self.get_parameter(
            "source_type"
        ).value
        if self.source_type not in ("left", "right"):
            raise ValueError("source_type must be 'left' or 'right'")

        self.base_link_name = self.get_parameter("base_link_name").value
        self.ee_link_name = self.get_parameter("ee_link_name").value
        self._ee_to_flange_matrix: np.ndarray | None = None
        self._tcp_transform_logged = False
        self.control_frequency_hz = float(
            self.get_parameter("control_frequency_hz").value
        )
        if self.control_frequency_hz <= 0.0:
            raise ValueError("control_frequency_hz must be positive")
        self.stale_timeout_s = float(
            self.get_parameter("stale_timeout_s").value
        )
        if not np.isfinite(self.stale_timeout_s) or self.stale_timeout_s <= 0:
            raise ValueError("stale_timeout_s must be finite and positive")
        self.ik_regularization_parameter = float(
            self.get_parameter("ik_regularization_parameter").value
        )
        if (
            not np.isfinite(self.ik_regularization_parameter)
            or self.ik_regularization_parameter < 0.0
        ):
            raise ValueError(
                "ik_regularization_parameter must be finite and non-negative"
            )

        self.reset_service_name = self.get_parameter("reset_service").value
        self.reset_client = self.create_client(
            Trigger,
            self.reset_service_name,
        )
        self._reset_future = None
        self._manager_resetting = False
        self._reset_generation = 0
        self._destroying = False

        reset_button = "X" if self.source_type == "left" else "A"
        self.teleop = VRTeleOp(
            source_type=self.source_type,
            urdf_path=self.urdf_path,
            base_link_name=self.base_link_name,
            ee_link_name=self.ee_link_name,
            scale_factor=float(
                self.get_parameter("translation_scale_factor").value
            ),
            pose_low_pass_alpha=float(
                self.get_parameter("pose_low_pass_alpha").value
            ),
            ik_regularization_parameter=self.ik_regularization_parameter,
            trigger_intent=GripperIntent(
                source_type=self.source_type,
                value_thresh=0.5,
                thresh=1.0,
            ),
            reset_intent=ResetIntent(
                source_type=reset_button,
                thresh=1.0,
            ),
            reset_callback=None,
            logger=self.get_logger(),
        )

        if self.teleop.ik_solver.num_joints != len(FRANKA_JOINT_NAMES):
            active_names = self.teleop.ik_solver.get_joint_names()
            raise ValueError(
                "Franka IK chain must contain exactly 7 active joints, "
                f"got {active_names}"
            )

        self.current_joint_positions: list[float] | None = None
        self._last_joint_warning_ns = 0
        self._last_input_times: dict[str, float | None] = dict.fromkeys(
            ("vr", "joints", "pose")
        )

        self.create_subscription(
            VRState,
            "vr_state",
            self._vr_state_callback,
            1,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self._joint_state_callback,
            1,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("ee_pose_topic").value,
            self._ee_pose_callback,
            1,
        )
        self._robot_state_subscription = self.create_subscription(
            FrankaRobotState,
            self.get_parameter("robot_state_topic").value,
            self._robot_state_callback,
            1,
        )
        self._control_status_subscription = self.create_subscription(
            ControlMode,
            "/robot/control/status",
            self._on_control_status,
            10,
        )

        self.joint_command_publisher = self.create_publisher(
            JointState,
            self.get_parameter("joint_command_topic").value,
            1,
        )
        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            "~/ee_pose_target",
            1,
        )

        self.timer = self.create_timer(
            1.0 / self.control_frequency_hz,
            self._control_callback,
        )

        self.get_logger().info(
            "Franka Pico teleop ready: side=%s, rate=%.1f Hz, joints=%s"
            % (
                self.source_type,
                self.control_frequency_hz,
                self.teleop.ik_solver.get_joint_names(),
            )
        )

    def _prepare_urdf(self, source_path: str) -> str:
        if not source_path.endswith(".xacro"):
            return source_path

        document = xacro.process_file(
            source_path,
            mappings={
                "arm_id": "fr3",
                "hand": "false",
                "ros2_control": "false",
            },
        )
        with tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".urdf",
            prefix="robo_orchard_fr3_",
            delete=False,
        ) as urdf_file:
            urdf_file.write(document.toprettyxml(indent="  "))
            self._generated_urdf_path = urdf_file.name
        return self._generated_urdf_path

    def destroy_node(self) -> bool:
        self._destroying = True
        self._reset_generation += 1
        self._reset_future = None
        generated_path = self._generated_urdf_path
        result = super().destroy_node()
        if generated_path and os.path.exists(generated_path):
            os.unlink(generated_path)
        return result

    def _on_control_status(self, message: ControlMode) -> None:
        resetting = message.data == ControlMode.RESETTING
        if resetting == self._manager_resetting:
            return
        self._manager_resetting = resetting
        if resetting:
            self.teleop.begin_reset()
        elif self._reset_future is None:
            self._finish_reset_state()

    def _vr_state_callback(self, message: VRState):
        now = monotonic()
        self._check_input_freshness(now)
        controller = (
            message.left_controller
            if self.source_type == "left"
            else message.right_controller
        )
        self._last_input_times["vr"] = now if controller.status != 0 else None
        action = self.teleop.update_vr_state(message)
        if action == Action.RESET:
            self.get_logger().info("Reset gesture received.")
            self._request_reset()

    def _joint_state_callback(self, message: JointState):
        positions_by_name = dict(
            zip(message.name, message.position, strict=False)
        )
        if not all(name in positions_by_name for name in FRANKA_JOINT_NAMES):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_joint_warning_ns > 2_000_000_000:
                self.get_logger().warning(
                    "Joint state does not contain all FR3 arm joints."
                )
                self._last_joint_warning_ns = now_ns
            return

        now = monotonic()
        self._check_input_freshness(now)
        self._last_input_times["joints"] = now
        self.current_joint_positions = [
            float(positions_by_name[name]) for name in FRANKA_JOINT_NAMES
        ]

    def _ee_pose_callback(self, message: PoseStamped):
        if self._ee_to_flange_matrix is None:
            return
        ee_matrix = pose_msg_to_matrix(message.pose)
        flange_pose = matrix_to_pose_msg(ee_matrix @ self._ee_to_flange_matrix)
        now = monotonic()
        self._check_input_freshness(now)
        self._last_input_times["pose"] = now
        self.teleop.update_robot_ee_pose(flange_pose)

    def _robot_state_callback(self, message: FrankaRobotState):
        if self._ee_to_flange_matrix is not None:
            return
        flange_to_ee = pose_msg_to_matrix(message.f_t_ee.pose)
        self._ee_to_flange_matrix = np.linalg.inv(flange_to_ee)
        if not self._tcp_transform_logged:
            translation = flange_to_ee[:3, 3]
            self.get_logger().info(
                "Using robot F_T_EE transform: xyz=[%.4f, %.4f, %.4f] m"
                % tuple(translation)
            )
            self._tcp_transform_logged = True

    def _check_input_freshness(self, now: float) -> bool:
        """Discard stale sessions using local monotonic receive times."""
        if all(
            received is not None and now - received <= self.stale_timeout_s
            for received in self._last_input_times.values()
        ):
            return True
        self.teleop.reset_session(require_rearm=True)
        return False

    def _control_callback(self):
        if (
            self._ee_to_flange_matrix is not None
            and self._robot_state_subscription is not None
        ):
            self.destroy_subscription(self._robot_state_subscription)
            self._robot_state_subscription = None

        if self._reset_future is not None or self._manager_resetting:
            return

        if not self._check_input_freshness(monotonic()):
            return

        if (
            self.current_joint_positions is None
            or self._ee_to_flange_matrix is None
        ):
            return

        self.teleop.update_robot_joint_state(self.current_joint_positions)
        result = self.teleop()
        if result is None:
            return

        self.target_pose_publisher.publish(
            PoseStamped(
                header=self._new_header(),
                pose=result.target_ee_pose,
            )
        )

        if result.solution is None:
            return
        if len(result.solution) != len(FRANKA_JOINT_NAMES):
            self.get_logger().error(
                f"IK returned {len(result.solution)} joints instead of 7."
            )
            return

        self.joint_command_publisher.publish(
            JointState(
                header=self._new_header(),
                name=FRANKA_JOINT_NAMES,
                position=[float(value) for value in result.solution],
            )
        )

    def _new_header(self) -> Header:
        return Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.base_link_name,
        )

    def _request_reset(self) -> None:
        if self._reset_future is not None or self._manager_resetting:
            self.get_logger().warning("Franka reset is already in progress.")
            return
        self.teleop.begin_reset()
        if not self.reset_client.service_is_ready():
            self.get_logger().warning(
                f"Franka reset service '{self.reset_service_name}' "
                "is unavailable."
            )
            self._finish_reset_state()
            return

        self.get_logger().info(
            f"Requesting Franka reset through '{self.reset_service_name}'."
        )
        try:
            future = self.reset_client.call_async(Trigger.Request())
        except Exception as error:
            self.get_logger().error(f"Franka reset request failed: {error}")
            self._finish_reset_state()
            return
        self._reset_generation += 1
        self._reset_future = future
        future.add_done_callback(
            partial(self._reset_result, self._reset_generation)
        )

    def _reset_result(self, generation: int, future) -> None:
        if self._destroying or generation != self._reset_generation:
            return
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Franka reset completed successfully.")
            else:
                self.get_logger().warning(
                    f"Franka reset failed: {response.message}"
                )
        except Exception as error:
            self.get_logger().error(f"Franka reset service failed: {error}")
        finally:
            self._finish_reset_state()
            self._reset_future = None

    def _finish_reset_state(self) -> None:
        if self._manager_resetting:
            return
        if self.current_joint_positions is not None:
            self.teleop.update_robot_joint_state(self.current_joint_positions)
        self.teleop.finish_reset()


def main(args=None):
    rclpy.init(args=args)
    node = FrankaPicoVRTeleOpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
