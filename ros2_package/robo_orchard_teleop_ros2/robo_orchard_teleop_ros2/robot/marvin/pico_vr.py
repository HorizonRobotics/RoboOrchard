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

from __future__ import annotations
import math
import os
from functools import partial
from typing import Literal

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from robo_orchard_pico_msg_ros2.msg import VRState
from robo_orchard_teleop_ros2.bridge.pico.intent import (
    GripperIntent,
    ResetIntent,
)
from robo_orchard_teleop_ros2.bridge.pico.teleop import Action, VRTeleOp

ArmSide = Literal["left", "right"]

MARVIN_JOINT_NAMES = {
    "left": [f"Joint{index}_L" for index in range(1, 8)],
    "right": [f"Joint{index}_R" for index in range(1, 8)],
}
MARVIN_EE_LINK_NAMES = {
    "left": "TCP_Link_L",
    "right": "TCP_Link_R",
}
MARVIN_RESET_BUTTONS = {
    "left": "X",
    "right": "A",
}


def _ordered_joint_positions(
    message: JointState, joint_names: list[str]
) -> list[float] | None:
    """Return joint positions in the requested order when all are present."""
    positions_by_name = dict(zip(message.name, message.position, strict=False))
    if not all(name in positions_by_name for name in joint_names):
        return None
    positions = [float(positions_by_name[name]) for name in joint_names]
    if not all(math.isfinite(value) for value in positions):
        return None
    return positions


class MarvinPicoVRTeleOpNode(Node):
    """Convert dual Pico controller motion into Marvin joint targets."""

    def __init__(self, **kwargs):
        super().__init__("marvin_pico_vr_teleop", **kwargs)

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("base_link_name", "robot_stand")
        self.declare_parameter(
            "left_ee_link_name", MARVIN_EE_LINK_NAMES["left"]
        )
        self.declare_parameter(
            "right_ee_link_name", MARVIN_EE_LINK_NAMES["right"]
        )
        self.declare_parameter("control_frequency_hz", 30.0)
        self.declare_parameter("translation_scale_factor", 1.0)
        self.declare_parameter("pose_low_pass_alpha", 0.25)

        self.urdf_path = str(self.get_parameter("urdf_path").value)
        if not os.path.isfile(self.urdf_path):
            raise FileNotFoundError(
                f"Marvin URDF does not exist: {self.urdf_path}"
            )

        self.base_link_name = str(self.get_parameter("base_link_name").value)
        ee_link_names = {
            "left": str(self.get_parameter("left_ee_link_name").value),
            "right": str(self.get_parameter("right_ee_link_name").value),
        }
        control_frequency_hz = float(
            self.get_parameter("control_frequency_hz").value
        )
        translation_scale_factor = float(
            self.get_parameter("translation_scale_factor").value
        )
        pose_low_pass_alpha = float(
            self.get_parameter("pose_low_pass_alpha").value
        )
        if (
            not math.isfinite(control_frequency_hz)
            or control_frequency_hz <= 0
        ):
            raise ValueError(
                "control_frequency_hz must be positive and finite"
            )
        if (
            not math.isfinite(translation_scale_factor)
            or translation_scale_factor <= 0
        ):
            raise ValueError(
                "translation_scale_factor must be positive and finite"
            )
        if (
            not math.isfinite(pose_low_pass_alpha)
            or not 0 < pose_low_pass_alpha <= 1
        ):
            raise ValueError("pose_low_pass_alpha must be in (0, 1]")

        self.teleops: dict[ArmSide, VRTeleOp] = {}
        for side in ("left", "right"):
            self.teleops[side] = VRTeleOp(
                source_type=side,
                urdf_path=self.urdf_path,
                base_link_name=self.base_link_name,
                ee_link_name=ee_link_names[side],
                scale_factor=translation_scale_factor,
                pose_low_pass_alpha=pose_low_pass_alpha,
                trigger_intent=GripperIntent(
                    source_type=side,
                    value_thresh=0.5,
                    thresh=1.0,
                ),
                reset_intent=ResetIntent(
                    source_type=MARVIN_RESET_BUTTONS[side],
                    thresh=1.0,
                ),
                reset_callback=None,
                logger=self.get_logger(),
            )
            actual_joint_names = self.teleops[side].ik_solver.get_joint_names()
            if actual_joint_names != MARVIN_JOINT_NAMES[side]:
                raise ValueError(
                    f"Marvin {side} IK chain joints must be "
                    f"{MARVIN_JOINT_NAMES[side]}, got {actual_joint_names}"
                )

        self.current_joint_positions: dict[ArmSide, list[float] | None] = {
            "left": None,
            "right": None,
        }
        self.reset_futures = {"left": None, "right": None}
        self.last_joint_warning_ns = {"left": 0, "right": 0}

        self.reset_clients = {
            side: self.create_client(Trigger, f"/robot/{side}/reset_ctrl")
            for side in ("left", "right")
        }
        self.command_publishers = {
            side: self.create_publisher(
                JointState, f"/robot/{side}/joint_cmd", 1
            )
            for side in ("left", "right")
        }
        self.target_pose_publishers = {
            side: self.create_publisher(
                PoseStamped, f"/robot/{side}/ee_pose_target", 1
            )
            for side in ("left", "right")
        }

        self.create_subscription(
            VRState,
            "vr_state",
            self._vr_state_callback,
            1,
        )
        for side in ("left", "right"):
            self.create_subscription(
                JointState,
                f"/robot/{side}/joint_state",
                partial(self._joint_state_callback, side),
                1,
            )
            self.create_subscription(
                PoseStamped,
                f"/robot/{side}/ee_pose",
                partial(self._ee_pose_callback, side),
                1,
            )

        self.timer = self.create_timer(
            1.0 / control_frequency_hz,
            self._control_callback,
        )
        self.get_logger().info(
            "Marvin Pico teleop ready: rate=%.1f Hz, base=%s, urdf=%s"
            % (control_frequency_hz, self.base_link_name, self.urdf_path)
        )

    def _vr_state_callback(self, message: VRState):
        for side in ("left", "right"):
            action = self.teleops[side].update_vr_state(message)
            if action == Action.RESET:
                self.get_logger().info(
                    f"Marvin {side} reset gesture received."
                )
                self._request_reset(side)

    def _joint_state_callback(
        self, side: ArmSide, message: JointState
    ) -> None:
        positions = _ordered_joint_positions(message, MARVIN_JOINT_NAMES[side])
        if positions is None:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_joint_warning_ns[side] > 2_000_000_000:
                self.get_logger().warning(
                    f"Marvin {side} joint state is incomplete or invalid."
                )
                self.last_joint_warning_ns[side] = now_ns
            return
        self.current_joint_positions[side] = positions
        self.teleops[side].update_robot_joint_state(positions)

    def _ee_pose_callback(self, side: ArmSide, message: PoseStamped) -> None:
        if message.header.frame_id not in ("", self.base_link_name):
            self.get_logger().warning(
                f"Ignoring Marvin {side} EE pose in frame "
                f"'{message.header.frame_id}', expected "
                f"'{self.base_link_name}'."
            )
            return
        self.teleops[side].update_robot_ee_pose(message.pose)

    def _control_callback(self) -> None:
        for side in ("left", "right"):
            if (
                self.reset_futures[side] is not None
                or self.current_joint_positions[side] is None
            ):
                continue
            result = self.teleops[side]()
            if result is None:
                continue

            header = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=self.base_link_name,
            )
            self.target_pose_publishers[side].publish(
                PoseStamped(header=header, pose=result.target_ee_pose)
            )
            if result.solution is None:
                continue
            if len(result.solution) != len(
                MARVIN_JOINT_NAMES[side]
            ) or not all(math.isfinite(value) for value in result.solution):
                self.get_logger().error(
                    f"Marvin {side} IK returned an invalid joint solution."
                )
                continue
            self.command_publishers[side].publish(
                JointState(
                    header=header,
                    name=MARVIN_JOINT_NAMES[side],
                    position=[float(value) for value in result.solution],
                )
            )

    def _request_reset(self, side: ArmSide) -> None:
        if self.reset_futures[side] is not None:
            self.get_logger().warning(
                f"Marvin {side} reset is already in progress."
            )
            return
        client = self.reset_clients[side]
        if not client.service_is_ready():
            self.get_logger().warning(
                f"Marvin {side} reset service is unavailable."
            )
            self.teleops[side].finish_reset()
            return

        self.get_logger().info(f"Requesting Marvin {side} reset.")
        future = client.call_async(Trigger.Request())
        self.reset_futures[side] = future
        future.add_done_callback(partial(self._reset_result, side))

    def _reset_result(self, side: ArmSide, future) -> None:
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Marvin {side} reset completed successfully."
                )
            else:
                self.get_logger().warning(
                    f"Marvin {side} reset failed: {response.message}"
                )
        except Exception as error:
            self.get_logger().error(
                f"Marvin {side} reset service failed: {error}"
            )
        finally:
            positions = self.current_joint_positions[side]
            if positions is not None:
                self.teleops[side].update_robot_joint_state(positions)
            self.teleops[side].finish_reset()
            self.reset_futures[side] = None


def main(args=None):
    rclpy.init(args=args)
    node = MarvinPicoVRTeleOpNode()
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
