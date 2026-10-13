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

import math
from typing import Any

import rclpy
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import JointState

from robo_orchard_pico_msg_ros2.msg import VRState


class HexfellowLiftPicoVRTeleOpNode(Node):
    """Integrate right-stick Y into lift position candidates in meters.

    Targets for joint1 require fresh position feedback and remain within a
    bounded lead of that feedback. Smaller positions correspond to lifting up.
    Candidates are routed by Control Manager, not sent directly to hardware.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__("hexfellow_lift_pico_vr_teleop", **kwargs)

        self.declare_parameter(
            "vr_topic",
            "/pico_bridge/vr_state",
            ParameterDescriptor(description="Input Pico VRState topic."),
        )
        self.declare_parameter(
            "joint_cmd_topic",
            "/hexfellow_teleop/lift_joint_cmd",
            ParameterDescriptor(description="Lift override JointState topic."),
        )
        self.declare_parameter(
            "state_topic",
            "/lift/motor_states",
            ParameterDescriptor(description="Input lift JointState topic."),
        )
        self.declare_parameter(
            "publish_hz",
            30.0,
            ParameterDescriptor(description="Command publish frequency."),
        )
        self.declare_parameter(
            "deadzone",
            0.08,
            ParameterDescriptor(description="Joystick deadzone in [0, 1]."),
        )
        self.declare_parameter(
            "stale_timeout_s",
            0.3,
            ParameterDescriptor(
                description="Timeout for VR input and lift position feedback."
            ),
        )
        self.declare_parameter(
            "min_position",
            0.0,
            ParameterDescriptor(description="Lift highest position."),
        )
        self.declare_parameter(
            "max_position",
            0.6,
            ParameterDescriptor(description="Lift lowest position."),
        )
        self.declare_parameter(
            "position_rate",
            0.08,
            ParameterDescriptor(
                description="Lift target position change rate in m/s."
            ),
        )
        self.declare_parameter(
            "target_lead_limit",
            0.015,
            ParameterDescriptor(
                description=(
                    "Max distance in meters that lift target can lead the "
                    "measured lift position."
                )
            ),
        )
        self.declare_parameter(
            "axis_y_sign",
            -1.0,
            ParameterDescriptor(
                description=(
                    "Sign for right axis_y -> lift position delta. "
                    "Default: push up decreases position, lifting up."
                )
            ),
        )

        self.vr_topic = self.get_parameter("vr_topic").value
        self.joint_cmd_topic = self.get_parameter("joint_cmd_topic").value
        self.state_topic = self.get_parameter("state_topic").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.stale_timeout_s = float(
            self.get_parameter("stale_timeout_s").value
        )
        self.min_position = float(self.get_parameter("min_position").value)
        self.max_position = float(self.get_parameter("max_position").value)
        self.target_position = self.min_position
        self.position_rate = float(self.get_parameter("position_rate").value)
        self.target_lead_limit = float(
            self.get_parameter("target_lead_limit").value
        )
        self.axis_y_sign = float(self.get_parameter("axis_y_sign").value)

        self._right_axis_y = 0.0
        self._last_valid_vr_time_s: float | None = None
        self._last_timer_time_s = self._now_s()
        self._last_state_time_s: float | None = None
        self._current_position: float | None = None
        self._motion_active = False

        self._lift_pub = self.create_publisher(
            JointState,
            self.joint_cmd_topic,
            1,
        )
        self._vr_sub = self.create_subscription(
            VRState,
            self.vr_topic,
            self._on_vr_state,
            1,
        )
        self._state_sub = self.create_subscription(
            JointState,
            self.state_topic,
            self._on_state,
            1,
        )
        self._timer = self.create_timer(
            1.0 / self.publish_hz,
            self._publish_joint_cmd,
        )

        self.get_logger().info(
            "Hexfellow lift Pico VR teleop initialized: "
            f"vr_topic={self.vr_topic}, "
            f"joint_cmd_topic={self.joint_cmd_topic}, "
            f"state_topic={self.state_topic}, "
            f"range=[{self.min_position}, {self.max_position}], "
            f"position_rate={self.position_rate}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _sanitize_axis(self, value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        value = max(-1.0, min(1.0, value))
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _on_vr_state(self, msg: VRState) -> None:
        now_s = self._now_s()
        right_controller = msg.right_controller
        self._last_valid_vr_time_s = now_s

        if right_controller.status == 0:
            self._right_axis_y = 0.0
        else:
            self._right_axis_y = self._sanitize_axis(right_controller.axis_y)

    def _on_state(self, msg: JointState) -> None:
        position_index = self._joint_position_index(msg)
        if position_index is None:
            return

        position = float(msg.position[position_index])
        if not math.isfinite(position):
            return
        current_position = self._clamp_position(position)
        self._current_position = current_position
        if not self._has_fresh_state() or not self._motion_active:
            self.target_position = current_position
        self._last_state_time_s = self._now_s()

    def _joint_position_index(self, msg: JointState) -> int | None:
        if msg.name:
            try:
                position_index = msg.name.index("joint1")
            except ValueError:
                return None
        elif len(msg.position) == 1:
            position_index = 0
        else:
            return None

        if position_index >= len(msg.position):
            return None
        return position_index

    def _has_fresh_vr_state(self) -> bool:
        if self._last_valid_vr_time_s is None:
            return False
        return (
            self._now_s() - self._last_valid_vr_time_s < self.stale_timeout_s
        )

    def _has_fresh_state(self) -> bool:
        if self._last_state_time_s is None:
            return False
        return self._now_s() - self._last_state_time_s < self.stale_timeout_s

    def _publish_joint_cmd(self) -> None:
        now_s = self._now_s()
        elapsed_s = now_s - self._last_timer_time_s
        self._last_timer_time_s = now_s

        if not self._has_fresh_state():
            # Keep the pending hold until feedback can replace the old target.
            return

        if not self._has_fresh_vr_state() or self._right_axis_y == 0.0:
            self._hold_at_current_position()
            return

        position_delta = (
            self.axis_y_sign
            * self._right_axis_y
            * self.position_rate
            * max(0.0, elapsed_s)
        )
        self.target_position = self._clamp_position(
            self.target_position + position_delta
        )
        self.target_position = self._clamp_target_to_current_window(
            self.target_position
        )
        self._publish_position(self.target_position)
        self._motion_active = True

    def _hold_at_current_position(self) -> None:
        if not self._motion_active:
            return
        assert self._current_position is not None
        self.target_position = self._current_position
        self._publish_position(self.target_position)
        self._motion_active = False

    def _publish_position(self, position: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1"]
        msg.position = [position]
        self._lift_pub.publish(msg)

    def _clamp_position(self, position: float) -> float:
        return max(
            self.min_position,
            min(self.max_position, position),
        )

    def _clamp_target_to_current_window(self, target_position: float) -> float:
        assert self._current_position is not None
        lead_limit = max(0.0, self.target_lead_limit)
        min_position = self._clamp_position(
            self._current_position - lead_limit
        )
        max_position = self._clamp_position(
            self._current_position + lead_limit
        )
        return max(min_position, min(max_position, target_position))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HexfellowLiftPicoVRTeleOpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
