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
from geometry_msgs.msg import Twist
from rclpy.node import Node, ParameterDescriptor

from robo_orchard_pico_msg_ros2.msg import VRState


class HexfellowChassisPicoVRTeleOpNode(Node):
    """Publish left-stick Twist candidates for Control Manager routing.

    Linear speed is in m/s and yaw rate is in rad/s. Holding the left stick
    button toggles local activation; this does not change the Manager mode.
    """

    def __init__(self, **kwargs: Any) -> None:
        super().__init__("hexfellow_chassis_pico_vr_teleop", **kwargs)

        self.declare_parameter(
            "vr_topic",
            "/pico_bridge/vr_state",
            ParameterDescriptor(description="Input Pico VRState topic."),
        )
        self.declare_parameter(
            "cmd_vel_topic",
            "/hexfellow_teleop/cmd_vel",
            ParameterDescriptor(description="Chassis override Twist topic."),
        )
        self.declare_parameter(
            "publish_hz",
            30.0,
            ParameterDescriptor(description="Command publish frequency."),
        )
        self.declare_parameter(
            "max_linear_x",
            0.2,
            ParameterDescriptor(description="Max forward/backward speed."),
        )
        self.declare_parameter(
            "max_angular_z",
            0.2,
            ParameterDescriptor(description="Max yaw rate."),
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
                description="Stop if no valid VR frame arrives."
            ),
        )
        self.declare_parameter(
            "toggle_hold_time_s",
            1.0,
            ParameterDescriptor(
                description=(
                    "Long-press duration for left axis click to toggle "
                    "chassis control."
                )
            ),
        )
        self.declare_parameter(
            "axis_y_sign",
            1.0,
            ParameterDescriptor(description="Sign for axis_y -> linear.x."),
        )
        self.declare_parameter(
            "axis_x_sign",
            -1.0,
            ParameterDescriptor(description="Sign for axis_x -> angular.z."),
        )

        self.vr_topic = self.get_parameter("vr_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.stale_timeout_s = float(
            self.get_parameter("stale_timeout_s").value
        )
        self.toggle_hold_time_s = float(
            self.get_parameter("toggle_hold_time_s").value
        )
        self.axis_y_sign = float(self.get_parameter("axis_y_sign").value)
        self.axis_x_sign = float(self.get_parameter("axis_x_sign").value)

        self._axis_x = 0.0
        self._axis_y = 0.0
        self._last_valid_vr_time_s: float | None = None
        self._toggle_press_start_s: float | None = None
        self._toggle_consumed = False
        self._control_active = False

        self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 1)
        self._vr_sub = self.create_subscription(
            VRState,
            self.vr_topic,
            self._on_vr_state,
            1,
        )
        self._timer = self.create_timer(
            1.0 / self.publish_hz,
            self._publish_cmd_vel,
        )

        self.get_logger().info(
            "Hexfellow chassis Pico VR teleop initialized: "
            f"vr_topic={self.vr_topic}, cmd_vel_topic={self.cmd_vel_topic}, "
            f"max_linear_x={self.max_linear_x}, "
            f"max_angular_z={self.max_angular_z}, deadzone={self.deadzone}, "
            f"toggle_hold_time_s={self.toggle_hold_time_s}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @property
    def control_active(self) -> bool:
        """Whether chassis control is currently active."""
        return self._control_active

    def _sanitize_axis(self, value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        value = max(-1.0, min(1.0, value))
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def _on_vr_state(self, msg: VRState) -> None:
        now_s = self._now_s()
        left_controller = msg.left_controller
        self._last_valid_vr_time_s = now_s

        if left_controller.status == 0:
            self._deactivate_and_stop("left controller is invalid")
        else:
            self._axis_x = self._sanitize_axis(left_controller.axis_x)
            self._axis_y = self._sanitize_axis(left_controller.axis_y)
            self._update_toggle_state(left_controller.axis_click, now_s)

    def _update_toggle_state(self, is_pressed: bool, now_s: float) -> None:
        if not is_pressed:
            self._reset_toggle_state()
            return

        if self._toggle_press_start_s is None:
            self._toggle_press_start_s = now_s
            return

        press_duration_s = now_s - self._toggle_press_start_s
        if (
            press_duration_s >= self.toggle_hold_time_s
            and not self._toggle_consumed
        ):
            self._control_active = not self._control_active
            self._toggle_consumed = True
            if self._control_active:
                self.get_logger().info("Hexfellow chassis control activated.")
            else:
                self.get_logger().info(
                    "Hexfellow chassis control deactivated."
                )
                self.publish_stop()

    def _reset_toggle_state(self) -> None:
        self._toggle_press_start_s = None
        self._toggle_consumed = False

    def _has_fresh_vr_state(self) -> bool:
        if self._last_valid_vr_time_s is None:
            return False
        return (
            self._now_s() - self._last_valid_vr_time_s < self.stale_timeout_s
        )

    def _deactivate_and_stop(self, reason: str) -> None:
        was_active = self._control_active
        self._control_active = False
        self._axis_x = 0.0
        self._axis_y = 0.0
        self._reset_toggle_state()
        if was_active:
            self.get_logger().warn(
                f"Hexfellow chassis control deactivated: {reason}."
            )
        self.publish_stop()

    def _publish_cmd_vel(self) -> None:
        if not self._has_fresh_vr_state():
            self._deactivate_and_stop("VR state is stale")
            return

        if not self._control_active:
            self.publish_stop()
            return

        msg = Twist()
        msg.linear.x = self.axis_y_sign * self._axis_y * self.max_linear_x
        msg.angular.z = self.axis_x_sign * self._axis_x * self.max_angular_z
        self._cmd_pub.publish(msg)

    def publish_stop(self) -> None:
        """Publish a zero-velocity candidate on the override topic."""
        self._cmd_pub.publish(Twist())


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HexfellowChassisPicoVRTeleOpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
