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
import os
from dataclasses import dataclass

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState


JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
)

DEFAULT_CENTER = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
DEFAULT_AMPLITUDES = (0.12, 0.10, 0.10, 0.12, 0.08, 0.10, 0.0)
DEFAULT_FREQUENCIES = (0.07, 0.11, 0.09, 0.13, 0.10, 0.15, 0.0)
DEFAULT_PHASES = (0.0, -1.5708, 3.1416, 0.7854, 2.3562, 3.9270, 0.0)


@dataclass
class ArmState:
    name: str
    current_positions: list[float] | None = None
    trajectory_center: list[float] | None = None


class ScriptedJointMasterNode(Node):
    # Publishes a deterministic joint-space trajectory for Piper followers.

    def __init__(self) -> None:
        super().__init__("scripted_joint_master")

        self.declare_parameter("left_command_topic", "/left_algo_cmd")
        self.declare_parameter("right_command_topic", "/right_algo_cmd")
        self.declare_parameter("left_state_topic", "/puppet/joint_left")
        self.declare_parameter("right_state_topic", "/puppet/joint_right")
        self.declare_parameter("publish_left", True)
        self.declare_parameter("publish_right", True)
        self.declare_parameter("use_current_state", True)
        self.declare_parameter("wait_for_state_timeout_s", 2.0)
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("start_delay_s", 1.0)
        self.declare_parameter("start_trigger_file", "")
        # Kept for launch/config compatibility; scripted motion ignores it.
        self.declare_parameter("ramp_s", 0.0)
        self.declare_parameter("duration_s", 10.0)
        self.declare_parameter("amplitude_scale", 1.0)
        self.declare_parameter("frequency_scale", 1.0)
        self.declare_parameter("mirror_right", False)
        self.declare_parameter("center_left", list(DEFAULT_CENTER))
        self.declare_parameter("center_right", list(DEFAULT_CENTER))
        self.declare_parameter("amplitudes", list(DEFAULT_AMPLITUDES))
        self.declare_parameter("frequencies", list(DEFAULT_FREQUENCIES))
        self.declare_parameter("phases", list(DEFAULT_PHASES))

        self.publish_left = self.get_parameter("publish_left").value
        self.publish_right = self.get_parameter("publish_right").value
        self.use_current_state = self.get_parameter("use_current_state").value
        self.wait_for_state_timeout_s = float(
            self.get_parameter("wait_for_state_timeout_s").value
        )
        self.rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.start_delay_s = max(
            0.0, float(self.get_parameter("start_delay_s").value)
        )
        self.start_trigger_file = str(
            self.get_parameter("start_trigger_file").value or ""
        )
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.amplitude_scale = float(
            self.get_parameter("amplitude_scale").value
        )
        self.frequency_scale = max(
            0.0, float(self.get_parameter("frequency_scale").value)
        )
        self.mirror_right = self.get_parameter("mirror_right").value

        self.center_left = self._joint_array_parameter("center_left")
        self.center_right = self._joint_array_parameter("center_right")
        self.amplitudes = self._joint_array_parameter("amplitudes")
        self.frequencies = self._joint_array_parameter("frequencies")
        self.phases = self._joint_array_parameter("phases")

        self.left = ArmState("left")
        self.right = ArmState("right")
        self._started = False
        self._done = False
        self._done_future: Future = Future()
        self._state_wait_logged = False
        self._start_time = self.get_clock().now()

        left_command_topic = self.get_parameter("left_command_topic").value
        right_command_topic = self.get_parameter("right_command_topic").value
        left_state_topic = self.get_parameter("left_state_topic").value
        right_state_topic = self.get_parameter("right_state_topic").value

        self.left_pub = self.create_publisher(JointState, left_command_topic, 1)
        self.right_pub = self.create_publisher(JointState, right_command_topic, 1)

        if self.use_current_state:
            self.create_subscription(
                JointState,
                left_state_topic,
                lambda msg: self._state_callback(self.left, msg),
                1,
            )
            self.create_subscription(
                JointState,
                right_state_topic,
                lambda msg: self._state_callback(self.right, msg),
                1,
            )

        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_callback)
        self.get_logger().info(
            "Scripted joint master publishing "
            f"left={self.publish_left} to {left_command_topic}, "
            f"right={self.publish_right} to {right_command_topic}, "
            f"rate={self.rate_hz:.1f} Hz, duration={self.duration_s:.1f}s"
        )

    def _joint_array_parameter(self, name: str) -> list[float]:
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != len(JOINT_NAMES):
            raise ValueError(
                f"Parameter {name} must have {len(JOINT_NAMES)} values; "
                f"got {len(values)}"
            )
        return values

    def _state_callback(self, arm: ArmState, msg: JointState) -> None:
        if len(msg.position) < len(JOINT_NAMES):
            return
        arm.current_positions = [
            float(value) for value in msg.position[: len(JOINT_NAMES)]
        ]

    def _ready_to_start(self, elapsed: float) -> bool:
        if elapsed < self.start_delay_s:
            return False
        if self.start_trigger_file and not os.path.exists(
            self.start_trigger_file
        ):
            return False
        if not self.use_current_state:
            return True

        left_ready = (
            (not self.publish_left) or self.left.current_positions is not None
        )
        right_ready = (
            (not self.publish_right) or self.right.current_positions is not None
        )
        if left_ready and right_ready:
            return True

        if (
            not self._state_wait_logged
            and elapsed >= self.start_delay_s + self.wait_for_state_timeout_s
        ):
            self._state_wait_logged = True
            self.get_logger().warning(
                "Waiting for follower joint state before starting scripted "
                "motion; no command will be published until the current "
                "position is known."
            )
        return False

    def _trajectory_offset_at_zero(self, right: bool) -> list[float]:
        offsets = []
        for idx, (amplitude, phase) in enumerate(
            zip(self.amplitudes, self.phases, strict=True)
        ):
            sign = -1.0 if right and self.mirror_right and idx < 6 else 1.0
            offsets.append(
                sign * self.amplitude_scale * amplitude * math.sin(phase)
            )
        return offsets

    def _initialize_trajectory_center(
        self, arm: ArmState, nominal_center: list[float], right: bool
    ) -> list[float]:
        if arm.trajectory_center is not None:
            return arm.trajectory_center

        if self.use_current_state and arm.current_positions is not None:
            offsets = self._trajectory_offset_at_zero(right=right)
            arm.trajectory_center = [
                current_position - offset
                for current_position, offset in zip(
                    arm.current_positions, offsets, strict=True
                )
            ]
            self.get_logger().info(
                f"{arm.name} scripted trajectory starts at current state: "
                f"{arm.current_positions}"
            )
        else:
            arm.trajectory_center = nominal_center.copy()
            self.get_logger().info(
                f"{arm.name} scripted trajectory uses configured center: "
                f"{arm.trajectory_center}"
            )
        return arm.trajectory_center

    def _trajectory(self, center: list[float], t: float, right: bool) -> list[float]:
        positions = []
        for idx, (base, amplitude, frequency, phase) in enumerate(
            zip(center, self.amplitudes, self.frequencies, self.phases, strict=True)
        ):
            sign = -1.0 if right and self.mirror_right and idx < 6 else 1.0
            offset = (
                sign
                * self.amplitude_scale
                * amplitude
                * math.sin(
                    2.0 * math.pi * frequency * self.frequency_scale * t
                    + phase
                )
            )
            positions.append(base + offset)
        return positions

    def _publish(self, publisher, positions: list[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = positions
        msg.velocity = [0.0] * len(JOINT_NAMES)
        msg.effort = [0.0] * len(JOINT_NAMES)
        publisher.publish(msg)

    def _timer_callback(self) -> None:
        if self._done:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9
        if not self._started:
            if not self._ready_to_start(elapsed):
                return
            self._started = True
            if self.publish_left:
                self._initialize_trajectory_center(
                    self.left, self.center_left, right=False
                )
            if self.publish_right:
                self._initialize_trajectory_center(
                    self.right, self.center_right, right=True
                )
            self._start_time = self.get_clock().now()
            elapsed = 0.0
            self.get_logger().info("Starting scripted joint trajectory.")

        if self.duration_s > 0.0 and elapsed > self.duration_s:
            self._done = True
            self.get_logger().info("Scripted joint trajectory complete.")
            if not self._done_future.done():
                self._done_future.set_result(None)
            return

        if self.publish_left:
            left_positions = self._trajectory(
                self.left.trajectory_center or self.center_left,
                elapsed,
                right=False,
            )
            self._publish(self.left_pub, left_positions)
        if self.publish_right:
            right_positions = self._trajectory(
                self.right.trajectory_center or self.center_right,
                elapsed,
                right=True,
            )
            self._publish(self.right_pub, right_positions)


def main(args=None):
    rclpy.init(args=args)
    node = ScriptedJointMasterNode()
    try:
        rclpy.spin_until_future_complete(node, node._done_future)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
