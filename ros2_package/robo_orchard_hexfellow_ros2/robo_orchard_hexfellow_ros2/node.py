# Project RoboOrchard
#
# Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
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
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions

from robo_orchard_deploy_msg_ros2.msg import InferenceStatus
from robo_orchard_hexfellow_ros2.tracking import Pose2, Settings, Tracker
from robo_orchard_teleop_msg_ros2.msg import ControlMode


def planar_pose(pose: object) -> Pose2:
    """Validate every pose component and project xyz/quaternion onto SE(2)."""
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.w, q.x, q.y, q.z)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Pose contains nonfinite components")
    norm = math.hypot(q.w, q.x, q.y, q.z)
    if norm < 1e-12:
        raise ValueError("Pose quaternion has zero norm")
    w, x, y, z = (v / norm for v in (q.w, q.x, q.y, q.z))
    return Pose2(
        p.x, p.y, math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    )


def stamp_ns(header: object) -> int:
    """Convert a ROS header stamp to integer nanoseconds."""
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class HexFellowTrackerNode(Node):
    """Single official-driver Twist publisher for AUTO tracking and teleop."""

    def __init__(self) -> None:
        super().__init__("hexfellow_tracker")
        defaults = {
            "target_topic": "/robot/chassis/pose_target",
            "teleop_topic": "/robot/chassis/teleop_cmd_vel",
            "output_topic": "/chassis/cmd_vel",
            "odom_topic": "/chassis/odom",
            "mode_topic": "/robot/control/status",
            "inference_status_topic": "/robot/inference_service/status",
            "odom_frame": "odom",
            "body_frame": "base_link",
            "control_rate": 100.0,
            **vars(Settings()),
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.tracker = Tracker(
            Settings(**{k: self._parameter(k) for k in vars(Settings())})
        )
        rate = self._parameter("control_rate")
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("control_rate must be finite and positive")
        for name in (
            defaults.keys() - vars(Settings()).keys() - {"control_rate"}
        ):
            if not self._parameter(name):
                raise ValueError(f"{name} must not be empty")
        topics = [
            self._parameter(name)
            for name in ("target_topic", "teleop_topic", "output_topic")
        ]
        if len(set(topics)) != len(topics):
            raise ValueError(
                "Command input and output topics must be distinct"
            )
        self.publisher = self.create_publisher(
            Twist, self._parameter("output_topic"), 10
        )
        self.create_subscription(
            Path, self._parameter("target_topic"), self._target, 10
        )
        self.create_subscription(
            Twist, self._parameter("teleop_topic"), self._teleop, 10
        )
        self.create_subscription(
            Odometry,
            self._parameter("odom_topic"),
            self._odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ControlMode, self._parameter("mode_topic"), self._mode, 10
        )
        self.create_subscription(
            InferenceStatus,
            self._parameter("inference_status_topic"),
            self._status,
            10,
        )
        self.create_timer(1 / rate, self._tick)

    def _parameter(self, name: str) -> object:
        return self.get_parameter(name).value

    def stop(self) -> None:
        """Cancel autonomous state and publish a neutral driver command."""
        self.tracker.cancel()
        self.publisher.publish(Twist())

    def _target(self, message: Path) -> None:
        try:
            if len(message.poses) != 1:
                raise ValueError("Target Path must contain exactly one pose")
            pose = message.poses[0]
            if message.header.frame_id != self._parameter("body_frame"):
                raise ValueError("Unexpected target reference frame")
            if pose.header.frame_id != message.header.frame_id:
                raise ValueError("Target pose frame must match Path frame")
            self.tracker.target_update(
                stamp_ns(message.header),
                planar_pose(pose.pose),
                time.monotonic(),
                sample_stamp=stamp_ns(pose.header),
            )
        except ValueError as exc:
            # Invalid AUTO input must not interrupt a valid teleop stream.
            self.tracker.cancel()
            if not self.tracker.teleop_allowed(time.monotonic()):
                self.stop()
            self.get_logger().warning(f"Rejected pose target: {exc}")

    def _odom(self, message: Odometry) -> None:
        try:
            if message.header.frame_id != self._parameter(
                "odom_frame"
            ) or message.child_frame_id != self._parameter("body_frame"):
                raise ValueError("Unexpected odometry frames")
            reset = self.tracker.odom_update(
                stamp_ns(message.header),
                planar_pose(message.pose.pose),
                time.monotonic(),
            )
            if reset:
                self.get_logger().warning(
                    "Odometry discontinuity; target cancelled"
                )
        except ValueError as exc:
            self.tracker.reset_odometry()
            reset = True
            self.get_logger().warning(f"Rejected odometry: {exc}")
        if reset and not self.tracker.teleop_allowed(time.monotonic()):
            self.stop()

    def _mode(self, message: ControlMode) -> None:
        if self.tracker.mode_update(message.data, time.monotonic()):
            self.stop()

    def _status(self, message: InferenceStatus) -> None:
        self.tracker.status_update(
            message.data == InferenceStatus.ENABLED, time.monotonic()
        )
        if not self.tracker.enabled and self.tracker.mode == ControlMode.AUTO:
            self.stop()

    def _teleop(self, message: Twist) -> None:
        now = time.monotonic()
        if not self.tracker.teleop_allowed(now):
            return
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if not all(math.isfinite(v) for v in values):
            self.stop()
            self.tracker.teleop_received = -math.inf
            self.get_logger().warning("Rejected nonfinite teleop Twist")
            return
        self.tracker.teleop_received = now
        self.publisher.publish(message)

    def _tick(self) -> None:
        command = self.tracker.command(time.monotonic())
        if command is not None:
            message = Twist()
            message.linear.x, message.angular.z = command
            self.publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the tracker; always request a stop before orderly shutdown."""
    # Keep the publisher's context valid until the final stop is requested.
    # rclpy's default signal handler shuts it down before spin unwinds.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    handlers = {}

    def interrupt(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            handlers[signum] = signal.signal(signum, interrupt)
        node = HexFellowTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    if rclpy.ok():
                        node.stop()
                finally:
                    node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for signum, previous in handlers.items():
                signal.signal(signum, previous)
