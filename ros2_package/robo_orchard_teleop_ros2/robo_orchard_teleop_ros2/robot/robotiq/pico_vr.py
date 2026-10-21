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

from math import isfinite
from time import monotonic

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from robo_orchard_pico_msg_ros2.msg import VRState
from robo_orchard_teleop_msg_ros2.msg import TeleopMatchState

MAX_OPENING_M = 0.085


class RobotiqPicoVRTeleOpNode(Node):
    """Produce standard-finger 2F-85 targets and initial-match heartbeats.

    Trigger zero maps to fully open, one to closed. Grip activation and
    command forwarding belong to a separate coordinator, not this source.
    """

    def __init__(self, **kwargs):
        super().__init__("robotiq_pico_vr_teleop", **kwargs)
        defaults = {
            "source_type": "right",
            "vr_topic": "/pico_bridge/vr_state",
            "joint_state_topic": "/robotiq/joint_states",
            "driver_status_topic": "/robotiq/status",
            "joint_command_topic": "/robotiq_teleop/candidate",
            "match_state_topic": "/robotiq_teleop/match_state",
            "joint_name": "robotiq_gripper",
            "publish_hz": 30.0,
            "stale_timeout_s": 0.3,
            "max_clock_skew_s": 0.05,
            "match_tolerance_m": 0.005,
            "match_dwell_s": 0.3,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.source_type = self.get_parameter("source_type").value
        if self.source_type not in ("left", "right"):
            raise ValueError("source_type must be 'left' or 'right'")
        self.joint_name = self.get_parameter("joint_name").value
        if not self.joint_name:
            raise ValueError("joint_name must not be empty")
        for name in (
            "publish_hz",
            "stale_timeout_s",
            "max_clock_skew_s",
            "match_tolerance_m",
            "match_dwell_s",
        ):
            value = float(self.get_parameter(name).value)
            if not isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
            if name in ("publish_hz", "stale_timeout_s") and value == 0.0:
                raise ValueError(f"{name} must be positive")
            setattr(self, name, value)

        self._received: dict[str, float | None] = dict.fromkeys(
            ("vr", "joints", "status")
        )
        self._source_stamps: dict[str, int] = dict.fromkeys(
            ("joints", "status"), 0
        )
        self._target: float | None = None
        self._measured: float | None = None
        self._driver_reason = "Waiting for driver status."
        self._match_since: float | None = None
        self._last_output_stamp = 0
        self._ready = False
        self._loss_id = 0
        self.create_subscription(
            VRState,
            self.get_parameter("vr_topic").value,
            self._vr_callback,
            1,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("joint_state_topic").value,
            self._joint_callback,
            1,
        )
        self.create_subscription(
            DiagnosticArray,
            self.get_parameter("driver_status_topic").value,
            self._status_callback,
            1,
        )
        self.joint_command_publisher = self.create_publisher(
            JointState, self.get_parameter("joint_command_topic").value, 1
        )
        self.match_state_publisher = self.create_publisher(
            TeleopMatchState,
            self.get_parameter("match_state_topic").value,
            1,
        )
        self.timer = self.create_timer(1.0 / self.publish_hz, self._publish)

    def _stamp_age_valid(self, stamp_ns: int) -> bool:
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        return (
            stamp_ns > 0
            and -self.max_clock_skew_s <= age_s <= self.stale_timeout_s
        )

    def _accept_stamp(self, source: str, header: Header) -> bool:
        self._readiness(monotonic())
        stamp = header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if (
            not self._stamp_age_valid(stamp_ns)
            or stamp_ns <= self._source_stamps[source]
        ):
            self._invalidate(source)
            return False
        self._source_stamps[source] = stamp_ns
        self._received[source] = monotonic()
        return True

    def _invalidate(self, source: str) -> None:
        self._received[source] = None
        self._record_loss()

    def _record_loss(self) -> None:
        if self._ready:
            self._loss_id += 1
        self._ready = False
        self._match_since = None

    def _vr_callback(self, message: VRState) -> None:
        now = monotonic()
        self._readiness(now)
        controller = (
            message.left_controller
            if self.source_type == "left"
            else message.right_controller
        )
        if (
            controller.status == 0
            or not isfinite(controller.trigger)
            or not 0.0 <= controller.trigger <= 1.0
        ):
            self._invalidate("vr")
            return
        # Like Franka, use receipt time: the headset clock need not match ROS.
        self._received["vr"] = now
        self._target = (1.0 - controller.trigger) * MAX_OPENING_M
        self._check_match()

    def _joint_callback(self, message: JointState) -> None:
        if (
            len(message.name) != len(message.position)
            or message.name.count(self.joint_name) != 1
        ):
            self._invalidate("joints")
            return
        positions = dict(zip(message.name, message.position, strict=False))
        measured = positions.get(self.joint_name)
        if (
            measured is None
            or not isfinite(measured)
            or not 0.0 <= measured <= MAX_OPENING_M
        ):
            self._invalidate("joints")
            return
        if self._accept_stamp("joints", message.header):
            self._measured = measured
            self._check_match()

    def _check_match(self) -> None:
        if (
            self._target is None
            or self._measured is None
            or abs(self._target - self._measured) > self.match_tolerance_m
        ):
            self._match_since = None

    def _status_callback(self, message: DiagnosticArray) -> None:
        diagnostics = [
            status
            for status in message.status
            if status.hardware_id == self.joint_name
        ]
        if len(diagnostics) != 1:
            self._driver_reason = "Expected one diagnostic for gripper joint."
            self._invalidate("status")
            return
        status = diagnostics[0]
        values = {item.key: item.value for item in status.values}
        required = {
            "online": "true",
            "motion_enabled": "true",
            "operation_busy": "false",
            "activated": "true",
            "activation_state": "3",  # SDK 1.1.0 ActivationState::Complete.
            "gripper_fault": "0",
            "controller_fault": "0",
        }
        for key, expected in required.items():
            if values.get(key) != expected:
                self._driver_reason = f"Driver {key} is not {expected}."
                self._invalidate("status")
                return
        if status.level != DiagnosticStatus.OK:
            self._driver_reason = "Driver diagnostic is not OK."
            self._invalidate("status")
            return
        if self._accept_stamp("status", message.header):
            self._driver_reason = ""

    def _readiness(self, now: float) -> tuple[bool, str]:
        for source, received in self._received.items():
            if (
                received is None
                or now - received > self.stale_timeout_s
                or (
                    source != "vr"
                    and not self._stamp_age_valid(self._source_stamps[source])
                )
            ):
                self._record_loss()
                reason = f"Missing, invalid or stale {source} input."
                if source == "status" and self._driver_reason:
                    reason = self._driver_reason
                return False, reason
        self._ready = True
        return True, "Ready."

    def _new_header(self) -> Header:
        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        self._last_output_stamp = max(stamp_ns, self._last_output_stamp + 1)
        stamp.sec, stamp.nanosec = divmod(
            self._last_output_stamp, 1_000_000_000
        )
        return Header(stamp=stamp)

    def _publish(self) -> None:
        now = monotonic()
        ready, reason = self._readiness(now)
        matched = False
        header = self._new_header()
        if ready:
            assert self._target is not None and self._measured is not None
            if abs(self._target - self._measured) <= self.match_tolerance_m:
                if self._match_since is None:
                    self._match_since = now
                matched = now - self._match_since >= self.match_dwell_s
                reason = "Matched." if matched else "Waiting for match dwell."
            else:
                self._match_since = None
                reason = "Trigger target does not match measured opening."
            self.joint_command_publisher.publish(
                JointState(
                    header=header,
                    name=[self.joint_name],
                    position=[self._target],
                )
            )
        self.match_state_publisher.publish(
            TeleopMatchState(
                header=header,
                candidate_topic=self.joint_command_publisher.topic_name,
                loss_id=self._loss_id,
                ready=ready,
                matched=matched,
                reason=reason,
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotiqPicoVRTeleOpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
