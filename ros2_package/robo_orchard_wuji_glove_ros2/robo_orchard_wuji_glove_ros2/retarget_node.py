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
import time
from typing import Any

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState

from robo_orchard_wuji_glove_msg_ros2.msg import HandSkeleton
from robo_orchard_wuji_glove_ros2.sdk_client import WujiSdkRetargeter


class WujiGloveRetargetNode(Node):
    """Retarget glove skeleton frames to first-generation hand commands."""

    def __init__(self, retargeter: Any | None = None) -> None:
        super().__init__("wuji_glove_retarget")
        self._hand_side = str(
            self.declare_parameter("hand_side", "right").value
        )
        self._hand_model = str(
            self.declare_parameter("hand_model", "wuji_hand").value
        )
        self._min_joint_confidence = float(
            self.declare_parameter("min_joint_confidence", 0.5).value
        )
        self._tracking_reset_gap_s = float(
            self.declare_parameter("tracking_reset_gap_s", 0.5).value
        )
        if self._hand_side not in ("left", "right"):
            raise ValueError("hand_side must be 'left' or 'right'")
        if self._hand_model != "wuji_hand":
            raise ValueError(
                "hand_model must be 'wuji_hand' for wujihandros2 1.1.0"
            )
        if not math.isfinite(self._min_joint_confidence) or not (
            0.0 <= self._min_joint_confidence <= 1.0
        ):
            raise ValueError("min_joint_confidence must be between 0 and 1")
        if (
            not math.isfinite(self._tracking_reset_gap_s)
            or self._tracking_reset_gap_s <= 0.0
        ):
            raise ValueError(
                "tracking_reset_gap_s must be positive and finite"
            )
        self._retargeter = retargeter or WujiSdkRetargeter(
            self._hand_model, self._hand_side
        )
        self._joint_names = [
            f"{self._hand_side}_finger{finger}_joint{joint}"
            for finger in range(1, 6)
            for joint in range(1, 5)
        ]
        self._last_error_log = 0.0
        self._tracking_valid = False
        self._tracking_error = "waiting for a valid skeleton frame"
        self._last_valid_frame_monotonic: float | None = None
        self._last_min_confidence = 0.0
        self._invalid_frames = 0
        self._reset_required = False

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._command_publisher = self.create_publisher(
            JointState, "retargeted_joint_commands", command_qos
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "diagnostics", 10
        )
        self._skeleton_subscription = self.create_subscription(
            HandSkeleton,
            "hand_skeleton",
            self._skeleton_callback,
            sensor_qos,
        )
        self._diagnostics_timer = self.create_timer(
            1.0, self._publish_diagnostics
        )
        self.get_logger().info(
            "Wuji Glove retarget ready: "
            f"model={self._hand_model} side={self._hand_side}"
        )

    def _skeleton_callback(self, message: HandSkeleton) -> None:
        now = time.monotonic()
        keypoints = np.asarray(
            [
                [
                    joint.pose.position.x,
                    joint.pose.position.y,
                    joint.pose.position.z,
                ]
                for joint in message.joints
            ],
            dtype=np.float32,
        )
        if keypoints.shape != (21, 3) or not np.isfinite(keypoints).all():
            self._mark_tracking_lost("invalid skeleton keypoints")
            return
        confidences = np.asarray(
            [joint.confidence for joint in message.joints], dtype=np.float32
        )
        if (
            confidences.shape != (21,)
            or not np.isfinite(confidences).all()
            or np.any(confidences < 0.0)
            or np.any(confidences > 1.0)
        ):
            self._mark_tracking_lost("invalid skeleton confidence values")
            return
        min_confidence = float(np.min(confidences))
        self._last_min_confidence = min_confidence
        if min_confidence < self._min_joint_confidence:
            self._mark_tracking_lost(
                "skeleton confidence below threshold: "
                f"minimum={min_confidence:.3f}, "
                f"required={self._min_joint_confidence:.3f}"
            )
            return
        if (
            self._last_valid_frame_monotonic is not None
            and now - self._last_valid_frame_monotonic
            > self._tracking_reset_gap_s
        ):
            self._reset_required = True
        if self._reset_required and not self._reset_retargeter(
            "tracking recovery"
        ):
            return
        try:
            positions = self._retargeter.step(keypoints)
            positions = np.asarray(positions, dtype=np.float32)
            if positions.shape != (20,) or not np.isfinite(positions).all():
                raise RuntimeError("retarget output must be 20 finite values")
        except Exception as error:
            self._mark_tracking_lost(
                f"retarget failed: {error}", force_reset=True
            )
            return

        command = JointState()
        command.header.stamp = message.header.stamp
        command.name = self._joint_names
        command.position = [float(value) for value in positions]
        try:
            self._command_publisher.publish(command)
        except Exception as error:
            self._mark_tracking_lost(
                f"command publish failed: {error}", force_reset=True
            )
            return
        self._tracking_valid = True
        self._tracking_error = ""
        self._last_valid_frame_monotonic = now

    def _mark_tracking_lost(
        self, message: str, *, force_reset: bool = False
    ) -> None:
        self._invalid_frames += 1
        needs_reset = self._tracking_valid or force_reset
        self._tracking_valid = False
        self._last_valid_frame_monotonic = None
        self._tracking_error = message
        self._report_error(message)
        if needs_reset:
            self._reset_required = True
            self._reset_retargeter("tracking loss")

    def _reset_retargeter(self, reason: str) -> bool:
        try:
            self._retargeter.reset()
        except Exception as error:
            self._tracking_valid = False
            self._last_valid_frame_monotonic = None
            self._reset_required = True
            self._tracking_error = (
                f"retarget reset failed during {reason}: {error}"
            )
            self._report_error(self._tracking_error)
            return False
        self._reset_required = False
        return True

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        if (
            self._tracking_valid
            and self._last_valid_frame_monotonic is not None
            and now - self._last_valid_frame_monotonic
            > self._tracking_reset_gap_s
        ):
            self._mark_tracking_lost(
                "skeleton tracking timed out after "
                f"{now - self._last_valid_frame_monotonic:.3f}s"
            )
        status = DiagnosticStatus()
        status.name = f"{self.get_fully_qualified_name()}/tracking"
        status.level = (
            DiagnosticStatus.OK
            if self._tracking_valid
            else DiagnosticStatus.WARN
        )
        status.message = (
            "tracking" if self._tracking_valid else self._tracking_error
        )
        status.values = [
            KeyValue(
                key="min_joint_confidence",
                value=str(self._min_joint_confidence),
            ),
            KeyValue(
                key="last_min_confidence",
                value=str(self._last_min_confidence),
            ),
            KeyValue(key="invalid_frames", value=str(self._invalid_frames)),
            KeyValue(
                key="reset_required", value=str(self._reset_required).lower()
            ),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics_publisher.publish(message)

    def _report_error(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log >= 1.0:
            self._last_error_log = now
            self.get_logger().error(message)


def main(args: list[str] | None = None) -> None:
    """Run the Wuji Glove retarget node."""
    rclpy.init(args=args)
    node = None
    try:
        node = WujiGloveRetargetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
