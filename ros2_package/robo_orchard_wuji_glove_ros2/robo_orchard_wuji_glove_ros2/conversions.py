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
from collections.abc import Sequence
from typing import Any

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Header

from robo_orchard_wuji_glove_msg_ros2.msg import (
    GloveInfo,
    HandJointAngles,
    HandSkeleton,
    PoseArrayWithConfidence,
    PoseWithConfidence,
    TactileFrame,
    TactileZones,
)
from robo_orchard_wuji_glove_ros2.sdk_client import GloveDeviceInfo

EARLIEST_UTC_TIMESTAMP_US = 1_577_836_800_000_000
FINGER_NAMES = ("thumb", "index", "middle", "ring", "pinky")
TACTILE_ROWS = 24


def to_ros_time(timestamp_us: int) -> Time:
    """Convert synchronized UTC microseconds to ROS time."""
    value = int(timestamp_us)
    if value < EARLIEST_UTC_TIMESTAMP_US:
        raise ValueError(f"device timestamp is not synchronized UTC: {value}")
    seconds, micros = divmod(value, 1_000_000)
    if seconds > 2_147_483_647:
        raise ValueError(f"device timestamp exceeds ROS Time range: {value}")
    return Time(sec=seconds, nanosec=micros * 1_000)


def to_ros_header(source: Any, frame_prefix: str = "") -> Header:
    """Convert a Wuji ``FrameHeader``."""
    message = Header()
    message.stamp = to_ros_time(int(source.timestamp_us))
    message.frame_id = _prefixed_frame_id(source.frame_id, frame_prefix)
    return message


def glove_info_to_message(info: GloveDeviceInfo, stamp: Time) -> GloveInfo:
    """Convert connection metadata."""
    message = GloveInfo()
    message.header.stamp = stamp
    message.source_namespace = info.source_namespace
    message.serial_number = info.serial_number
    message.device_name = info.device_name
    message.firmware_version = info.firmware_version
    message.sdk_version = info.sdk_version
    message.handedness = info.handedness
    message.ip_address = info.ip_address
    message.data_port = info.data_port
    message.connected = info.connected
    return message


def tactile_to_message(source: Any, frame_prefix: str = "") -> TactileFrame:
    """Convert a tactile, contact-binary, or residual matrix frame."""
    message = TactileFrame()
    message.header = to_ros_header(source.header, frame_prefix)
    message.sequence = int(source.header.seq)
    message.device_timestamp_us = int(source.header.timestamp_us)
    message.rows, message.columns = _tactile_shape(len(source.data))
    message.data = [float(value) for value in source.data]
    return message


def tactile_zones_to_message(
    source: Any, frame_prefix: str = ""
) -> TactileZones:
    """Convert calibrated tactile zones."""
    message = TactileZones()
    message.header = to_ros_header(source.header, frame_prefix)
    message.sequence = int(source.header.seq)
    message.device_timestamp_us = int(source.header.timestamp_us)
    for name in ("palm", *FINGER_NAMES):
        setattr(
            message,
            name,
            [float(value) for value in getattr(source, name)],
        )
    return message


def poses_to_message(
    source: Any, frame_prefix: str = ""
) -> PoseArrayWithConfidence:
    """Convert EMF or fingertip poses in documented finger order."""
    message = PoseArrayWithConfidence()
    message.header = to_ros_header(source.header, frame_prefix)
    message.sequence = int(source.header.seq)
    message.device_timestamp_us = int(source.header.timestamp_us)
    for index, source_pose in enumerate(source.poses):
        pose = PoseWithConfidence()
        pose.name = (
            FINGER_NAMES[index]
            if index < len(FINGER_NAMES)
            else f"pose_{index}"
        )
        pose.pose = _pose_to_message(source_pose.pose)
        pose.confidence = float(source_pose.confidence)
        message.poses.append(pose)
    return message


def hand_joint_angles_to_message(
    source: Any, frame_prefix: str = ""
) -> HandJointAngles:
    """Convert the five fixed-size finger angle arrays."""
    if len(source.fingers) != 5:
        raise ValueError(f"expected 5 fingers, got {len(source.fingers)}")
    message = HandJointAngles()
    message.header = to_ros_header(source.header, frame_prefix)
    message.sequence = int(source.header.seq)
    message.device_timestamp_us = int(source.header.timestamp_us)
    for index, finger in enumerate(source.fingers):
        angles = [float(value) for value in finger.angles]
        if len(angles) != 5:
            raise ValueError(
                f"expected 5 angle slots for finger {index}, got {len(angles)}"
            )
        message.fingers[index].angles = angles
        message.fingers[index].confidence = float(finger.confidence)
        message.fingers[index].valid_angle_count = 5 if index == 0 else 4
    return message


def hand_skeleton_to_message(
    source: Any, frame_prefix: str = ""
) -> HandSkeleton:
    """Convert all 21 MediaPipe-order skeleton joints."""
    if len(source.joints) != 21:
        raise ValueError(f"expected 21 joints, got {len(source.joints)}")
    message = HandSkeleton()
    message.header = to_ros_header(source.header, frame_prefix)
    message.sequence = int(source.header.seq)
    message.device_timestamp_us = int(source.header.timestamp_us)
    for index, joint in enumerate(source.joints):
        message.joints[index].name = str(joint.name)
        message.joints[index].pose = _pose_to_message(joint.pose)
        message.joints[index].confidence = float(joint.confidence)
    return message


def imu_to_message(source: Any, frame_prefix: str = "") -> Imu:
    """Convert the documented glove palm IMU to ``sensor_msgs/Imu``."""
    message = Imu()
    message.header = to_ros_header(source.header, frame_prefix)
    message.orientation.x = float(source.orientation.x)
    message.orientation.y = float(source.orientation.y)
    message.orientation.z = float(source.orientation.z)
    message.orientation.w = float(source.orientation.w)
    message.orientation_covariance = _fixed_floats(
        source.orientation_covariance, 9
    )
    _copy_vector(source.angular_velocity, message.angular_velocity)
    message.angular_velocity_covariance = _fixed_floats(
        source.angular_velocity_covariance, 9
    )
    _copy_vector(source.linear_acceleration, message.linear_acceleration)
    message.linear_acceleration_covariance = _fixed_floats(
        source.linear_acceleration_covariance, 9
    )
    return message


def point_cloud_to_message(source: Any, frame_prefix: str = "") -> PointCloud2:
    """Convert the SDK's ROS-compatible point-cloud byte layout."""
    point_step = int(source.point_stride)
    data = list(source.data)
    if point_step <= 0:
        raise ValueError("point_stride must be positive")
    if len(data) % point_step:
        raise ValueError(
            f"point cloud data length {len(data)} is not divisible by "
            f"point_stride {point_step}"
        )
    message = PointCloud2()
    message.header = to_ros_header(source.header, frame_prefix)
    if source.frame_id:
        message.header.frame_id = _prefixed_frame_id(
            source.frame_id, frame_prefix
        )
    message.height = 1
    message.point_step = point_step
    message.data = data
    message.width = len(data) // point_step
    message.row_step = message.width * message.point_step
    message.is_bigendian = False
    message.is_dense = False
    for source_field in source.fields:
        field = PointField()
        field.name = str(source_field.name)
        field.offset = int(source_field.offset)
        field.datatype = int(source_field.type)
        field.count = 1
        message.fields.append(field)
    return message


def transforms_to_messages(
    source: Any, frame_prefix: str = ""
) -> list[TransformStamped]:
    """Convert a Wuji ``FrameTransforms`` collection."""
    messages = []
    for transform in source.transforms:
        message = TransformStamped()
        message.header.stamp = to_ros_time(int(transform.timestamp_us))
        message.header.frame_id = _prefixed_frame_id(
            transform.parent_frame_id, frame_prefix
        )
        message.child_frame_id = _prefixed_frame_id(
            transform.child_frame_id, frame_prefix
        )
        _copy_xyz(transform.translation, message.transform.translation)
        message.transform.rotation.x = float(transform.rotation.x)
        message.transform.rotation.y = float(transform.rotation.y)
        message.transform.rotation.z = float(transform.rotation.z)
        message.transform.rotation.w = float(transform.rotation.w)
        messages.append(message)
    return messages


def _pose_to_message(source: Any) -> Pose:
    message = Pose()
    _copy_xyz(source.position, message.position)
    message.orientation.x = float(source.orientation.x)
    message.orientation.y = float(source.orientation.y)
    message.orientation.z = float(source.orientation.z)
    message.orientation.w = float(source.orientation.w)
    return message


def _prefixed_frame_id(frame_id: Any, frame_prefix: str) -> str:
    value = str(frame_id)
    if not value or not frame_prefix:
        return value
    return f"{frame_prefix}/{value}"


def _copy_xyz(source: Any, destination: Any) -> None:
    if hasattr(source, "x"):
        destination.x = float(source.x)
        destination.y = float(source.y)
        destination.z = float(source.z)
    else:
        destination.x = float(source[0])
        destination.y = float(source[1])
        destination.z = float(source[2])


def _copy_vector(source: Any, destination: Any) -> None:
    destination.x = float(source.x)
    destination.y = float(source.y)
    destination.z = float(source.z)


def _fixed_floats(values: Sequence[float], length: int) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != length:
        raise ValueError(f"expected {length} values, got {len(result)}")
    return result


def _tactile_shape(value_count: int) -> tuple[int, int]:
    columns, remainder = divmod(value_count, TACTILE_ROWS)
    if not columns or remainder:
        raise ValueError(
            f"tactile data length {value_count} is not a non-empty "
            f"{TACTILE_ROWS}-row matrix"
        )
    return TACTILE_ROWS, columns
