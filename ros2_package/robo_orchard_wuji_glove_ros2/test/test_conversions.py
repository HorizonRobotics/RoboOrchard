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

from types import SimpleNamespace as Namespace

import pytest
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointField

from robo_orchard_wuji_glove_ros2.conversions import (
    glove_info_to_message,
    hand_joint_angles_to_message,
    hand_skeleton_to_message,
    imu_to_message,
    point_cloud_to_message,
    poses_to_message,
    tactile_to_message,
    tactile_zones_to_message,
    to_ros_time,
    transforms_to_messages,
)
from robo_orchard_wuji_glove_ros2.sdk_client import GloveDeviceInfo


def _header(sequence=7, timestamp_us=1_700_000_000_123_456):
    return Namespace(
        seq=sequence, timestamp_us=timestamp_us, frame_id="r_wrist"
    )


def _quaternion(w=1.0):
    return Namespace(x=0.0, y=0.0, z=0.0, w=w)


def _pose(x=0.0):
    return Namespace(position=[x, 0.02, 0.03], orientation=_quaternion())


def test_time_requires_synchronized_utc_in_ros_range():
    with pytest.raises(ValueError, match="not synchronized UTC"):
        to_ros_time(123_456)
    with pytest.raises(ValueError, match="exceeds ROS Time range"):
        to_ros_time(2_147_483_648_000_000)

    converted = to_ros_time(1_700_000_000_123_456)
    assert converted.sec == 1_700_000_000
    assert converted.nanosec == 123_456_000


def test_glove_info_records_canonical_source_namespace():
    message = glove_info_to_message(
        GloveDeviceInfo(
            source_namespace="/wuji_glove/left_a",
            serial_number="WG1JA001",
            device_name="wuji_glove_left_a",
        ),
        Time(sec=123),
    )

    assert message.source_namespace == "/wuji_glove/left_a"
    assert message.serial_number == "WG1JA001"


def test_tactile_conversion_preserves_current_and_legacy_layouts():
    current = tactile_to_message(
        Namespace(header=_header(), data=[0.0] * (24 * 31)),
        "glove_a",
    )
    legacy = tactile_to_message(
        Namespace(header=_header(), data=[0.0] * (24 * 32)),
        "glove_a",
    )
    zones = tactile_zones_to_message(
        Namespace(
            header=_header(),
            palm=[0.0],
            thumb=[0.0],
            index=[0.0],
            middle=[0.0],
            ring=[0.0],
            pinky=[0.0],
        ),
        "glove_a",
    )

    assert (current.rows, current.columns) == (24, 31)
    assert (legacy.rows, legacy.columns) == (24, 32)
    assert current.sequence == 7
    assert current.header.frame_id == "glove_a/r_wrist"
    assert legacy.header.frame_id == "glove_a/r_wrist"
    assert zones.header.frame_id == "glove_a/r_wrist"


def test_tracking_conversions_keep_documented_order_and_confidence():
    poses = poses_to_message(
        Namespace(
            header=_header(),
            poses=[
                Namespace(pose=_pose(index / 100.0), confidence=0.9)
                for index in range(5)
            ],
        ),
        "glove_a",
    )
    angles = hand_joint_angles_to_message(
        Namespace(
            header=_header(),
            fingers=[
                Namespace(angles=[float(index)] * 5, confidence=0.8)
                for index in range(5)
            ],
        ),
        "glove_a",
    )
    skeleton = hand_skeleton_to_message(
        Namespace(
            header=_header(),
            joints=[
                Namespace(
                    name=f"joint_{index}",
                    pose=_pose(index / 100.0),
                    confidence=0.95,
                )
                for index in range(21)
            ],
        ),
        "glove_a",
    )

    assert [pose.name for pose in poses.poses] == [
        "thumb",
        "index",
        "middle",
        "ring",
        "pinky",
    ]
    assert angles.fingers[0].valid_angle_count == 5
    assert angles.fingers[1].valid_angle_count == 4
    assert skeleton.joints[20].name == "joint_20"
    assert skeleton.joints[20].pose.position.x == 0.2
    assert poses.header.frame_id == "glove_a/r_wrist"
    assert angles.header.frame_id == "glove_a/r_wrist"
    assert skeleton.header.frame_id == "glove_a/r_wrist"


def test_standard_imu_point_cloud_and_tf_conversions():
    vector = Namespace(x=0.1, y=-0.2, z=9.8)
    imu = imu_to_message(
        Namespace(
            header=_header(),
            orientation=_quaternion(0.75),
            orientation_covariance=[-1.0] + [0.0] * 8,
            angular_velocity=vector,
            angular_velocity_covariance=[0.0] * 9,
            linear_acceleration=vector,
            linear_acceleration_covariance=[0.0] * 9,
        ),
        "glove_a",
    )
    cloud = point_cloud_to_message(
        Namespace(
            header=_header(),
            frame_id="r_wrist",
            point_stride=16,
            fields=[
                Namespace(name="x", offset=0, type=PointField.FLOAT32),
                Namespace(name="pressure", offset=12, type=PointField.FLOAT32),
            ],
            data=[0] * 32,
        ),
        "glove_a",
    )
    transforms = transforms_to_messages(
        Namespace(
            transforms=[
                Namespace(
                    timestamp_us=1_700_000_000_123_456,
                    parent_frame_id="waist",
                    child_frame_id="r_wrist",
                    translation=[0.0, -0.3, -0.5],
                    rotation=_quaternion(),
                )
            ]
        ),
        "glove_a",
    )

    assert imu.orientation.w == 0.75
    assert imu.header.frame_id == "glove_a/r_wrist"
    assert imu.orientation_covariance[0] == -1.0
    assert imu.linear_acceleration.z == 9.8
    assert cloud.width == 2
    assert cloud.header.frame_id == "glove_a/r_wrist"
    assert cloud.fields[1].name == "pressure"
    assert transforms[0].header.frame_id == "glove_a/waist"
    assert transforms[0].child_frame_id == "glove_a/r_wrist"


@pytest.mark.parametrize(
    ("point_stride", "data", "message"),
    [
        (0, [], "point_stride must be positive"),
        (16, [0] * 17, "data length 17 is not divisible"),
    ],
)
def test_point_cloud_rejects_inconsistent_byte_layout(
    point_stride, data, message
):
    source = Namespace(
        header=_header(),
        frame_id="r_wrist",
        point_stride=point_stride,
        fields=[],
        data=data,
    )

    with pytest.raises(ValueError, match=message):
        point_cloud_to_message(source)


@pytest.mark.parametrize("value_count", [0, 743, 745])
def test_tactile_rejects_inconsistent_matrix_shape(value_count):
    source = Namespace(
        header=_header(),
        data=[0.0] * value_count,
    )

    with pytest.raises(
        ValueError, match=f"tactile data length {value_count} is not"
    ):
        tactile_to_message(source)
