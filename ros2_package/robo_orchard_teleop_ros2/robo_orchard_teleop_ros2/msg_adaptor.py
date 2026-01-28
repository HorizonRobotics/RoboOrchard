# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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

import numpy as np
from geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    Transform,
    Vector3,
)
from scipy.spatial.transform import Rotation

__all__ = [
    "pose_msg_to_matrix",
    "matrix_to_pose_msg",
    "pose_msg_2_transform_msg",
]


def pose_msg_to_matrix(pose: Pose) -> np.ndarray:
    """Converts a geometry_msgs/Pose message to a 4x4 homogeneous matrix."""
    r = Rotation.from_quat(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    matrix = np.identity(4)
    matrix[:3, :3] = r.as_matrix()
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return matrix


def matrix_to_pose_msg(matrix: np.ndarray) -> Pose:
    r = Rotation.from_matrix(matrix[:3, :3])
    quat = r.as_quat()
    pos = matrix[:3, 3]

    return Pose(
        position=Point(x=pos[0], y=pos[1], z=pos[2]),
        orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
    )


def pose_msg_2_transform_msg(msg: Pose) -> Transform:
    return Transform(
        translation=Vector3(
            x=msg.position.x,
            y=msg.position.y,
            z=msg.position.z,
        ),
        rotation=msg.orientation,
    )
