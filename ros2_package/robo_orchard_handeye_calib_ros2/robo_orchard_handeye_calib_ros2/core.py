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

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def quaternion_to_homo(position, orientation):
    quat_array = [orientation.x, orientation.y, orientation.z, orientation.w]
    rotation_matrix = Rotation.from_quat(quat_array).as_matrix()

    if hasattr(position, "x"):
        pos_array = [position.x, position.y, position.z]
    else:
        pos_array = position

    _homo = np.eye(4)
    _homo[:3, :3] = rotation_matrix
    _homo[:3, 3] = pos_array
    return _homo


def homo_to_quaternion(homo):
    translation_vector = homo[:3, 3]
    rotation_matrix = homo[:3, :3]
    quaternion = Rotation.from_matrix(rotation_matrix).as_quat()
    return translation_vector, quaternion


def run_eye_in_hand_calibration(aruco_poses_list, ee_poses_list, cnt):
    aruco_pose_r = []
    aruco_pose_t = []
    gripper2base_r = []
    gripper2base_t = []
    for idx in range(cnt):
        aruco_pose = aruco_poses_list[idx]
        ee_pose = ee_poses_list[idx]

        aruco_pose_position = aruco_pose.pose.position
        aruco_pose_orientation = aruco_pose.pose.orientation
        aruco_pose_homo = quaternion_to_homo(
            aruco_pose_position, aruco_pose_orientation
        )

        ee_pose_position = ee_pose.pose.position
        ee_pose_orientation = ee_pose.pose.orientation
        ee_pose_homo = quaternion_to_homo(
            ee_pose_position, ee_pose_orientation
        )

        aruco_pose_r.append(aruco_pose_homo[:3, :3])
        aruco_pose_t.append(aruco_pose_homo[:3, 3])
        gripper2base_r.append(ee_pose_homo[:3, :3])
        gripper2base_t.append(ee_pose_homo[:3, 3])

    cam2ee_r, cam2ee_t = cv2.calibrateHandEye(
        gripper2base_r,
        gripper2base_t,
        aruco_pose_r,
        aruco_pose_t,
        cv2.CALIB_HAND_EYE_TSAI,
    )
    is_identity_rot = np.allclose(cam2ee_r, np.eye(3), atol=1e-6)
    is_zero_trans = np.allclose(cam2ee_t, np.zeros((3, 1)), atol=1e-6)

    if is_identity_rot and is_zero_trans:
        raise RuntimeError(
            "Calibration Algorithm Failed (Silent Error): "
            "Result is Identity/Zero, which indicates the algorithm failed to converge. "  # noqa: E501
            "Cause: Not enough rotation data. "
            "Solution: Please record poses with larger rotation differences (tilt the gripper!)."  # noqa: E501
        )
    cam2ee_pose_homo = np.eye(4)
    cam2ee_pose_homo[:3, :3] = cam2ee_r
    cam2ee_pose_homo[:3, 3] = cam2ee_t[:, 0]
    cam2ee_position, cam2ee_orientation = homo_to_quaternion(cam2ee_pose_homo)
    return cam2ee_position.tolist(), cam2ee_orientation.tolist()


def run_eye_to_hand_calibration(aruco_poses_list, ee_poses_list, cnt):
    aruco_pose_r = []
    aruco_pose_t = []
    base2gripper_r = []
    base2gripper_t = []
    for idx in range(cnt):
        aruco_pose = aruco_poses_list[idx]
        ee_pose = ee_poses_list[idx]

        aruco_pose_position = aruco_pose.pose.position
        aruco_pose_orientation = aruco_pose.pose.orientation
        aruco_pose_homo = quaternion_to_homo(
            aruco_pose_position, aruco_pose_orientation
        )
        aruco_pose_r.append(aruco_pose_homo[:3, :3])
        aruco_pose_t.append(aruco_pose_homo[:3, 3])

        gripper2base_position = ee_pose.pose.position
        gripper2base_orientation = ee_pose.pose.orientation
        gripper2base_homo = quaternion_to_homo(
            gripper2base_position, gripper2base_orientation
        )

        # eye to hand need trans gripper2base to base2gripper
        base2gripper_homo = np.linalg.inv(gripper2base_homo)
        base2gripper_r.append(base2gripper_homo[:3, :3])
        base2gripper_t.append(base2gripper_homo[:3, 3])

    cam2base_r, cam2base_t = cv2.calibrateHandEye(
        base2gripper_r,
        base2gripper_t,
        aruco_pose_r,
        aruco_pose_t,
        cv2.CALIB_HAND_EYE_DANIILIDIS,
    )
    is_identity_rot = np.allclose(cam2base_r, np.eye(3), atol=1e-6)
    is_zero_trans = np.allclose(cam2base_t, np.zeros((3, 1)), atol=1e-6)

    if is_identity_rot and is_zero_trans:
        raise RuntimeError(
            "Calibration Algorithm Failed (Silent Error): "
            "Result is Identity/Zero, which indicates the algorithm failed to converge. "  # noqa: E501
            "Cause: Not enough rotation data. "
            "Solution: Please record poses with larger rotation differences (tilt the gripper!)."  # noqa: E501
        )  # noqa: E501

    cam2base_homo = np.eye(4)
    cam2base_homo[:3, :3] = cam2base_r
    cam2base_homo[:3, 3] = cam2base_t[:, 0]
    cam2base_position, cam2base_orientation = homo_to_quaternion(cam2base_homo)
    return cam2base_position.tolist(), cam2base_orientation.tolist()
