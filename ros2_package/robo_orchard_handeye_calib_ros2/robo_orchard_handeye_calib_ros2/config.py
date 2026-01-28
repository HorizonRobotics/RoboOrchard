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

from typing import Literal

from pydantic import BaseModel, Field


class HandEyeCalibrationConfig(BaseModel):
    mode: Literal["eye_in_hand", "eye_to_hand"] = Field(
        ..., description="Mode of calibration: 'eye_in_hand' or 'eye_to_hand'"
    )
    end_effector_frame_name: str = Field(
        ..., description="Name of the end effector frame"
    )
    camera_frame_name: str = Field(..., description="Name of the camera frame")
    aruco_marker_frame_name: str = Field(
        ..., description="Name of the ArUco marker frame"
    )
    base_frame_name: str = Field(..., description="Name of the base frame")
    aruco_marker_pose_topic_name: str = Field(
        ..., description="Topic name for the ArUco marker pose"
    )
    end_effector_pose_topic_name: str = Field(
        ..., description="Topic name for the end effector pose"
    )
    result_file: str = Field(..., description="File path to record result.")
