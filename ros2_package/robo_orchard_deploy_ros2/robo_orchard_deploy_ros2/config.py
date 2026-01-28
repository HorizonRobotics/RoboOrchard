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

from typing import Dict, List

from pydantic import BaseModel, Field

__all__ = [
    "RobotConfig",
    "ObservationConfig",
    "ControlConfig",
    "DeployConfig",
]


class RobotConfig(BaseModel):
    num_joints: int = Field(
        default=7, description="Number of joints for the robot arm."
    )
    joint_names: List[str] = Field(
        default=[
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ],
        description="List of joint names for the robot arm.",
    )
    joint_velocities: List[float] = Field(
        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
        description="List of joint velocities for the robot arm.",
    )
    joint_efforts: List[float] = Field(
        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5],
        description="List of joint efforts for the robot arm.",
    )


class ObservationConfig(BaseModel):
    color_topics: Dict[str, str] = Field(
        description="Camera color topics.",
    )
    depth_topics: Dict[str, str] = Field(
        description="Camera depth topics.",
    )
    intrinsic_topics: Dict[str, str] = Field(
        description="Color intrinsic topics.",
    )
    arm_state_topics: Dict[str, str] = Field(
        description="Topics for left and right arm states.",
    )


class ControlConfig(BaseModel):
    left_arm_control_topic: str = Field(
        default="/left_algo_cmd", description="Control topic for the left arm."
    )
    right_arm_control_topic: str = Field(
        default="/right_algo_cmd",
        description="Control topic for the right arm.",
    )
    control_frequency: float = Field(
        default=25.0, description="Control frequency in Hz."
    )
    reset_arm_service_name: List[str] = Field(
        default=["/robot/left/reset_ctrl", "/robot/right/reset_ctrl"],
        description="Service names to reset the robot arm controllers.",
    )


class DeployConfig(BaseModel):
    robot_config: RobotConfig | None = None
    observation_config: ObservationConfig | None = None
    control_config: ControlConfig | None = None
    server_url: str = Field(
        default="http://localhost:5000/openpi", description="Server URL."
    )
    infer_frequency: float = Field(
        default=1.0, description="Inference frequency in Hz."
    )
    delay_horizon: int = Field(
        default=10,
        description="Delay horizon (in number of steps) under model frame rate.",  # noqa: E501
    )
    max_delay_horizon: int | None = Field(
        default=None,
        description="Maximum delay horizon (in number of steps) under control frame rate.",  # noqa: E501
    )
