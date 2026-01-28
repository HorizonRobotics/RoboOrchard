# Project RoboOrchard
#
# Copyright (c) 2025 Horizon Robotics. All Rights Reserved.
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

import pydantic

__all__ = ["TFConfig", "TFNodeConfig"]


class TFConfig(pydantic.BaseModel):
    """Represents a single coordinate frame transformation configuration.

    Attributes:
        parent_frame_id (str): Identifier of the parent coordinate frame.
            Example: "world" or "base_link".
        child_frame_id (str): Identifier of the child coordinate frame.
            Example: "sensor_lidar" or "camera_front".
        xyz (tuple[float, float, float]): Cartesian translation
            components (x,y,z) in meters relative to parent frame.
        quat (tuple[float, float, float, float]): Rotation represented
            as quaternion format.
        scalar_first (bool): Flag indicating if scalar component comes first
            in quaternion representation. When True, expects (w,x,y,z) format.
    """

    parent_frame_id: str
    child_frame_id: str
    xyz: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    scalar_first: bool


class TFNodeConfig(pydantic.BaseModel):
    """Defines a collection of coordinate frame transformations for a node.

    Attributes:
        tf (list[TFConfig]): List of coordinate transformation configurations.
            Defaults to empty list when no transformations are specified.
    """

    tf: list[TFConfig] = pydantic.Field(default_factory=lambda: [])
