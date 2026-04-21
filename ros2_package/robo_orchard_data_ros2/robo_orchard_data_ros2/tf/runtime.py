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

from pathlib import Path

import pydantic

from robo_orchard_data_ros2.tf.config import TFConfig


class _CalibrationResult(pydantic.BaseModel):
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


class _ResultFilePayload(pydantic.BaseModel):
    parent_frame: str = pydantic.Field(min_length=1)
    child_frame: str = pydantic.Field(min_length=1)
    result: _CalibrationResult


def load_tf_config_from_result_file(result_file: str) -> TFConfig:
    payload = _ResultFilePayload.model_validate_json(
        Path(result_file).read_text()
    )
    return TFConfig(
        parent_frame_id=payload.parent_frame,
        child_frame_id=payload.child_frame,
        xyz=payload.result.position,
        quat=payload.result.orientation,
        scalar_first=False,
    )
