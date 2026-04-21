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

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robo_orchard_data_ros2.tf.config import TFConfig
from robo_orchard_data_ros2.tf.runtime import load_tf_config_from_result_file


def _write_handeye_result(
    path,
    position,
    orientation,
    parent_frame="left_end_effector",
    child_frame="left_camera_color_optical_frame",
):
    path.write_text(
        json.dumps(
            {
                "parent_frame": parent_frame,
                "child_frame": child_frame,
                "result": {
                    "position": position,
                    "orientation": orientation,
                },
            }
        )
    )


def test_load_tf_config_from_result_file_returns_transform_from_file(tmp_path):
    result_file = tmp_path / "left.json"
    _write_handeye_result(
        result_file,
        [0.1, 0.2, 0.3],
        [0.4, 0.5, 0.6, 0.7],
        parent_frame="tool0",
        child_frame="camera0",
    )

    tf_config = load_tf_config_from_result_file(str(result_file))

    assert tf_config == TFConfig(
        parent_frame_id="tool0",
        child_frame_id="camera0",
        xyz=(0.1, 0.2, 0.3),
        quat=(0.4, 0.5, 0.6, 0.7),
        scalar_first=False,
    )


@pytest.mark.parametrize(
    ("payload", "message"),
    [
        (
            {
                "child_frame": "camera0",
                "result": {
                    "position": [0.1, 0.2, 0.3],
                    "orientation": [0.4, 0.5, 0.6, 0.7],
                },
            },
            "parent_frame",
        ),
        (
            {
                "parent_frame": "tool0",
                "result": {
                    "position": [0.1, 0.2, 0.3],
                    "orientation": [0.4, 0.5, 0.6, 0.7],
                },
            },
            "child_frame",
        ),
        (
            {
                "parent_frame": "tool0",
                "child_frame": "camera0",
                "result": {
                    "position": [0.1, 0.2],
                    "orientation": [0.4, 0.5, 0.6, 0.7],
                },
            },
            "position",
        ),
        (
            {
                "parent_frame": "tool0",
                "child_frame": "camera0",
                "result": {
                    "position": [0.1, 0.2, 0.3],
                    "orientation": [0.4, 0.5, 0.6],
                },
            },
            "orientation",
        ),
    ],
)
def test_load_tf_config_from_result_file_rejects_invalid_payload(
    tmp_path, payload, message
):
    invalid_result = tmp_path / "invalid.json"
    invalid_result.write_text(json.dumps(payload))

    with pytest.raises(ValueError, match=message):
        load_tf_config_from_result_file(str(invalid_result))


