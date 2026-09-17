# Project RoboOrchard
#
# Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
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

import importlib.util
import json
import sys
from pathlib import Path

import pytest
from pydantic import ValidationError

from robo_orchard_handeye_calib_ros2.config import HandEyeCalibrationConfig


@pytest.fixture
def calibration_fields():
    return {
        "mode": "eye_in_hand",
        "end_effector_frame_name": "tool",
        "camera_frame_name": "camera",
        "aruco_marker_frame_name": "marker",
        "base_frame_name": "base",
        "aruco_marker_pose_topic_name": "/marker/pose",
        "end_effector_pose_topic_name": "/robot/ee_pose",
    }


@pytest.mark.parametrize("field", ["result_file", "output_root"])
def test_config_accepts_one_output_mode(calibration_fields, field):
    config = HandEyeCalibrationConfig(**calibration_fields, **{field: "out"})
    assert getattr(config, field) == "out"


@pytest.mark.parametrize(
    "output",
    [
        {},
        {"result_file": None, "output_root": None},
        {"result_file": "result.json", "output_root": "results"},
        {"result_file": ""},
        {"output_root": ""},
    ],
)
def test_config_rejects_invalid_output_modes(calibration_fields, output):
    with pytest.raises(ValidationError):
        HandEyeCalibrationConfig(**calibration_fields, **output)


@pytest.fixture
def generator(tmp_path, monkeypatch):
    script = (
        Path(__file__).resolve().parents[3]
        / "projects/HoloBrain/handeye_calib/gen_handeye_calib_config.py"
    )
    spec = importlib.util.spec_from_file_location("handeye_generator", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "__file__", str(tmp_path / script.name))
    monkeypatch.setattr(
        sys,
        "argv",
        [
            script.name,
            "--mode",
            "eye_in_hand",
            "--camera_frame_name",
            "camera",
            "--marker_frame",
            "marker",
            "--end_effector_frame_name",
            "tool",
            "--base_frame_name",
            "base",
            "--end_effector_pose_topic_name",
            "/robot/ee_pose",
        ],
    )
    return module


@pytest.mark.parametrize("field", ["result_file", "output_root"])
def test_generator_writes_selected_output_mode(
    generator, tmp_path, monkeypatch, field
):
    monkeypatch.setattr(sys, "argv", sys.argv + [f"--{field}", "results"])
    generator.main()
    data = json.loads((tmp_path / "handeye_calib_config.json").read_text())
    config = HandEyeCalibrationConfig.model_validate(data)
    assert getattr(config, field) == "results"
    other = "output_root" if field == "result_file" else "result_file"
    assert other not in data


@pytest.mark.parametrize(
    "arguments",
    [[], ["--result_file", "result.json", "--output_root", "results"]],
)
def test_generator_requires_exactly_one_output_mode(
    generator, tmp_path, monkeypatch, arguments
):
    monkeypatch.setattr(sys, "argv", sys.argv + arguments)
    with pytest.raises(SystemExit) as error:
        generator.main()
    assert error.value.code == 2
    assert not (tmp_path / "handeye_calib_config.json").exists()
