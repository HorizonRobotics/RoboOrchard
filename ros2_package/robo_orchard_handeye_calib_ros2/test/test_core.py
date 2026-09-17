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

import numpy as np
import pytest

from robo_orchard_handeye_calib_ros2 import core


@pytest.mark.parametrize(
    ("calibrate", "expected_method"),
    [
        (core.run_eye_in_hand_calibration, core.cv2.CALIB_HAND_EYE_TSAI),
        (core.run_eye_to_hand_calibration, core.cv2.CALIB_HAND_EYE_DANIILIDIS),
    ],
)
def test_calibration_selects_solver_by_keyword(
    monkeypatch, calibrate, expected_method
):
    def solve(robot_r, robot_t, marker_r, marker_t, *, method):
        assert method == expected_method
        return np.eye(3), np.array([[1.0], [2.0], [3.0]])

    monkeypatch.setattr(core.cv2, "calibrateHandEye", solve)
    position, orientation = calibrate([], [], 0)
    assert position == [1.0, 2.0, 3.0]
    assert orientation == [0.0, 0.0, 0.0, 1.0]
