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


import json
from pathlib import Path

import pytest


def test_import():
    import robo_orchard_inference_app  # noqa: F401


def test_collecting_state_writes_mit_params(tmp_path):
    from robo_orchard_inference_app.state import CollectingState

    state = CollectingState()
    state.episode_meta.user_name = "collector"
    state.episode_meta.task_name = "task"
    state.prepare(str(tmp_path))
    state.prepare_recording_path()
    Path(state.current_data_uri).mkdir(parents=True, exist_ok=True)
    state.at_start_recording()

    mit_params = {
        "param_names": ["mit_kp"],
        "master": [
            {
                "node_name": "/robot/left_master/controller",
                "status": "confirmed",
                "params": {"mit_kp": 10.0},
            }
        ],
        "follower": [],
    }
    state.at_stop_recording(mit_params=mit_params)

    meta_path = (
        tmp_path / state.session_time_str / "data" / "collector" / "task"
    )
    episode_meta_paths = list(meta_path.glob("episode_*/episode_meta.json"))
    assert len(episode_meta_paths) == 1
    episode_meta = json.loads(episode_meta_paths[0].read_text())
    assert episode_meta["mit_params"] == mit_params


def test_mit_parameter_helpers_include_mode():
    from robo_orchard_inference_app.ros_bridge import (
        MIT_PARAM_NAMES,
        RosServiceHelper,
    )

    assert "enable_mit_ctrl" in MIT_PARAM_NAMES
    assert "mit_velocity_feedforward" in MIT_PARAM_NAMES
    assert "mit_velocity_feedforward_source" in MIT_PARAM_NAMES
    assert "mit_gravity_compensation_enabled" in MIT_PARAM_NAMES
    assert "mit_gravity_compensation_urdf_path" in MIT_PARAM_NAMES
    assert "mit_gravity_compensation_scale" in MIT_PARAM_NAMES
    assert "mit_gravity_compensation_max_t_ref" in MIT_PARAM_NAMES
    assert RosServiceHelper._python_value_to_parameter_value(False) == {
        "type": 1,
        "bool_value": False,
    }
    assert RosServiceHelper._python_value_to_parameter_value(1.25) == {
        "type": 3,
        "double_value": 1.25,
    }
    assert RosServiceHelper._python_value_to_parameter_value("robot.urdf") == {
        "type": 4,
        "string_value": "robot.urdf",
    }


def test_scripted_motion_float_params_stay_double_like():
    from robo_orchard_inference_app.components.main_control import (
        MainControlComponent,
    )

    assert MainControlComponent._scripted_param_value(100.0) == "100.0"
    assert MainControlComponent._scripted_param_value(10.0) == "10.0"
    assert MainControlComponent._scripted_param_value(0.25) == "0.25"
    assert MainControlComponent._scripted_param_value(2) == "2"
    assert MainControlComponent._scripted_param_value(True) == "true"
    assert MainControlComponent._scripted_param_value(
        [0.0, 0.1, -0.7]
    ) == "[0.0, 0.1, -0.7]"


def test_scripted_motion_recording_subscriber_gate_params():
    from robo_orchard_inference_app.components.main_control import (
        MainControlComponent,
    )
    from robo_orchard_inference_app.config import LaunchCfg

    component = object.__new__(MainControlComponent)
    component.launch_cfg = LaunchCfg()

    args = component._scripted_motion_args(
        duration_s=10.0,
        amplitude_scale=1.0,
        frequency_scale=1.0,
        use_current_state=False,
        use_start_position=True,
        start_position_left=[0.0, 0.1, -0.7, 0.0, 0.0, 0.0, 0.0],
        start_position_right=[0.0, 0.1, -0.7, 0.0, 0.0, 0.0, 0.0],
        min_command_subscribers=2,
        command_subscriber_wait_timeout_s=2.5,
    )

    assert "use_current_state:=false" in args
    assert "use_start_position:=true" in args
    assert "start_position_left:=[0.0, 0.1, -0.7, 0.0, 0.0, 0.0, 0.0]" in args
    assert "start_position_right:=[0.0, 0.1, -0.7, 0.0, 0.0, 0.0, 0.0]" in args
    assert "min_command_subscribers:=2" in args
    assert "command_subscriber_wait_timeout_s:=2.5" in args


if __name__ == "__main__":
    pytest.main(["-s", __file__])
