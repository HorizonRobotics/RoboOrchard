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

import os

from robo_orchard_inference_app.config import (
    FoxgloveCfg,
    LaunchCfg,
    MITControlTuningCfg,
    ROSBridgeCfg,
    ScriptedMotionCfg,
    UIControlCfg,
)


def main():
    config = LaunchCfg(
        workspace="/data/holobrain/",
        foxglove=FoxgloveCfg(
            host="https://app.foxglove.dev",
            remote_file_layout_id="",
            remote_file_endpoint_template="{host}/?{query}",
            websocket_layout_id="",
            websocket_url="ws://localhost:8765",
            display_type="link_button",
        ),
        ros_bridge=ROSBridgeCfg(
            host="localhost",
            port=9090,
            takeover_service_name=[
                "/robot/left/aloha_orchestrator/takeover",
                "/robot/right/aloha_orchestrator/takeover",
            ],
            release_service_name=[
                "/robot/left/aloha_orchestrator/auto",
                "/robot/right/aloha_orchestrator/auto",
            ],
            stop_service_name=[
                "/robot/left/takeover_muxer/stop",
                "/robot/right/takeover_muxer/stop",
            ],
            enable_inference_service_name=[
                "/robot/inference_service/enable",
            ],
            disable_inference_service_name=[
                "/robot/inference_service/disable",
            ],
            inference_node_candidates=[
                "/robot/inference_service/sync_node",
                "/robot/inference_service/async_node",
            ],
            mit_control=MITControlTuningCfg(
                master_param_node_names=[
                    "/robot/left_master/robot_left_master_controller",
                    "/robot/right_master/robot_right_master_controller",
                ],
                follower_param_node_names=[
                    "/robot/left/robot_left_controller",
                    "/robot/right/robot_right_controller",
                ],
                default_master_enabled=True,
                default_follower_enabled=True,
                default_master_kp=10.0,
                default_master_kd=0.8,
                default_master_vel_ref=45.0,
                default_master_torque_ref=0.0,
                default_follower_kp=10.0,
                default_follower_kd=0.8,
                default_follower_vel_ref=45.0,
                default_follower_torque_ref=0.0,
                default_follower_gravity_compensation_alpha=0.0,
                default_follower_gravity_compensation_urdf_path=(
                    "/data/holobrain/urdf/piper_x_description.urdf"
                ),
            ),
            enable_arm_service_name=[
                "/robot/left_master/enable_ctrl",
                "/robot/right_master/enable_ctrl",
            ],
            reset_arm_service_name=[
                "/robot/left_master/reset_ctrl",
                "/robot/left/reset_ctrl",
                "/robot/right_master/reset_ctrl",
                "/robot/right/reset_ctrl",
            ],
            save_handeye_calib_service_name="/handeye_calib/save_data",
            static_transform_service_name="/set_static_transforms",
        ),
        ui_control=UIControlCfg(
            start_keyboard="s",
            stop_keyboard="f",
        ),
        scripted_motion=ScriptedMotionCfg(
            command=[
                "python3",
                "/opt/roboorchard/ros2_package/robo_orchard_teleop_ros2/"
                "robo_orchard_teleop_ros2/scripted/joint_master.py",
            ],
            duration_s=10.0,
            start_delay_s=3.0,
            amplitude_scale=1.0,
            frequency_scale=1.0,
            rate_hz=100.0,
            reset_position=[0.0, 0.10, -0.70, 0.0, 0.0, 0.0, 0.0],
        ),
        file_server_uri="http://localhost:8000",
    )

    with open(
        os.path.join(
            os.path.dirname(__file__), "inference_app_launch_cfg.json"
        ),
        "w",
    ) as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
