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
    ROSBridgeCfg,
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
                "/robot/left/takeover_muxer/trigger_takeover",
                "/robot/right/takeover_muxer/trigger_takeover",
            ],
            release_service_name=[
                "/robot/left/takeover_muxer/release_control",
                "/robot/right/takeover_muxer/release_control",
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
            enable_arm_service_name=[
                "/robot/left_master/enable_ctrl",
                "/robot/right_master/enable_ctrl",
            ],
            disable_arm_service_name=[
                "/robot/left_master/disable_ctrl",
                "/robot/right_master/disable_ctrl",
            ],
            reset_arm_service_name=[
                "/robot/left/reset_ctrl",
                "/robot/right/reset_ctrl",
            ],
            record_handeye_calib_service_name="/handeye_calib/record_data",
            save_handeye_calib_service_name="/handeye_calib/save_data",
        ),
        ui_control=UIControlCfg(
            start_keyboard="s",
            stop_keyboard="f",
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
