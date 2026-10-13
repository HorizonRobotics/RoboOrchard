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

from __future__ import annotations
import os
from pathlib import Path

from robo_orchard_control_manager_ros2.config import (
    CommandChannel,
    ControlManagerConfig,
)


def build_config(
    teleop_source: str, with_hexfellow: bool = False
) -> ControlManagerConfig:
    """Build Manager wiring for the selected arm and optional base hardware.

    Hexfellow requires Pico input. Its autonomous command topics are reserved
    for future Deploy channels; only the arm channels have Deploy outputs.
    """
    if with_hexfellow and teleop_source != "pico":
        raise ValueError("WITH_HEXFELLOW requires TELEOP_SOURCE=pico")
    if teleop_source == "aloha":
        override_root = "/master/joint_"
    elif teleop_source == "pico":
        override_root = "/pico_teleop/joint_"
    else:
        raise ValueError(
            f"Unsupported teleop source {teleop_source!r}; "
            "expected aloha or pico"
        )
    channels = [
        CommandChannel(
            name=f"{side}_arm",
            msg_type="sensor_msgs/msg/JointState",
            autonomous_topic=f"/{side}_algo_cmd",
            override_topic=f"{override_root}{side}",
            output_topic=f"/robot/{side}/joint_cmd",
        )
        for side in ("left", "right")
    ]
    if with_hexfellow:
        channels.extend(
            [
                CommandChannel(
                    name="chassis",
                    msg_type="geometry_msgs/msg/Twist",
                    autonomous_topic="/chassis_algo_cmd",
                    override_topic="/hexfellow_teleop/cmd_vel",
                    output_topic="/chassis/cmd_vel",
                ),
                CommandChannel(
                    name="lift",
                    msg_type="sensor_msgs/msg/JointState",
                    autonomous_topic="/lift_algo_cmd",
                    override_topic="/hexfellow_teleop/lift_joint_cmd",
                    output_topic="/lift/joint_cmd",
                ),
            ]
        )
    if teleop_source == "aloha":
        enable_services = [
            "/robot/left_master/enable_ctrl",
            "/robot/left/enable_ctrl",
            "/robot/right_master/enable_ctrl",
            "/robot/right/enable_ctrl",
        ]
        reset_services = [
            "/robot/left_master/reset_ctrl",
            "/robot/left/reset_ctrl",
            "/robot/right_master/reset_ctrl",
            "/robot/right/reset_ctrl",
        ]
    elif teleop_source == "pico":
        enable_services = [
            "/robot/left/enable_ctrl",
            "/robot/right/enable_ctrl",
        ]
        reset_services = [
            "/robot/left/reset_ctrl",
            "/robot/right/reset_ctrl",
        ]
    return ControlManagerConfig(
        channels=channels,
        enable_services=enable_services,
        reset_services=reset_services,
        inference_disable_services=["/robot/inference_service/disable"],
        inference_node_candidates=[
            "/robot/inference_service/sync_node",
            "/robot/inference_service/async_node",
        ],
        replay_time_s=0.0,
    )


def main() -> None:
    teleop_source = os.environ.get("TELEOP_SOURCE", "aloha").strip().lower()
    hexfellow = os.environ.get("WITH_HEXFELLOW", "false").strip().lower()
    if hexfellow not in ("true", "false"):
        raise ValueError("WITH_HEXFELLOW must be true or false")
    config = build_config(teleop_source, with_hexfellow=hexfellow == "true")
    output = Path(__file__).with_name("control_manager.json")
    output.write_text(config.model_dump_json(indent=4), encoding="utf-8")


if __name__ == "__main__":
    main()
