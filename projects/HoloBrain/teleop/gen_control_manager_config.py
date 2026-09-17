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
import sys
from pathlib import Path

INFERENCE_DIR = Path(__file__).resolve().parents[1] / "inference"
sys.path.insert(0, str(INFERENCE_DIR))

from control_channels import COMMAND_CHANNELS, override_topic  # noqa: E402

from robo_orchard_control_manager_ros2.config import (  # noqa: E402
    CommandChannel,
    ControlManagerConfig,
)


def build_config(teleop_source: str) -> ControlManagerConfig:
    """Build project Manager wiring from the shared command channels."""
    channels = [
        CommandChannel(
            name=channel.name,
            msg_type=channel.msg_type,
            autonomous_topic=channel.autonomous_topic,
            override_topic=override_topic(channel, teleop_source),
            output_topic=channel.output_topic,
        )
        for channel in COMMAND_CHANNELS
    ]
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
    else:
        raise ValueError(
            f"Unsupported teleop source {teleop_source!r}; "
            "expected aloha or pico"
        )
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
    config = build_config(teleop_source)
    output = Path(__file__).with_name("control_manager.json")
    output.write_text(config.model_dump_json(indent=4), encoding="utf-8")


if __name__ == "__main__":
    main()
