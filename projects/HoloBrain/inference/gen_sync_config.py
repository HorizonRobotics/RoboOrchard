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

from robo_orchard_deploy_ros2.config import (
    CameraInfoChannel,
    ControlConfig,
    DeployConfig,
    ImageChannel,
    JointCommandChannel,
    JointStateChannel,
    ObservationConfig,
)

CAMERA_PREFIX = "/agilex"
CAMERA_SIDES = ("left", "right", "middle")
ARM_SIDES = ("left", "right")
PIPER_JOINT_NAMES = [f"joint{index}" for index in range(1, 8)]


def build_obs_channels():
    """Declare the Piper observation channels sent to the model server."""
    channels = []
    for side in CAMERA_SIDES:
        camera = f"{CAMERA_PREFIX}/{side}_camera"
        channels.append(
            ImageChannel(
                server_input_key=f"{side}_color",
                topic=f"{camera}/color/image_raw",
                encoding="bgr8",
            )
        )
        channels.append(
            ImageChannel(
                server_input_key=f"{side}_depth",
                topic=f"{camera}/aligned_depth_to_color/image_raw",
                encoding="passthrough",
            )
        )
        channels.append(
            CameraInfoChannel(
                server_input_key=f"{side}_intrinsic",
                topic=f"{camera}/color/camera_info",
            )
        )
    for side in ARM_SIDES:
        channels.append(
            JointStateChannel(
                server_input_key=f"{side}_arm_state",
                topic=f"/puppet/joint_{side}",
            )
        )
    return channels


def build_action_channels():
    """Declare the Piper action channels driven by the model response."""
    return [
        JointCommandChannel(
            server_output_key=f"{side}_arm_actions",
            topic=f"/{side}_algo_cmd",
            joint_names=PIPER_JOINT_NAMES,
            velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
            efforts=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5],
        )
        for side in ARM_SIDES
    ]


def main():
    config = DeployConfig(
        observation_config=ObservationConfig(channels=build_obs_channels()),
        control_config=ControlConfig(
            channels=build_action_channels(),
            control_frequency=200.0,
        ),
        server_url="http://localhost:2000/holobrain",
        infer_frequency=3.0,
        delay_horizon=32,
    )

    with open(
        os.path.join(os.path.dirname(__file__), "sync_inference.json"), "w"
    ) as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
