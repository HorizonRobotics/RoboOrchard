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
    ControlConfig,
    DeployConfig,
    ObservationConfig,
    RobotConfig,
)


def main():
    camera_prefix = "/agilex"
    config = DeployConfig(
        robot_config=RobotConfig(
            num_joints=7,
            joint_names=[
                "joint1",
                "joint2",
                "joint3",
                "joint4",
                "joint5",
                "joint6",
                "joint7",
            ],
            joint_velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
            joint_efforts=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5],
        ),
        observation_config=ObservationConfig(
            color_topics={
                "left_color": f"{camera_prefix}/left_camera/color/image_raw",
                "right_color": f"{camera_prefix}/right_camera/color/image_raw",
                "middle_color": f"{camera_prefix}/middle_camera/color/image_raw",  # noqa E501
            },
            depth_topics={
                "left_depth": f"{camera_prefix}/left_camera/aligned_depth_to_color/image_raw",  # noqa E501
                "right_depth": f"{camera_prefix}/right_camera/aligned_depth_to_color/image_raw",  # noqa E501
                "middle_depth": f"{camera_prefix}/middle_camera/aligned_depth_to_color/image_raw",  # noqa E501
            },
            intrinsic_topics={
                "left_intrinsic": f"{camera_prefix}/left_camera/color/camera_info",  # noqa E501
                "right_intrinsic": f"{camera_prefix}/right_camera/color/camera_info",  # noqa E501
                "middle_intrinsic": f"{camera_prefix}/middle_camera/color/camera_info",  # noqa E501
            },
            arm_state_topics={
                "left_arm_state": "/puppet/joint_left",
                "right_arm_state": "/puppet/joint_right",
            },
        ),
        control_config=ControlConfig(
            left_arm_control_topic="/left_algo_cmd",
            right_arm_control_topic="/right_algo_cmd",
            control_frequency=200.0,
        ),
        server_url="http://localhost:6050/sem",
        infer_frequency=5.0,
        delay_horizon=16,
    )

    with open(
        os.path.join(os.path.dirname(__file__), "async_inference.json"), "w"
    ) as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
