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

from robo_orchard_data_ros2.tf.config import TFConfig, TFNodeConfig


def main():
    config = TFNodeConfig(
        tf=[
            TFConfig(
                parent_frame_id="left_end_effector",
                child_frame_id="left_camera_color_optical_frame",
                xyz=[
                    -0.06867924193484086,
                    -0.0005945544447201671,
                    0.03843362824412718,
                ],
                quat=[
                    -0.14277810176817451,
                    0.1236499359266293,
                    -0.6680764786273947,
                    0.7197214222917346,
                ],
                scalar_first=False,
            ),
            TFConfig(
                parent_frame_id="right_end_effector",
                child_frame_id="right_camera_color_optical_frame",
                xyz=[
                    -0.07333788908459828,
                    0.00991803705544634,
                    0.03390080995535155,
                ],
                quat=[
                    0.1296176811682453,
                    -0.12171535345636147,
                    0.717362436615576,
                    -0.673628802824318,
                ],
                scalar_first=False,
            ),
            TFConfig(
                parent_frame_id="left_base_link",
                child_frame_id="middle_camera_color_optical_frame",
                xyz=[
                    -0.010783568385050412,
                    -0.2559182030838615,
                    0.5173197227547938,
                ],
                quat=[
                    -0.6344593881273598,
                    0.6670669773214551,
                    -0.2848079166270871,
                    0.2671467447131103,
                ],
                scalar_first=False,
            ),
            TFConfig(
                parent_frame_id="world",
                child_frame_id="left_base_link",
                xyz=[0.0, 0.0, 0.0],
                quat=[0.0, 0.0, 0.0, 1.0],
                scalar_first=False,
            ),
            TFConfig(
                parent_frame_id="left_base_link",
                child_frame_id="right_base_link",
                xyz=[0.0, -0.6, 0.0],
                quat=[0.0, 0.0, 0.0, 1.0],
                scalar_first=False,
            ),
        ]
    )

    with open(
        os.path.join(os.path.dirname(__file__), "tf_publisher.json"), "w"
    ) as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
