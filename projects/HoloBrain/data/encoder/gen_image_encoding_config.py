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

from robo_orchard_data_ros2.codec.image.codec import (
    JpegCodecConfig,
)
from robo_orchard_data_ros2.codec.image.config import ImageEncodingConfig


def main():
    camera_namespace = "/agilex"

    config = ImageEncodingConfig(
        codec=JpegCodecConfig(),
        num_workers=8,
        topic_mapping={
            f"{camera_namespace}/left_camera/color/image_raw": "/left_camera/color/image_raw/compressed_data",  # noqa: E501
            f"{camera_namespace}/middle_camera/color/image_raw": "/middle_camera/color/image_raw/compressed_data",  # noqa: E501
            f"{camera_namespace}/right_camera/color/image_raw": "/right_camera/color/image_raw/compressed_data",  # noqa: E501
        },
        max_queue_size=128,
    )
    with open(
        os.path.join(os.path.dirname(__file__), "image_encoding.json"), "w"
    ) as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
