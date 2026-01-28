# Project RoboOrchard
#
# Copyright (c) 2024 Horizon Robotics. All Rights Reserved.
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

from typing import Literal, Mapping

from pydantic import BaseModel

from robo_orchard_data_ros2.codec.image.codec import (
    JpegCodecConfig,
    PngCodecConfig,
)

__all__ = ["ImageEncodingConfig"]


class ImageEncodingConfig(BaseModel):
    """Configuration for image encoding in ROS 2.

    This class defines the configuration used for encoding images,
    including the codec type (JPEG or PNG), the number of workers,
    the maximum queue size, and the mapping between source and
    destination topics.

    Attributes:
        codec (JpegCodecConfig | PngCodecConfig): The image encoding
            codec configuration. Defaults to `PngCodecConfig()`.
        num_workers (int): The number of worker threads for encoding.
            Defaults to 0 (single-threaded execution).
        max_queue_size (int): The maximum size of the encoding task queue.
            Defaults to 0 (unlimited queue size).
        topic_mapping (Mapping[str, str]): A dictionary mapping source
            ROS topics to destination ROS topics.
        queue_full_action (Literal["raise", "drop_last", "drop_first"]): Action
            to take when the task queue is full (i.e., reaches max_queue_size).
            Defaults to "raise".
            Options are:
            - "raise": Raise a TaskQueueFulledError.
            - "drop_last": Discard the newest task (the one being added).
            - "drop_first": Discard the oldest task in the queue.
    """

    codec: JpegCodecConfig | PngCodecConfig = PngCodecConfig()
    num_workers: int = 0
    max_queue_size: int = 0
    topic_mapping: Mapping[str, str]
    queue_full_action: Literal["raise", "drop_last", "drop_first"] = (
        "drop_first"
    )
