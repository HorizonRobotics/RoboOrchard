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

import functools
import os
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node, ParameterDescriptor
from sensor_msgs.msg import CompressedImage, Image

from robo_orchard_core.utils.task_executor import (
    DataNotReadyError,
    OrderedTaskExecutor,
    TaskQueueFulledError,
)
from robo_orchard_data_ros2.codec.image.codec import (
    EncodingError,
    JpegCodec,
    PngCodec,
)
from robo_orchard_data_ros2.codec.image.config import ImageEncodingConfig

__all__ = ["ImageEncodingNode"]


CODEC_TYPE = JpegCodec | PngCodec


class _CodecFunction:
    """Wrapper for encoding function calls with exception handling.

    This class acts as a callable function that wraps around an image
    codec class's `encode` method. It catches `EncodingError` exceptions
    and ensures that encoding failures do not crash the pipeline.

    Attributes:
        codec_class (Callable): The codec class that provides the `encode`
            method.
    """

    def __init__(self, codec_class: CODEC_TYPE):
        """Initializes the codec function wrapper.

        Args:
            codec_class (Callable): The codec class used for encoding.
        """
        self.codec_class = codec_class

    def __call__(
        self, src_topic: str, dst_topic: str, *args, **kwargs
    ) -> Tuple[Tuple[str, str], Optional[CompressedImage]]:
        """Encodes an image message and handles errors.

        Args:
            src_topic (str): Source ROS topic name.
            dst_topic (str): Destination ROS topic name.
            *args: Additional arguments for the codec.
            **kwargs: Additional keyword arguments for the codec.

        Returns:
            Tuple[Tuple[str, str], Optional[CompressedImage]]: A tuple
            containing the topic mapping and the encoded image message,
            or `None` if encoding fails.
        """
        try:
            ret = self.codec_class.encode(*args, **kwargs)
        except EncodingError:
            ret = None
        return (src_topic, dst_topic), ret


class ImageEncodingNode(Node):
    """ROS 2 Node for encoding image messages.

    This node subscribes to a set of ROS topics containing raw image data,
    encodes them using a specified codec (JPEG or PNG), and republishes the
    encoded images to corresponding output topics.
    """

    def __init__(self):
        super().__init__("image_encoding")
        self._sub = dict()
        self._pub = dict()
        self._submit_cnt = 0
        self._pub_cnt = 0
        self._hint_freq = 512
        self._initialize()
        self.create_timer(0.01, self._timer_publish)

    def _message_callback(self, msg, src_topic: str, dst_topic: str):
        """Handles incoming image messages and queues them for encoding.

        Args:
            msg (Image): The incoming ROS image message.
            src_topic (str): The source topic name.
            dst_topic (str): The destination topic name.

        Raises:
            TaskQueueFulledError: If the task queue is full, the message
            is dropped.
        """
        try:
            self._executor.put(src_topic, dst_topic, msg)
            self._submit_cnt += 1
            if (
                self.config.max_queue_size > 0
                and self._submit_cnt == 2 * self.config.num_workers
            ):
                self.get_logger().info("Warming up for 10s")
                time.sleep(10)
                self.get_logger().info("Finish warming up...")
        except TaskQueueFulledError:
            self.get_logger().info("Drop message due to full queue.")

    def _timer_publish(self):
        """Retrieves encoded images from the queue and publishes them."""
        try:
            (src_topic, dst_topic), msg = self._executor.get()
            if msg is not None:
                self._pub[dst_topic].publish(msg)
                self._pub_cnt += 1
                if self._pub_cnt % self._hint_freq == 0:
                    self.get_logger().info(
                        f"Encoding {self._pub_cnt}-th message"
                    )
            else:
                self.get_logger().error(
                    f"Failed to encode message of topic {src_topic}"
                )
        except DataNotReadyError:
            pass
        except Exception as e:
            self.get_logger().error(f"Error while publishing message: {e}")

    def _initialize(self):
        """Initialization.

        Initializes parameters, loads configuration, and sets up subscriptions
        and publishers.
        """
        self.declare_parameter(
            "config_file",
            "",
            descriptor=ParameterDescriptor(description="Config path"),
        )
        config_file: str = (
            self.get_parameter("config_file")
            .get_parameter_value()
            .string_value
        )
        if not os.path.exists(config_file):
            raise FileNotFoundError(
                "config file {} does not exists!".format(config_file)
            )
        with open(config_file, "r") as handle:
            self.config: ImageEncodingConfig = (
                ImageEncodingConfig.model_validate_json(handle.read())
            )

        for src_topic, dst_topic in self.config.topic_mapping.items():
            self._sub[src_topic] = self.create_subscription(
                Image,
                src_topic,
                functools.partial(
                    self._message_callback,
                    src_topic=src_topic,
                    dst_topic=dst_topic,
                ),
                10,
            )
            self._pub[dst_topic] = self.create_publisher(
                CompressedImage, dst_topic, 10
            )

        codec_class: CODEC_TYPE = self.config.codec.class_type(
            self.config.codec  # type: ignore
        )
        self._executor = OrderedTaskExecutor(
            fn=_CodecFunction(codec_class),
            num_workers=self.config.num_workers,
            max_queue_size=self.config.max_queue_size,
            queue_full_action=self.config.queue_full_action,
        )


def main(args=None):
    rclpy.init(args=args)

    node = ImageEncodingNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
