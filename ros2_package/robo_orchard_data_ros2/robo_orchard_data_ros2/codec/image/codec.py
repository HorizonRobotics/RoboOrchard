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

from typing import Optional

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

from robo_orchard_core.utils.config import ClassConfig, ClassType_co

__all__ = [
    "EncodingError",
    "DecodingError",
    "JpegCodec",
    "JpegCodecConfig",
    "PngCodec",
    "PngCodecConfig",
]


class EncodingError(Exception):
    """Exception raised for errors in encoding images."""

    pass


class DecodingError(Exception):
    """Exception raised for errors in decoding images."""

    pass


class JpegCodec:
    """A codec for encoding and decoding images using JPEG format.

    This class provides methods for encoding ROS `sensor_msgs/Image`
    messages into `sensor_msgs/CompressedImage` using JPEG compression
    and for decoding them back.

    Attributes:
        config (JpegCodecConfig): The configuration parameters for
            JPEG encoding.
        bridge (CvBridge): A bridge to convert between ROS image messages
            and OpenCV images.
    """

    def __init__(self, config: Optional["JpegCodecConfig"] = None):
        """Initializes the JPEG codec.

        Args:
            config (Optional[JpegCodecConfig]): Configuration for JPEG
                encoding. If None, defaults to JpegCodecConfig.
        """
        self.config = JpegCodecConfig() if config is None else config
        self.bridge = CvBridge()

    def encode(self, msg: Image) -> CompressedImage:
        """Encodes a ROS Image message into a JPEG-compressed image.

        Args:
            msg (Image): The input ROS image message.

        Returns:
            CompressedImage: The JPEG-encoded compressed image message.

        Raises:
            EncodingError: If the encoding process fails.
        """
        cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding=self.config.desired_encoding
        )
        success, encoded_image = cv2.imencode(
            ".jpg",
            cv_image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.config.jpeg_quality],
        )
        if not success:
            raise EncodingError

        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = "jpeg"
        compressed_msg.data = encoded_image.tobytes()

        return compressed_msg

    def decode(self, msg: CompressedImage) -> Image:
        """Decodes a JPEG-compressed image message into a ROS Image message.

        Args:
            msg (CompressedImage): The input JPEG-compressed image message.

        Returns:
            Image: The decoded ROS image message.
        """
        data = self.bridge.compressed_imgmsg_to_cv2(msg)
        new_msg = self.bridge.cv2_to_imgmsg(data, header=msg.header)
        return new_msg


class JpegCodecConfig(ClassConfig[JpegCodec]):
    """Configuration for the JPEG codec.

    Attributes:
        class_type (ClassType_co[JpegCodec]): The associated codec class type.
        desired_encoding (str): The desired encoding format for OpenCV
            conversion (default: "bgr8").
        jpeg_quality (int): The JPEG quality level (default: 95).
    """

    class_type: ClassType_co[JpegCodec] = JpegCodec
    desired_encoding: str = "bgr8"
    jpeg_quality: int = 95


class PngCodec:
    """A codec for encoding and decoding images using PNG format.

    This class provides methods for encoding ROS `sensor_msgs/Image`
    messages into `sensor_msgs/CompressedImage` using PNG compression
    and for decoding them back.

    Attributes:
        config (PngCodecConfig): The configuration parameters for PNG encoding.
        bridge (CvBridge): A bridge to convert between ROS image messages and
            OpenCV images.
    """

    def __init__(self, config: Optional["PngCodecConfig"] = None):
        """Initializes the PNG codec.

        Args:
            config (Optional[PngCodecConfig]): Configuration for PNG encoding.
                If None, defaults to PngCodecConfig.
        """
        self.config = PngCodecConfig() if config is None else config
        self.bridge = CvBridge()

    def encode(self, msg: Image) -> CompressedImage:
        """Encodes a ROS Image message into a PNG-compressed image.

        Args:
            msg (Image): The input ROS image message.

        Returns:
            CompressedImage: The PNG-encoded compressed image message.

        Raises:
            EncodingError: If the encoding process fails.
        """

        cv_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )
        success, encoded_depth = cv2.imencode(
            ".png",
            cv_depth,
            [int(cv2.IMWRITE_PNG_COMPRESSION), self.config.compress_level],
        )

        if not success:
            raise EncodingError

        compressed_msg = CompressedImage()
        compressed_msg.header = msg.header
        compressed_msg.format = "png"
        compressed_msg.data = encoded_depth.tobytes()

        return compressed_msg

    def decode(self, msg: CompressedImage) -> Image:
        """Decodes a PNG-compressed image message into a ROS Image message.

        Args:
            msg (CompressedImage): The input PNG-compressed image message.

        Returns:
            Image: The decoded ROS image message.
        """

        data = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8), cv2.IMREAD_UNCHANGED
        )

        new_msg = self.bridge.cv2_to_imgmsg(data, header=msg.header)
        return new_msg


class PngCodecConfig(ClassConfig[PngCodec]):
    """Configuration for the PNG codec.

    Attributes:
        class_type (ClassType_co[PngCodec]): The associated codec class type.
        compress_level (int): The compression level for PNG encoding
            (default: 1).
    """

    class_type: ClassType_co[PngCodec] = PngCodec
    compress_level: int = 1
