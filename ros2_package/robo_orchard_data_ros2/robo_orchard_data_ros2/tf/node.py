# Project RoboOrchard
#
# Copyright (c) 2025 Horizon Robotics. All Rights Reserved.
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

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node, ParameterDescriptor
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from robo_orchard_data_ros2.tf.config import TFNodeConfig


class StaticTFPublisherNode(Node):
    def __init__(self):
        super().__init__("static_tf_publisher")
        self._init()
        self._br = StaticTransformBroadcaster(self)
        self._publish_tf()

    def _publish_tf(self):
        static_transforms = []

        for tf_i in self.config.tf:
            if tf_i.scalar_first:
                qw, qx, qy, qz = tf_i.quat
            else:
                qx, qy, qz, qw = tf_i.quat

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = tf_i.parent_frame_id
            t.child_frame_id = tf_i.child_frame_id
            t.transform.translation.x = tf_i.xyz[0]
            t.transform.translation.y = tf_i.xyz[1]
            t.transform.translation.z = tf_i.xyz[2]
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            static_transforms.append(t)

        if static_transforms:
            self._br.sendTransform(static_transforms)

            for t in static_transforms:
                self.get_logger().info(
                    "Publish static transform [{}] -> [{}]".format(
                        t.header.frame_id, t.child_frame_id
                    )
                )
        else:
            self.get_logger().warn("No tf is published!")

    def _init(self):
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
            self.config: TFNodeConfig = TFNodeConfig.model_validate_json(
                handle.read()
            )


def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
