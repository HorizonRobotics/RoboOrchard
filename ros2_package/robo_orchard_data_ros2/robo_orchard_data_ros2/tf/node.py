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

import uuid
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from robo_orchard_data_msg_ros2.srv import SetStaticTransforms
from robo_orchard_data_ros2.tf.config import TFConfig
from robo_orchard_data_ros2.tf.runtime import load_tf_config_from_result_file


class TFPublisherNode(Node):
    def __init__(self):
        super().__init__("static_tf_publisher")
        self.entries: list[TFConfig] = []
        self._br = StaticTransformBroadcaster(self)
        self.declare_parameter("startup_id", str(uuid.uuid4()))
        self.create_service(
            SetStaticTransforms,
            "set_static_transforms",
            self._set_static_transforms_service_callback,
        )
        self._publish_tf()
        self._log_active_transforms("TF publisher starting empty")

    def _publish_tf(self):
        # NOTE: /tf_static is a latched topic and the TF2 protocol has no
        # "remove frame" primitive. Republishing a reduced set only updates
        # the current latched payload for new subscribers; long-running
        # listeners retain any frame they previously received. To recover
        # from a frame that must be withdrawn, restart the downstream
        # listeners (e.g. inference / rviz nodes) rather than this publisher.
        static_transforms = []

        for tf_i in self.entries:
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

    def _log_active_transforms(self, prefix: str) -> None:
        if not self.entries:
            self.get_logger().warn(f"{prefix}: no tf is published")
            return

        tf_summary = ", ".join(
            f"[{tf_i.parent_frame_id}] -> [{tf_i.child_frame_id}]"
            for tf_i in self.entries
        )
        self.get_logger().info(f"{prefix}: {tf_summary}")

    def _set_static_transforms_service_callback(self, request, response):
        directory = Path(request.directory)
        if not directory.is_dir():
            response.success = False
            response.message = f"Not a directory: {request.directory}"
            self.get_logger().error(response.message)
            return response

        new_entries: list[TFConfig] = []
        for json_file in sorted(directory.glob("*.json")):
            try:
                new_entries.append(
                    load_tf_config_from_result_file(str(json_file))
                )
            except (ValueError, OSError) as exc:
                response.success = False
                response.message = str(exc)
                self.get_logger().error(
                    f"Failed to load static transform from "
                    f"'{json_file}': {exc}"
                )
                return response

        self.entries = new_entries
        self._publish_tf()
        self._log_active_transforms("Updated active TF config")
        response.success = True
        response.message = ""
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
