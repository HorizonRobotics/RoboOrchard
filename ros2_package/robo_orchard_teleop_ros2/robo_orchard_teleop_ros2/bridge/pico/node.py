# Project RoboOrchard
#
# Copyright (c) 2024-2025 Horizon Robotics. All Rights Reserved.
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

import atexit
import json
from typing import Literal

import numpy as np
import pydantic
import rclpy
import xrobotoolkit_sdk as xrt
from geometry_msgs.msg import (
    Point,
    Pose,
    Quaternion,
    TransformStamped,
)
from rclpy.node import Node, ParameterDescriptor
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header

from robo_orchard_pico_msg_ros2.msg import (
    Head,
    LeftController,
    RightController,
    VRState,
)
from robo_orchard_teleop_ros2.msg_adaptor import (
    pose_msg_2_transform_msg,
)

__all__ = ["PicoBridge"]


class PicoBridgeConfig(pydantic.BaseModel):
    fps: float = 90
    timestamp_source: Literal["VR", "ROS"] = "VR"
    pub_tf: bool = True


class PicoBridge(Node):
    """A ROS2 node that bridges Pico VR device data to ROS topics.

    It reads raw data from the XRT SDK, parses it, and publishes it
    as a structured VRState message.
    """  # noqa: E501

    def __init__(self):
        super().__init__("pico_bridge")
        self._initialize()

        self._cnt = 0
        self._last_vr_time_ns = 0

        # This 4x4 homogeneous matrix defines the static, fixed rotation required  # noqa: E501
        # to change the basis from the Pico's native coordinate system to the
        # standard ROS coordinate system (REP-105: X-forward, Y-left, Z-up).
        # This matrix acts as a "translator" between the two coordinate "languages".  # noqa: E501
        self.t_pico2ros = np.array(
            [
                [0.0, 0.0, -1.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        # Pre-calculate the inverse of the transformation matrix.
        # This is a performance optimization, as the inverse is needed for every  # noqa: E501
        # pose calculation. Calculating it once here avoids redundant computation  # noqa: E501
        # inside the high-frequency xr_monitor loop.
        self.t_pico2ros_inv = np.linalg.inv(self.t_pico2ros)

        self._publisher = self.create_publisher(VRState, "vr_state", 10)

        if self.config.pub_tf:
            from tf2_ros import TransformBroadcaster

            self.tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(1.0 / self.config.fps, self.xr_monitor)

    def _parse_vr_state(self) -> VRState | None:
        """Fetches the latest message from the SDK, parses it, and returns a VRState object.

        Returns None if no new or valid data is available.
        """  # noqa: E501
        vr_device_id, raw_vr_msg_str = xrt.get_latest_message()

        if not raw_vr_msg_str:
            self.get_logger().debug("No new message from SDK.")
            return None

        # NOTE: The data source seems to double-encode the JSON.
        # This is inefficient but handled here as required.
        # The structure is: '{"value": "{\\"Head\\":...}"}'
        value_str = json.loads(raw_vr_msg_str)["value"]
        raw_vr_msg = json.loads(value_str)

        current_timestamp_ns = int(raw_vr_msg["timeStampNs"])
        if current_timestamp_ns <= self._last_vr_time_ns:
            self.get_logger().warn(
                f"Timestamp did not increase. Skipping frame. "
                f"Last: {self._last_vr_time_ns}, Current: {current_timestamp_ns}"  # noqa: E501
            )
            return None

        self._last_vr_time_ns = current_timestamp_ns

        header = Header()
        header.frame_id = vr_device_id
        if self.config.timestamp_source == "ROS":
            header.stamp = self.get_clock().now().to_msg()
        else:  # "VR"
            NS_PER_SECOND = 1_000_000_000  # noqa: N806
            header.stamp.sec = current_timestamp_ns // NS_PER_SECOND
            header.stamp.nanosec = current_timestamp_ns % NS_PER_SECOND

        def _parse_pose(data: str) -> Pose:
            """Parses a raw Pico pose and converts its basis to the ROS standard.

            This function handles the crucial conversion of absolute pose data from
            the Pico SDK's native coordinate system to the ROS standard (REP-105).

            The core insight is that the raw data, while representing an
            absolute pose, is mathematically a **transformation operator** defined
            in the Pico coordinate basis. A simple coordinate rotation is
            insufficient as it fails to transform the basis in which the
            operator itself is expressed.

            The correct method is to perform a **change of basis** for the
            operator. This is achieved via the similarity transform:
            `M_ros = C @ M_pico @ C⁻¹`, where `C` is the change-of-basis
            matrix (`t_pico2ros`) and `M_pico` is the original transformation
            operator from the SDK.

            The resulting matrix `M_ros` is the equivalent transformation
            operator, but expressed purely in the ROS basis. Its components
            (translation and rotation) represent the final absolute pose in the
            ROS world, which are then packaged into a `Pose` message.
            """  # noqa: E501

            raw_pose = [float(data_i) for data_i in data.split(",")]
            if len(raw_pose) != 7:
                raise ValueError(
                    f"Pico pose data must have 7 elements, but got {len(raw_pose)}"  # noqa: E501
                )

            # Represent the raw Pico transform as a 4x4 homogeneous transformation matrix.  # noqa: E501
            pico_transform = np.identity(4)
            pico_transform[:3, :3] = Rotation.from_quat(
                raw_pose[3:]
            ).as_matrix()
            pico_transform[:3, 3] = raw_pose[:3]

            # Apply the similarity transform to change the basis of the operator.  # noqa: E501
            ros_transform = (
                self.t_pico2ros @ pico_transform @ self.t_pico2ros_inv
            )

            # Extract the final position and orientation from the new ROS-native matrix.  # noqa: E501
            ros_transform_ts = ros_transform[:3, 3]
            ros_transform_rot = Rotation.from_matrix(ros_transform[:3, :3])
            ros_transform_quat = ros_transform_rot.as_quat()

            position = Point(
                x=ros_transform_ts[0],
                y=ros_transform_ts[1],
                z=ros_transform_ts[2],
            )
            orientation = Quaternion(
                x=ros_transform_quat[0],
                y=ros_transform_quat[1],
                z=ros_transform_quat[2],
                w=ros_transform_quat[3],
            )

            pose = Pose(
                position=position,
                orientation=orientation,
            )
            return pose

        def _parse_controller(data: dict, is_left: bool) -> dict:
            """Parses a controller data dictionary into a format for the ROS message."""  # noqa: E501
            ret = dict(
                pose=_parse_pose(data["pose"]),
                trigger=data["trigger"],
                gripper=data["grip"],
                menu_button=data["menuButton"],
                axis_click=data["axisClick"],
                axis_x=data["axisX"],
                axis_y=data["axisY"],
            )
            if is_left:
                ret["x_button"] = data["primaryButton"]
                ret["y_button"] = data["secondaryButton"]
            else:
                ret["a_button"] = data["primaryButton"]
                ret["b_button"] = data["secondaryButton"]
            return ret

        if "Head" in raw_vr_msg:
            head = Head(
                pose=_parse_pose(raw_vr_msg["Head"]["pose"]),
                status=1,
            )
        else:
            self.get_logger().warn("Missing head message")
            head = Head(status=0)

        controller_msg = raw_vr_msg.get("Controller", dict())
        if "left" in controller_msg:
            left_controller = LeftController(
                **_parse_controller(controller_msg["left"], is_left=True),
                status=1,
            )
        else:
            self.get_logger().warn("Missing left_controller message")
            left_controller = LeftController(status=0)

        if "right" in controller_msg:
            right_controller = RightController(
                **_parse_controller(controller_msg["right"], is_left=False),
                status=1,
            )
        else:
            self.get_logger().warn("Missing right_controller message")
            right_controller = RightController(status=0)

        return VRState(
            header=header,
            head=head,
            left_controller=left_controller,
            right_controller=right_controller,
        )

    def _broadcast_transform(self, msg: VRState):
        transforms_to_broadcast = []

        if msg.head.status != 0:
            transforms_to_broadcast.append(
                TransformStamped(
                    header=msg.header,
                    child_frame_id="head",
                    transform=pose_msg_2_transform_msg(msg.head.pose),
                )
            )
        if msg.left_controller.status != 0:
            transforms_to_broadcast.append(
                TransformStamped(
                    header=msg.header,
                    child_frame_id="left_controller",
                    transform=pose_msg_2_transform_msg(
                        msg.left_controller.pose
                    ),
                )
            )
        if msg.right_controller.status != 0:
            transforms_to_broadcast.append(
                TransformStamped(
                    header=msg.header,
                    child_frame_id="right_controller",
                    transform=pose_msg_2_transform_msg(
                        msg.right_controller.pose
                    ),
                )
            )

        if transforms_to_broadcast:
            self.tf_broadcaster.sendTransform(transforms_to_broadcast)

    def xr_monitor(self):
        msg = self._parse_vr_state()

        if msg is None:
            return

        self._cnt += 1
        if self._cnt % 4096 == 0:
            self.get_logger().info(f"Publish {self._cnt}-th message")

        self._publisher.publish(msg)

        if self.config.pub_tf:
            self._broadcast_transform(msg)

    def _initialize(self):
        self.declare_parameter(
            "fps",
            90,
            descriptor=ParameterDescriptor(description="fps."),
        )
        self.declare_parameter(
            "timestamp_source",
            "VR",
            descriptor=ParameterDescriptor(description="timestamp source."),
        )
        self.declare_parameter(
            "pub_tf",
            True,
            descriptor=ParameterDescriptor(
                description="Whether or not publish tf."
            ),
        )

        self.config = PicoBridgeConfig(
            fps=self.get_parameter("fps").value,
            timestamp_source=self.get_parameter("timestamp_source").value,
            pub_tf=self.get_parameter("pub_tf").value,
        )

        xrt.init()

        atexit.register(xrt.close)


def main(args=None):
    rclpy.init(args=args)

    node = PicoBridge()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
