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

import json
import os

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node, ParameterDescriptor
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from robo_orchard_handeye_calib_ros2.config import (
    HandEyeCalibrationConfig,
)
from robo_orchard_handeye_calib_ros2.core import (
    run_eye_in_hand_calibration,
    run_eye_to_hand_calibration,
)


class CalibrationNode(Node):
    def __init__(self):
        super().__init__("handeye_calibration_node")
        self._tf_broadcaster = StaticTransformBroadcaster(self)

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
                f"config file does not exists!({config_file})"
            )
        with open(config_file, "r") as f:
            self.config: HandEyeCalibrationConfig = (
                HandEyeCalibrationConfig.model_validate_json(f.read())
            )  # noqa: E501

        self._eepose_sub = Subscriber(
            self,
            PoseStamped,
            self.config.end_effector_pose_topic_name,
        )
        self._aruco_sub = Subscriber(
            self,
            PoseStamped,
            self.config.aruco_marker_pose_topic_name,
        )
        self.observation_subs = [self._aruco_sub, self._eepose_sub]
        self.sync = ApproximateTimeSynchronizer(
            self.observation_subs, queue_size=1, slop=0.05
        )
        self.sync.registerCallback(self._obs_callback)

        self.cur_aruco_pose = None
        self.cur_ee_pose = None
        self.aruco_poses_list = []
        self.ee_poses_list = []
        self.record_data_cnt = 0

        self.data_cb = MutuallyExclusiveCallbackGroup()
        self.create_service(
            srv_type=Trigger,
            srv_name="record_data",
            callback=self._record_data_service_callback,
            callback_group=self.data_cb,
        )

        self.create_service(
            srv_type=Trigger,
            srv_name="save_data",
            callback=self._save_data_service_callback,
            callback_group=self.data_cb,
        )
        self.get_logger().info("Hand-eye calibration node initialized.")

    def _obs_callback(self, aruco_msg: PoseStamped, ee_msg: PoseStamped):
        self.cur_aruco_pose = aruco_msg
        self.cur_ee_pose = ee_msg

    def _record_data_service_callback(self, request, response):
        if self.cur_aruco_pose is not None and self.cur_ee_pose is not None:
            # Check message freshness - ensure messages are recent
            current_time = self.get_clock().now().nanoseconds / 1e9

            aruco_pose_timestamp = self.cur_aruco_pose.header.stamp
            ee_pose_timestamp = self.cur_ee_pose.header.stamp
            aruco_pose_time = (
                aruco_pose_timestamp.sec + aruco_pose_timestamp.nanosec / 1e9
            )
            ee_pose_time = (
                ee_pose_timestamp.sec + ee_pose_timestamp.nanosec / 1e9
            )

            decay_threshold = 0.1
            if (
                abs(aruco_pose_time - current_time) > decay_threshold
                or abs(ee_pose_time - current_time) > decay_threshold
            ):
                response.success = False
                response.message = (
                    "Message is too old, check the topics and record again."
                )
                self.cur_aruco_pose = None
                self.cur_ee_pose = None
                return response

            if abs(aruco_pose_time - ee_pose_time) > decay_threshold:
                response.success = False
                response.message = (
                    "Aruco pose and end effector pose timestamps differ "
                    f"by more than {decay_threshold} seconds, "
                    "check aruco marker whether in camera field."
                )
                return response

            self.aruco_poses_list.append(self.cur_aruco_pose)
            self.ee_poses_list.append(self.cur_ee_pose)
            self.record_data_cnt += 1
            self.cur_aruco_pose = None
            self.cur_ee_pose = None
            response.success = True
            response.message = f"Recorded {self.record_data_cnt} data."
            return response
        else:
            response.success = False
            response.message = (
                "Current aruco pose or end effector pose is None, "
                "please check the topics."
            )
            return response

    def _save_data_service_callback(self, request, response):
        if len(self.aruco_poses_list) < 3 or len(self.ee_poses_list) < 3:
            response.success = False
            response.message = (
                "Not enough data, please record at least 3 poses."
            )
            return response
        self.get_logger().info(
            f"Recording finished, result will be saved to"
            f" {self.config.result_file}"
        )
        parent_frame = None
        child_frame = None
        try:
            if self.config.mode == "eye_in_hand":
                res_position, res_orientation = run_eye_in_hand_calibration(  # noqa: E501
                    self.aruco_poses_list,
                    self.ee_poses_list,
                    self.record_data_cnt,
                )
                parent_frame = self.config.end_effector_frame_name
                child_frame = self.config.camera_frame_name
            elif self.config.mode == "eye_to_hand":
                res_position, res_orientation = run_eye_to_hand_calibration(  # noqa: E501
                    self.aruco_poses_list,
                    self.ee_poses_list,
                    self.record_data_cnt,
                )
                parent_frame = self.config.base_frame_name
                child_frame = self.config.camera_frame_name
            else:
                response.success = False
                response.message = (
                    "Invalid mode, please check the config file."
                )
                return response
        except Exception as e:
            response.success = False
            response.message = f"Calibration failed: {e}"
            return response
        with open(self.config.result_file, "w") as f:
            json.dump(
                {
                    "parent_frame": parent_frame,
                    "child_frame": child_frame,
                    "result": {
                        "position": res_position,
                        "orientation": res_orientation,
                    },
                },
                f,
                indent=4,
            )
        # publish to ros2 tf_tree
        self._publish_tf(
            res_position, res_orientation, parent_frame, child_frame
        )
        response.success = True
        response.message = (
            f"Calibration result saved in {self.config.result_file}."
        )
        return response

    def _publish_tf(
        self, res_position, res_orientation, parent_frame, child_frame
    ):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = res_position[0]
        t.transform.translation.y = res_position[1]
        t.transform.translation.z = res_position[2]
        t.transform.rotation.x = res_orientation[0]
        t.transform.rotation.y = res_orientation[1]
        t.transform.rotation.z = res_orientation[2]
        t.transform.rotation.w = res_orientation[3]
        self._tf_broadcaster.sendTransform(t)
        self.get_logger().info("Result saved and broadcasted to tf tree.")


def main(args=None):
    rclpy.init()
    try:
        node = CalibrationNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n[INFO] Ctrl+C pressed, shutting down...")
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
