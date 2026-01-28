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

import argparse
import os

from robo_orchard_handeye_calib_ros2.config import HandEyeCalibrationConfig


def main():
    parser = argparse.ArgumentParser(
        description="Generate Hand-Eye Calibration Config"
    )
    parser.add_argument(
        "--mode",
        type=str,
        help="Mode",
        required=True,
    )
    parser.add_argument(
        "--camera_frame_name",
        type=str,
        help="Camera frame name",
        required=True,
    )
    parser.add_argument(
        "--marker_frame",
        type=str,
        help="Aruco marker frame name",
        required=True,
    )
    parser.add_argument(
        "--end_effector_frame_name",
        type=str,
        help="End effector frame name",
        required=True,
    )
    parser.add_argument(
        "--base_frame_name",
        type=str,
        help="Base frame name",
        required=True,
    )
    parser.add_argument(
        "--end_effector_pose_topic_name",
        type=str,
        help="End effector pose topic name",
        required=True,
    )
    parser.add_argument(
        "--result_file",
        type=str,
        help="Result file name",
        required=True,
    )
    args = parser.parse_args()
    config = HandEyeCalibrationConfig(
        mode=args.mode,
        end_effector_frame_name=args.end_effector_frame_name,
        camera_frame_name=args.camera_frame_name,
        aruco_marker_frame_name=args.marker_frame,
        base_frame_name=args.base_frame_name,
        aruco_marker_pose_topic_name="/handeye_calib/aruco_single_node/pose",
        end_effector_pose_topic_name=args.end_effector_pose_topic_name,
        result_file=args.result_file,
    )
    with open(
        os.path.join(os.path.dirname(__file__), "handeye_calib_config.json"),
        "w",
    ) as f:
        f.write(config.model_dump_json(indent=4))


if __name__ == "__main__":
    main()
