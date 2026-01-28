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
from io import BytesIO

import numpy as np
import requests
from rclpy.node import Node

from robo_orchard_deploy_ros2.config import DeployConfig


class ModelInferencer:
    def __init__(self, node: Node, config: DeployConfig):
        self._node = node
        self._config = config

    def _encode_np_array(self, data) -> BytesIO:
        """Encode numpy array into a binary stream for transmission.

        Args:
            data (np.ndarray): The numpy array to encode.

        Returns:
            BytesIO: A binary stream containing the encoded data.
        """
        buff = BytesIO()
        np.save(buff, data)
        buff.seek(0)
        return buff

    def _pack_request_data(self, observation):
        request_file = {}
        request_data = {}
        for key, value in observation.items():
            if value is not None and isinstance(value, np.ndarray):
                request_file[key] = (
                    f"{key}.bin",
                    self._encode_np_array(observation[key]),
                    "application/octet-stream",
                )
        request_data["instruction"] = (
            self._node.get_parameter("instruction")
            .get_parameter_value()
            .string_value
        )
        request_data["delay_horizon"] = (
            self._config.delay_horizon if self._config.delay_horizon else None
        )
        return request_file, request_data

    def request_inference(self, observation):
        """Send a request to the model infer server.

        Args:
            request_files: Request files to send to the model server.
            request_datas: Request data to send to the model server.

        Returns:
            model restult
        """
        request_files, request_datas = self._pack_request_data(observation)
        if not request_files:
            self._node.get_logger().error(
                "No request files to send to model server."
            )
            return None
        elif not request_datas:
            self._node.get_logger().error(
                "No request data to send to model server."
            )
            return None

        try:
            response = requests.post(
                self._config.server_url,
                files=request_files,
                data=request_datas,
                timeout=30,
            )
            if response.status_code != requests.codes.ok:
                self._node.get_logger().error(
                    f"Get an error when request {self._config.server_url}: "
                    f"{response.content}"
                )
            try:
                predict_actions = json.loads(response.content)
                return predict_actions
            except json.JSONDecodeError as e:
                self._node.get_logger().error(
                    f"Failed to decode JSON from server response: {e}"
                )
                return None
        except requests.exceptions.RequestException as e:
            self._node.get_logger().error(
                f"Request to model server failed: {e}"
            )
            return None
