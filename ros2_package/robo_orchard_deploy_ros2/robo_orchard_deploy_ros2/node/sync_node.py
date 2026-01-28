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

import os
import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node, ParameterDescriptor
from std_srvs.srv import Trigger

from robo_orchard_deploy_ros2.action_exec import ActionExecutor
from robo_orchard_deploy_ros2.config import DeployConfig
from robo_orchard_deploy_ros2.model_request import ModelInferencer
from robo_orchard_deploy_ros2.obs_manager import ObservationManager


class NodeState:
    INIT = 0
    IDLE = 1
    EXECUTING = 2
    PAUSED = 3


class DeployNode(Node):
    """A ROS2 node that deploys a dual arm robot system synchronously.

    The node subscribes to various sensor topics, synchronizes them,
    and sends the observations to a model server for inference. The inferred
    actions are then executed on the robot arms at a controlled frequency.
    """  # noqa: E501

    def __init__(self):
        super().__init__("sync_deploy_node")
        self._initialize()
        self.obs_manager = ObservationManager(self, self.config)
        self.model_inferencer = ModelInferencer(self, self.config)
        self.action_executor = ActionExecutor(self, self.config)

        self._model_infer_callback_group = MutuallyExclusiveCallbackGroup()
        self._model_infer_timer = self.create_timer(
            1 / self.config.infer_frequency,
            self._model_infer_callback,
            self._model_infer_callback_group,
        )
        self._action_callback_group = MutuallyExclusiveCallbackGroup()
        self._action_timer = self.create_timer(
            1 / self.config.control_config.control_frequency,
            self._action_timer_callback,
            self._action_callback_group,
        )
        self.current_actions = None
        self.current_action_index = 0
        self.state = NodeState.INIT
        self.state_lock = threading.Lock()

        self.create_service(
            Trigger,
            "enable",
            self._enable_inference_callback,
        )
        self.create_service(
            Trigger,
            "disable",
            self._disable_inference_callback,
        )

        self.get_logger().info(
            f"Initialized SyncDeployNode with model server "
            f"{self.config.server_url}"
        )

    def _initialize(self):
        self.declare_parameter(
            "config_file",
            "",
            descriptor=ParameterDescriptor(description="Config path"),
        )
        self.declare_parameter(
            "instruction",
            "Do something.",
            descriptor=ParameterDescriptor(description="Task instruction"),
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
        with open(config_file, "r") as f:
            self.config: DeployConfig = DeployConfig.model_validate_json(
                f.read()
            )

    def _model_infer_callback(self):
        """Timer callback to request new actions from the model server."""
        with self.state_lock:
            if self.state != NodeState.IDLE:
                return
        cur_obs = self.obs_manager.get_observations()
        if not cur_obs:
            self.get_logger().warn(
                "No observations received yet.", throttle_duration_sec=1
            )
            return
        predict_actions = self.model_inferencer.request_inference(cur_obs)
        if predict_actions is None:
            self.get_logger().warn("No actions received from model server.")
            return
        else:
            with self.state_lock:
                self.current_actions = predict_actions
                self.current_action_index = 0
                if self.state == NodeState.IDLE:
                    self.state = NodeState.EXECUTING

    def _action_timer_callback(self):
        """Timer callback to execute actions at the control frequency."""
        with self.state_lock:
            if self.state != NodeState.EXECUTING:
                return

            if self.current_action_index >= self.current_actions.get(
                "action_horizon", 0
            ):
                self.get_logger().info(
                    "[IDLE]: Wait for new actions.", throttle_duration_sec=1
                )
                self.current_actions = None
                self.state = NodeState.IDLE
                return

            try:
                self.action_executor.send_action(
                    self.current_actions, self.current_action_index
                )
                self.get_logger().debug(
                    f"[EXECUTING]: Action {self.current_action_index} sent."
                )
                self.current_action_index += 1
            except Exception as e:
                self.get_logger().error(
                    f"[EXECUTING]: Error when sending action: {e}"
                )
                self.current_actions = None
                self.current_action_index = 0
                self.state = NodeState.IDLE
                return

    def _enable_inference_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Callback to resume the inference and action execution."""
        with self.state_lock:
            if self.state != NodeState.EXECUTING:
                self.current_actions = None
                self.current_action_index = 0
                self.state = NodeState.IDLE
                response.success = True
                response.message = "Node executing."
                self.get_logger().info("Node executing.")
            else:
                response.success = True
                response.message = "Node is already executing."
                self.get_logger().warning("Node is already executing.")
        return response

    def _disable_inference_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Callback to pause the inference and action execution."""
        with self.state_lock:
            if (
                self.state == NodeState.EXECUTING
                or self.state == NodeState.IDLE
            ):  # noqa: E501
                self.state = NodeState.PAUSED
                response.success = True
                self.current_actions = None
                self.current_action_index = 0
                response.message = "Node paused."
                self.get_logger().info("Node paused.")
            elif self.state == NodeState.PAUSED:
                response.success = True
                response.message = "Node is already paused."
                self.get_logger().warning("Node is already paused.")
            else:
                response.success = True
                response.message = "Node is not executing."
                self.get_logger().warning("Node is in INIT state.")
        return response


def main(args=None):
    rclpy.init(args=args)
    excutor = rclpy.executors.MultiThreadedExecutor()
    excutor.add_node(DeployNode())

    try:
        excutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        excutor.shutdown()
