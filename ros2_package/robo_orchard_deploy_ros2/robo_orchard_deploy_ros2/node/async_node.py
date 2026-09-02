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
    EXECUTING = 0
    PAUSED = 1


class DeployNode(Node):
    """A ROS2 node that deploys a robot asynchronously.

    The node synchronizes the configured observation channels and requests
    inference on its own timer, without waiting for the current action
    sequence to run out. Each response is spliced onto the step the robot
    has reached, so control keeps running while a request is in flight.

    How many arms or hands the embodiment has comes from the config; the
    node itself is embodiment agnostic.
    """

    def __init__(self):
        super().__init__("async_deploy_node")
        self._initialize()
        self.obs_manager = ObservationManager(self, self.config)
        self.model_inferencer = ModelInferencer(self, self.config)
        self.action_executor = ActionExecutor(self, self.config)

        self._model_infer_callback_group = MutuallyExclusiveCallbackGroup()
        self.model_infer_timer = self.create_timer(
            1 / self.config.infer_frequency,
            self._model_infer_timer_callback,
            callback_group=self._model_infer_callback_group,
        )

        self._control_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_timer = self.create_timer(
            1 / self.config.control_config.control_frequency,
            self._action_timer_callback,
            callback_group=self._control_callback_group,
        )
        self.current_actions = None
        self.current_action_idx = 0
        self.state = NodeState.PAUSED
        self.shared_state_lock = threading.Lock()

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
            f"Initialized AsyncDeployNode with model server "
            f"{self.config.server_url}"
        )

        if self.config.max_delay_horizon is None:
            self.max_delay_horizon = (
                self.config.control_config.control_frequency / 2
            )
        else:
            self.max_delay_horizon = self.config.max_delay_horizon

    def _extract_remaining_actions(self):
        """Return the unpublished steps and the index they follow."""
        if self.current_actions is None:
            return {}, None
        remaining = self.action_executor.remaining_actions(
            self.current_actions, self.current_action_idx
        )
        if not remaining:
            return {}, None
        return remaining, self.current_action_idx

    def _model_infer_timer_callback(self):
        """Timer callback to request model inference in given frequency.

        This function collects the current observations, sends them to the
        model inference server, calulates the action idex based on
        the time taken for inference, and updates the current actions
        accordingly.
        """
        with self.shared_state_lock:
            if self.state != NodeState.EXECUTING:
                return
        current_observations = self.obs_manager.get_observations()
        if not current_observations:
            self.get_logger().warning("No observations received yet.")
            return

        remaining_actions_start_idx = None

        with self.shared_state_lock:
            remaining_actions, remaining_actions_start_idx = (
                self._extract_remaining_actions()
            )
        current_observations.update(remaining_actions)
        predict_actions = self.model_inferencer.request_inference(
            current_observations
        )
        if predict_actions is None:
            self.get_logger().error("Model server returns no actions.")
            return
        new_action = predict_actions.copy()
        with self.shared_state_lock:
            old_action = (
                self.current_actions.copy()
                if self.current_actions is not None
                else None
            )
            if old_action is None:
                self.current_actions = new_action.copy()
                self.current_action_idx = 0
            elif new_action != old_action:
                if remaining_actions_start_idx is not None:
                    cur_delay_horizon = (
                        self.current_action_idx - remaining_actions_start_idx
                    )
                    if cur_delay_horizon > self.max_delay_horizon:
                        self.get_logger().warning(
                            "Excessive latency detected, exceeding the limit: "
                            "{cur_delay_horizon} > {self.max_delay_horizon}."
                        )
                        return
                    else:
                        self.current_action_idx = cur_delay_horizon
                else:
                    self.current_action_idx = 0
                self.current_actions = new_action.copy()

    def _action_timer_callback(self):
        """Timer callback to execute actions at the control frequency.

        This function checks the current state and action index, and
        sends the appropriate action to the robot arms by time elapsed.
        """
        with self.shared_state_lock:
            if self.state != NodeState.EXECUTING:
                return

            if self.current_actions is None:
                return

            step_count = self.action_executor.action_step_count(
                self.current_actions
            )
            if self.current_action_idx >= step_count:
                self.get_logger().info(
                    "Wait for new actions.", throttle_duration_sec=1
                )
                return
            current_action_local = self.current_actions
            current_idx_local = self.current_action_idx
            self.current_action_idx += 1

        self.action_executor.send_action(
            current_action_local, current_idx_local
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

    def _enable_inference_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Callback to resume the inference and action execution."""
        with self.shared_state_lock:
            if self.state == NodeState.PAUSED:
                self.current_actions = None
                self.current_action_idx = 0
                self.state = NodeState.EXECUTING
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
        with self.shared_state_lock:
            if self.state == NodeState.EXECUTING:
                self.state = NodeState.PAUSED
                response.success = True
                self.current_actions = None
                self.current_action_idx = 0
                response.message = "Node paused."
                self.get_logger().info("Node paused.")
            else:
                response.success = True
                response.message = "Node is already paused."
                self.get_logger().warning("Node is already paused.")
        return response


def main(args=None):
    rclpy.init(args=args)
    excutor = rclpy.executors.MultiThreadedExecutor()
    deploy_node = DeployNode()
    excutor.add_node(deploy_node)

    try:
        excutor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        excutor.shutdown()
