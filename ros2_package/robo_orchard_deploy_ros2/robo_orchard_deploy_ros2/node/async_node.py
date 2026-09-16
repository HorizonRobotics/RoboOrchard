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

import math
import os
import threading
from time import perf_counter

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node, ParameterDescriptor
from std_srvs.srv import Trigger

from robo_orchard_deploy_msg_ros2.msg import InferenceEvent, InferenceStatus
from robo_orchard_deploy_ros2.action_exec import ActionExecutor
from robo_orchard_deploy_ros2.config import DeployConfig
from robo_orchard_deploy_ros2.model_request import ModelInferencer
from robo_orchard_deploy_ros2.obs_manager import ObservationManager
from robo_orchard_deploy_ros2.trajectory_stitcher import TrajectoryStitcher


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

    Stitched chunks target a future control step, allowing the old chunk
    to keep running during the solve. The previous solve duration plus one
    control period sets the lead time. A result that misses its scheduled
    step is dropped rather than switched at an unconstrained point.
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
        # Where the live chunk began executing, so the next solve knows how
        # far it got.
        self._chunk_start_idx = 0
        self._pending_actions: tuple[int, dict, int] | None = None
        self._stitch_lead_steps = 1
        self.state = NodeState.PAUSED
        self.shared_state_lock = threading.Lock()
        self._inference_generation = 0
        self._status_publisher = self.create_publisher(
            InferenceStatus, "status", 10
        )
        self._event_publisher = self.create_publisher(
            InferenceEvent, "events", 10
        )
        self._status_timer = self.create_timer(1.0, self._publish_status)

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

        self._stitcher = TrajectoryStitcher(
            self.config.trajectory_stitch,
            self.config.control_config.control_frequency,
            [
                channel.server_output_key
                for channel in self.config.control_config.channels
            ],
            self.get_logger(),
        )
        if self._stitcher.enabled:
            self.get_logger().info("Chunk stitching enabled.")
        self._publish_status()

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
            if (
                self.state != NodeState.EXECUTING
                or self._pending_actions is not None
            ):
                return
            request_generation = self._inference_generation
        current_observations = self.obs_manager.get_observations()
        if not current_observations:
            self.get_logger().warning("No observations received yet.")
            return

        remaining_actions_start_idx = None

        with self.shared_state_lock:
            if (
                request_generation != self._inference_generation
                or self.state != NodeState.EXECUTING
            ):
                return
            requested_actions = self.current_actions
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
            if (
                self.state != NodeState.EXECUTING
                or request_generation != self._inference_generation
                or self.current_actions is not requested_actions
            ):
                self.get_logger().debug("Discarding stale inference response.")
                return
            if self.current_actions is None:
                install_idx, prev_ran = 0, 0
            elif new_action == self.current_actions:
                return
            else:
                prev_ran = self.current_action_idx - self._chunk_start_idx
                if remaining_actions_start_idx is None:
                    install_idx = 0
                else:
                    install_idx = (
                        self.current_action_idx - remaining_actions_start_idx
                    )
                    if install_idx > self.max_delay_horizon:
                        self.get_logger().warning(
                            "Excessive latency detected, exceeding the "
                            f"limit: {install_idx} > "
                            f"{self.max_delay_horizon}."
                        )
                        # Nothing to undo: the solve is promoted on install,
                        # so the stitcher still holds the chunk in flight.
                        return

            switch_idx = self.current_action_idx
            held_actions = None
            if self._stitcher.enabled and self.current_actions is not None:
                current_step_count = self.action_executor.action_step_count(
                    self.current_actions
                )
                if current_step_count > 0 and switch_idx >= current_step_count:
                    held_actions = self.current_actions
                lead_steps = max(
                    0,
                    min(
                        self._stitch_lead_steps,
                        int(self.max_delay_horizon) - install_idx,
                        current_step_count - switch_idx - 1,
                        self.action_executor.action_step_count(new_action)
                        - install_idx
                        - 1,
                    ),
                )
                switch_idx += lead_steps
                install_idx += lead_steps
                prev_ran += lead_steps

        if self._stitcher.enabled:
            solve_started = perf_counter()
            new_action = self._stitcher.stitch(
                new_action,
                install_idx,
                prev_ran,
                held_actions=held_actions,
            )
            with self.shared_state_lock:
                if (
                    self.state != NodeState.EXECUTING
                    or request_generation != self._inference_generation
                    or self.current_actions is not requested_actions
                ):
                    self._stitcher.discard_pending()
                    return
                self._stitch_lead_steps = (
                    math.ceil(
                        (perf_counter() - solve_started)
                        * self.config.control_config.control_frequency
                    )
                    + 1
                )
                if self.current_action_idx > switch_idx:
                    self.get_logger().warning(
                        "Missed the planned handover step; keeping the "
                        "current actions."
                    )
                    return
                if self.current_action_idx == switch_idx:
                    self._switch_actions(new_action, install_idx)
                else:
                    self._pending_actions = (
                        switch_idx,
                        new_action,
                        install_idx,
                    )
            return

        with self.shared_state_lock:
            if (
                self.state != NodeState.EXECUTING
                or request_generation != self._inference_generation
                or self.current_actions is not requested_actions
            ):
                return
            if remaining_actions_start_idx is not None:
                install_idx = max(
                    install_idx,
                    self.current_action_idx - remaining_actions_start_idx,
                )
                if install_idx > self.max_delay_horizon:
                    self.get_logger().warning(
                        "Excessive latency before the handover, "
                        f"exceeding the limit: {install_idx} > "
                        f"{self.max_delay_horizon}."
                    )
                    return
            self._switch_actions(new_action, install_idx)

    def _switch_actions(self, actions: dict, start_idx: int) -> None:
        """Switch chunks while holding ``shared_state_lock``."""
        self.current_actions = actions
        self.current_action_idx = start_idx
        self._chunk_start_idx = start_idx
        self._pending_actions = None
        if self._stitcher.enabled:
            self._stitcher.commit()

    def _action_timer_callback(self):
        """Timer callback to execute actions at the control frequency.

        This function checks the current state and action index, and
        sends the appropriate action to the robot arms by time elapsed.
        """
        with self.shared_state_lock:
            if self.state != NodeState.EXECUTING:
                return

            if self._pending_actions is not None:
                switch_idx, actions, start_idx = self._pending_actions
                if self.current_action_idx == switch_idx:
                    self._switch_actions(actions, start_idx)

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

    def _invalidate_actions_locked(self) -> None:
        """Drop all action state while holding ``shared_state_lock``."""
        self._inference_generation += 1
        self.current_actions = None
        self.current_action_idx = 0
        self._chunk_start_idx = 0
        self._pending_actions = None
        self._stitch_lead_steps = 1
        self.action_executor.reset_limiter()
        self._stitcher.reset()

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

    def _publish_status(self) -> None:
        with self.shared_state_lock:
            self._publish_status_locked()

    def _publish_status_locked(self) -> None:
        message = InferenceStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.data = (
            InferenceStatus.ENABLED
            if self.state == NodeState.EXECUTING
            else InferenceStatus.DISABLED
        )
        self._status_publisher.publish(message)

    def _publish_lifecycle_change_locked(self, enabled: bool) -> None:
        self._publish_status_locked()
        event = InferenceEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.event_type = (
            InferenceEvent.ENABLE_TRIGGERED
            if enabled
            else InferenceEvent.DISABLE_TRIGGERED
        )
        event.details = (
            "Inference enabled." if enabled else "Inference disabled."
        )
        self._event_publisher.publish(event)

    def _enable_inference_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Callback to resume the inference and action execution."""
        with self.shared_state_lock:
            if self.state == NodeState.PAUSED:
                self._invalidate_actions_locked()
                self.state = NodeState.EXECUTING
                response.success = True
                response.message = "Node executing."
                self.get_logger().info("Node executing.")
                self._publish_lifecycle_change_locked(True)
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
            self._invalidate_actions_locked()
            if self.state == NodeState.EXECUTING:
                self.state = NodeState.PAUSED
                response.success = True
                response.message = "Node paused."
                self.get_logger().info("Node paused.")
                self._publish_lifecycle_change_locked(False)
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
