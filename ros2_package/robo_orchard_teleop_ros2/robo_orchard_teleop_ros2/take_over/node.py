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

import collections
from enum import Enum
from functools import partial

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node, ParameterDescriptor
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Trigger

from robo_orchard_teleop_msg_ros2.msg import (
    ControlMode as ControlModeMsg,
    TakeOverEvent,
)


class ControlMode(Enum):
    """Enumeration for the control states of the muxer."""

    AUTONOMOUS = "autonomous"
    OVERRIDE = "override"
    STOP = "stop"


class TakeOverMuxerNode(Node):
    """A flexible, stateful multiplexer for managing human takeover of a robot.

    This node acts as a stateful controller. It operates in one of three modes:

    - AUTONOMOUS: Forwards commands from a specified algorithm topic.

    - OVERRIDE: Forwards commands from a human/override topic, triggered by a
      service call. This mode can also be configured to be silent.

    - STOP: Blocks all incoming commands and sends a single zero-command to
      ensure the robot is stationary. This mode is triggered by a service call.

    The node continuously publishes its current control mode. Control can be
    returned to the autonomous system from either OVERRIDE or STOP mode via
    another service call.
    """  # noqa: E501

    def __init__(self):
        super().__init__("takeover_muxer_node")

        # --- Parameters ---
        self.declare_parameter(
            "message_type",
            "",
            ParameterDescriptor(
                description="The ROS2 message type for control commands."
            ),
        )
        self.declare_parameter(
            "algo_topic",
            "/algo_cmd",
            ParameterDescriptor(
                description="Topic for autonomous control commands."
            ),
        )
        self.declare_parameter(
            "override_topic",
            "/teach_cmd",
            ParameterDescriptor(
                description="Topic for human override commands."
            ),
        )
        self.declare_parameter(
            "output_topic",
            "/control_cmd",
            ParameterDescriptor(
                description="The topic to publish the final command to."
            ),
        )
        self.declare_parameter(
            "replay_time_s",
            2.0,
            ParameterDescriptor(
                description="Time in seconds to look back for the state reset command."  # noqa: E501
            ),
        )
        self.declare_parameter(
            "mode_publish_rate_hz",
            1.0,
            ParameterDescriptor(
                description="Rate to publish the current control mode."
            ),
        )
        self.declare_parameter(
            "override_mode_behavior",
            "forward",
            ParameterDescriptor(
                description="Behavior in override mode: 'forward' or 'silent'."
            ),
        )

        # Get parameter values
        self._message_type_str = self.get_parameter("message_type").value
        if not self._message_type_str:
            raise ValueError("You should provide message type")
        self._algo_topic = self.get_parameter("algo_topic").value
        self._override_topic = self.get_parameter("override_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._replay_time_s = self.get_parameter("replay_time_s").value
        mode_publish_rate = self.get_parameter("mode_publish_rate_hz").value

        self._override_behavior = self.get_parameter(
            "override_mode_behavior"
        ).value
        if self._override_behavior not in ["forward", "silent"]:
            raise ValueError(
                f"Invalid override_mode_behavior: {self._override_behavior}. "
                "Must be `forward` or `silent`. Shutting down."
            )

        # --- Internal State ---
        self._current_mode = ControlMode.AUTONOMOUS
        buffer_size = int(200 * (self._replay_time_s + 2.0))
        self._history = collections.deque(maxlen=buffer_size)

        # --- Load Message Type and Create Communications ---
        try:
            self._msg_type = get_message(self._message_type_str)
        except (ModuleNotFoundError, AttributeError) as e:
            raise TypeError(
                f"Failed to load message type {self._message_type_str}: {e}"
            )

        self._command_publisher = self.create_publisher(
            self._msg_type, self._output_topic, 1
        )
        self._mode_publisher = self.create_publisher(
            ControlModeMsg, "control_mode", 10
        )
        self._event_publisher = self.create_publisher(
            TakeOverEvent, "events", 10
        )

        # The subscriber for the algorithm topic is always needed for the history buffer.  # noqa: E501
        self.create_subscription(
            self._msg_type,
            self._algo_topic,
            partial(self._input_callback, topic_name=self._algo_topic),
            1,
        )

        # Conditionally create the subscriber for the override topic.
        if self._override_behavior == "forward":
            self.get_logger().info(
                f"Behavior is `forward`, creating subscriber for override topic: {self._override_topic}"  # noqa: E501
            )
            self.create_subscription(
                self._msg_type,
                self._override_topic,
                partial(self._input_callback, topic_name=self._override_topic),
                1,
            )
        else:  # Behavior is "silent"
            self.get_logger().info(
                "Behavior is `silent`, skipping creation of override topic subscriber."  # noqa: E501
            )

        # --- Services for state transition ---
        self.create_service(
            Trigger, "trigger_takeover", self._takeover_service_callback
        )
        self.create_service(
            Trigger, "release_control", self._release_service_callback
        )
        self.create_service(Trigger, "stop", self._stop_service_callback)

        # --- Timer for mode publishing ---
        self.create_timer(1.0 / mode_publish_rate, self._publish_current_mode)

        self.get_logger().info(
            f"TakeoverMuxerNode initialized. Default mode: AUTONOMOUS. Override behavior: {self._override_behavior}"  # noqa: E501
        )

    def _input_callback(self, msg, topic_name: str):
        """Processes incoming messages based on the current control mode and behavior."""  # noqa: E501
        if self._current_mode == ControlMode.STOP:
            return

        if (
            self._current_mode == ControlMode.AUTONOMOUS
            and topic_name == self._algo_topic
        ):
            self._history.append((self.get_clock().now(), msg))
            self._command_publisher.publish(msg)
        elif (
            self._current_mode == ControlMode.OVERRIDE
            and topic_name == self._override_topic
        ):
            if self._override_behavior == "forward":
                self._command_publisher.publish(msg)

    def _takeover_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Service to switch from AUTONOMOUS to OVERRIDE and perform replay."""
        if self._current_mode == ControlMode.OVERRIDE:
            response.success = True
            response.message = "Already in OVERRIDE mode."
            return response

        self._publish_event(
            TakeOverEvent.TAKEOVER_TRIGGERED, "Takeover service called."
        )

        self.get_logger().info(
            "Takeover triggered. Switching to OVERRIDE mode."
        )
        self._current_mode = ControlMode.OVERRIDE

        if self._replay_time_s <= 0.0:
            response.success = True
            response.message = "Takeover successful, but without any replay."
            self.get_logger().info(response.message)
            self._publish_current_mode()
            return response

        if not self._history:
            response.success = True
            response.message = "Takeover successful, but cannot replay: history buffer is empty."  # noqa: E501
            self.get_logger().warn(response.message)
            self._publish_current_mode()
            return response

        target_time = self.get_clock().now() - Duration(
            seconds=self._replay_time_s
        )

        found_message = None
        for timestamp, msg in reversed(self._history):
            if timestamp <= target_time:
                found_message = msg
                break

        if found_message:
            self.get_logger().info(
                f"Resetting state to ~{self._replay_time_s:.2f}s ago."
            )
            self._command_publisher.publish(found_message)

            details = (
                f"Replayed command from approx {self._replay_time_s:.2f}s ago."
            )
            self._publish_event(TakeOverEvent.REPLAY_COMMAND_SENT, details)

            response.message = "Takeover successful. Replay command published."
        else:
            response.message = "Takeover successful. No message is replay."

        response.success = True
        self._publish_current_mode()
        return response

    def _release_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Service to switch from OVERRIDE back to AUTONOMOUS."""
        if self._current_mode == ControlMode.AUTONOMOUS:
            response.success = True
            response.message = "Already in AUTONOMOUS mode."
            return response

        # Publish an event to record that the release was triggered.
        self._publish_event(
            TakeOverEvent.RELEASE_TRIGGERED, "Release control service called."
        )

        self.get_logger().info(
            "Control released. Switching back to AUTONOMOUS mode."
        )
        self._current_mode = ControlMode.AUTONOMOUS
        self._history.clear()
        response.success = True
        response.message = "Switched to AUTONOMOUS mode."
        self._publish_current_mode()
        return response

    def _stop_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """Service to enter STOP mode and publish a zero-command."""
        if self._current_mode == ControlMode.STOP:
            response.success = True
            response.message = "Already in STOP mode."
            return response

        self.get_logger().info("Stop triggered. Switching to STOP mode.")
        self._current_mode = ControlMode.STOP

        self._publish_event(
            TakeOverEvent.STOP_TRIGGERED, "Stop service called."
        )
        self._publish_current_mode()

        response.success = True
        response.message = "Switched to STOP mode and sent a zero-command."
        return response

    def _publish_current_mode(self):
        """Helper function to publish the current mode string."""
        mode_msg = ControlModeMsg()
        mode_msg.header.stamp = self.get_clock().now().to_msg()
        mode_msg.data = self._current_mode.value
        self._mode_publisher.publish(mode_msg)

    def _publish_event(self, event_type: str, details: str):
        """Helper function to publish a TakeoverEvent."""
        event_msg = TakeOverEvent()
        event_msg.header.stamp = self.get_clock().now().to_msg()
        event_msg.event_type = event_type
        event_msg.details = details
        self._event_publisher.publish(event_msg)


def main(args=None):
    """Main function to initialize and spin the node."""
    rclpy.init(args=args)
    node = TakeOverMuxerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
