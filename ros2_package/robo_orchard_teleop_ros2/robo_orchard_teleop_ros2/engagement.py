# Project RoboOrchard
#
# Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
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

from dataclasses import dataclass
from functools import partial
from math import isfinite
from time import monotonic
from typing import Any

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from robo_orchard_teleop_msg_ros2.msg import (
    ControlMode,
    TeleopActivationState,
    TeleopMatchState,
)
from robo_orchard_teleop_msg_ros2.srv import RecaptureBaseline


def load_config(path: str) -> dict[str, Any]:
    """Load one engagement group's YAML route and source configuration."""
    with open(path, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    if not isinstance(config, dict):
        raise ValueError("Engagement config must be a mapping")
    for name in ("match_state_topic", "recapture_service"):
        if not isinstance(config.get(name), str) or not config[name].strip():
            raise ValueError(f"{name} must be a non-empty string")
    routes = config.get("command_routes")
    if not isinstance(routes, list) or not routes:
        raise ValueError("command_routes must be a non-empty list")
    for route in routes:
        if not isinstance(route, dict) or any(
            not isinstance(route.get(name), str) or not route[name].strip()
            for name in ("input_topic", "output_topic")
        ):
            raise ValueError("Each route needs input_topic and output_topic")
    return config


@dataclass
class Snapshot:
    message: Any
    received: float
    stamp_ns: int


class EngagementNode(Node):
    """Gate one group of JointState routes after verified baseline recapture.

    Sources own readiness and matching. This node never selects Manager mode
    or calls hardware services; forwarded messages retain their source stamp.
    """

    def __init__(self, **kwargs):
        super().__init__("teleop_engagement", **kwargs)
        self.declare_parameter("config_file", "")
        self.declare_parameter("activation_topic", "/teleop/activation/state")
        self.declare_parameter("manager_status_topic", "/robot/control/status")
        self.declare_parameter("status_topic", "~/status")
        for name, default in (
            ("publish_hz", 30.0),
            ("stale_timeout_s", 0.3),
            ("manager_status_timeout_s", 2.5),
            ("service_timeout_s", 2.0),
            ("max_clock_skew_s", 0.05),
        ):
            self.declare_parameter(name, default)
            value = float(self.get_parameter(name).value)
            if (
                not isfinite(value)
                or value < 0
                or (name != "max_clock_skew_s" and value == 0)
            ):
                raise ValueError(f"{name} has an invalid value")
            setattr(self, name, value)
        config = load_config(self.get_parameter("config_file").value)
        pairs = [
            (
                self.resolve_topic_name(route["input_topic"]),
                self.resolve_topic_name(route["output_topic"]),
            )
            for route in config["command_routes"]
        ]
        inputs, outputs = zip(*pairs, strict=True)
        if (
            len(set(inputs)) != len(inputs)
            or len(set(outputs)) != len(outputs)
            or set(inputs) & set(outputs)
        ):
            raise ValueError("Routes contain duplicates or input/output loops")
        self._outputs = {
            source: self.create_publisher(JointState, target, 1)
            for source, target in pairs
        }
        self._samples: dict[str, Snapshot] = {}
        self._last_stamps: dict[str, int] = {}
        self._phase = "idle"
        self._reason = "Release required."
        self._armed = False
        self._future = None
        self._deadline = 0.0
        self._barrier_topic = ""
        self._barrier = 0
        self._cutover_stamps: dict[str, int] = {}
        for topic in inputs:
            self.create_subscription(
                JointState, topic, partial(self._candidate, topic), 1
            )
        for key, message_type, topic in (
            (
                "activation",
                TeleopActivationState,
                self.get_parameter("activation_topic").value,
            ),
            (
                "manager",
                ControlMode,
                self.get_parameter("manager_status_topic").value,
            ),
            ("match", TeleopMatchState, config["match_state_topic"]),
        ):
            self.create_subscription(
                message_type, topic, partial(self._state_message, key), 1
            )
        self._client = self.create_client(
            RecaptureBaseline, config["recapture_service"]
        )
        self._status_publisher = self.create_publisher(
            DiagnosticArray, self.get_parameter("status_topic").value, 1
        )
        self._timer = self.create_timer(1.0 / self.publish_hz, self._tick)

    @staticmethod
    def _stamp(message) -> int:
        stamp = message.header.stamp
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def _stamp_valid(self, stamp_ns: int, key: str = "") -> bool:
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        return stamp_ns > 0 and (
            -self.max_clock_skew_s <= age <= self._timeout(key)
        )

    def _timeout(self, key: str) -> float:
        return (
            self.manager_status_timeout_s
            if key == "manager"
            else self.stale_timeout_s
        )

    def _fresh(self, key: str) -> bool:
        sample = self._samples.get(key)
        return sample is not None and (
            monotonic() - sample.received <= self._timeout(key)
            and self._stamp_valid(sample.stamp_ns, key)
        )

    def _cache(self, key: str, message) -> bool:
        stamp_ns = self._stamp(message)
        if not self._stamp_valid(stamp_ns, key) or (
            stamp_ns <= self._last_stamps.get(key, 0)
        ):
            self._samples.pop(key, None)
            self._close(f"Invalid or regressing {key} stamp.")
            return False
        self._last_stamps[key] = stamp_ns
        self._samples[key] = Snapshot(message, monotonic(), stamp_ns)
        return True

    def _close(self, reason: str) -> None:
        self._phase = "idle"
        self._reason = reason
        self._armed = False
        self._future = None
        self._barrier_topic = ""
        self._barrier = 0
        self._cutover_stamps = {}
        for topic in self._outputs:
            self._samples.pop(topic, None)

    def _state_message(self, key: str, message) -> None:
        # Observe loss before replacement; a fresh delivery cannot hide a gap.
        self._check_health()
        previous = self._samples.get(key)
        if not self._cache(key, message):
            return
        if key == "activation":
            state = message.state
            if state == TeleopActivationState.INACTIVE:
                self._close("Released.")
                self._armed = True
            elif state == TeleopActivationState.PENDING:
                if self._phase != "idle":
                    self._close("Activation pending; release required.")
            elif state == TeleopActivationState.ACTIVE:
                was_active = previous is not None and (
                    previous.message.state == TeleopActivationState.ACTIVE
                )
                if not was_active and self._armed:
                    if self._prerequisites():
                        self._phase = "matching"
                        self._reason = "Waiting for source match."
                    else:
                        self._close("Activation prerequisites unavailable.")
            else:
                self._close("Activation unavailable.")
        elif key == "manager" and message.data != ControlMode.TAKEOVER:
            self._close("Manager is not in TAKEOVER.")
        elif key == "match":
            if previous is not None and (
                message.loss_id != previous.message.loss_id
            ):
                self._close("Source observed input loss; release required.")
            elif not message.ready:
                self._close("Source unavailable: " + message.reason)
        self._check_health()

    def _prerequisites(self) -> bool:
        return all(
            self._fresh(key) for key in ("activation", "manager", "match")
        ) and (
            self._samples["activation"].message.state
            == TeleopActivationState.ACTIVE
            and self._samples["manager"].message.data == ControlMode.TAKEOVER
            and self._samples["match"].message.ready
        )

    def _check_health(self) -> None:
        if self._phase == "idle":
            if self._armed and any(
                not self._fresh(key)
                for key in ("activation", "manager", "match")
                if key in self._samples
            ):
                self._close("Engagement heartbeat lost; release required.")
            return
        if not self._prerequisites():
            self._close("Engagement prerequisites lost.")
        elif self._phase in ("recapturing", "verifying") and (
            not self._samples["match"].message.matched
        ):
            self._close("Initial match lost during baseline cutover.")
        elif self._phase == "active" and not all(
            self._fresh(topic) for topic in self._outputs
        ):
            self._close("Candidate stream lost.")
        elif self._phase in ("recapturing", "verifying") and (
            monotonic() >= self._deadline
        ):
            self._close("Baseline cutover timed out.")

    def _candidate(self, topic: str, message: JointState) -> None:
        self._check_health()
        # Queued old-baseline deliveries are discarded, not session faults.
        if (
            topic == self._barrier_topic
            and self._stamp(message) <= self._barrier
        ):
            return
        if (
            not message.name
            or len(message.name) != len(set(message.name))
            or any(not name for name in message.name)
            or len(message.position) != len(message.name)
            or any(
                len(values) not in (0, len(message.name))
                for values in (message.velocity, message.effort)
            )
            or not all(
                isfinite(value)
                for value in (
                    list(message.position)
                    + list(message.velocity)
                    + list(message.effort)
                )
            )
        ):
            self._close("Invalid JointState candidate.")
            return
        self._cache(topic, message)

    def _request_recapture(self) -> None:
        if not self._client.service_is_ready():
            self._close("Recapture service unavailable.")
            return
        for topic in self._outputs:
            self._samples.pop(topic, None)
        self._cutover_stamps = {
            topic: self._last_stamps.get(topic, 0) for topic in self._outputs
        }
        self._phase = "recapturing"
        self._reason = "Recapturing baseline."
        self._deadline = monotonic() + self.service_timeout_s
        try:
            future = self._client.call_async(RecaptureBaseline.Request())
        except Exception as error:
            self.get_logger().error(f"Baseline request failed: {error}")
            self._close("Baseline request failed.")
            return
        self._future = future
        # Poll completion on the default callback group: response validation,
        # input callbacks and all-route forwarding remain serialized.

    def _recapture_result(self, future) -> None:
        self._check_health()
        if future is not self._future or self._phase != "recapturing":
            return
        self._future = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f"Baseline response failed: {error}")
            self._close("Baseline response failed.")
            return
        stamp = response.stamp_barrier
        barrier = stamp.sec * 1_000_000_000 + stamp.nanosec
        if (
            not response.success
            or response.candidate_topic not in self._outputs
            or not self._stamp_valid(barrier)
            or barrier < self._cutover_stamps.get(response.candidate_topic, 0)
        ):
            self._close("Invalid baseline response.")
            return
        self._barrier_topic = response.candidate_topic
        self._barrier = barrier
        sample = self._samples.get(self._barrier_topic)
        if sample is not None and sample.stamp_ns <= barrier:
            self._samples.pop(self._barrier_topic)
        self._phase = "verifying"
        self._reason = "Waiting for post-baseline candidates."

    def _tick(self) -> None:
        self._check_health()
        if self._future is not None and self._future.done():
            self._recapture_result(self._future)
        if (
            self._phase == "matching"
            and self._samples["match"].message.matched
        ):
            self._request_recapture()
        if self._phase == "verifying":
            match = self._samples["match"]
            topic = match.message.candidate_topic
            if (
                topic in self._outputs
                and all(self._fresh(route) for route in self._outputs)
                and self._samples[topic].stamp_ns == match.stamp_ns
            ):
                self._phase = "active"
                self._reason = "Engaged."
        if self._phase == "active":
            for topic, publisher in self._outputs.items():
                publisher.publish(self._samples[topic].message)
        diagnostic = DiagnosticStatus(
            name=self.get_fully_qualified_name(),
            level=DiagnosticStatus.OK,
            message=self._reason,
            values=[KeyValue(key="state", value=self._phase)],
        )
        self._status_publisher.publish(
            DiagnosticArray(
                header=Header(stamp=self.get_clock().now().to_msg()),
                status=[diagnostic],
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EngagementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
