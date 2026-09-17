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

import threading
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from enum import Enum
from functools import partial

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node, ParameterDescriptor
from rosidl_runtime_py.utilities import get_message
from std_srvs.srv import Trigger

from robo_orchard_control_manager_ros2.config import (
    CommandChannel,
    ControlManagerConfig,
    load_control_manager_config,
)
from robo_orchard_teleop_msg_ros2.msg import ControlMode, TakeOverEvent

STATUS_TOPIC = "/robot/control/status"
EVENTS_TOPIC = "/robot/control/events"
_HISTORY_MARGIN_S = 2.0
AUTO_SERVICE = "/robot/control/auto"
TAKEOVER_SERVICE = "/robot/control/takeover"
STOP_SERVICE = "/robot/control/stop"
RESET_SERVICE = "/robot/control/reset"


class ControlState(Enum):
    """Global command routing states."""

    AUTO = ControlMode.AUTO
    TAKEOVER = ControlMode.TAKEOVER
    STOP = ControlMode.STOP
    RESETTING = ControlMode.RESETTING


@dataclass
class _CommandChannelRuntime:
    """ROS endpoints and replay history for one configured channel."""

    config: CommandChannel
    publisher: object
    history: deque[tuple[int, object]] = field(default_factory=deque)


@dataclass(frozen=True)
class _ServiceResult:
    """Outcome of one downstream Trigger call."""

    service_name: str
    success: bool
    detail: str


def resolve_message_types(
    config: ControlManagerConfig,
) -> dict[str, type]:
    """Resolve each configured ROS message type by channel name.

    Raises:
        TypeError: If a configured message class cannot be resolved.
    """
    resolved = {}
    for channel in config.channels:
        try:
            resolved[channel.name] = get_message(channel.msg_type)
        except Exception as exc:
            raise TypeError(
                f"Channel '{channel.name}' message type "
                f"'{channel.msg_type}' could not be resolved: {exc}"
            ) from exc
    return resolved


class ControlManagerNode(Node):
    """Route all configured command channels under one global state."""

    def __init__(self) -> None:
        super().__init__("control_manager_node")
        self.declare_parameter(
            "config_file",
            "",
            descriptor=ParameterDescriptor(
                description="Path to a Control Manager YAML or JSON config."
            ),
        )
        config_file = str(self.get_parameter("config_file").value)
        if not config_file:
            raise ValueError("The config_file parameter must not be empty.")

        self.config = load_control_manager_config(config_file)
        self.message_types = resolve_message_types(self.config)
        self._state_lock = threading.Lock()
        self._transition_lock = threading.Lock()
        self._state = ControlState.STOP
        self._command_callback_group = ReentrantCallbackGroup()
        self._transition_callback_group = ReentrantCallbackGroup()
        self._downstream_callback_group = ReentrantCallbackGroup()

        self._status_publisher = self.create_publisher(
            ControlMode, STATUS_TOPIC, 10
        )
        self._event_publisher = self.create_publisher(
            TakeOverEvent, EVENTS_TOPIC, 10
        )
        self._channels = {}
        self._command_subscriptions = []
        for channel_config in self.config.channels:
            runtime = _CommandChannelRuntime(
                config=channel_config,
                publisher=self.create_publisher(
                    self.message_types[channel_config.name],
                    channel_config.output_topic,
                    1,
                ),
            )
            self._channels[channel_config.name] = runtime
            self._command_subscriptions.extend(
                [
                    self.create_subscription(
                        self.message_types[channel_config.name],
                        channel_config.autonomous_topic,
                        partial(self._autonomous_callback, runtime),
                        1,
                        callback_group=self._command_callback_group,
                    ),
                    self.create_subscription(
                        self.message_types[channel_config.name],
                        channel_config.override_topic,
                        partial(self._override_callback, runtime),
                        1,
                        callback_group=self._command_callback_group,
                    ),
                ]
            )

        self._status_timer = self.create_timer(
            1.0 / self.config.status_publish_rate_hz,
            self._publish_status,
            callback_group=self._command_callback_group,
        )
        self._enable_clients = self._create_clients(
            self.config.enable_services
        )
        self._reset_clients = self._create_clients(self.config.reset_services)
        self._inference_disable_clients = self._create_clients(
            self.config.inference_disable_services
        )
        self._transition_services = [
            self.create_service(
                Trigger,
                AUTO_SERVICE,
                self._auto_callback,
                callback_group=self._transition_callback_group,
            ),
            self.create_service(
                Trigger,
                TAKEOVER_SERVICE,
                self._takeover_callback,
                callback_group=self._transition_callback_group,
            ),
            self.create_service(
                Trigger,
                STOP_SERVICE,
                self._stop_callback,
                callback_group=self._transition_callback_group,
            ),
            self.create_service(
                Trigger,
                RESET_SERVICE,
                self._reset_callback,
                callback_group=self._transition_callback_group,
            ),
        ]
        self._publish_status()
        self.get_logger().info(
            f"Loaded {len(self.config.channels)} control command channels."
        )

    def _create_clients(
        self, service_names: list[str]
    ) -> list[tuple[str, object]]:
        return [
            (
                service_name,
                self.create_client(
                    Trigger,
                    service_name,
                    callback_group=self._downstream_callback_group,
                ),
            )
            for service_name in service_names
        ]

    @property
    def state(self) -> ControlState:
        """Return the single authoritative routing state."""
        with self._state_lock:
            return self._state

    def _autonomous_callback(
        self, channel: _CommandChannelRuntime, message: object
    ) -> None:
        with self._state_lock:
            if self._state != ControlState.AUTO:
                return
            if self.config.replay_time_s > 0.0:
                now_ns = self.get_clock().now().nanoseconds
                channel.history.append((now_ns, message))
                self._prune_history_locked(channel, now_ns)
            channel.publisher.publish(message)

    def _override_callback(
        self, channel: _CommandChannelRuntime, message: object
    ) -> None:
        with self._state_lock:
            if self._state == ControlState.TAKEOVER:
                channel.publisher.publish(message)

    def _prune_history_locked(
        self, channel: _CommandChannelRuntime, now_ns: int
    ) -> None:
        retention_ns = int(
            (self.config.replay_time_s + _HISTORY_MARGIN_S) * 1e9
        )
        cutoff_ns = now_ns - retention_ns
        while channel.history and channel.history[0][0] < cutoff_ns:
            channel.history.popleft()

    def _clear_histories_locked(self) -> None:
        for channel in self._channels.values():
            channel.history.clear()

    def _replay_locked(self) -> bool:
        if self.config.replay_time_s <= 0.0:
            return False

        target_ns = self.get_clock().now().nanoseconds - int(
            self.config.replay_time_s * 1e9
        )
        replay_messages = []
        for channel in self._channels.values():
            message = next(
                (
                    message
                    for timestamp_ns, message in reversed(channel.history)
                    if timestamp_ns <= target_ns
                ),
                None,
            )
            if message is None:
                return False
            replay_messages.append((channel, message))

        for channel, message in replay_messages:
            channel.publisher.publish(message)
        self._publish_event_locked(
            TakeOverEvent.REPLAY_COMMAND_SENT,
            "Replayed one autonomous command for every channel.",
        )
        return True

    def _change_state(
        self, target: ControlState, *, emit_event: bool = True
    ) -> bool:
        """Apply routing state and its DAgger marker without orchestration.

        This is the internal boundary used by transition services. It
        deliberately performs no downstream service calls.

        Returns:
            Whether a multi-channel replay was published.
        """
        with self._state_lock:
            if target == self._state:
                return False

            previous = self._state
            replayed = False
            if (
                target == ControlState.TAKEOVER
                and previous == ControlState.AUTO
            ):
                replayed = self._replay_locked()

            if target in (
                ControlState.AUTO,
                ControlState.STOP,
                ControlState.RESETTING,
            ):
                self._clear_histories_locked()

            self._state = target
            self._publish_status_locked()
            event = {
                ControlState.AUTO: (
                    TakeOverEvent.RELEASE_TRIGGERED,
                    "Autonomous control requested.",
                ),
                ControlState.TAKEOVER: (
                    TakeOverEvent.TAKEOVER_TRIGGERED,
                    "Takeover requested.",
                ),
                ControlState.STOP: (
                    TakeOverEvent.STOP_TRIGGERED,
                    "Stop requested.",
                ),
            }.get(target)
            if (
                target == ControlState.STOP
                and previous == ControlState.RESETTING
            ):
                event = None
            if emit_event and event is not None:
                self._publish_event_locked(*event)
            return replayed

    def _try_transition(self, response: Trigger.Response) -> bool:
        if self._transition_lock.acquire(blocking=False):
            return True
        response.success = False
        response.message = "Another control transition is already in progress."
        return False

    def _auto_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._try_transition(response):
            return response
        try:
            current = self.state
            if current == ControlState.AUTO:
                response.success = True
                response.message = "Already in AUTO."
                return response
            if current not in (ControlState.STOP, ControlState.TAKEOVER):
                response.success = False
                response.message = f"Cannot enter AUTO from {current.value}."
                return response

            if current != ControlState.STOP:
                self._change_state(ControlState.STOP, emit_event=False)
            failures = self._call_clients_sequentially(
                self._enable_clients, stop_on_failure=True
            )
            if failures:
                response.success = False
                response.message = self._format_failures(
                    "Failed to prepare AUTO", failures
                )
                self.get_logger().error(response.message)
                return response

            with self._state_lock:
                self._clear_histories_locked()
            self._change_state(ControlState.AUTO)
            response.success = True
            response.message = "Entered AUTO."
            return response
        finally:
            self._transition_lock.release()

    def _takeover_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._try_transition(response):
            return response
        try:
            current = self.state
            if current == ControlState.TAKEOVER:
                response.success = True
                response.message = "Already in TAKEOVER."
                return response
            if current not in (ControlState.AUTO, ControlState.STOP):
                response.success = False
                response.message = (
                    f"Cannot enter TAKEOVER from {current.value}."
                )
                return response

            replayed = self._change_state(ControlState.TAKEOVER)
            response.success = True
            response.message = (
                "Entered TAKEOVER with replay."
                if replayed
                else "Entered TAKEOVER."
            )
            return response
        finally:
            self._transition_lock.release()

    def _stop_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._try_transition(response):
            return response
        try:
            current = self.state
            if current == ControlState.STOP:
                response.success = True
                response.message = "Already in STOP."
                return response
            if current not in (ControlState.AUTO, ControlState.TAKEOVER):
                response.success = False
                response.message = f"Cannot enter STOP from {current.value}."
                return response

            self._change_state(ControlState.STOP)
            response.success = True
            response.message = "Entered STOP."
            return response
        finally:
            self._transition_lock.release()

    def _reset_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if not self._try_transition(response):
            return response
        entered_resetting = False
        try:
            current = self.state
            if current not in (
                ControlState.AUTO,
                ControlState.TAKEOVER,
                ControlState.STOP,
            ):
                response.success = False
                response.message = f"Cannot reset from {current.value}."
                return response

            self._change_state(ControlState.RESETTING)
            entered_resetting = True
            disable_failures = self._disable_inference_for_reset()
            if disable_failures:
                response.success = False
                response.message = self._format_failures(
                    "Failed to disable inference; hardware reset skipped",
                    disable_failures,
                )
                self.get_logger().error(response.message)
                return response

            reset_failures = self._call_clients_concurrently(
                self._reset_clients
            )
            if reset_failures:
                response.success = False
                response.message = self._format_failures(
                    "Hardware reset failed", reset_failures
                )
                self.get_logger().error(response.message)
                return response

            response.success = True
            response.message = "Reset completed; control remains stopped."
            return response
        except Exception as exc:
            self.get_logger().error(f"Unexpected reset failure: {exc}")
            response.success = False
            response.message = f"Unexpected reset failure: {exc}"
            return response
        finally:
            try:
                if entered_resetting:
                    self._change_state(ControlState.STOP)
            finally:
                self._transition_lock.release()

    def _disable_inference_for_reset(self) -> list[_ServiceResult]:
        """Skip absent optional inference nodes; otherwise require disable."""
        candidates = set(self.config.inference_node_candidates)
        if candidates:
            node_names = {
                f"{namespace.rstrip('/')}/{name}"
                for name, namespace in self.get_node_names_and_namespaces()
            }
            if not candidates.intersection(node_names) and not any(
                client.service_is_ready()
                for _, client in self._inference_disable_clients
            ):
                self.get_logger().info(
                    "No configured inference node or disable service "
                    "currently discovered; skipping inference disable "
                    "for this reset."
                )
                return []
        return self._call_clients_sequentially(self._inference_disable_clients)

    def _call_clients_sequentially(
        self,
        clients: list[tuple[str, object]],
        *,
        stop_on_failure: bool = False,
    ) -> list[_ServiceResult]:
        failures = []
        for service_name, client in clients:
            result = self._call_trigger(service_name, client)
            if not result.success:
                failures.append(result)
                if stop_on_failure:
                    break
        return failures

    def _call_clients_concurrently(
        self, clients: list[tuple[str, object]]
    ) -> list[_ServiceResult]:
        if not clients:
            return []
        with ThreadPoolExecutor(max_workers=len(clients)) as executor:
            results = list(
                executor.map(lambda item: self._call_trigger(*item), clients)
            )
        return [result for result in results if not result.success]

    def _call_trigger(
        self, service_name: str, client: object
    ) -> _ServiceResult:
        try:
            available = client.wait_for_service(
                timeout_sec=self.config.service_wait_timeout_s
            )
        except Exception as exc:
            return _ServiceResult(service_name, False, str(exc))
        if not available:
            return _ServiceResult(
                service_name,
                False,
                "service was unavailable before the discovery timeout",
            )

        try:
            future = client.call_async(Trigger.Request())
        except Exception as exc:
            return _ServiceResult(service_name, False, str(exc))
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(self.config.service_response_timeout_s):
            # Forget the local request; this does not cancel hardware work.
            client.remove_pending_request(future)
            return _ServiceResult(
                service_name,
                False,
                "response timed out",
            )

        try:
            downstream_response = future.result()
        except Exception as exc:
            return _ServiceResult(service_name, False, str(exc))
        if downstream_response is None:
            return _ServiceResult(service_name, False, "returned no response")
        return _ServiceResult(
            service_name,
            downstream_response.success,
            downstream_response.message or "downstream service failed",
        )

    @staticmethod
    def _format_failures(prefix: str, failures: list[_ServiceResult]) -> str:
        details = "; ".join(
            f"{result.service_name}: {result.detail}" for result in failures
        )
        return f"{prefix}: {details}"

    def _publish_status(self) -> None:
        with self._state_lock:
            self._publish_status_locked()

    def _publish_status_locked(self) -> None:
        message = ControlMode()
        message.header.stamp = self.get_clock().now().to_msg()
        message.data = self._state.value
        self._status_publisher.publish(message)

    def _publish_event_locked(self, event_type: str, details: str) -> None:
        message = TakeOverEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.event_type = event_type
        message.details = details
        self._event_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    """Run the Control Manager node."""
    rclpy.init(args=args)
    node = ControlManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
