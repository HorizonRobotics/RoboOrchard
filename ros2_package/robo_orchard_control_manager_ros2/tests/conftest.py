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

import sys
import threading
import types


class FakeParameterDescriptor:
    def __init__(self, description=""):
        self.description = description


class FakeLogger:
    def __init__(self):
        self.info_messages = []
        self.error_messages = []

    def info(self, message, **kwargs):
        self.info_messages.append(message)

    def error(self, message, **kwargs):
        self.error_messages.append(message)


class FakeReentrantCallbackGroup:
    pass


class FakeMultiThreadedExecutor:
    def add_node(self, node):
        self.node = node

    def spin(self):
        pass

    def shutdown(self):
        pass


class FakeTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def to_msg(self):
        return self.nanoseconds


class FakeClock:
    def __init__(self):
        self.nanoseconds = 0

    def now(self):
        return FakeTime(self.nanoseconds)


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.published = []

    def publish(self, message):
        self.published.append(message)


class FakeSubscription:
    def __init__(self, topic, callback):
        self.topic = topic
        self.callback = callback


class FakeTimer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback


class FakeService:
    def __init__(self, name, callback, callback_group):
        self.name = name
        self.callback = callback
        self.callback_group = callback_group


class FakeTrigger:
    class Request:
        """Empty Trigger request."""

        pass

    class Response:
        """Trigger result returned by test services."""

        def __init__(self, success=False, message=""):
            self.success = success
            self.message = message


class FakeFuture:
    def __init__(self):
        self._event = threading.Event()
        self._result = None
        self._exception = None
        self._callbacks = []
        self._lock = threading.Lock()

    def add_done_callback(self, callback):
        with self._lock:
            if self._event.is_set():
                call_now = True
            else:
                self._callbacks.append(callback)
                call_now = False
        if call_now:
            callback(self)

    def set_result(self, result):
        self._finish(result=result)

    def set_exception(self, exception):
        self._finish(exception=exception)

    def _finish(self, result=None, exception=None):
        with self._lock:
            self._result = result
            self._exception = exception
            self._event.set()
            callbacks = self._callbacks
            self._callbacks = []
        for callback in callbacks:
            callback(self)

    def result(self):
        if self._exception is not None:
            raise self._exception
        return self._result


class FakeServiceBehavior:
    def __init__(
        self,
        *,
        available=True,
        success=True,
        message="",
        pending=False,
    ):
        self.available = available
        self.response = FakeTrigger.Response(success, message)
        self.pending = pending


class FakeClient:
    def __init__(self, node, name, callback_group):
        self.node = node
        self.name = name
        self.callback_group = callback_group
        self.wait_timeouts = []
        self.futures = []
        self.removed_requests = []
        self.called = threading.Event()

    def remove_pending_request(self, future):
        self.removed_requests.append(future)

    def wait_for_service(self, timeout_sec):
        self.wait_timeouts.append(timeout_sec)
        return self.node.service_behaviors[self.name].available

    def service_is_ready(self):
        return self.node.service_behaviors[self.name].available

    def call_async(self, request):
        self.node.service_call_order.append(self.name)
        future = FakeFuture()
        self.futures.append(future)
        behavior = self.node.service_behaviors[self.name]
        self.called.set()
        if not behavior.pending:
            future.set_result(behavior.response)
        return future


class FakeNode:
    config_file = ""
    service_behaviors = {}

    @classmethod
    def reset_service_behaviors(cls):
        cls.service_behaviors = {}

    @classmethod
    def configure_service(cls, name, **kwargs):
        cls.service_behaviors[name] = FakeServiceBehavior(**kwargs)

    def __init__(self, name):
        self.name = name
        self.logger = FakeLogger()
        self.clock = FakeClock()
        self.publishers = {}
        self.subscriptions = {}
        self._subscriptions = []
        self.timers = []
        self.clients = {}
        self.services = {}
        self.service_call_order = []
        self.graph_nodes = []

    def declare_parameter(self, name, default, descriptor=None):
        pass

    def get_parameter(self, name):
        return types.SimpleNamespace(value=self.config_file)

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return self.clock

    def get_node_names_and_namespaces(self):
        return self.graph_nodes

    def create_publisher(self, msg_type, topic, depth):
        publisher = FakePublisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(
        self, msg_type, topic, callback, depth, callback_group=None
    ):
        subscription = FakeSubscription(topic, callback)
        subscription.callback_group = callback_group
        self.subscriptions[topic] = subscription
        self._subscriptions.append(subscription)
        return subscription

    def destroy_node(self):
        while self._subscriptions:
            subscription = self._subscriptions.pop(0)
            del self.subscriptions[subscription.topic]

    def create_timer(self, period, callback, callback_group=None):
        timer = FakeTimer(period, callback)
        timer.callback_group = callback_group
        self.timers.append(timer)
        return timer

    def create_client(self, srv_type, name, callback_group=None):
        self.service_behaviors.setdefault(name, FakeServiceBehavior())
        client = FakeClient(self, name, callback_group)
        self.clients[name] = client
        return client

    def create_service(self, srv_type, name, callback, callback_group=None):
        service = FakeService(name, callback, callback_group)
        self.services[name] = service
        return service


class FakeJointState:
    def __init__(self, data=None):
        self.data = data


class FakeHeader:
    def __init__(self):
        self.stamp = None


class FakeControlMode:
    AUTO = "auto"
    TAKEOVER = "takeover"
    STOP = "stop"
    RESETTING = "resetting"

    def __init__(self):
        self.header = FakeHeader()
        self.data = ""


class FakeTakeOverEvent:
    TAKEOVER_TRIGGERED = "takeover_triggered"
    REPLAY_COMMAND_SENT = "replay_command_sent"
    RELEASE_TRIGGERED = "release_triggered"
    STOP_TRIGGERED = "stop_triggered"

    def __init__(self):
        self.header = FakeHeader()
        self.event_type = ""
        self.details = ""


def _install_stub_modules():
    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_node.ParameterDescriptor = FakeParameterDescriptor
    rclpy.node = rclpy_node
    rclpy_callback_groups = types.ModuleType("rclpy.callback_groups")
    rclpy_callback_groups.ReentrantCallbackGroup = FakeReentrantCallbackGroup
    rclpy.callback_groups = rclpy_callback_groups
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.MultiThreadedExecutor = FakeMultiThreadedExecutor
    rclpy.executors = rclpy_executors
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.callback_groups"] = rclpy_callback_groups
    sys.modules["rclpy.executors"] = rclpy_executors
    sys.modules["rclpy.node"] = rclpy_node

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = FakeTrigger
    std_srvs.srv = std_srvs_srv
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs_srv

    teleop_msgs = types.ModuleType("robo_orchard_teleop_msg_ros2")
    teleop_msgs_msg = types.ModuleType("robo_orchard_teleop_msg_ros2.msg")
    teleop_msgs_msg.ControlMode = FakeControlMode
    teleop_msgs_msg.TakeOverEvent = FakeTakeOverEvent
    teleop_msgs.msg = teleop_msgs_msg
    sys.modules["robo_orchard_teleop_msg_ros2"] = teleop_msgs
    sys.modules["robo_orchard_teleop_msg_ros2.msg"] = teleop_msgs_msg

    rosidl_runtime_py = types.ModuleType("rosidl_runtime_py")
    rosidl_utilities = types.ModuleType("rosidl_runtime_py.utilities")

    def get_message(msg_type):
        if msg_type == "sensor_msgs/msg/JointState":
            return FakeJointState
        raise ModuleNotFoundError(msg_type)

    rosidl_utilities.get_message = get_message
    rosidl_runtime_py.utilities = rosidl_utilities
    sys.modules["rosidl_runtime_py"] = rosidl_runtime_py
    sys.modules["rosidl_runtime_py.utilities"] = rosidl_utilities


_install_stub_modules()
