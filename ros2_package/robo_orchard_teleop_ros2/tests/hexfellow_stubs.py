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

import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, patch


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class JointState:
    def __init__(self, *, name=(), position=()):
        self.header = SimpleNamespace(stamp=None)
        self.name = list(name)
        self.position = list(position)


class Node:
    def __init__(self, name, *, parameter_overrides=()):
        self.parameters = {
            parameter.name: parameter.value
            for parameter in parameter_overrides
        }
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self.time_s = 0.0
        self.logger = Mock()

    def declare_parameter(self, name, value, descriptor=None):
        self.parameters.setdefault(name, value)

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def create_publisher(self, message_type, topic, qos):
        messages = []
        publisher = SimpleNamespace(
            topic=topic, messages=messages, publish=messages.append
        )
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(subscription)
        return subscription

    def create_timer(self, period, callback):
        timer = SimpleNamespace(period=period, callback=callback)
        self.timers.append(timer)
        return timer

    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                nanoseconds=int(self.time_s * 1e9),
                to_msg=lambda: SimpleNamespace(sec=int(self.time_s)),
            )
        )

    def get_logger(self):
        return self.logger


def load_node_module(component):
    """Load a Hexfellow adapter with isolated ROS stubs, without an SDK."""
    modules = {}
    for name, attributes in {
        "rclpy": {},
        "rclpy.node": {
            "Node": Node,
            "ParameterDescriptor": SimpleNamespace,
        },
        "geometry_msgs": {},
        "geometry_msgs.msg": {"Twist": Twist},
        "sensor_msgs": {},
        "sensor_msgs.msg": {"JointState": JointState},
        "robo_orchard_pico_msg_ros2": {},
        "robo_orchard_pico_msg_ros2.msg": {"VRState": SimpleNamespace},
    }.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        modules[name] = module
    path = (
        Path(__file__).resolve().parents[1]
        / "robo_orchard_teleop_ros2/robot/hexfellow"
        / component
        / "pico_vr.py"
    )
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location(component, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


def vr_state(**controller_values):
    controller = dict(status=1, axis_x=0.0, axis_y=0.0, axis_click=False)
    controller.update(controller_values)
    return SimpleNamespace(
        left_controller=SimpleNamespace(**controller),
        right_controller=SimpleNamespace(**controller),
    )
