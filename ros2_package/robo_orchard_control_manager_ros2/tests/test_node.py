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

import yaml
from rclpy.node import Node

from robo_orchard_control_manager_ros2.node import (
    ControlManagerNode,
    resolve_message_types,
)


def _config_data(msg_type="sensor_msgs/msg/JointState"):
    return {
        "channels": [
            {
                "name": "left_arm",
                "kind": "joint_command",
                "msg_type": msg_type,
                "autonomous_topic": "/left_algo_cmd",
                "override_topic": "/left_override_cmd",
                "output_topic": "/robot/left/joint_cmd",
            }
        ]
    }


def test_node_loads_config_file_parameter_and_resolves_types(tmp_path):
    config_path = tmp_path / "control_manager.yaml"
    config_path.write_text(yaml.safe_dump(_config_data()), encoding="utf-8")
    Node.config_file = str(config_path)

    node = ControlManagerNode()

    assert node.config.channels[0].name == "left_arm"
    assert node.message_types["left_arm"].__name__ == "FakeJointState"
    assert node.logger.info_messages == ["Loaded 1 control command channels."]


def test_unresolvable_message_type_has_channel_context(tmp_path):
    config_path = tmp_path / "control_manager.yaml"
    config_path.write_text(
        yaml.safe_dump(_config_data("missing_msgs/msg/Command")),
        encoding="utf-8",
    )
    Node.config_file = str(config_path)

    try:
        ControlManagerNode()
    except TypeError as exc:
        message = str(exc)
    else:
        raise AssertionError("Expected message type resolution to fail.")

    assert "left_arm" in message
    assert "missing_msgs/msg/Command" in message


def test_command_subscriptions_preserve_node_ownership(tmp_path):
    config_path = tmp_path / "control_manager.yaml"
    config_path.write_text(yaml.safe_dump(_config_data()), encoding="utf-8")
    Node.config_file = str(config_path)
    node = ControlManagerNode()

    assert len(node._subscriptions) == 2
    assert len({id(subscription) for subscription in node._subscriptions}) == 2
    node.destroy_node()
    assert node._subscriptions == []
    assert node.subscriptions == {}


def test_resolve_message_types_returns_one_class_per_channel(tmp_path):
    config_path = tmp_path / "control_manager.yaml"
    config_path.write_text(yaml.safe_dump(_config_data()), encoding="utf-8")
    Node.config_file = str(config_path)
    node = ControlManagerNode()

    assert resolve_message_types(node.config)["left_arm"].__name__ == (
        "FakeJointState"
    )
