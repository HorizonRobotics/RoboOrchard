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

"""One-sided pose/velocity routing with independent joint replay."""

import copy

import pytest
import yaml
from conftest import FakePath, FakePoseStamped
from rclpy.node import Node

from robo_orchard_control_manager_ros2.node import (
    EVENTS_TOPIC,
    ControlManagerNode,
    ControlState,
)
from robo_orchard_teleop_msg_ros2.msg import TakeOverEvent


def _channel(name, msg_type, *, autonomous=True, override=True):
    channel = {
        "name": name,
        "msg_type": msg_type,
        "output_topic": f"/{name}/output",
    }
    if autonomous:
        channel["autonomous_topic"] = f"/{name}/auto"
    if override:
        channel["override_topic"] = f"/{name}/override"
    return channel


@pytest.fixture(
    params=[
        "geometry_msgs/msg/PoseStamped",
        "nav_msgs/msg/Path",
        "geometry_msgs/PoseStamped",
        "nav_msgs/Path",
    ]
)
def pose_type(request):
    return request.param


def _make_node(tmp_path, pose_type, joints=False):
    channels = [
        _channel("pose", pose_type, override=False),
        _channel("velocity", "geometry_msgs/msg/Twist", autonomous=False),
    ]
    if joints:
        channels += [
            _channel("joint", "sensor_msgs/msg/JointState"),
            _channel(
                "auto_joint", "sensor_msgs/msg/JointState", override=False
            ),
            _channel(
                "override_joint",
                "sensor_msgs/msg/JointState",
                autonomous=False,
            ),
        ]
    path = tmp_path / "manager.yaml"
    path.write_text(yaml.safe_dump({"channels": channels}), encoding="utf-8")
    Node.config_file = str(path)
    return ControlManagerNode()


def _pose(node):
    message = node.message_types["pose"]()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.header.frame_id = "reference_body"
    pose = FakePoseStamped() if isinstance(message, FakePath) else message
    pose.pose.position.x = 0.3
    pose.pose.orientation.w = 1.0
    if isinstance(message, FakePath):
        pose.header.frame_id = message.header.frame_id
        pose.header.stamp.sec = 124
        pose.header.stamp.nanosec = 789
        message.poses = [pose]
    return message


def _twist(node):
    message = node.message_types["velocity"]()
    message.linear.x = 0.4
    message.angular.z = -0.5
    return message


def test_constructor_subscribes_only_configured_sides_and_types(
    tmp_path, pose_type
):
    node = _make_node(tmp_path, pose_type)
    assert set(node.subscriptions) == {"/pose/auto", "/velocity/override"}
    assert len(node._command_subscriptions) == 2
    for topic, name in (
        ("/pose/auto", "pose"),
        ("/velocity/override", "velocity"),
    ):
        subscription = node.subscriptions[topic]
        assert subscription.msg_type is node.message_types[name]
        assert subscription.callback_group is node._command_callback_group
    assert node.publishers["/pose/output"].published == []
    assert len(node.publishers["/velocity/output"].published) == 1
    node.destroy_node()
    assert node._subscriptions == []


@pytest.mark.parametrize("state", list(ControlState))
def test_one_sided_state_routing_preserves_pose_header_and_twist(
    tmp_path, state, pose_type
):
    node = _make_node(tmp_path, pose_type)
    node._change_state(state)
    pose, twist = _pose(node), _twist(node)
    original_pose = copy.deepcopy(pose)
    node.publishers["/velocity/output"].published.clear()
    node.subscriptions["/pose/auto"].callback(pose)
    node.subscriptions["/velocity/override"].callback(twist)
    assert node.publishers["/pose/output"].published == (
        [pose] if state == ControlState.AUTO else []
    )
    assert node.publishers["/velocity/output"].published == (
        [twist] if state == ControlState.TAKEOVER else []
    )
    assert pose.header == original_pose.header
    if isinstance(pose, FakePath):
        assert len(pose.poses) == 1
        assert vars(pose.poses[0]) == vars(original_pose.poses[0])
    else:
        assert vars(pose) == vars(original_pose)
    assert not node._channels["pose"].history
    assert not node._channels["velocity"].history


def test_pose_never_replayed_or_replaced_by_a_fake_stop_target(
    tmp_path, pose_type
):
    node = _make_node(tmp_path, pose_type)
    node._change_state(ControlState.AUTO)
    pose = _pose(node)
    node.subscriptions["/pose/auto"].callback(pose)
    node.clock.nanoseconds = 10_000_000_000
    for state in (
        ControlState.TAKEOVER,
        ControlState.STOP,
        ControlState.RESETTING,
        ControlState.STOP,
        ControlState.AUTO,
    ):
        assert node._change_state(state) is False
        assert node.publishers["/pose/output"].published == [pose]
        zero = node.publishers["/velocity/output"].published[-1]
        assert zero.linear.x == zero.angular.z == 0
    assert TakeOverEvent.REPLAY_COMMAND_SENT not in [
        event.event_type for event in node.publishers[EVENTS_TOPIC].published
    ]


@pytest.mark.parametrize("missing_joint_history", [False, True])
def test_joint_replay_ignores_pose_velocity_and_override_only_routes(
    tmp_path, missing_joint_history, pose_type
):
    node = _make_node(tmp_path, pose_type, joints=True)
    node._change_state(ControlState.AUTO)
    node.clock.nanoseconds = 1_000_000_000
    old = {}
    for name in ("joint", "auto_joint"):
        if missing_joint_history and name == "auto_joint":
            continue
        message = node.message_types[name](data=name)
        node.subscriptions[f"/{name}/auto"].callback(message)
        old[name] = message
    node.subscriptions["/pose/auto"].callback(_pose(node))
    for publisher in node.publishers.values():
        publisher.published.clear()
    node.clock.nanoseconds = 3_500_000_000
    assert (
        node._change_state(ControlState.TAKEOVER) is not missing_joint_history
    )
    for name in ("joint", "auto_joint"):
        assert node.publishers[f"/{name}/output"].published == (
            [] if missing_joint_history else [old[name]]
        )
    assert node.publishers["/pose/output"].published == []
    assert node.publishers["/override_joint/output"].published == []
    assert not node._channels["override_joint"].history
    assert not node._channels["pose"].history
    assert not node._channels["velocity"].history
    zero = node.publishers["/velocity/output"].published[0]
    assert zero.linear.x == zero.angular.z == 0
