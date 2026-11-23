# Project RoboOrchard
#
# Copyright (c) 2026 Horizon Robotics. All Rights Reserved.
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

import importlib
import math
import sys
from types import ModuleType, SimpleNamespace

import pytest


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


class Twist:
    def __init__(self):
        self.linear = vector()
        self.angular = vector()


def pose(stamp=10, frame="base_link", x=0.2):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp, nanosec=0),
            frame_id=frame,
        ),
        pose=SimpleNamespace(
            position=vector(x=x),
            orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
        ),
    )


def target(stamp=10, frame="base_link", x=0.2, sample_offset=5_000_000):
    element = pose(stamp=stamp, frame=frame, x=x)
    element.header.stamp.sec, element.header.stamp.nanosec = divmod(
        stamp * 1_000_000_000 + sample_offset, 1_000_000_000
    )
    return SimpleNamespace(
        header=pose(stamp=stamp, frame=frame).header,
        poses=[element],
    )


@pytest.fixture
def ros(monkeypatch):
    """Exercise the ROS adapter without requiring ROS or touching hardware."""

    class FakeNode:
        overrides = {}

        def __init__(self, name):
            self.parameters = {}
            self.subscriptions = []
            self.messages = []
            self.destroyed = False
            self.logger = SimpleNamespace(warning=lambda text: None)

        def declare_parameter(self, name, default):
            self.parameters[name] = self.overrides.get(name, default)

        def get_parameter(self, name):
            return SimpleNamespace(value=self.parameters[name])

        def create_publisher(self, kind, topic, qos):
            self.output_topic = topic
            return SimpleNamespace(publish=self.messages.append)

        def create_subscription(self, kind, topic, callback, qos):
            self.subscriptions.append((topic, callback))

        def create_timer(self, period, callback):
            self.timer = callback

        def get_logger(self):
            return self.logger

        def destroy_node(self):
            self.destroyed = True

    modules = {
        "rclpy": {
            "init": lambda **kw: None,
            "ok": lambda: True,
            "shutdown": lambda: None,
            "spin": lambda node: None,
        },
        "rclpy.node": {"Node": FakeNode},
        "rclpy.qos": {"qos_profile_sensor_data": object()},
        "rclpy.signals": {"SignalHandlerOptions": SimpleNamespace(NO=0)},
        "geometry_msgs.msg": {"Twist": Twist},
        "nav_msgs.msg": {"Odometry": object, "Path": object},
        "robo_orchard_deploy_msg_ros2.msg": {
            "InferenceStatus": SimpleNamespace(ENABLED="enabled"),
        },
        "robo_orchard_teleop_msg_ros2.msg": {
            "ControlMode": SimpleNamespace(AUTO="auto"),
        },
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        vars(module).update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    name = "robo_orchard_hexfellow_ros2.node"
    monkeypatch.delitem(sys.modules, name, raising=False)
    module = importlib.import_module(name)
    monkeypatch.setattr(module.time, "monotonic", lambda: 40.0)
    yield module, FakeNode
    sys.modules.pop(name, None)


def make_ready(module):
    node = module.HexFellowTrackerNode()
    node._mode(SimpleNamespace(data="auto"))
    node._status(SimpleNamespace(data="enabled"))
    observation = pose()
    node._odom(
        SimpleNamespace(
            header=SimpleNamespace(
                frame_id="odom", stamp=observation.header.stamp
            ),
            child_frame_id="base_link",
            pose=SimpleNamespace(pose=pose(x=0).pose),
        )
    )
    return node


def test_node_defaults_and_auto_output_only_planar(ros):
    module, _ = ros
    node = make_ready(module)
    assert node.output_topic == "/chassis/cmd_vel"
    assert node.parameters["control_rate"] == 100.0
    assert "target_rate" not in node.parameters
    assert "/robot/inference_service/status" in dict(node.subscriptions)
    node._target(target())
    node._tick()
    command = node.messages[-1]
    assert 0 < command.linear.x <= 0.3
    assert command.linear.y == command.linear.z == 0
    assert command.angular.x == command.angular.y == 0


def test_pose_sample_time_reaches_tracker_and_callbacks(ros):
    module, _ = ros
    node = make_ready(module)
    node._target(target(x=0))
    node._target(target(x=0.001, sample_offset=15_000_000))
    assert node.tracker.reference_velocity == pytest.approx((0.1, 0))
    node._tick()
    assert node.messages[-1].linear.x == pytest.approx(0.102)
    node._tick()
    assert node.messages[-1].linear.x == pytest.approx(0.002)


def test_finite_pose_with_overflowing_feedback_cancels_and_publishes_zero(ros):
    module, _ = ros
    node = make_ready(module)
    node._target(target(x=1e308))
    assert node.tracker.goal is not None

    node._tick()
    command = node.messages[-1]
    assert (command.linear.x, command.angular.z) == (0.0, 0.0)
    assert node.tracker.goal is None


@pytest.mark.parametrize(
    "bad",
    ["frame", "pose_frame", "nan", "quaternion", "empty", "multiple", "time"],
)
def test_invalid_target_cancels_and_stops(ros, bad):
    module, _ = ros
    node = make_ready(module)
    node._target(target())
    message = target()
    if bad == "frame":
        message.header.frame_id = "other"
    elif bad == "pose_frame":
        message.poses[0].header.frame_id = "other"
    elif bad == "nan":
        message.poses[0].pose.position.z = math.nan
    elif bad == "quaternion":
        message.poses[0].pose.orientation.w = 0
    elif bad == "empty":
        message.poses = []
    elif bad == "multiple":
        message.poses *= 2
    else:
        message.poses[0].header.stamp = message.header.stamp
    node._target(message)
    assert node.tracker.goal is None
    assert node.messages[-1].linear.x == 0


def test_invalid_odometry_clears_history_and_stop(ros):
    module, _ = ros
    node = make_ready(module)
    node._target(target())
    node._odom(
        SimpleNamespace(
            header=target(frame="bad").header,
            child_frame_id="base_link",
        )
    )
    assert node.tracker.goal is None
    assert not node.tracker.history.samples
    assert node.messages[-1].linear.x == 0


def test_teleop_passthrough_without_inference_and_no_timer_overwrite(ros):
    module, _ = ros
    node = module.HexFellowTrackerNode()
    node._mode(SimpleNamespace(data="takeover"))
    command = Twist()
    command.linear = vector(0.1, 0.2, 0.3)
    command.angular = vector(0.4, 0.5, 0.6)
    node._teleop(command)
    count = len(node.messages)
    node._tick()
    node._status(SimpleNamespace(data="disabled"))
    node._target(target(frame="bad"))
    assert len(node.messages) == count
    assert node.messages[-1] is command
    module.time.monotonic = lambda: 40.6
    node._tick()
    assert node.messages[-1].linear.x == 0
    assert node.messages[-1].angular.z == 0


def test_teleop_ignored_outside_takeover_and_rejects_nonfinite(ros):
    module, _ = ros
    node = make_ready(module)
    command = Twist()
    command.linear.x = 0.2
    count = len(node.messages)
    node._teleop(command)
    assert len(node.messages) == count
    node._mode(SimpleNamespace(data="takeover"))
    command.angular.y = math.nan
    node._teleop(command)
    assert node.messages[-1].linear.x == 0


@pytest.mark.parametrize(
    "parameters",
    [
        {"control_rate": 0},
        {"body_frame": ""},
        {"output_topic": "/robot/chassis/teleop_cmd_vel"},
    ],
)
def test_bad_node_parameters(ros, parameters):
    module, fake_node = ros
    fake_node.overrides = parameters
    with pytest.raises(ValueError):
        module.HexFellowTrackerNode()


def test_main_stops_on_executor_error(ros, monkeypatch):
    module, _ = ros
    node = make_ready(module)
    node._target(target())
    monkeypatch.setattr(module, "HexFellowTrackerNode", lambda: node)

    def error(_node):
        raise RuntimeError("executor failure")

    monkeypatch.setattr(module.rclpy, "spin", error)
    with pytest.raises(RuntimeError, match="executor failure"):
        module.main()
    assert node.destroyed
    assert node.tracker.goal is None
    assert node.messages[-1].linear.x == 0
