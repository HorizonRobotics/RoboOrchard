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

import json
import sys
import types
from pathlib import Path

import pytest


def _install_stub_modules():
    class FakeTime:
        def __init__(self, nanoseconds=0):
            self.nanoseconds = nanoseconds

        def __sub__(self, other):
            return FakeTime(self.nanoseconds - other.nanoseconds)

        def to_msg(self):
            return self

    class FakeClock:
        def __init__(self):
            self.ns = 0

        def now(self):
            return FakeTime(self.ns)

    class FakePublisher:
        def __init__(self, topic):
            self.topic = topic
            self.messages = []

        def publish(self, msg):
            self.messages.append(msg)

        def get_subscription_count(self):
            return 1

    class FakeNode:
        parameter_overrides = {}

        def __init__(self, *args, **kwargs):
            self._declared_parameters = {}
            self._publishers = []
            self.clock = FakeClock()
            self._logs = {"info": [], "warning": [], "error": []}

        def declare_parameter(self, name, value):
            self._declared_parameters[name] = FakeNode.parameter_overrides.get(
                name, value
            )

        def get_parameter(self, name):
            return types.SimpleNamespace(value=self._declared_parameters[name])

        def create_publisher(self, msg_type, topic, qos):
            publisher = FakePublisher(topic)
            self._publishers.append(publisher)
            return publisher

        def create_subscription(self, msg_type, topic, callback, qos):
            return types.SimpleNamespace(topic=topic, callback=callback)

        def create_timer(self, period, callback):
            return types.SimpleNamespace(period=period, callback=callback)

        def get_clock(self):
            return self.clock

        def get_logger(self):
            logs = self._logs
            return types.SimpleNamespace(
                info=lambda m: logs["info"].append(m),
                warning=lambda m: logs["warning"].append(m),
                error=lambda m: logs["error"].append(m),
            )

        def destroy_node(self):
            pass

    class FakeFuture:
        def __init__(self):
            self._done = False

        def done(self):
            return self._done

        def set_result(self, value):
            self._done = True

    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.init = lambda *args, **kwargs: None
    fake_rclpy.shutdown = lambda *args, **kwargs: None
    fake_rclpy.ok = lambda: True
    fake_rclpy.spin_until_future_complete = lambda *args, **kwargs: None

    fake_node_module = types.ModuleType("rclpy.node")
    fake_node_module.Node = FakeNode
    fake_executors = types.ModuleType("rclpy.executors")

    class ExternalShutdownException(Exception):  # noqa: N818
        pass

    fake_executors.ExternalShutdownException = ExternalShutdownException
    fake_task = types.ModuleType("rclpy.task")
    fake_task.Future = FakeFuture

    fake_sensor_msgs = types.ModuleType("sensor_msgs")
    fake_sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    class JointState:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None)
            self.name = []
            self.position = []
            self.velocity = []
            self.effort = []

    fake_sensor_msgs_msg.JointState = JointState

    fake_rclpy.node = fake_node_module
    fake_rclpy.executors = fake_executors
    fake_rclpy.task = fake_task
    sys.modules["rclpy"] = fake_rclpy
    sys.modules["rclpy.node"] = fake_node_module
    sys.modules["rclpy.executors"] = fake_executors
    sys.modules["rclpy.task"] = fake_task
    sys.modules["sensor_msgs"] = fake_sensor_msgs
    sys.modules["sensor_msgs.msg"] = fake_sensor_msgs_msg
    return FakeNode, JointState


FakeNode, JointState = _install_stub_modules()
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robo_orchard_teleop_ros2.scripted.joint_master import (  # noqa: E402
    ScriptedJointMasterNode,
)


def _write_plan(path: Path, side: str, seconds=2.0, rate=100.0):
    """A J1 half-sine sweep 0 -> 0.5 -> 0 rad, everything else still."""
    import math

    n = int(seconds * rate) + 1
    ts = [i / rate for i in range(n)]
    qs = [
        [0.5 * math.sin(math.pi * t / seconds), 0.1, -0.2, 0.0, 0.0, 0.0]
        for t in ts
    ]
    qds = [
        [
            0.5 * math.pi / seconds * math.cos(math.pi * t / seconds),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        for t in ts
    ]
    payload = {"side": side, "rate": rate, "t": ts, "q": qs, "qd": qds}
    path.write_text(json.dumps(payload))
    return payload


def _make_node(tmp_path, **overrides):
    left = tmp_path / "circle_traj_left.json"
    right = tmp_path / "circle_traj_right.json"
    if not left.exists():
        _write_plan(left, "left")
    if not right.exists():
        _write_plan(right, "right")
    FakeNode.parameter_overrides = {
        "mode": "circle",
        "circle_trajectory_left": str(left),
        "circle_trajectory_right": str(right),
        "start_delay_s": 0.0,
        "settle_s": 0.4,
        "speed_scale": 1.0,
        "duration_s": 0.0,
        **overrides,
    }
    try:
        return ScriptedJointMasterNode()
    finally:
        FakeNode.parameter_overrides = {}


def _feed_state(node, positions):
    msg = JointState()
    msg.position = list(positions)
    node._state_callback(node.left, msg)
    node._state_callback(node.right, msg)


def _run_until(node, seconds, step=0.01):
    for _ in range(int(seconds / step)):
        node.clock.ns += int(step * 1e9)
        node._timer_callback()
        if node._done:
            break


def test_circle_ramp_starts_at_current_pose(tmp_path):
    node = _make_node(tmp_path)
    _feed_state(node, [0.3, 0.2, -0.1, 0.0, 0.0, 0.0, 0.05])
    _run_until(node, 0.05)
    first = node.left_pub.messages[0]
    assert first.position[0] == pytest.approx(0.3, abs=0.02)
    # gripper follows the current opening through ramp and playback
    assert first.position[6] == pytest.approx(0.05, abs=1e-6)
    # ramp publishes zero velocity
    assert all(v == 0.0 for v in first.velocity)


def test_circle_playback_tracks_plan_and_finishes(tmp_path):
    node = _make_node(tmp_path)
    _feed_state(node, [0.0] * 7)
    _run_until(node, 60.0)
    assert node._done
    total = node._circle_total_s
    # playback = ramp + settle + 2.0 s plan
    assert total == pytest.approx(node._circle_play_t0 + 2.0, abs=1e-6)
    # mid-playback sample matches the planned J1 half-sine peak
    peak = max(m.position[0] for m in node.left_pub.messages)
    assert peak == pytest.approx(0.5, abs=0.02)
    # playback carries the planned velocities
    peak_vel = max(m.velocity[0] for m in node.left_pub.messages)
    assert peak_vel == pytest.approx(0.5 * 3.14159 / 2.0, rel=0.1)
    # command ends on the plan's final sample
    last = node.left_pub.messages[-1]
    assert last.position[0] == pytest.approx(0.0, abs=0.02)


def test_circle_speed_scale_slows_and_clamps(tmp_path):
    node = _make_node(tmp_path, speed_scale=0.5)
    _feed_state(node, [0.0] * 7)
    _run_until(node, 60.0)
    assert node._circle_total_s == pytest.approx(
        node._circle_play_t0 + 4.0, abs=1e-6
    )
    # velocities are chain-rule scaled
    peak_vel = max(m.velocity[0] for m in node.left_pub.messages)
    assert peak_vel == pytest.approx(0.5 * 0.5 * 3.14159 / 2.0, rel=0.1)

    fast = _make_node(tmp_path, speed_scale=1.5)
    _feed_state(fast, [0.0] * 7)
    _run_until(fast, 1.0)
    assert fast._circle_speed == 1.0
    assert any("clamps speed_scale" in m for m in fast._logs["warning"])


def test_circle_missing_or_mismatched_plan_fails_fast(tmp_path):
    with pytest.raises(ValueError, match="failed to load"):
        _make_node(
            tmp_path,
            circle_trajectory_left=str(tmp_path / "nope.json"),
        )
    swapped = tmp_path / "swapped.json"
    _write_plan(swapped, "right")
    with pytest.raises(ValueError, match="planned for side"):
        _make_node(tmp_path, circle_trajectory_left=str(swapped))


def test_stress_scenario_supplies_reviewed_waypoints():
    FakeNode.parameter_overrides = {
        "scenario": "stress",
        "start_delay_s": 0.0,
    }
    try:
        node = ScriptedJointMasterNode()
    finally:
        FakeNode.parameter_overrides = {}
    assert node.mode == "waypoints"
    assert len(node.waypoints_left) == 7
    assert node.waypoints_left == node.waypoints_right
    assert node.pass_through_indices == {3}
    assert node.duration_s == 0.0
    assert node.use_current_state is True
