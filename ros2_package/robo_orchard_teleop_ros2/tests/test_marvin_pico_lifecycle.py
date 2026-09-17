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
import types
from concurrent.futures import Future

import pytest

_ROS_STUB_MODULE_PREFIXES = (
    "geometry_msgs",
    "rclpy",
    "robo_orchard_pico_msg_ros2",
    "robo_orchard_teleop_msg_ros2",
    "robo_orchard_teleop_ros2.bridge.pico",
    "robo_orchard_teleop_ros2.ik",
    "robo_orchard_teleop_ros2.msg_adaptor",
    "robo_orchard_teleop_ros2.robot.marvin.pico_vr",
    "robo_orchard_teleop_ros2.robot.piper.pico_vr",
    "sensor_msgs",
    "std_msgs",
    "std_srvs",
    "test_pico_vr_node",
    "test_pico_teleop",
    "test_topic_activation_intent",
)


def _is_ros_stub_module(name: str) -> bool:
    return any(
        name == prefix or name.startswith(f"{prefix}.")
        for prefix in _ROS_STUB_MODULE_PREFIXES
    )


# The shared node-test helper installs its ROS doubles at import time. Restore
# the module registry immediately so collecting this file cannot change the
# implementation imported by later test modules.
_saved_modules = {
    name: module
    for name, module in sys.modules.items()
    if _is_ros_stub_module(name)
}
try:
    import test_pico_teleop as teleop_stubs
    import test_pico_vr_node as ros_stubs
    from test_topic_activation_intent import _load_module

    from robo_orchard_teleop_ros2.robot.marvin.pico_vr import (
        MarvinPicoVRTeleOpNode,
    )
    from robo_orchard_teleop_ros2.robot.piper.pico_vr import ArmEngageState

    TopicActivationIntent = _load_module().TopicActivationIntent
finally:
    for _module_name in tuple(sys.modules):
        if _is_ros_stub_module(_module_name):
            sys.modules.pop(_module_name)
    sys.modules.update(_saved_modules)


class _Intent:
    def __init__(self, events):
        self.events = events
        self.armed = True

    def update(self, message):
        self.last_message = message
        self.events.append("activation_update")

    def status(self):
        return self.armed, "active" if self.armed else "release_required"

    def require_rearm(self):
        self.events.append("rearm")
        self.armed = False


class _Teleop:
    def __init__(self, side, events):
        self.side = side
        self.events = events

    def begin_reset(self):
        self.events.append(f"begin:{self.side}")

    def finish_reset(self):
        self.events.append(f"finish:{self.side}")

    def update_vr_state(self, message):
        self.events.append((self.side, message))
        return None


class _LateFuture:
    def __init__(self):
        self.result_called = False

    def result(self):
        self.result_called = True
        raise AssertionError("late reset response was accessed")


def test_keyboard_activation_and_reset_only_update_local_teleop_state():
    events = []
    node = object.__new__(MarvinPicoVRTeleOpNode)
    node._topic_activation_intent = _Intent(events)
    node._keyboard_sides = lambda: ("left", "right")
    node.teleops = {side: _Teleop(side, events) for side in ("left", "right")}
    node._request_reset = lambda: events.append("request_reset")

    node._on_keyboard_reset(ros_stubs._Empty())

    assert events == [
        "rearm",
        "request_reset",
    ]

    active = object()
    node._on_keyboard_activation(active)
    assert events[-1] == "activation_update"
    assert node._topic_activation_intent.last_message is active


def test_vr_activation_only_updates_local_teleop_state():
    events = []
    message = object()
    node = object.__new__(MarvinPicoVRTeleOpNode)
    node.teleops = {side: _Teleop(side, events) for side in ("left", "right")}

    node._vr_state_callback(message)

    assert events == [("left", message), ("right", message)]


def test_reset_response_after_destroy_is_ignored():
    events = []
    future = _LateFuture()
    node = object.__new__(MarvinPicoVRTeleOpNode)
    node._destroying = False
    node._reset_generation = 7
    node.reset_future = future

    node.destroy_node()
    node._reset_result(7, future)

    assert events == []
    assert node.reset_future is None
    assert future.result_called is False


@pytest.mark.parametrize("reset_side", ["left", "right"])
@pytest.mark.parametrize("response_success", [True, False])
@pytest.mark.parametrize(
    "response_order", ["before_status", "during", "after"]
)
def test_global_reset_discards_both_held_teleop_sessions(
    reset_side, response_success, response_order, monkeypatch
):
    node = object.__new__(MarvinPicoVRTeleOpNode)
    node._logger = ros_stubs._Logger()
    node._destroying = False
    node._reset_generation = 0
    node.reset_future = None
    node._manager_resetting = False
    node._topic_activation_intent = None
    future = Future()
    node.reset_client = types.SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: future,
    )
    node.current_joint_positions = {
        side: [0.0] * 7 for side in ("left", "right")
    }
    node.teleops = {
        side: teleop_stubs.VRTeleOp(
            source_type=side,
            urdf_path="robot.urdf",
            base_link_name="base_link",
            ee_link_name=f"{side}_ee",
            trigger_intent=teleop_stubs._LongPressIntent(side),
            reset_intent=teleop_stubs._ResetIntent(side),
        )
        for side in ("left", "right")
    }
    monkeypatch.setitem(
        node._vr_state_callback.__globals__, "Action", teleop_stubs.Action
    )
    message = teleop_stubs._VRState(
        **{
            f"{side}_controller": teleop_stubs._Controller(
                status=1, pose=teleop_stubs._make_pose(), gripper=1.0
            )
            for side in ("left", "right")
        }
    )
    for teleop in node.teleops.values():
        teleop.update_robot_ee_pose(teleop_stubs._make_pose(x=1.0))
    node._vr_state_callback(message)
    assert all(
        teleop.control_state.is_active for teleop in node.teleops.values()
    )

    reset_intent = node.teleops[reset_side].control_state.reset_intent
    reset_intent.should_reset = lambda message: True
    node._vr_state_callback(message)
    reset_intent.should_reset = lambda message: False
    assert node.reset_future is future
    assert all(
        teleop.control_state.reset_in_progress
        for teleop in node.teleops.values()
    )
    response = types.SimpleNamespace(success=response_success, message="done")
    if response_order == "before_status":
        future.set_result(response)
    node._on_control_status(types.SimpleNamespace(data="resetting"))
    if response_order == "during":
        future.set_result(response)
    assert all(
        teleop.control_state.reset_in_progress
        for teleop in node.teleops.values()
    )
    node._on_control_status(types.SimpleNamespace(data="stop"))
    if response_order == "after":
        assert all(
            teleop.control_state.reset_in_progress
            for teleop in node.teleops.values()
        )
        future.set_result(response)
    assert node.reset_future is None

    for teleop in node.teleops.values():
        state = teleop.control_state
        assert not state.is_active
        assert state.initial_vr_pose is None
        assert state.initial_ee_pose is None
        assert state.filtered_target_ee_pose is None
        assert state.rearm_required
        assert teleop.update_vr_state(message) == teleop_stubs.Action.DEACTIVE
        assert teleop() is None

    for side, teleop in node.teleops.items():
        controller = getattr(message, f"{side}_controller")
        controller.gripper = 0.0
        teleop.update_vr_state(message)
        new_pose = teleop_stubs._make_pose(x=2.0)
        teleop.update_robot_ee_pose(new_pose)
        controller.gripper = 1.0
        assert teleop.update_vr_state(message) == teleop_stubs.Action.ACTIVE
        assert teleop.control_state.initial_ee_pose is new_pose


def test_marvin_external_reset_consumes_status_subscription(monkeypatch):
    events = []
    node_globals = MarvinPicoVRTeleOpNode.__init__.__globals__

    def make_teleop(**kwargs):
        side = kwargs["source_type"]
        teleop = _Teleop(side, events)
        teleop.ik_solver = types.SimpleNamespace(
            get_joint_names=lambda: node_globals["MARVIN_JOINT_NAMES"][side]
        )
        return teleop

    monkeypatch.setitem(node_globals, "VRTeleOp", make_teleop)
    monkeypatch.setitem(
        node_globals,
        "os",
        types.SimpleNamespace(
            path=types.SimpleNamespace(isfile=lambda _: True)
        ),
    )
    node = MarvinPicoVRTeleOpNode()
    subscriptions = [
        args
        for args, _ in node.subscriptions
        if args[1] == "/robot/control/status"
    ]
    assert len(subscriptions) == 1
    assert subscriptions[0][3] == 10
    callback = subscriptions[0][2]
    callback(types.SimpleNamespace(data="resetting"))
    callback(types.SimpleNamespace(data="resetting"))
    assert events == ["begin:left", "begin:right"]
    callback(types.SimpleNamespace(data="stop"))
    callback(types.SimpleNamespace(data="stop"))
    assert events == [
        "begin:left",
        "begin:right",
        "finish:left",
        "finish:right",
    ]


@pytest.mark.parametrize("robot", ["piper", "marvin"])
@pytest.mark.parametrize("input_source", ["pico", "keyboard"])
def test_external_reset_requires_release_and_new_baselines(
    robot, input_source
):
    node_type = (
        ros_stubs.PiperPicoVRTeleOpNode
        if robot == "piper"
        else MarvinPicoVRTeleOpNode
    )
    node = object.__new__(node_type)
    node._manager_resetting = False
    node._reset_pending = False
    node.reset_future = None
    node.current_joint_positions = dict.fromkeys(("left", "right"))
    node._arm_state = dict.fromkeys(("left", "right"), ArmEngageState.ACTIVE)
    node._topic_activation_intent = (
        TopicActivationIntent(timeout_s=10.0)
        if input_source == "keyboard"
        else None
    )
    intent = node._topic_activation_intent
    if intent is not None:
        intent.update(types.SimpleNamespace(state=1))
        intent.update(types.SimpleNamespace(state=2))
    teleops = {
        side: teleop_stubs.VRTeleOp(
            source_type=side,
            urdf_path="robot.urdf",
            base_link_name="base_link",
            ee_link_name=f"{side}_ee",
            trigger_intent=intent or teleop_stubs._LongPressIntent(side),
            reset_intent=teleop_stubs._ResetIntent(side),
        )
        for side in ("left", "right")
    }
    node.teleops = teleops
    node.left_teleop = teleops["left"]
    node.right_teleop = teleops["right"]
    message = teleop_stubs._VRState(
        **{
            f"{side}_controller": teleop_stubs._Controller(
                status=1, pose=teleop_stubs._make_pose(), gripper=1.0
            )
            for side in teleops
        }
    )
    for teleop in teleops.values():
        teleop.update_robot_ee_pose(teleop_stubs._make_pose(x=1.0))
        assert teleop.update_vr_state(message) == teleop_stubs.Action.ACTIVE

    node._on_control_status(types.SimpleNamespace(data="stop"))
    assert all(teleop.control_state.is_active for teleop in teleops.values())
    node._on_control_status(types.SimpleNamespace(data="resetting"))
    for teleop in teleops.values():
        state = teleop.control_state
        assert state.reset_in_progress
        assert not state.is_active
        assert state.initial_ee_pose is None
        assert state.initial_vr_pose is None
        assert state.filtered_target_ee_pose is None
        assert teleop.update_vr_state(message) == teleop_stubs.Action.DEACTIVE
        assert teleop() is None
        teleop.update_robot_joint_state([1.0])
    node._on_control_status(types.SimpleNamespace(data="resetting"))
    assert all(
        teleop.control_state.current_joint_state == [1.0]
        for teleop in teleops.values()
    )
    node._on_control_status(types.SimpleNamespace(data="stop"))
    node._on_control_status(types.SimpleNamespace(data="takeover"))
    for teleop in teleops.values():
        assert not teleop.control_state.reset_in_progress
        assert teleop.update_vr_state(message) == teleop_stubs.Action.DEACTIVE
        assert teleop() is None
    if robot == "piper":
        assert all(not node._should_drive_side(side) for side in teleops)

    if intent is not None:
        intent.update(types.SimpleNamespace(state=1))
        node._on_control_status(types.SimpleNamespace(data="stop"))
    for side, teleop in teleops.items():
        getattr(message, f"{side}_controller").gripper = 0.0
        teleop.update_vr_state(message)
    if intent is not None:
        intent.update(types.SimpleNamespace(state=2))
    for side, teleop in teleops.items():
        new_pose = teleop_stubs._make_pose(x=2.0)
        teleop.update_robot_ee_pose(new_pose)
        getattr(message, f"{side}_controller").gripper = 1.0
        assert teleop.update_vr_state(message) == teleop_stubs.Action.ACTIVE
        assert teleop.control_state.initial_ee_pose is new_pose
