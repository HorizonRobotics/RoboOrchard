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

from __future__ import annotations
# ruff: noqa: E402
import math
import sys
import types
from dataclasses import dataclass, field
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))


class _ParameterValue:
    def __init__(self, value):
        self.value = value
        self.string_value = str(value) if value is not None else ""
        self.bool_value = bool(value)
        try:
            self.double_value = float(value)
        except (TypeError, ValueError):
            self.double_value = 0.0


class _Parameter:
    def __init__(self, name, value):
        self.name = name
        self.value = value

    def get_parameter_value(self):
        return _ParameterValue(self.value)


class _Logger:
    def __init__(self):
        self.messages = []

    def debug(self, msg, *args, **kwargs):
        self.messages.append(("debug", msg))

    def info(self, msg, *args, **kwargs):
        self.messages.append(("info", msg))

    def warn(self, msg, *args, **kwargs):
        self.messages.append(("warn", msg))

    warning = warn

    def error(self, msg, *args, **kwargs):
        self.messages.append(("error", msg))


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Node:
    def __init__(self, node_name, **kwargs):
        self.node_name = node_name
        self._parameter_overrides = {
            item.name: item.value
            for item in kwargs.get("parameter_overrides", []) or []
        }
        self._parameters = {}
        self._logger = _Logger()
        self.subscriptions = []
        self.publishers = []
        self.timers = []

    def declare_parameter(self, name, default_value, descriptor=None):
        value = self._parameter_overrides.get(name, default_value)
        self._parameters[name] = value
        return _Parameter(name, value)

    def get_parameter(self, name):
        return _Parameter(name, self._parameters[name])

    def create_subscription(self, *args, **kwargs):
        self.subscriptions.append((args, kwargs))
        return object()

    def create_publisher(self, *args, **kwargs):
        publisher = _Publisher()
        self.publishers.append(publisher)
        return publisher

    def create_timer(self, period_s, callback):
        timer = types.SimpleNamespace(
            period_s=period_s, callback=callback, cancelled=False
        )
        timer.cancel = lambda: setattr(timer, "cancelled", True)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(
                to_msg=lambda: types.SimpleNamespace(sec=123, nanosec=456)
            )
        )

    def destroy_node(self):
        pass


@dataclass
class _Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class _Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class _Pose:
    position: _Point = field(default_factory=_Point)
    orientation: _Quaternion = field(default_factory=_Quaternion)


@dataclass
class _PoseStamped:
    header: object | None = None
    pose: _Pose = field(default_factory=_Pose)


geometry_msgs = sys.modules.get(
    "geometry_msgs", types.ModuleType("geometry_msgs")
)
geometry_msgs_msg = sys.modules.get(
    "geometry_msgs.msg", types.ModuleType("geometry_msgs.msg")
)
geometry_msgs_msg.Point = _Point
geometry_msgs_msg.Quaternion = _Quaternion
geometry_msgs_msg.Pose = _Pose
geometry_msgs_msg.PoseStamped = _PoseStamped
geometry_msgs.msg = geometry_msgs_msg
sys.modules["geometry_msgs"] = geometry_msgs
sys.modules["geometry_msgs.msg"] = geometry_msgs_msg


if "sensor_msgs" not in sys.modules:

    @dataclass
    class _JointState:
        header: object | None = None
        name: list[str] = field(default_factory=list)
        position: list[float] = field(default_factory=list)
        velocity: list[float] = field(default_factory=list)
        effort: list[float] = field(default_factory=list)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = _JointState
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg


if "std_msgs" not in sys.modules:

    @dataclass
    class _Header:
        frame_id: str = ""
        stamp: object | None = None

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Header = _Header
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg


if "std_srvs" not in sys.modules:

    @dataclass
    class _TriggerRequest:
        pass

    @dataclass
    class _TriggerResponse:
        success: bool = False
        message: str = ""

    class _Trigger:
        Request = _TriggerRequest
        Response = _TriggerResponse

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = _Trigger
    std_srvs.srv = std_srvs_srv
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs_srv


if "robo_orchard_pico_msg_ros2" not in sys.modules:

    @dataclass
    class _Controller:
        status: int = 1
        trigger: float = 0.0
        gripper: float = 0.0
        pose: object = field(
            default_factory=lambda: (
                sys.modules["geometry_msgs.msg"].PoseStamped().pose
            )
        )

    @dataclass
    class _VRState:
        left_controller: _Controller = field(default_factory=_Controller)
        right_controller: _Controller = field(default_factory=_Controller)
        header: object = field(
            default_factory=lambda: sys.modules["std_msgs.msg"].Header()
        )

    pico_msgs = types.ModuleType("robo_orchard_pico_msg_ros2")
    pico_msgs_msg = types.ModuleType("robo_orchard_pico_msg_ros2.msg")
    pico_msgs_msg.VRState = _VRState
    pico_msgs.msg = pico_msgs_msg
    sys.modules["robo_orchard_pico_msg_ros2"] = pico_msgs
    sys.modules["robo_orchard_pico_msg_ros2.msg"] = pico_msgs_msg


rclpy = types.ModuleType("rclpy")
rclpy.init = lambda *args, **kwargs: None
rclpy.shutdown = lambda *args, **kwargs: None
rclpy.spin = lambda *args, **kwargs: None
rclpy_node = types.ModuleType("rclpy.node")
rclpy_node.Node = _Node
rclpy_node.ParameterDescriptor = lambda description="": types.SimpleNamespace(
    description=description
)
rclpy.node = rclpy_node
sys.modules["rclpy"] = rclpy
sys.modules["rclpy.node"] = rclpy_node


intent_module = sys.modules.get(
    "robo_orchard_teleop_ros2.bridge.pico.intent",
    types.ModuleType("robo_orchard_teleop_ros2.bridge.pico.intent"),
)
intent_module.GripperIntent = lambda *args, **kwargs: object()
intent_module.ResetIntent = lambda *args, **kwargs: object()
sys.modules["robo_orchard_teleop_ros2.bridge.pico.intent"] = intent_module


teleop_module = types.ModuleType("robo_orchard_teleop_ros2.bridge.pico.teleop")


class _Action:
    RESET = "RESET"
    ACTIVE = "ACTIVE"
    DEACTIVE = "DEACTIVE"
    INVALID_VR_MSG = "INVALID_VR_MSG"


@dataclass
class _TeleOpResult:
    target_ee_pose: object
    solution: list[float] | None


class _VRTeleOp:
    instances = []

    def __init__(self, *args, **kwargs):
        self.__class__.instances.append(self)
        self.init_kwargs = kwargs
        self.latest_vr_state = types.SimpleNamespace(
            left_controller=types.SimpleNamespace(trigger=0.0),
            right_controller=types.SimpleNamespace(trigger=0.0),
        )
        self.next_action = _Action.ACTIVE
        self.next_result = None

    def update_vr_state(self, msg):
        self.latest_vr_state = msg
        return self.next_action

    def update_robot_ee_pose(self, msg):
        self.current_ee_pose = msg

    def update_robot_joint_state(self, msg):
        self.current_joint_state = msg

    def __call__(self):
        return self.next_result


teleop_module.Action = _Action
teleop_module.TeleOpResult = _TeleOpResult
teleop_module.VRTeleOp = _VRTeleOp
sys.modules["robo_orchard_teleop_ros2.bridge.pico.teleop"] = teleop_module


from robo_orchard_teleop_ros2.bridge.pico.teleop import VRTeleOp
from robo_orchard_teleop_ros2.robot.piper.pico_vr import (
    JOINT_NAMES,
    PIPER_MAX_GRIPPER_OPENING_M,
    PiperPicoVRTeleOpNode,
    _trigger_to_gripper_position,
)


def test_trigger_to_gripper_position_is_inverted():
    assert math.isclose(
        _trigger_to_gripper_position(0.0), PIPER_MAX_GRIPPER_OPENING_M
    )
    assert math.isclose(_trigger_to_gripper_position(1.0), 0.0)
    assert math.isclose(_trigger_to_gripper_position(0.25), 0.075)


def test_pico_node_passes_pose_low_pass_alpha_to_vr_teleop():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()

    node = PiperPicoVRTeleOpNode()

    assert "pose_low_pass_alpha" not in node._parameters
    assert VRTeleOp.instances[0].init_kwargs["pose_low_pass_alpha"] == 0.25
    assert VRTeleOp.instances[1].init_kwargs["pose_low_pass_alpha"] == 0.25


def test_pico_node_passes_translation_scale_factor_to_vr_teleop():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()

    PiperPicoVRTeleOpNode()

    assert VRTeleOp.instances[0].init_kwargs["scale_factor"] == 1.2
    assert VRTeleOp.instances[1].init_kwargs["scale_factor"] == 1.2


def test_pico_node_passes_configured_ee_link_name_to_vr_teleop():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()
    parameter_overrides = [
        _Parameter("urdf_path", "/tmp/robot.urdf"),
        _Parameter("ee_link_name", "link6"),
    ]

    PiperPicoVRTeleOpNode(parameter_overrides=parameter_overrides)

    assert VRTeleOp.instances[0].init_kwargs["ee_link_name"] == "link6"
    assert VRTeleOp.instances[1].init_kwargs["ee_link_name"] == "link6"


def test_pico_node_uses_higher_control_rate():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()

    node = PiperPicoVRTeleOpNode()

    assert len(node.timers) == 1
    assert node.timers[0].period_s == 1.0 / 30.0


def test_pico_node_publishes_joint_command_in_teleop_timer():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()

    node = PiperPicoVRTeleOpNode()
    node._arm_state["left"] = pico_vr_module.ArmEngageState.ACTIVE
    left_teleop = VRTeleOp.instances[0]
    left_teleop.latest_vr_state.left_controller.trigger = 0.25
    left_teleop.next_result = _TeleOpResult(
        target_ee_pose=_Pose(),
        solution=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    )

    node.timer_callback()

    assert len(node.left_cmd_pub.messages) == 1
    msg = node.left_cmd_pub.messages[0]
    assert msg.position[:6] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert math.isclose(msg.position[6], 0.075)
    assert msg.name == JOINT_NAMES


def test_pico_node_does_not_publish_when_teleop_returns_none():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()

    node = PiperPicoVRTeleOpNode()
    left_teleop = VRTeleOp.instances[0]
    left_teleop.next_result = None

    node.timer_callback()

    assert node.left_cmd_pub.messages == []
