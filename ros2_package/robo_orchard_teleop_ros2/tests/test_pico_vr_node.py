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

# ruff: noqa: E402

import math
import sys
import types
from dataclasses import dataclass, field
from pathlib import Path

import pytest

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

    @dataclass
    class _Empty:
        pass

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Empty = _Empty
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


if "robo_orchard_teleop_msg_ros2" not in sys.modules:

    @dataclass
    class _TeleopActivationState:
        UNAVAILABLE = 0
        INACTIVE = 1
        ACTIVE = 2

        header: object = field(
            default_factory=lambda: sys.modules["std_msgs.msg"].Header()
        )
        state: int = UNAVAILABLE
        transition_id: int = 0

    teleop_msgs = types.ModuleType("robo_orchard_teleop_msg_ros2")
    teleop_msgs_msg = types.ModuleType("robo_orchard_teleop_msg_ros2.msg")
    teleop_msgs_msg.TeleopActivationState = _TeleopActivationState
    teleop_msgs.msg = teleop_msgs_msg
    sys.modules["robo_orchard_teleop_msg_ros2"] = teleop_msgs
    sys.modules["robo_orchard_teleop_msg_ros2.msg"] = teleop_msgs_msg


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


class _Intent:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class _TopicActivationIntent(_Intent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.messages = []
        self.require_rearm_count = 0

    def update(self, message):
        self.messages.append(message)

    def require_rearm(self):
        self.require_rearm_count += 1


intent_module.DisabledResetIntent = _Intent
intent_module.GripperIntent = _Intent
intent_module.InactiveActivationIntent = _Intent
intent_module.PicoActivationIntent = _Intent
intent_module.ResetIntent = _Intent
intent_module.TopicActivationIntent = _TopicActivationIntent
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
        self.reset_session_count = 0
        self.recapture_count = 0

    def update_vr_state(self, msg):
        self.latest_vr_state = msg
        return self.next_action

    def update_robot_ee_pose(self, msg):
        self.current_ee_pose = msg

    def update_robot_joint_state(self, msg):
        self.current_joint_state = msg

    def finish_reset(self):
        self.finish_reset_count = getattr(self, "finish_reset_count", 0) + 1

    def recapture_baseline(self):
        self.recapture_count += 1
        return True

    def reset_session(self):
        self.reset_session_count += 1

    def __call__(self):
        return self.next_result


teleop_module.Action = _Action
teleop_module.TeleOpResult = _TeleOpResult
teleop_module.VRTeleOp = _VRTeleOp
sys.modules["robo_orchard_teleop_ros2.bridge.pico.teleop"] = teleop_module


from robo_orchard_teleop_ros2.bridge.pico.teleop import VRTeleOp
from robo_orchard_teleop_ros2.robot.piper.pico_vr import (
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


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize(
    "names",
    [
        [f"joint{index}" for index in range(1, 7)] + ["gripper"],
        [f"left_joint{index}" for index in range(1, 7)] + ["left_gripper"],
        ["shoulder", "upper", "elbow", "forearm", "wrist", "tool", "opening"],
        ["joint6", "joint5", "joint4", "joint3", "joint2", "joint1", "joint7"],
    ],
)
def test_pico_publishes_configured_names_in_hardware_ik_order(side, names):
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()

    node = PiperPicoVRTeleOpNode(
        parameter_overrides=[_Parameter(f"{side}_joint_names", names)]
    )
    node._arm_state[side] = pico_vr_module.ArmEngageState.ACTIVE
    feedback = _JointState(
        name=list(names), position=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.02]
    )
    teleop = getattr(node, f"{side}_teleop")
    updates = []
    teleop.update_robot_joint_state = updates.append
    getattr(node, f"sub_{side}_joint_state_callback")(feedback)
    assert updates == [feedback.position[:6]]
    getattr(teleop.latest_vr_state, f"{side}_controller").trigger = 0.25
    teleop.next_result = _TeleOpResult(
        target_ee_pose=_Pose(),
        solution=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    )

    node.timer_callback()

    publisher = getattr(node, f"{side}_cmd_pub")
    assert len(publisher.messages) == 1
    msg = publisher.messages[0]
    assert msg.position[:6] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert math.isclose(msg.position[6], 0.075)
    assert msg.name == names
    assert msg.name is not feedback.name
    assert node.joint_names[side] is not names


@pytest.mark.parametrize("names", [None, [], ["joint1"] * 7, [" "] * 7])
def test_pico_does_not_publish_commands_without_valid_feedback_names(names):
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    node = PiperPicoVRTeleOpNode()
    feedback = (
        None if names is None else _JointState(name=names, position=[0.0] * 7)
    )

    node._handle_teleop_result(
        side="left",
        ret=_TeleOpResult(target_ee_pose=_Pose(), solution=[0.1] * 6),
        gripper=0.02,
        header=object(),
        joint_state_cmd_publisher=node.left_cmd_pub,
        target_pose_publisher=node.left_target_pub,
        joint_state=feedback,
    )

    assert node.left_cmd_pub.messages == []
    assert node.get_logger().messages[-1][0] == "error"


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize(
    "configured_names",
    [
        [f"joint{index}" for index in range(1, 7)] + ["gripper"],
        ["shoulder", "upper", "elbow", "forearm", "wrist", "tool", "opening"],
        ["joint6", "joint5", "joint4", "joint3", "joint2", "joint1", "joint7"],
    ],
)
def test_reordered_feedback_never_seeds_ik_or_labels_its_solution(
    side, configured_names
):
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    node = PiperPicoVRTeleOpNode(
        parameter_overrides=[
            _Parameter(f"{side}_joint_names", configured_names)
        ]
    )
    getattr(node, f"sub_{side}_joint_state_callback")(
        _JointState(name=list(configured_names), position=[0.0] * 7)
    )
    assert getattr(node, f"{side}_joint_state_msg") is not None
    names = list(configured_names)
    names[0], names[1] = names[1], names[0]
    feedback = _JointState(
        name=names, position=[0.2, 0.1, 0.3, 0.4, 0.5, 0.6, 0.02]
    )
    teleop = getattr(node, f"{side}_teleop")
    updates = []
    teleop.update_robot_joint_state = updates.append

    getattr(node, f"sub_{side}_joint_state_callback")(feedback)

    assert updates == []
    assert getattr(node, f"{side}_joint_state_msg") is None
    node._handle_teleop_result(
        side=side,
        ret=_TeleOpResult(target_ee_pose=_Pose(), solution=[0.1] * 6),
        gripper=0.02,
        header=object(),
        joint_state_cmd_publisher=getattr(node, f"{side}_cmd_pub"),
        target_pose_publisher=getattr(node, f"{side}_target_pub"),
        joint_state=feedback,
    )
    assert getattr(node, f"{side}_cmd_pub").messages == []


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize(
    "names",
    [
        None,
        "abcdefg",
        [],
        ["name"] * 7,
        [f"name{index}" for index in range(6)],
        [f"name{index}" for index in range(8)],
        [" ", "b", "c", "d", "e", "f", "g"],
        [1, "b", "c", "d", "e", "f", "g"],
    ],
)
def test_invalid_joint_configuration_fails_before_ik_creation(side, names):
    VRTeleOp.instances.clear()
    with pytest.raises(ValueError, match=f"{side}_joint_names"):
        PiperPicoVRTeleOpNode(
            parameter_overrides=[_Parameter(f"{side}_joint_names", names)]
        )
    assert VRTeleOp.instances == []


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize(
    "invalid", ["duplicate", "foreign", "missing", "length", "nan", "inf"]
)
def test_malformed_custom_feedback_clears_cache_and_blocks_commands(
    side, invalid
):
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    names = ["shoulder", "upper", "elbow", "forearm", "wrist", "tool", "jaw"]
    node = PiperPicoVRTeleOpNode(
        parameter_overrides=[_Parameter(f"{side}_joint_names", names)]
    )
    callback = getattr(node, f"sub_{side}_joint_state_callback")
    callback(_JointState(name=list(names), position=[0.0] * 7))
    assert getattr(node, f"{side}_joint_state_msg") is not None
    feedback = _JointState(name=list(names), position=[0.1] * 7)
    if invalid == "duplicate":
        feedback.name[1] = feedback.name[0]
    elif invalid == "foreign":
        feedback.name[0] = "joint1"
    elif invalid == "missing":
        feedback.name.pop()
        feedback.position.pop()
    elif invalid == "length":
        feedback.position.pop()
    else:
        feedback.position[0] = float(invalid)
    updates = []
    getattr(node, f"{side}_teleop").update_robot_joint_state = updates.append

    callback(feedback)

    assert updates == []
    assert getattr(node, f"{side}_joint_state_msg") is None
    node._handle_teleop_result(
        side=side,
        ret=_TeleOpResult(target_ee_pose=_Pose(), solution=[0.1] * 6),
        gripper=0.02,
        header=object(),
        joint_state_cmd_publisher=getattr(node, f"{side}_cmd_pub"),
        target_pose_publisher=getattr(node, f"{side}_target_pub"),
        joint_state=feedback,
    )
    assert getattr(node, f"{side}_cmd_pub").messages == []


def test_pico_node_does_not_publish_when_teleop_returns_none():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()

    node = PiperPicoVRTeleOpNode()
    left_teleop = VRTeleOp.instances[0]
    left_teleop.next_result = None

    node.timer_callback()

    assert node.left_cmd_pub.messages == []


def test_pico_is_the_default_operator_input_source():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()

    node = PiperPicoVRTeleOpNode()

    assert node._operator_input_source == "pico"
    assert node.keyboard_activation_sub is None
    assert node.keyboard_reset_sub is None
    assert isinstance(
        VRTeleOp.instances[0].init_kwargs["trigger_intent"], _Intent
    )


def test_keyboard_mode_selects_only_the_configured_side():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()
    node = PiperPicoVRTeleOpNode(
        parameter_overrides=[
            _Parameter("operator_input_source", "keyboard"),
            _Parameter("keyboard_control_side", "left"),
        ]
    )

    assert node.keyboard_activation_sub is not None
    assert node.keyboard_reset_sub is not None
    assert (
        VRTeleOp.instances[0].init_kwargs["trigger_intent"]
        is node._topic_activation_intent
    )
    assert isinstance(
        VRTeleOp.instances[1].init_kwargs["trigger_intent"],
        _Intent,
    )


def test_keyboard_activation_message_updates_topic_intent():
    import robo_orchard_teleop_ros2.robot.piper.pico_vr as pico_vr_module

    pico_vr_module.os.path.exists = lambda _path: True
    VRTeleOp.instances.clear()
    node = PiperPicoVRTeleOpNode(
        parameter_overrides=[
            _Parameter("operator_input_source", "keyboard"),
        ]
    )
    message = sys.modules[
        "robo_orchard_teleop_msg_ros2.msg"
    ].TeleopActivationState(state=2)

    node._on_keyboard_activation(message)

    assert node._topic_activation_intent.messages == [message]
