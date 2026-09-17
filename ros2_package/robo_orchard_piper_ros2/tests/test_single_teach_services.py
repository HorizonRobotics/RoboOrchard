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
from unittest.mock import Mock

import pytest


def _install_stub_modules():
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.node = types.ModuleType("rclpy.node")

    class FakeNode:
        def __init__(self, *args, **kwargs):
            self._parameters = {}
            self._created_services = []

        def declare_parameter(self, name, value):
            self._parameters[name] = value

        def get_parameter(self, name):
            value = self._parameters[name]
            parameter_value = types.SimpleNamespace(
                string_value=value if isinstance(value, str) else "",
                bool_value=bool(value) if isinstance(value, bool) else False,
                integer_value=value if isinstance(value, int) else 0,
                double_array_value=list(value)
                if isinstance(value, (list, tuple))
                else [],
            )
            return types.SimpleNamespace(
                value=value, get_parameter_value=lambda: parameter_value
            )

        def get_logger(self):
            return types.SimpleNamespace(
                info=lambda *args, **kwargs: None,
                warn=lambda *args, **kwargs: None,
                warning=lambda *args, **kwargs: None,
                error=lambda *args, **kwargs: None,
            )

        def create_publisher(self, *args, **kwargs):
            return types.SimpleNamespace(publish=lambda *a, **k: None)

        def create_service(self, service_type, name, callback):
            self._created_services.append(name)
            return types.SimpleNamespace(name=name, callback=callback)

        def create_subscription(self, *args, **kwargs):
            return types.SimpleNamespace()

        def create_timer(self, *args, **kwargs):
            return types.SimpleNamespace()

    fake_rclpy.node.Node = FakeNode
    sys.modules["rclpy"] = fake_rclpy
    sys.modules["rclpy.node"] = fake_rclpy.node

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs.msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs.msg.PoseStamped = type("PoseStamped", (), {})

    class FakeTransformStamped:
        def __init__(self) -> None:
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.child_frame_id = ""
            self.transform = types.SimpleNamespace(
                translation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            )

    geometry_msgs.msg.TransformStamped = FakeTransformStamped
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs.msg

    tf2_ros = types.ModuleType("tf2_ros")
    tf2_ros.TransformBroadcaster = lambda node: types.SimpleNamespace(
        sendTransform=Mock()
    )
    sys.modules["tf2_ros"] = tf2_ros

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.JointState = type("JointState", (), {})
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs.msg

    std_srvs = types.ModuleType("std_srvs")
    std_srvs.srv = types.ModuleType("std_srvs.srv")
    std_srvs.srv.Trigger = type(
        "Trigger",
        (),
        {"Request": object, "Response": object},
    )
    sys.modules["std_srvs"] = std_srvs
    sys.modules["std_srvs.srv"] = std_srvs.srv

    piper_msg = types.ModuleType("robo_orchard_piper_msg_ros2")
    piper_msg.msg = types.ModuleType("robo_orchard_piper_msg_ros2.msg")
    piper_msg.msg.PiperStatusMsg = type("PiperStatusMsg", (), {})
    sys.modules["robo_orchard_piper_msg_ros2"] = piper_msg
    sys.modules["robo_orchard_piper_msg_ros2.msg"] = piper_msg.msg

    fake_bridge = types.ModuleType("robo_orchard_piper_ros2.ros_bridge")
    fake_bridge.DEFAULT_JOINT_NAMES = tuple(
        [f"joint{index}" for index in range(1, 7)] + ["gripper"]
    )
    fake_bridge.validate_joint_names = list
    fake_bridge.create_piper = lambda *args, **kwargs: types.SimpleNamespace(
        GetArmStatus=lambda: types.SimpleNamespace(
            arm_status=types.SimpleNamespace(ctrl_mode=0x01, teach_status=0)
        )
    )
    fake_bridge.enable_arm_ctrl = lambda *args, **kwargs: None
    fake_bridge.get_arm_ee_pose = lambda *args, **kwargs: None
    fake_bridge.get_arm_state = lambda *args, **kwargs: None
    fake_bridge.get_arm_status = lambda *args, **kwargs: None
    fake_bridge.joint_control = lambda *args, **kwargs: None
    fake_bridge.switch_piper_ctrl_mode = lambda *args, **kwargs: True
    fake_bridge.set_ctrl_method = lambda *args, **kwargs: None
    sys.modules["robo_orchard_piper_ros2.ros_bridge"] = fake_bridge


_install_stub_modules()

sys.path.insert(0, "ros2_package/robo_orchard_piper_ros2")

import robo_orchard_piper_ros2.single as single_module  # noqa: E402
from robo_orchard_piper_ros2.single import PiperSingleControlNode  # noqa: E402


def _build_node(
    ctrl_mode: int,
    teach_status: int = 0,
    enable_mit_ctrl: bool = False,
):
    node = PiperSingleControlNode.__new__(PiperSingleControlNode)
    arm_status = types.SimpleNamespace(
        ctrl_mode=ctrl_mode, teach_status=teach_status
    )
    node.piper = types.SimpleNamespace(
        GetArmStatus=lambda: types.SimpleNamespace(arm_status=arm_status)
    )
    node._arm_status = arm_status
    node.enable_mit_ctrl = enable_mit_ctrl
    node._enable_flag = False
    node.get_logger = lambda: types.SimpleNamespace(
        warn=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
    )
    return node


def test_is_controlable_accepts_raw_sdk_can_mode_value():
    node = _build_node(ctrl_mode=0x01)
    node._enable_flag = True

    assert node.is_controlable() is True


def test_joint_callback_passes_names_and_rejects_invalid_commands(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x01)
    node._enable_flag = True
    node.joint_names = [f"axis{index}" for index in range(7)]
    node.gripper_exist = True
    node.gripper_val_mutiple = 1
    errors = []
    node.get_logger = lambda: types.SimpleNamespace(error=errors.append)
    calls = []

    def reject_command(piper, **kwargs):
        calls.append(kwargs)
        raise ValueError("Invalid names")

    monkeypatch.setattr(single_module, "joint_control", reject_command)
    message = types.SimpleNamespace(name=["right_joint1"], position=[0.1])

    node.joint_callback(message)

    assert calls[0]["joint_names"] == node.joint_names
    assert calls[0]["joint_data"] is message
    assert errors == ["Rejecting joint command: Invalid names"]


def test_reset_uses_the_same_names_for_feedback_and_commands(monkeypatch):
    node = _build_node(ctrl_mode=0x01)
    node._enable_flag = True
    node.joint_names = [f"axis{index}" for index in range(7)]
    node.gripper_exist = True
    node.gripper_val_mutiple = 1
    node.reset_joint_position = [0.0] * 7
    node.get_logger = lambda: types.SimpleNamespace(
        info=lambda *args: None, error=lambda *args: None
    )
    feedback_names = []
    commands = []
    names = node.joint_names

    def feedback(piper, configured_names):
        feedback_names.append(configured_names)
        return types.SimpleNamespace(
            name=names, position=[0.1] * 7, velocity=[], effort=[]
        )

    monkeypatch.setattr(single_module, "get_arm_state", feedback)
    monkeypatch.setattr(
        single_module,
        "joint_control",
        lambda piper, **kwargs: commands.append(kwargs),
    )
    monkeypatch.setattr(single_module.time, "sleep", lambda seconds: None)
    response = types.SimpleNamespace(success=False, message="")

    node._reset_ctrl_service_callback(None, response)

    assert response.success
    assert feedback_names == [names]
    assert len(commands) == 600
    assert all(command["joint_names"] == names for command in commands)
    assert all(command["joint_data"].name == names for command in commands)


def test_enable_arm_ctrl_in_active_teach_mode_does_not_attempt_recovery(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=1)

    calls = []

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        calls.append("set_ctrl_method")

    monkeypatch.setattr(
        single_module,
        "switch_piper_ctrl_mode",
        lambda *args, **kwargs: calls.append("switch_ctrl_mode"),
    )
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl()

    assert ret is False
    assert node._enable_flag is False
    assert calls == []


def test_enable_arm_ctrl_in_post_teach_mode_succeeds_after_ctrl_mode_recovery(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=2)

    calls = []

    def fake_switch(*args, **kwargs):
        calls.append("switch_ctrl_mode")
        node._arm_status.ctrl_mode = 0x01
        return True

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        calls.append("set_ctrl_method")

    monkeypatch.setattr(
        single_module,
        "switch_piper_ctrl_mode",
        fake_switch,
    )
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl()

    assert ret is True
    assert node._enable_flag is True
    assert calls == ["switch_ctrl_mode"]


def test_enable_arm_ctrl_in_post_teach_mode_does_not_fail_on_immediate_status(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=2)

    calls = []

    def fake_switch(*args, **kwargs):
        calls.append("switch_ctrl_mode")
        return True

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        calls.append("set_ctrl_method")

    monkeypatch.setattr(
        single_module,
        "switch_piper_ctrl_mode",
        fake_switch,
    )
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl()

    assert ret is True
    assert node._enable_flag is True
    assert calls == ["switch_ctrl_mode"]


def test_enable_arm_ctrl_in_fresh_power_on_does_not_fail_on_stale_status(
    monkeypatch,
):
    # Simulate a cold boot where the piper SDK's first GetArmStatus returns
    # a zero-initialised struct (ctrl_mode=0x00) because no CAN status frame
    # has arrived yet, and where the post-MotionCtrl_2 status read still
    # reflects the pre-command value.
    node = _build_node(ctrl_mode=0x00, teach_status=0)

    calls = []

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        # set_ctrl_method issues MotionCtrl_2(0x01, ...) but the status
        # frame has not refreshed yet; leave node._arm_status.ctrl_mode
        # at its stale value to exercise the race.
        calls.append("set_ctrl_method")

    monkeypatch.setattr(
        single_module,
        "switch_piper_ctrl_mode",
        lambda *args, **kwargs: calls.append("switch_ctrl_mode"),
    )
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl()

    assert ret is True
    assert node._enable_flag is True
    assert calls == ["enable", "set_ctrl_method"]


def test_enable_ctrl_service_fails_when_ctrl_mode_switch_times_out(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=2)
    logs = []
    node.get_logger = lambda: types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warn=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
        error=lambda message: logs.append(message),
    )

    def fake_switch(*args, **kwargs):
        raise TimeoutError("ctrl mode switch timed out")

    monkeypatch.setattr(single_module, "switch_piper_ctrl_mode", fake_switch)
    monkeypatch.setattr(
        single_module, "set_ctrl_method", lambda *args, **kwargs: None
    )

    response = types.SimpleNamespace(success=None, message=None)

    ret = node._enable_ctrl_service_callback(object(), response)

    assert ret is response
    assert response.success is False
    assert "unexpected error occurred" in response.message.lower()
    assert "ctrl mode switch timed out" in response.message
    assert node._enable_flag is False
    assert logs == ["Error while enabling arm: ctrl mode switch timed out"]


def test_enable_ctrl_service_retries_when_flag_set_but_not_controlable(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x00, teach_status=0)
    node._enable_flag = True
    calls = []
    node.get_logger = lambda: types.SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warn=lambda *args, **kwargs: None,
        warning=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None,
    )

    def fake_enable_arm_ctrl():
        calls.append("enable")
        node._arm_status.ctrl_mode = 0x01
        return True

    monkeypatch.setattr(node, "enable_arm_ctrl", fake_enable_arm_ctrl)

    response = types.SimpleNamespace(success=None, message=None)

    ret = node._enable_ctrl_service_callback(object(), response)

    assert ret is response
    assert response.success is True
    assert response.message == "Arm enabled successfully."
    assert calls == ["enable"]


def test_auto_enable_timeout_does_not_abort_node_startup(monkeypatch):
    base_node = PiperSingleControlNode.__mro__[1]
    original_declare_parameter = base_node.declare_parameter
    logs = []

    def fake_declare_parameter(self, name, value):
        if name == "auto_enable_arm_ctrl":
            value = True
        return original_declare_parameter(self, name, value)

    def fake_get_logger(self):
        return types.SimpleNamespace(
            info=lambda *args, **kwargs: None,
            warn=lambda *args, **kwargs: None,
            warning=lambda message: logs.append(message),
            error=lambda *args, **kwargs: None,
        )

    def fake_enable_arm_ctrl(self):
        raise TimeoutError("enable timed out")

    monkeypatch.setattr(base_node, "declare_parameter", fake_declare_parameter)
    monkeypatch.setattr(base_node, "get_logger", fake_get_logger)
    monkeypatch.setattr(
        PiperSingleControlNode, "enable_arm_ctrl", fake_enable_arm_ctrl
    )

    node = PiperSingleControlNode()

    assert node._enable_flag is False
    assert node._created_services == ["enable_ctrl", "reset_ctrl"]
    assert logs == ["Auto enable timed out. enable timed out"]


def test_node_does_not_register_disable_ctrl_service():
    node = PiperSingleControlNode()

    assert node._created_services == ["enable_ctrl", "reset_ctrl"]


def _override_parameters(
    monkeypatch: pytest.MonkeyPatch, overrides: dict[str, str | bool]
) -> None:
    base_node = PiperSingleControlNode.__mro__[1]
    original_declare_parameter = base_node.declare_parameter

    def declare_parameter(self, name: str, value: object) -> None:
        original_declare_parameter(self, name, overrides.get(name, value))

    monkeypatch.setattr(base_node, "declare_parameter", declare_parameter)


def test_ee_tf_is_enabled_by_default() -> None:
    node = PiperSingleControlNode()

    assert node.base_frame_id == "base_link"
    assert node.ee_frame_id == "end_effector"
    assert node.publish_ee_tf is True
    assert node._tf_broadcaster is not None


@pytest.mark.parametrize("publish_ee_tf", [True, False])
@pytest.mark.parametrize("role", ["left", "right", "left_master"])
def test_ee_pose_and_tf_use_the_same_feedback_and_timestamp(
    monkeypatch: pytest.MonkeyPatch, publish_ee_tf: bool, role: str
) -> None:
    base_frame_id = f"{role}_base_link"
    ee_frame_id = f"{role}_end_effector"
    joint_names = [f"{role}_axis{index}" for index in range(7)]
    _override_parameters(
        monkeypatch,
        {
            "joint_names": joint_names,
            "base_frame_id": base_frame_id,
            "ee_frame_id": ee_frame_id,
            "publish_ee_tf": publish_ee_tf,
        },
    )
    broadcaster_factory = Mock()
    monkeypatch.setattr(
        single_module, "TransformBroadcaster", broadcaster_factory
    )
    node = PiperSingleControlNode()
    assert node.joint_names == joint_names
    node.arm_status_pub = Mock()
    node.joint_pub = Mock()
    node.end_pose_pub = Mock()
    node.get_clock = Mock()
    arm_status = object()
    joint_state = types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=None)
    )
    monkeypatch.setattr(
        single_module, "get_arm_status", Mock(return_value=arm_status)
    )
    get_joint_state = Mock(return_value=joint_state)
    monkeypatch.setattr(single_module, "get_arm_state", get_joint_state)
    get_ee_pose = Mock()
    monkeypatch.setattr(single_module, "get_arm_ee_pose", get_ee_pose)

    for sample in range(2):
        stamp = types.SimpleNamespace(sec=sample + 1, nanosec=100)
        ee_pose = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=None, frame_id=""),
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.1 + sample, y=-0.2, z=0.3),
                orientation=types.SimpleNamespace(x=0.5, y=-0.5, z=0.5, w=0.5),
            ),
        )
        get_ee_pose.return_value = ee_pose
        node.get_clock.return_value.now.return_value.to_msg.side_effect = [
            stamp,
            stamp,
        ]

        node.publish_callback()

        get_joint_state.assert_called_with(node.piper, joint_names)
        node.arm_status_pub.publish.assert_called_with(arm_status)
        node.joint_pub.publish.assert_called_with(joint_state)
        node.end_pose_pub.publish.assert_called_with(ee_pose)
        assert ee_pose.header.frame_id == base_frame_id
        assert ee_pose.header.stamp == stamp
        assert get_ee_pose.call_count == sample + 1
        if publish_ee_tf:
            broadcaster_factory.assert_called_once_with(node)
            send_transform = node._tf_broadcaster.sendTransform
            assert send_transform.call_count == sample + 1
            transform = send_transform.call_args.args[0]
            assert transform.header.frame_id == base_frame_id
            assert transform.header.stamp == ee_pose.header.stamp
            assert transform.child_frame_id == ee_frame_id
            assert transform.transform.translation == ee_pose.pose.position
            assert transform.transform.rotation == ee_pose.pose.orientation
        else:
            broadcaster_factory.assert_not_called()
            assert node._tf_broadcaster is None


@pytest.mark.parametrize(
    "base_frame_id,ee_frame_id",
    [
        ("", "end_effector"),
        ("base_link", ""),
        (" ", "end_effector"),
        ("base_link", "base_link"),
    ],
)
def test_invalid_ee_frames_fail_before_connecting_to_can(
    monkeypatch: pytest.MonkeyPatch, base_frame_id: str, ee_frame_id: str
) -> None:
    _override_parameters(
        monkeypatch,
        {"base_frame_id": base_frame_id, "ee_frame_id": ee_frame_id},
    )
    create_piper = Mock()
    monkeypatch.setattr(single_module, "create_piper", create_piper)

    with pytest.raises(ValueError, match="base_frame_id and ee_frame_id"):
        PiperSingleControlNode()

    create_piper.assert_not_called()
