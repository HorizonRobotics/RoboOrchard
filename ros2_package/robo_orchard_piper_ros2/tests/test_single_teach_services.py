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


def _install_stub_modules():
    fake_rclpy = types.ModuleType("rclpy")
    fake_rclpy.node = types.ModuleType("rclpy.node")
    fake_rclpy.node.Node = object
    sys.modules.setdefault("rclpy", fake_rclpy)
    sys.modules.setdefault("rclpy.node", fake_rclpy.node)

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs.msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs.msg.PoseStamped = type("PoseStamped", (), {})
    sys.modules.setdefault("geometry_msgs", geometry_msgs)
    sys.modules.setdefault("geometry_msgs.msg", geometry_msgs.msg)

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.JointState = type("JointState", (), {})
    sys.modules.setdefault("sensor_msgs", sensor_msgs)
    sys.modules.setdefault("sensor_msgs.msg", sensor_msgs.msg)

    std_srvs = types.ModuleType("std_srvs")
    std_srvs.srv = types.ModuleType("std_srvs.srv")
    std_srvs.srv.Trigger = type(
        "Trigger",
        (),
        {"Request": object, "Response": object},
    )
    sys.modules.setdefault("std_srvs", std_srvs)
    sys.modules.setdefault("std_srvs.srv", std_srvs.srv)

    piper_msg = types.ModuleType("robo_orchard_piper_msg_ros2")
    piper_msg.msg = types.ModuleType("robo_orchard_piper_msg_ros2.msg")
    piper_msg.msg.PiperStatusMsg = type("PiperStatusMsg", (), {})
    sys.modules.setdefault("robo_orchard_piper_msg_ros2", piper_msg)
    sys.modules.setdefault("robo_orchard_piper_msg_ros2.msg", piper_msg.msg)

    fake_bridge = types.ModuleType("robo_orchard_piper_ros2.ros_bridge")
    fake_bridge.create_piper = lambda *args, **kwargs: None
    fake_bridge.disable_arm_ctrl = lambda *args, **kwargs: None
    fake_bridge.enable_arm_ctrl = lambda *args, **kwargs: None
    fake_bridge.get_arm_ee_pose = lambda *args, **kwargs: None
    fake_bridge.get_arm_state = lambda *args, **kwargs: None
    fake_bridge.get_arm_status = lambda *args, **kwargs: None
    fake_bridge.joint_control = lambda *args, **kwargs: None
    fake_bridge.reset_piper_ctrl_mode = lambda *args, **kwargs: True
    fake_bridge.set_ctrl_method = lambda *args, **kwargs: None
    sys.modules.setdefault("robo_orchard_piper_ros2.ros_bridge", fake_bridge)


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
        warn=lambda *args, **kwargs: None
    )
    return node


def test_enable_arm_ctrl_in_active_teach_mode_does_not_attempt_recovery(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=1)

    calls = []

    def fake_reset(*args, **kwargs):
        calls.append("reset")
        return True

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        calls.append("set_ctrl_method")

    monkeypatch.setattr(single_module, "reset_piper_ctrl_mode", fake_reset)
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl(force_reset=True)

    assert ret is False
    assert node._enable_flag is False
    assert calls == []


def test_enable_arm_ctrl_in_post_teach_mode_succeeds_after_ctrl_mode_recovery(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=2)

    calls = []

    def fake_reset(*args, **kwargs):
        calls.append("reset")
        node._arm_status.ctrl_mode = 0x01
        return True

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        calls.append("set_ctrl_method")

    monkeypatch.setattr(single_module, "reset_piper_ctrl_mode", fake_reset)
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl(force_reset=True)

    assert ret is True
    assert node._enable_flag is True
    assert calls == ["reset", "set_ctrl_method"]


def test_enable_arm_ctrl_in_post_teach_mode_fails_if_ctrl_mode_stays_0x02(
    monkeypatch,
):
    node = _build_node(ctrl_mode=0x02, teach_status=2)

    calls = []

    def fake_reset(*args, **kwargs):
        calls.append("reset")
        return True

    def fake_enable(*args, **kwargs):
        calls.append("enable")

    def fake_set_ctrl_method(*args, **kwargs):
        calls.append("set_ctrl_method")

    monkeypatch.setattr(single_module, "reset_piper_ctrl_mode", fake_reset)
    monkeypatch.setattr(single_module, "enable_arm_ctrl", fake_enable)
    monkeypatch.setattr(single_module, "set_ctrl_method", fake_set_ctrl_method)

    ret = node.enable_arm_ctrl(force_reset=True)

    assert ret is False
    assert node._enable_flag is False
    assert calls == ["reset", "set_ctrl_method"]
