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

import importlib
import sys
import types

import pytest


def _install_stub_modules():
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs.msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs.msg.PoseStamped = type("PoseStamped", (), {})
    sys.modules["geometry_msgs"] = geometry_msgs
    sys.modules["geometry_msgs.msg"] = geometry_msgs.msg

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.msg.JointState = type("JointState", (), {})
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs.msg

    piper_sdk = types.ModuleType("piper_sdk")
    piper_sdk.C_PiperInterface = type("C_PiperInterface", (), {})
    sys.modules["piper_sdk"] = piper_sdk

    piper_msg = types.ModuleType("robo_orchard_piper_msg_ros2")
    piper_msg.msg = types.ModuleType("robo_orchard_piper_msg_ros2.msg")
    piper_msg.msg.PiperStatusMsg = type("PiperStatusMsg", (), {})
    sys.modules["robo_orchard_piper_msg_ros2"] = piper_msg
    sys.modules["robo_orchard_piper_msg_ros2.msg"] = piper_msg.msg

    scipy = types.ModuleType("scipy")
    scipy.spatial = types.ModuleType("scipy.spatial")
    scipy.spatial.transform = types.ModuleType("scipy.spatial.transform")

    class FakeRotation:
        @staticmethod
        def from_euler(*args, **kwargs):
            return types.SimpleNamespace(as_quat=lambda: [0.0, 0.0, 0.0, 1.0])

    scipy.spatial.transform.Rotation = FakeRotation
    sys.modules["scipy"] = scipy
    sys.modules["scipy.spatial"] = scipy.spatial
    sys.modules["scipy.spatial.transform"] = scipy.spatial.transform


_install_stub_modules()

sys.path.insert(0, "ros2_package/robo_orchard_piper_ros2")
sys.modules.pop("robo_orchard_piper_ros2.ros_bridge", None)

ros_bridge = importlib.import_module("robo_orchard_piper_ros2.ros_bridge")


def test_switch_piper_ctrl_mode_retries_until_target_mode_is_observed(
    monkeypatch,
):
    arm_status = types.SimpleNamespace(ctrl_mode=0x02)
    motion_calls = []
    clock = {"now": 0.0}

    def fake_motion(*args, **kwargs):
        motion_calls.append(args)
        if len(motion_calls) == 3:
            arm_status.ctrl_mode = 0x01

    piper = types.SimpleNamespace(
        GetArmStatus=lambda: types.SimpleNamespace(arm_status=arm_status),
        MotionCtrl_2=fake_motion,
    )

    monkeypatch.setattr(ros_bridge.time, "time", lambda: clock["now"])
    monkeypatch.setattr(
        ros_bridge.time,
        "sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )

    ros_bridge.switch_piper_ctrl_mode(piper, 0x01, timeout=5.0)
    assert len(motion_calls) == 3


def test_switch_piper_ctrl_mode_times_out_when_target_mode_stays_unmatched(
    monkeypatch,
):
    arm_status = types.SimpleNamespace(ctrl_mode=0x02)
    motion_calls = []
    clock = {"now": 0.0}

    piper = types.SimpleNamespace(
        GetArmStatus=lambda: types.SimpleNamespace(arm_status=arm_status),
        MotionCtrl_2=lambda *args, **kwargs: motion_calls.append(args),
    )

    monkeypatch.setattr(ros_bridge.time, "time", lambda: clock["now"])
    monkeypatch.setattr(
        ros_bridge.time,
        "sleep",
        lambda seconds: clock.__setitem__("now", clock["now"] + seconds),
    )

    with pytest.raises(TimeoutError):
        ros_bridge.switch_piper_ctrl_mode(piper, 0x01, timeout=0.5)

    assert motion_calls
